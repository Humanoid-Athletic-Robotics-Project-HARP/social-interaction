"""Bounded, class-agnostic tracking of obstacle points already expressed in odom.

The caller must remove the floor/self returns and use transforms at scan time.
Voxel components describe observed surfaces, not complete object reconstruction.
No component is discarded for being small, static, or large. Capacity failures
raise ValueError instead of returning an incomplete map that looks clear.
"""

from dataclasses import dataclass, replace
import math

import numpy as np
from scipy.optimize import linear_sum_assignment


STRUCTURE_ID = 10**9    # IDs of static surface tiles start here, beyond any object ID
MEASUREMENT_NOISE = .05   # m, 1 sigma of a fitted centre (2 cm range noise, partial views)
ACCELERATION_NOISE = 1.   # m/s^2, 1 sigma: walkers start, stop and turn
FIT_PADDING = .06         # m beyond the farthest visible return of a fitted disc


def _spread(covariance):
    """Planning uncertainty: position spread plus a share of velocity spread."""
    return float(math.sqrt(max(covariance[0, 0], covariance[1, 1]))
                 + .3*math.sqrt(max(covariance[2, 2], covariance[3, 3])))


@dataclass(frozen=True)
class Track:
    id: int
    x: float
    y: float
    vx: float
    vy: float
    radius: float
    uncertainty: float
    last_seen: float
    hits: int


class Tracker:
    """Voxel clustering, gated association and constant-velocity Kalman filtering.

    Coordinates and sizes are meters, times seconds. ``tracks_at`` returns
    predicted copies; querying does not advance or change the filter. Missed
    tracks coast for ``max_age`` with increasing uncertainty. A sensor watchdog
    remains necessary: expiry here cannot establish that unobserved space is free.
    """

    def __init__(self, voxel=0.18, padding=0.10, max_age=2.5,
                 max_points=100000, max_cells=4096, max_tracks=128):
        if (not all(math.isfinite(v) for v in (voxel, padding, max_age))
                or voxel <= 0 or padding < 0 or max_age <= 0
                or min(max_points, max_cells, max_tracks) <= 0):
            raise ValueError("Invalid tracker limits")
        self.voxel = voxel
        self.padding = padding
        self.max_age = max_age
        self.max_points = max_points
        self.max_cells = max_cells
        self.max_tracks = max_tracks
        self._tracks = []
        self._structure = {}
        self._stamp = None
        self._next_id = 1
        self._next_structure_id = STRUCTURE_ID
        self._covariance = {}
        # Each object's own size, learned from consistent views only, and how
        # many consistent scans remain before velocity learning resumes.
        self._size = {}
        self._settle = {}

    def _clusters(self, points):
        points = np.asarray(points, dtype=float)
        if points.size == 0:
            points = points.reshape((0, 2))
        if (points.ndim != 2 or points.shape[1] != 2
                or len(points) > self.max_points or not np.isfinite(points).all()
                or (len(points) and np.max(np.abs(points)) > 1e6)):
            raise ValueError("Invalid or excessive obstacle points")
        if not len(points):
            return []
        cells, inverse = np.unique(np.floor(points / self.voxel).astype(np.int64),
                                   axis=0, return_inverse=True)
        if len(cells) > self.max_cells:
            raise ValueError("Obstacle voxel capacity exceeded")
        # Extents of every original return survive downsampling.
        low = np.full((len(cells), 2), np.inf)
        high = np.full((len(cells), 2), -np.inf)
        np.minimum.at(low, inverse, points)
        np.maximum.at(high, inverse, points)
        remaining = {tuple(cell): i for i, cell in enumerate(cells)}
        components = []
        while remaining:
            seed = next(iter(remaining))
            pending = [seed]
            members = [remaining.pop(seed)]
            while pending:
                x, y = pending.pop()
                for dx in (-1, 0, 1):
                    for dy in (-1, 0, 1):
                        key = (x + dx, y + dy)
                        index = remaining.pop(key, None)
                        if index is not None:
                            pending.append(key)
                            members.append(index)
            lo, hi = low[members].min(axis=0), high[members].max(axis=0)
            # A wall's single circumscribed circle would fill its free corridor.
            # Keep compact objects together; tile long components, preserving every
            # observed return rather than silently dropping large objects.
            if np.linalg.norm(hi - lo) > 1.4 or self._thin(cells[members]):
                member_array = np.array(members)
                _, tile_index = np.unique(cells[member_array] // 2, axis=0,
                                          return_inverse=True)
                groups = [(member_array[tile_index == i], tuple(cells[member_array[tile_index == i][0]] // 2))
                          for i in range(tile_index.max() + 1)]
            else:
                groups = [(members, None)]
            for group, key in groups:
                members_of_group = np.flatnonzero(np.isin(inverse, group)) if key is None else None
                lo, hi = low[group].min(axis=0), high[group].max(axis=0)
                center = (lo + hi) * 0.5
                # A measured wall face has no hidden depth on the robot's side:
                # full padding would push every 0.36 m tile 0.1 m into a hallway.
                padding = self.padding if key is None or min(hi - lo) > .1 else .05
                radius = float(np.linalg.norm(hi - lo) * 0.5 + padding)
                components.append((center, radius, key, members_of_group))
            if (sum(c[2] is None for c in components) > self.max_tracks
                    or len(components) > self.max_cells):
                raise ValueError("Obstacle component capacity exceeded")
        return components

    def _thin(self, cells):
        """A partly hidden wall: a long, one-voxel-thin line of returns.

        Shorter than the 1.4 m tiling size, it would otherwise be tracked as an
        object whose ID churns with every change in which part is visible.
        """
        if len(cells) < 5:
            return False
        centered = cells - cells.mean(axis=0)
        axes = np.linalg.svd(centered.astype(float), full_matrices=False)[2]
        spans = np.ptp(centered @ axes.T, axis=0)*self.voxel
        return spans[0] >= .9 and spans[1] <= .2

    def tracks_at(self, stamp):
        if not math.isfinite(stamp) or (self._stamp is not None and stamp < self._stamp):
            raise ValueError("Track query time must be finite and monotonic")
        return self._project(self._tracks, stamp)+self._project(self._structure.values(), stamp)

    def _project(self, tracks, stamp):
        result = []
        for track in tracks:
            age = stamp - track.last_seen
            if age > self.max_age:
                continue
            covariance = self._covariance.get(track.id)
            if covariance is None:   # static structure
                uncertainty = track.uncertainty + 0.15 * age + 0.05 * age * age
            else:
                uncertainty = _spread(self._predict(covariance, age))
            result.append(replace(track, x=track.x + track.vx * age, y=track.y + track.vy * age,
                                  uncertainty=uncertainty))
        return result

    @staticmethod
    def _predict(covariance, dt):
        # Constant velocity, white acceleration noise of ACCELERATION_NOISE (1 sigma).
        f = np.eye(4)
        f[0, 2] = f[1, 3] = dt
        g = np.array([dt*dt/2, dt*dt/2, dt, dt])
        q = np.zeros((4, 4))
        for axis in (0, 1):
            index = [axis, axis+2]
            q[np.ix_(index, index)] = np.outer(g[index], g[index])
        return f @ covariance @ f.T + ACCELERATION_NOISE**2*q

    def _fit(self, points, origin):
        """Centre and radius of a compact object from its visible side.

        The visible side of a convex body is its front. Its centre lies about
        half its visible width behind the nearest returns, not at the middle of
        the returns' bounding box, which sits toward the sensor and inflates the
        disc on the robot's side. Deep or irregular shapes keep the bounding box.
        """
        low, high = points.min(axis=0), points.max(axis=0)
        center = (low+high)/2
        box = (center, float(np.linalg.norm(high-low)/2+self.padding))
        if origin is None or len(points) < 3:
            return box
        ray = center-origin
        reach = float(np.linalg.norm(ray))
        if reach < .3:
            return box
        u = ray/reach
        w = np.array([-u[1], u[0]])
        lateral, depth = points @ w, points @ u
        width, deep = float(np.ptp(lateral)), float(np.ptp(depth))
        if deep > .75*width+.05:
            return box
        half = max(width/2, .05)
        center = w*(lateral.min()+lateral.max())/2+u*(depth.min()+half)
        return center, float(np.max(np.linalg.norm(points-center, axis=1)))+FIT_PADDING

    def clear_observed_free(self, ids):
        """Caller must provide positive ray evidence; elapsed time is insufficient."""
        ids = set(ids)
        self._tracks = [t for t in self._tracks if t.id not in ids]
        self._covariance = {k: v for k, v in self._covariance.items() if k not in ids}
        self._size = {k: v for k, v in self._size.items() if k not in ids}
        self._settle = {k: v for k, v in self._settle.items() if k not in ids}
        self._structure = {k: t for k, t in self._structure.items() if t.id not in ids}

    def update(self, points_xy_in_odom, stamp_seconds, origin=None):
        """origin: sensor xy at scan time, for shape-aware centres (optional)."""
        stamp = float(stamp_seconds)
        origin = None if origin is None else np.asarray(origin, dtype=float)[:2]
        if (not math.isfinite(stamp)
                or (self._stamp is not None and stamp <= self._stamp)):
            raise ValueError("Scan stamps must strictly increase")
        points = np.asarray(points_xy_in_odom, dtype=float)
        if points.size == 0:
            points = points.reshape((0, 2))
        if (points.ndim != 2 or points.shape[1] != 2 or len(points) > self.max_points
                or not np.isfinite(points).all() or (len(points) and np.max(np.abs(points)) > 1e6)):
            raise ValueError("Invalid or excessive obstacle points")
        # A walker beside a wall (or people abreast) joins a long component that
        # would become zero-velocity wall tiles. Returns inside a recently seen
        # moving track's predicted disc are claimed for it before tiling.
        claimed, claims = np.zeros(len(points), bool), []
        # The claim uses the walker's own learned size, so it never reaches into
        # the wall beside them.
        movers = [t for t in self._project(self._tracks, stamp) if stamp-t.last_seen <= .35
                  and t.hits >= 3 and math.hypot(t.vx, t.vy) > .15
                  and self._size.get(t.id, t.radius) < .6]
        if movers and len(points):
            centers = np.array([[t.x, t.y] for t in movers])
            gaps = np.linalg.norm(points[:, None]-centers[None], axis=2)
            inside = gaps <= np.array([self._size.get(t.id, t.radius)+.03 for t in movers])[None]
            owner = np.where(inside.any(axis=1), np.argmin(np.where(inside, gaps, np.inf), axis=1), -1)
            for k in range(len(movers)):
                index = np.flatnonzero(owner == k)
                if len(index) >= 3:
                    claims.append(index)
                    claimed[index] = True
        rest = np.flatnonzero(~claimed)
        components = [(c, r, k, None if m is None else rest[m])
                      for c, r, k, m in self._clusters(points[rest])]
        # Tiles of long surfaces (walls) are static structure: keyed by their
        # fixed grid tile, re-measured every scan, never given a velocity or
        # matched against moving objects. Tracking them as objects let shifting
        # visible extents spawn new IDs until the tracker overflowed in a building.
        structure = {k: t for k, t in self._structure.items() if stamp - t.last_seen <= self.max_age}
        next_structure = self._next_structure_id
        for center, radius, key, _ in components:
            if key is None:
                continue
            old = structure.get(key)
            ident = old.id if old else next_structure
            next_structure += old is None
            structure[key] = Track(ident, float(center[0]), float(center[1]), 0., 0., radius,
                                   .05, stamp, (old.hits if old else 0)+1)
        if len(structure) > self.max_cells:
            raise ValueError("Obstacle structure capacity exceeded")
        previous = {t.id: t for t in self._tracks if stamp - t.last_seen <= self.max_age}
        predicted = self._project(self._tracks, stamp)
        # Separate a merged return with the previous motion estimates as seeds.
        # Every return belongs to a resulting occupied disc, including singletons.
        observations = [self._fit(points[index], origin) for index in claims]
        for center, radius, key, members in components:
            if key is not None:
                continue
            # Seeds may have missed a scan or two: a briefly unseen walker brushing
            # past a column must still split from it.
            seeds = [t for t in predicted if stamp-t.last_seen <= .35 and t.hits >= 3
                     and np.linalg.norm(center-[t.x,t.y]) < radius
                     and .18 < t.radius < .5]
            if (len(seeds) >= 2 and radius < 1.3
                    and any(math.hypot(t.vx,t.vy) > .15 for t in seeds)):
                local = points[members]
                centers = np.array([[t.x,t.y] for t in seeds])
                labels = np.argmin(np.linalg.norm(local[:,None]-centers[None],axis=2),axis=1)
                observations += [self._fit(local[labels == i], origin)
                                 for i in range(len(seeds)) if np.any(labels == i)]
            else:
                observations.append(self._fit(points[members], origin))
        # Match the whole frame jointly, so one locally closest match cannot
        # steal the only plausible observation from another crossing track.
        costs = np.full((len(predicted),len(observations)+len(predicted)),1e6)
        for row,track in enumerate(predicted):
            age = stamp - track.last_seen
            gate = .35+3.*age
            # Missing an older track must not cost MORE than missing a fresh one.
            # Otherwise Hungarian steals a fresh object's exact observation to
            # avoid the large dummy cost of a distant stale ID (identity jumps).
            costs[row,len(observations)+row] = .95
            missed = max(0., self._stamp-track.last_seen)
            for index, (center, radius) in enumerate(observations):
                distance = float(np.linalg.norm(center - [track.x, track.y]))
                if distance <= gate:
                    costs[row,index] = distance+.3*abs(radius-track.radius)+.5*missed
        rows,columns = linear_sum_assignment(costs)
        pairs = [(costs[row,col],predicted[row].id,col) for row,col in zip(rows,columns)
                 if col < len(observations) and costs[row,col] < 1e6]
        used_tracks, used_observations = set(), set()
        updated = []
        covariances = {t.id: self._covariance[t.id] for t in previous.values()}
        sizes = {t.id: self._size.get(t.id, t.radius) for t in previous.values()}
        settles = {t.id: self._settle.get(t.id, 0) for t in previous.values()}
        for _, ident, index in sorted(pairs):
            if ident in used_tracks or index in used_observations:
                continue
            old = previous[ident]
            center, radius = observations[index]
            dt = stamp - old.last_seen
            state = np.array([old.x + old.vx * dt, old.y + old.vy * dt, old.vx, old.vy])
            covariance = self._predict(self._covariance[ident], dt)
            gain = covariance[:, :2] @ np.linalg.inv(covariance[:2, :2]+MEASUREMENT_NOISE**2*np.eye(2))
            size, settle = self._size.get(ident, radius), self._settle.get(ident, 0)
            # Normal views of a walker vary (occlusion, few returns at range);
            # a merge grows the blob well beyond the object's own size. A new
            # track's first views are often partial: learn its size first.
            if old.hits < 8:
                size += .4*(radius-size)
            elif radius > 1.35*size+.05 or radius < size/1.5-.08:
                # A view unlike the object's own size is a merge or split, not
                # acceleration: move the disc, but never let a column inherit a
                # walker's motion, during the merge or as it snaps back after.
                settle = 2
            else:
                # Grow slowly, shrink fast: a lasting merge must not redefine
                # the object as large, while a clear view restores it quickly.
                size += (.03 if radius > size else .2)*(radius-size)
                settle = max(0, settle-1)
            if settle:
                gain[2:] = 0.
            sizes[ident], settles[ident] = size, settle
            state = state+gain @ (center-state[:2])
            # Joseph form: stays symmetric and valid when the velocity gain is zeroed.
            update = np.eye(4)-gain @ np.eye(2, 4)
            covariance = update @ covariance @ update.T+MEASUREMENT_NOISE**2*gain @ gain.T
            magnitude = np.linalg.norm(state[2:])
            if magnitude > 3.0:
                state[2:] *= 3.0 / magnitude
            covariances[ident] = covariance
            # The filtered center must still enclose the latest observed extent.
            radius = max(radius + float(np.linalg.norm(state[:2] - center)), old.radius * 0.95)
            updated.append(Track(ident, float(state[0]), float(state[1]),
                                 float(state[2]), float(state[3]), radius,
                                 _spread(covariance), stamp, old.hits + 1))
            used_tracks.add(ident)
            used_observations.add(index)
        # Preserve missing objects at their last measurement; projection is read-only.
        updated.extend(t for t in previous.values() if t.id not in used_tracks)
        next_id = self._next_id
        for index, (center, radius) in enumerate(observations):
            if index not in used_observations:
                # Velocity unknown until a second scan: large initial spread.
                covariances[next_id] = np.diag([MEASUREMENT_NOISE**2]*2+[1.]*2)
                sizes[next_id], settles[next_id] = radius, 0
                updated.append(Track(next_id, float(center[0]), float(center[1]),
                                     0.0, 0.0, radius, _spread(covariances[next_id]), stamp, 1))
                next_id += 1
        if len(updated) > self.max_tracks:
            raise ValueError("Tracked obstacle capacity exceeded")
        # Failed updates leave the previous state intact.
        self._tracks = sorted(updated, key=lambda t: t.id)
        self._covariance = {t.id: covariances[t.id] for t in updated}
        self._size = {t.id: sizes[t.id] for t in updated}
        self._settle = {t.id: settles[t.id] for t in updated}
        self._next_id = next_id
        self._structure, self._next_structure_id = structure, next_structure
        self._stamp = stamp
        return self.tracks_at(stamp)
