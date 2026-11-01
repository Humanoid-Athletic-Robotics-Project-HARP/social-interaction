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
    """Voxel clustering, gated association and alpha-beta velocity estimation.

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
        self._stamp = None
        self._next_id = 1

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
            if np.linalg.norm(hi - lo) > 1.4:
                member_array = np.array(members)
                _, tile_index = np.unique(cells[member_array] // 4, axis=0,
                                          return_inverse=True)
                groups = [member_array[tile_index == i] for i in range(tile_index.max() + 1)]
            else:
                groups = [members]
            for group in groups:
                lo, hi = low[group].min(axis=0), high[group].max(axis=0)
                center = (lo + hi) * 0.5
                radius = float(np.linalg.norm(hi - lo) * 0.5 + self.padding)
                components.append((center, radius))
            if len(components) > self.max_tracks:
                raise ValueError("Obstacle component capacity exceeded")
        return components

    def tracks_at(self, stamp):
        if not math.isfinite(stamp) or (self._stamp is not None and stamp < self._stamp):
            raise ValueError("Track query time must be finite and monotonic")
        result = []
        for track in self._tracks:
            age = stamp - track.last_seen
            if age <= self.max_age:
                result.append(replace(track, x=track.x + track.vx * age,
                                      y=track.y + track.vy * age,
                                      uncertainty=track.uncertainty + 0.15 * age + 0.05 * age * age))
        return result

    def clear_observed_free(self, ids):
        """Caller must provide positive ray evidence; elapsed time is insufficient."""
        ids = set(ids)
        self._tracks = [t for t in self._tracks if t.id not in ids]

    def update(self, points_xy_in_odom, stamp_seconds):
        stamp = float(stamp_seconds)
        if (not math.isfinite(stamp)
                or (self._stamp is not None and stamp <= self._stamp)):
            raise ValueError("Scan stamps must strictly increase")
        observations = self._clusters(points_xy_in_odom)
        previous = {t.id: t for t in self._tracks if stamp - t.last_seen <= self.max_age}
        predicted = self.tracks_at(stamp)
        # Separate a merged return with the previous motion estimates as seeds.
        # Every return belongs to a resulting occupied disc, including singletons.
        points = np.asarray(points_xy_in_odom, dtype=float).reshape(-1, 2)
        separated = []
        for center, radius in observations:
            seeds = [t for t in predicted if t.last_seen == self._stamp and t.hits >= 5
                     and np.linalg.norm(center-[t.x,t.y]) < radius
                     and .18 < t.radius < .5]
            if (len(seeds) >= 2 and radius < 1.3
                    and any(math.hypot(t.vx,t.vy) > .15 for t in seeds)):
                local = points[np.linalg.norm(points-center, axis=1) <= radius]
                centers = np.array([[t.x,t.y] for t in seeds])
                labels = np.argmin(np.linalg.norm(local[:,None]-centers[None],axis=2),axis=1)
                for i in range(len(seeds)):
                    group = local[labels == i]
                    if len(group):
                        lo, hi = group.min(axis=0), group.max(axis=0)
                        separated.append(((lo+hi)/2, float(np.linalg.norm(hi-lo)/2+self.padding)))
            else:
                separated.append((center,radius))
        observations = separated
        # Match the whole frame jointly, so one locally closest match cannot
        # steal the only plausible observation from another crossing track.
        costs = np.full((len(predicted),len(observations)+len(predicted)),1e6)
        for row,track in enumerate(predicted):
            age = stamp - track.last_seen
            gate = .35+3.*age
            costs[row,len(observations)+row] = gate+.3
            for index, (center, radius) in enumerate(observations):
                distance = float(np.linalg.norm(center - [track.x, track.y]))
                if distance <= gate:
                    costs[row,index] = distance+.3*abs(radius-track.radius)
        rows,columns = linear_sum_assignment(costs)
        pairs = [(costs[row,col],predicted[row].id,col) for row,col in zip(rows,columns)
                 if col < len(observations) and costs[row,col] < 1e6]
        used_tracks, used_observations = set(), set()
        updated = []
        for _, ident, index in sorted(pairs):
            if ident in used_tracks or index in used_observations:
                continue
            old = previous[ident]
            center, radius = observations[index]
            dt = stamp - old.last_seen
            predicted_position = np.array([old.x + old.vx * dt, old.y + old.vy * dt])
            residual = center - predicted_position
            position = predicted_position + 0.8 * residual
            velocity = np.array([old.vx, old.vy]) + 0.3 * residual / dt
            magnitude = np.linalg.norm(velocity)
            if magnitude > 3.0:
                velocity *= 3.0 / magnitude
            # The filtered center must still enclose the latest observed extent.
            radius = max(radius + float(np.linalg.norm(position - center)), old.radius * 0.95)
            uncertainty = max(0.05, min(0.5, float(np.linalg.norm(residual)) * 0.5))
            updated.append(Track(ident, float(position[0]), float(position[1]),
                                 float(velocity[0]), float(velocity[1]), radius,
                                 uncertainty, stamp, old.hits + 1))
            used_tracks.add(ident)
            used_observations.add(index)
        # Preserve missing objects at their last measurement; projection is read-only.
        updated.extend(t for t in previous.values() if t.id not in used_tracks)
        next_id = self._next_id
        for index, (center, radius) in enumerate(observations):
            if index not in used_observations:
                updated.append(Track(next_id, float(center[0]), float(center[1]),
                                     0.0, 0.0, radius, 0.2, stamp, 1))
                next_id += 1
        if len(updated) > self.max_tracks:
            raise ValueError("Tracked obstacle capacity exceeded")
        # Failed updates leave the previous state intact.
        self._tracks = sorted(updated, key=lambda t: t.id)
        self._next_id = next_id
        self._stamp = stamp
        return self.tracks_at(stamp)
