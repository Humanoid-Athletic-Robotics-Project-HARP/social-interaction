"""Obstacles that move, such as a person crossing the robot's path, for our simulated
lidar (harp/lidar.py). HIM's simulator doesn't move them, so they exist only in the
lidar's copy of the world: the robot sees them but can't touch them.

In a world file, a body with mocap="true" is a mover; its geoms are its shape.
It walks in a straight line at a constant speed, back and forth:
  - the body's pos is where it starts,
  - a site <body>_end in the worldbody is where it turns back,
  - a numeric <body>_duration is how many seconds one way takes,
  - an optional numeric <body>_delay: it waits until the robot sets off, then
    this many seconds more. Setting off is the course test's goal being sent
    (course_goal's goal_sent event), or when driving by hand, the robot
    walking START_DISTANCE from where it stood when HIM's sim handed control to
    our policy.
    Without it, the mover starts at sim time 0.
  - an optional numeric <body>_trips: how many one-way walks before it stops
    where it is (1 = start to end once). Without it, it never stops.
  - an optional numeric <body>_route, x0 y0 x1 y1 ...: instead of a straight
    line, it walks these points in turn (through doors, along a hallway) at a
    constant speed, one way in <body>_duration, and turns back at either end.
    The site <body>_end is then optional (the last point).
Each other numeric holds one finite number: duration positive, delay and trips not negative.
These scripted movers never react to the robot, so a course can replay them exactly.
With the task's "crowd": "aware" (course.py --crowd aware), the lidar uses AwareMovers
instead: the same routes, but they give way to the robot. Where they went then depends
on the robot, so the lidar logs it, and scoring and the replay follow that log.

  <custom>
    <numeric name="person_duration" data="5"/>
    <numeric name="person_delay" data="3"/>
    <numeric name="person_trips" data="1"/>
  </custom>
  <body name="person" mocap="true" pos="2.5 -3 0">
    <geom name="person" type="cylinder" size="0.25 0.85" pos="0 0 0.86"/>
  </body>
  <site name="person_end" pos="2.5 3 0"/>

Counting the delay from the robot setting off makes the mover meet the
robot at about the same point however long the controller and Nav2 take to
start. The lidar logs a mover_start event (mover, starts_at: the sim time it
starts walking).
"""

import json
import math

import numpy as np

from harp import events

START_DISTANCE = 0.3   # m the pelvis has to move from where it started
CROWDS = ("scripted", "aware")   # the task's "crowd": scripted (the default) or aware movers
# How an aware crowd's movers walk and give way (AwareMovers): every call (each lidar scan)
# each picks, from a few headings and speeds around its route's, the cheapest by how much
# progress along its route it gives up, how far it is from the route, and how near it would
# pass the robot (if the robot keeps its velocity) or a wall; then speeds up or slows to it.
STEP = 0.02               # s per integration step (the lidar calls every 0.1 s)
MAX_STEPS = 500           # steps per call at most (a long gap takes longer steps)
LOOKAHEAD = 0.5           # m along its route ahead, where its route direction is taken
ROUTE_PULL = 0.5          # 1/s: speed back toward its route per m off it
RETURN = 0.5              # share of its speed it returns to the route at, at most
LATERAL = 0.5             # weight of returning to the route, against progress along it
ARRIVE = 0.1              # m from a route's end where a mover turns back
ACCELERATION = 0.9        # m/s^2
TURNS = np.radians([0, -25, 25, -50, 50, -75, 75, -100, 100])   # right first: ties keep right
PACES = (1.0, 0.6, 0.3)   # shares of its speed, besides standing still
SEES = 4.0                # m from the robot beyond which a mover ignores it
ROBOT_RADIUS = 0.35       # m from the robot's base to the edge of its body, arms included
COMFORT = 0.45            # m of gap a mover would rather keep from the robot's body
SOON = 0.3                # s ahead the nearest pass is counted from (now can't change)
URGENCY = 2.0             # s: sooner passes weigh more, by exp(-time / URGENCY); later ones
                          # still count, or slowing down would look safe and it would creep
CLOSE_COST = 10.0         # cost of passing through the robot (a 90 deg turn costs 2)
MIN_GAP = 0.15            # m of gap it never heads to within CONTACT_TIME, if any choice avoids it
CONTACT_TIME = 2.5        # s
BLOCKED_COST = 100.0
SIDE_COST = 0.5           # passing on the other side from the one it has started on
WALL_LOOK = (0.2, 0.4, 0.6)   # s ahead its positions are checked against walls
WALL_COST = 1.0           # heading into a wall or block (it would slide along it)
ROBOT_WINDOW = 0.5        # s the robot's velocity is measured over (its gait sways)
GIVE_UP = 6.0             # s a mover stands blocked (by the robot in a door, say) before turning back
OTHERS = 3.0              # m within which a mover gives way to another, at half the weight
                          # (the other gives way too), never letting their bodies overlap
OTHERS_COMFORT = 0.25     # m of gap it would rather keep from another mover
OTHERS_GAP = 0.05         # m of gap it never heads to within CONTACT_TIME, if avoidable
BOX = 6                   # mujoco.mjtGeom.mjGEOM_BOX; and sphere, capsule, ellipsoid, cylinder:
ROUND = (2, 3, 4, 5)


class Movers:
    def __init__(self, model, started=None, path=None):
        """started: {name: sim time} from mover_start events, to replay a run. path:
        (names, times, positions) from logged_path, to replay an aware crowd."""
        self.names, self.mocap, self.start, self.end, self.routes = [], [], [], [], []
        self.duration, self.delay, self.trips = [], [], []
        for b in range(model.nbody):
            if model.body_mocapid[b] < 0:
                continue
            name = model.body(b).name
            route = optional_route(model, name + "_route")
            try:
                end = model.site_pos[model.site(name + "_end").id].copy() if route is None else None
                duration = numeric(model, name + "_duration")
            except KeyError:
                raise SystemExit(f"Mover {name} needs a site {name}_end (or a {name}_route) and a "
                                 f"numeric {name}_duration (see submission/harp/movers.py)")
            if duration <= 0:
                raise SystemExit(f"Mover {name}_duration must be positive")
            delay, trips = optional(model, name + "_delay"), optional(model, name + "_trips")
            for suffix, value in (("delay", delay), ("trips", trips)):
                if value is not None and value < 0:
                    raise SystemExit(f"Mover {name}_{suffix} must not be negative")
            self.names.append(name)
            self.mocap.append(model.body_mocapid[b])
            start = model.body_pos[b].copy()
            if route is not None:
                # Straight lines between the body's position and each route point.
                points = np.vstack((start[:2], route))
                self.routes.append((points, np.r_[0., np.cumsum(np.linalg.norm(np.diff(points, axis=0), axis=1))]))
                end = np.r_[points[-1], start[2]]
            else:
                self.routes.append(None)
            self.start.append(start)
            self.end.append(end)
            self.duration.append(duration)
            self.delay.append(delay)
            self.trips.append(trips)
        # Sim time each mover starts walking; None until the robot has moved.
        self.started = [0.0 if d is None else None for d in self.delay]
        for i, name in enumerate(self.names):
            if started and name in started:
                self.started[i] = started[name]
        self.robot_start = None
        self.path = None
        if path is not None:
            names, times, positions = path
            if sorted(names) != sorted(self.names):
                raise ValueError(f"logged movers {list(names)} aren't the world's {self.names}")
            order = [list(names).index(n) for n in self.names]
            self.path = (np.asarray(times, dtype=float), np.asarray(positions, dtype=float)[:, order])

    def update(self, xy, t, released, goal_sent):
        """Every scan: starts movers once the robot sets off (our policy has control
        and a goal was sent, or it walked). xy is the pelvis position, t the sim time.
        Returns (name, sim time it starts walking) for each mover triggered now."""
        if all(s is not None for s in self.started) or not released:
            return []
        if not goal_sent:
            if self.robot_start is None:
                self.robot_start = xy.copy()
            if np.linalg.norm(xy - self.robot_start) < START_DISTANCE:
                return []
        begun = []
        for i, delay in enumerate(self.delay):
            if self.started[i] is None:
                self.started[i] = t + delay
                begun.append((self.names[i], self.started[i]))
        return begun

    def apply(self, data, t):
        """Moves every mover to where it is at sim time t."""
        if self.path is not None:
            times, positions = self.path
            for i, mocap in enumerate(self.mocap):
                data.mocap_pos[mocap][:2] = (np.interp(t, times, positions[:, i, 0]),
                                             np.interp(t, times, positions[:, i, 1]))
                data.mocap_pos[mocap][2] = self.start[i][2]
            return
        for i, mocap in enumerate(self.mocap):
            legs = 0.0 if self.started[i] is None else max(0.0, t - self.started[i]) / self.duration[i]
            if self.trips[i] is not None:
                legs = min(legs, self.trips[i])
            phase = legs % 2.0
            share = phase if phase <= 1.0 else 2.0 - phase   # out, then back
            if self.routes[i] is None:
                data.mocap_pos[mocap] = self.start[i] + share * (self.end[i] - self.start[i])
            else:
                points, along = self.routes[i]
                data.mocap_pos[mocap][:2] = point_at(points, along, share * along[-1])
                data.mocap_pos[mocap][2] = self.start[i][2]


class AwareMovers(Movers):
    """Movers that see the robot (the task's "crowd": "aware"). Each walks the same route
    at the same mean speed as a scripted mover, but speeds up, slows to turn back and
    changes direction gradually, and gives way to the robot: it goes round it, slows to
    let it cross or steps aside (right, if straight at each other), then heads back to
    its route, keeping off walls and blocks (boxes named wall* or block*) and giving way to
    each other the same way (each taking half), their bodies never overlapping.
    data.qpos[:2] holds the robot's true position when apply is called (the lidar's copy
    of the world, posed from HIM's state)."""

    def __init__(self, model):
        super().__init__(model)
        import mujoco
        self.paths = [route if route is not None else
                      (np.vstack((start[:2], end[:2])), np.r_[0., np.linalg.norm(end[:2] - start[:2])])
                      for start, end, route in zip(self.start, self.end, self.routes)]
        self.speed = np.array([along[-1] / d for (_, along), d in zip(self.paths, self.duration)])
        self.positions = np.array([start[:2] for start in self.start], dtype=float).reshape(-1, 2)
        self.velocity = np.zeros_like(self.positions)
        self.progress = np.zeros(len(self.names))   # m along its route, from the start
        self.heading = np.ones(len(self.names))     # 1 out, -1 back, 0 done (trips)
        self.legs = [0] * len(self.names)           # one-way walks finished
        self.rest = [None] * len(self.names)        # where a mover done with its trips stands
        self.side = np.zeros(len(self.names))       # side it is passing the robot on (+1 left
                                                    # of their relative motion, -1 right), 0 none
        self.blocked = np.zeros(len(self.names))    # s it has stood though it wants to walk
        # Its size: how far its shapes reach from its centre, across the floor. Its centre
        # stays this far from walls, and gaps to the robot are counted from it.
        self.radius = np.zeros(len(self.names))
        for i, mocap in enumerate(self.mocap):
            body = int(np.flatnonzero(model.body_mocapid == mocap)[0])
            for g in np.flatnonzero(model.geom_bodyid == body):
                size = model.geom_size[g]
                reach = (math.hypot(size[0], size[1]) if model.geom_type[g] == BOX else
                         max(size[0], size[1]) if model.geom_type[g] == 4 else
                         size[0] if model.geom_type[g] in ROUND else model.geom_rbound[g])
                self.radius[i] = max(self.radius[i], float(np.linalg.norm(model.geom_pos[g][:2])) + reach)
        boxes = [g for g in range(model.ngeom) if model.geom_bodyid[g] == 0 and model.geom_type[g] == BOX
                 and model.geom(g).name.split("/")[-1].startswith(("wall", "block"))]
        turns = np.zeros((len(boxes), 9))
        for turn, g in zip(turns, boxes):
            mujoco.mju_quat2Mat(turn, model.geom_quat[g])
        self.wall_center = model.geom_pos[boxes, :2].reshape(-1, 2)
        self.wall_half = model.geom_size[boxes, :2].reshape(-1, 2)
        self.wall_turn = turns.reshape(-1, 3, 3)[:, :2, :2]
        self.at, self.seen = None, []

    def apply(self, data, t):
        """Walks every mover on to sim time t: a velocity chosen for each now, then reached
        (ACCELERATION) and followed in steps of at most STEP."""
        robot = np.array(data.qpos[:2], dtype=float)
        if self.at is None:
            self.at = t
        self.seen = [(s, p) for s, p in self.seen if t - s < ROBOT_WINDOW] + [(t, robot)]
        (then, before) = self.seen[0]
        robot_velocity = (robot - before) / (t - then) if t > then else np.zeros(2)
        gap = t - self.at
        if gap > 0:
            walking = [s is not None and t >= s for s in self.started]
            chosen = np.zeros_like(self.velocity)
            for i in range(len(self.names)):
                chosen[i] = self._choose(i, *self._preferred(i, walking[i]), robot, robot_velocity)
            steps = min(int(math.ceil(gap / STEP - 1e-9)), MAX_STEPS)
            for _ in range(steps):
                change = chosen - self.velocity
                size = np.linalg.norm(change, axis=1, keepdims=True)
                self.velocity += change * np.minimum(1.0, ACCELERATION * gap / steps / np.maximum(size, 1e-9))
                self.positions += self.velocity * gap / steps
                self._separate()
                self._clear_walls()
            for i in np.flatnonzero(walking):
                self._advance(i)
                # Standing short of where it heads (the robot in a door, or at its turning
                # point) for GIVE_UP: it turns back.
                stuck = (self.heading[i] != 0 and np.linalg.norm(self.velocity[i]) < 0.05
                         and np.linalg.norm(self._end(i) - self.positions[i]) > 2 * ARRIVE)
                self.blocked[i] = self.blocked[i] + gap if stuck else 0.0
                if self.blocked[i] >= GIVE_UP:
                    self.heading[i], self.blocked[i], self.side[i] = -self.heading[i], 0.0, 0.0
        self.at = max(self.at, t)
        for i, mocap in enumerate(self.mocap):
            data.mocap_pos[mocap][:2] = self.positions[i]
            data.mocap_pos[mocap][2] = self.start[i][2]

    def _preferred(self, i, walking):
        """(velocity it would walk at, frame for _choose) with nothing in its way."""
        target, speed, direction, error = self._route(i, walking)
        if direction is None:   # waiting or done: to its spot
            offset = target - self.positions[i]
            distance = float(np.linalg.norm(offset))
            return (speed * offset / distance if distance > 0.02 else np.zeros(2)), None
        normal = np.array([-direction[1], direction[0]])
        across = float(np.clip(-ROUTE_PULL * error, -RETURN * speed, RETURN * speed))
        return speed * direction + across * normal, (direction, normal, speed, across)

    def _choose(self, i, preferred, frame, robot, robot_velocity):
        """The cheapest of standing still and a few headings and paces around preferred.
        frame: (route direction, its left normal, speed along it, speed back to it), or
        None to compare with preferred alone."""
        away = self.positions[i] - robot
        size = float(np.linalg.norm(preferred))
        if size > 1e-3:
            base = preferred / size
        elif np.linalg.norm(away) > 1e-6:
            base = away / np.linalg.norm(away)   # standing: out of the robot's way, if at all
        else:
            base = np.array([1.0, 0.0])
        directions = np.stack([np.cos(TURNS) * base[0] - np.sin(TURNS) * base[1],
                               np.sin(TURNS) * base[0] + np.cos(TURNS) * base[1]], axis=1)
        reach = max(size, 0.5 * self.speed[i])
        candidates = np.vstack([np.zeros((1, 2))] + [pace * reach * directions for pace in PACES])
        if frame is None:
            cost = np.sum((candidates - preferred) ** 2, axis=1)
        else:
            direction, normal, along, across = frame
            cost = (candidates @ direction - along) ** 2 + LATERAL * (candidates @ normal - across) ** 2
        cost = cost / max(self.speed[i], 0.1) ** 2
        engaged = np.linalg.norm(away) < SEES
        personal = self.radius[i] + ROBOT_RADIUS + COMFORT   # between centres
        if engaged:
            relative = candidates - robot_velocity
            when = np.maximum(-(relative @ away) / np.maximum(np.sum(relative ** 2, axis=1), 1e-9),
                              SOON)
            miss = away + when[:, None] * relative
            nearest = np.linalg.norm(miss, axis=1)
            cost += CLOSE_COST * np.exp(-when / URGENCY) * np.maximum(0.0, 1.0 - nearest / personal) ** 2
            touch = away + np.minimum(when, CONTACT_TIME)[:, None] * relative
            close = np.linalg.norm(touch, axis=1) < self.radius[i] + ROBOT_RADIUS + MIN_GAP
            cost += np.where(close, BLOCKED_COST, 0.0)
            side = np.sign(relative[:, 0] * miss[:, 1] - relative[:, 1] * miss[:, 0])
            if self.side[i]:
                cost += np.where(side == -self.side[i], SIDE_COST, 0.0)
        others = np.flatnonzero((np.arange(len(self.names)) != i)
                                & (np.linalg.norm(self.positions - self.positions[i], axis=1) < OTHERS))
        if len(others):
            apart = self.positions[i] - self.positions[others]                      # (m, 2)
            moving = candidates[:, None, :] - self.velocity[others][None]         # (n, m, 2)
            soon = np.maximum(-np.sum(moving * apart, axis=2)
                              / np.maximum(np.sum(moving ** 2, axis=2), 1e-9), SOON)
            passing = np.linalg.norm(apart + soon[..., None] * moving, axis=2)
            bodies = self.radius[i] + self.radius[others]
            cost += 0.5 * CLOSE_COST * np.sum(
                np.exp(-soon / URGENCY) * np.maximum(0.0, 1.0 - passing / (bodies + OTHERS_COMFORT)) ** 2, axis=1)
            contact = apart + np.minimum(soon, CONTACT_TIME)[..., None] * moving
            cost += np.where((np.linalg.norm(contact, axis=2) < bodies + OTHERS_GAP).any(axis=1), BLOCKED_COST, 0.0)
        if len(self.wall_center):
            ahead = self.positions[i] + np.multiply.outer(WALL_LOOK, candidates)   # (looks, n, 2)
            gap = self._wall_gap(ahead.reshape(-1, 2)).reshape(len(WALL_LOOK), -1)
            cost += WALL_COST * np.maximum(0.0, 1.0 - gap / self.radius[i]).sum(axis=0)
        best = int(np.argmin(cost))
        self.side[i] = side[best] if engaged and nearest[best] < personal + 0.5 else 0.0
        return candidates[best]

    def _wall_local(self, points):
        """points (n, 2) in every wall's and block's own frame: (n, walls, 2)."""
        return np.einsum("nwk,wkj->nwj", points[:, None, :] - self.wall_center, self.wall_turn)

    def _wall_gap(self, points):
        """Distance from each point to the nearest wall or block surface (0 inside one)."""
        local = self._wall_local(points)
        return np.linalg.norm(local - np.clip(local, -self.wall_half, self.wall_half), axis=2).min(axis=1)

    def _route(self, i, walking):
        """(spot, speed, direction, distance left of the route): walking, its route's
        direction LOOKAHEAD ahead and its speed along it, slowing to turn back at an end;
        waiting or done with its trips, direction None and the spot it stands at."""
        points, along = self.paths[i]
        if not walking:
            return points[0], self.speed[i], None, 0.0
        if self.heading[i] == 0:
            return self.rest[i], self.speed[i], None, 0.0
        s, heading = self.progress[i], self.heading[i]
        foot = point_at(points, along, s)
        direction = point_at(points, along, min(max(s + heading * LOOKAHEAD, 0.0), along[-1])) - foot
        if np.linalg.norm(direction) < 1e-6:   # at an end: along its last piece of route
            direction = heading * (points[-1] - points[-2] if heading > 0 else points[1] - points[0])
        direction = direction / max(float(np.linalg.norm(direction)), 1e-9)
        remaining = float(np.linalg.norm(self._end(i) - self.positions[i]))
        speed = min(self.speed[i], math.sqrt(2 * ACCELERATION * remaining) + 0.05)
        error = float((self.positions[i] - foot) @ np.array([-direction[1], direction[0]]))
        return None, speed, direction, error

    def _end(self, i):
        """The end of its route it is walking to."""
        points, _ = self.paths[i]
        return points[-1] if self.heading[i] > 0 else points[0]

    def _advance(self, i):
        """Its progress: the nearest point of its route a little behind or ahead of the
        last, never going back; at an end, it turns back (or stops, after its trips)."""
        points, along = self.paths[i]
        s, heading = self.progress[i], self.heading[i]
        if heading == 0 or along[-1] < 1e-9:
            return
        low, high = (s - 0.3, s + 1.0) if heading > 0 else (s - 1.0, s + 0.3)
        best, nearest = math.inf, s
        for k in range(len(points) - 1):
            length = along[k + 1] - along[k]
            first, last = max(low, along[k]), min(high, along[k + 1])
            if length < 1e-9 or first > last:
                continue
            segment = points[k + 1] - points[k]
            f = float(np.clip((self.positions[i] - points[k]) @ segment / length ** 2,
                              (first - along[k]) / length, (last - along[k]) / length))
            gap = float(np.linalg.norm(self.positions[i] - points[k] - f * segment))
            if gap < best:
                best, nearest = gap, along[k] + f * length
        self.progress[i] = max(s, nearest) if heading > 0 else min(s, nearest)
        end = points[-1] if heading > 0 else points[0]
        if np.linalg.norm(self.positions[i] - end) < ARRIVE:
            self.legs[i] += 1
            self.progress[i] = along[-1] if heading > 0 else 0.0
            if self.trips[i] is not None and self.legs[i] >= self.trips[i]:
                self.heading[i], self.rest[i] = 0, end
            else:
                self.heading[i] = -heading

    def _separate(self):
        """Pushes apart any two movers whose bodies overlap, half each."""
        for _ in range(2):
            apart = self.positions[:, None] - self.positions[None]
            distance = np.linalg.norm(apart, axis=2)
            overlap = self.radius[:, None] + self.radius[None] - distance
            np.fill_diagonal(overlap, -1.0)
            for i, j in zip(*np.nonzero(np.triu(overlap > 0))):
                direction = (apart[i, j] / distance[i, j] if distance[i, j] > 1e-9
                             else np.array([1.0, 0.0]))
                self.positions[i] += 0.5 * overlap[i, j] * direction
                self.positions[j] -= 0.5 * overlap[i, j] * direction

    def _clear_walls(self):
        """Keeps every mover's centre its radius from every wall and block."""
        if not len(self.wall_center):
            return
        local = self._wall_local(self.positions)
        gap = np.linalg.norm(local - np.clip(local, -self.wall_half, self.wall_half), axis=2)
        for i, w in zip(*np.nonzero(gap < self.radius[:, None])):
            center, half, turn = self.wall_center[w], self.wall_half[w], self.wall_turn[w]
            point = (self.positions[i] - center) @ turn
            inside = np.clip(point, -half, half)
            outside = point - inside
            length = float(np.linalg.norm(outside))
            if length >= self.radius[i]:   # an earlier push this step already cleared it
                continue
            if length < 1e-9:   # centre inside the box: out by its nearest side
                axis = int(np.argmin(half - np.abs(point)))
                direction = np.zeros(2)
                direction[axis] = math.copysign(1.0, point[axis])
                inside[axis] = direction[axis] * half[axis]
            else:
                direction = outside / length
            self.positions[i] = center + turn @ (inside + self.radius[i] * direction)


def point_at(points, along, s):
    """The point s m along a polyline (points, cumulative lengths along)."""
    k = min(int(np.searchsorted(along, s, side="right")) - 1, len(points) - 2)
    f = (s - along[k]) / max(along[k + 1] - along[k], 1e-9)
    return points[k] + f * (points[k + 1] - points[k])


def crowd_mode(task_text):
    """The task's crowd (G1_TASK_COMMAND's "crowd"): "scripted", the default, or "aware".
    ValueError for anything else."""
    try:
        task = json.loads(task_text or "null")
    except ValueError:
        task = None   # other task readers tolerate bad JSON too
    mode = task.get("crowd", "scripted") if isinstance(task, dict) else "scripted"
    if mode not in CROWDS:
        raise ValueError("crowd must be scripted or aware")
    return mode


def crowd_event(names):
    """The lidar's first event with an aware crowd (harp/events.py): the movers its path
    events list."""
    return events.record("lidar", "crowd", mode="aware", movers=list(names))


def path_event(t, positions):
    """Where an aware crowd's movers are at sim time t, every scan (harp/events.py)."""
    return events.record("lidar", "movers", sim_time=t,
                         xy=[[round(float(x), 3), round(float(y), 3)] for x, y in positions])


def logged_path(events):
    """(names, times, positions) from a run's crowd and movers events, for
    Movers(path=...), or None if its movers were scripted."""
    crowd = next((e for e in events if e.get("event") == "crowd"), None)
    samples = [e for e in events if e.get("event") == "movers"]
    if not crowd or crowd.get("mode") != "aware" or not samples:
        return None
    return (list(crowd["movers"]), np.array([e.get("sim_time", e.get("t")) for e in samples], dtype=float),
            np.array([e["xy"] for e in samples], dtype=float).reshape(len(samples), -1, 2))


def numeric(model, name):
    i = model.numeric(name).id
    value = float(model.numeric_data[model.numeric_adr[i]])
    if model.numeric_size[i] != 1 or not math.isfinite(value):
        raise SystemExit(f"Mover {name} must hold one finite number")
    return value


def optional_route(model, name):
    """An even number of finite coordinates, at least one point, or None if absent."""
    try:
        i = model.numeric(name).id
    except KeyError:
        return None
    adr, size = model.numeric_adr[i], model.numeric_size[i]
    values = np.asarray(model.numeric_data[adr:adr + size], dtype=float)
    if size < 2 or size % 2 or not np.isfinite(values).all():
        raise SystemExit(f"Mover {name} must hold x y pairs of finite numbers")
    return values.reshape(-1, 2)


def optional(model, name):
    try:
        return numeric(model, name)
    except KeyError:
        return None


def obstacle_geoms(model):
    """Every geom the robot shouldn't touch: fixed in the world or on a mover, except the floor."""
    return [g for g in range(model.ngeom)
            if (model.geom_bodyid[g] == 0 or model.body_mocapid[model.geom_bodyid[g]] >= 0)
            and model.geom(g).name != "safety_floor"]
