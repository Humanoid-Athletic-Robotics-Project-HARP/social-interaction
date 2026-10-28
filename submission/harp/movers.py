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

import numpy as np

START_DISTANCE = 0.3   # m the pelvis has to move from where it started


class Movers:
    def __init__(self, model, started=None):
        """started: {name: sim time} from mover_start events, to replay a run."""
        self.names, self.mocap, self.start, self.end = [], [], [], []
        self.duration, self.delay, self.trips = [], [], []
        for b in range(model.nbody):
            if model.body_mocapid[b] < 0:
                continue
            name = model.body(b).name
            try:
                end = model.site(name + "_end").id
                duration = numeric(model, name + "_duration")
            except KeyError:
                raise SystemExit(f"Mover {name} needs a site {name}_end and a numeric "
                                 f"{name}_duration (see submission/harp/movers.py)")
            self.names.append(name)
            self.mocap.append(model.body_mocapid[b])
            self.start.append(model.body_pos[b].copy())
            self.end.append(model.site_pos[end].copy())
            self.duration.append(duration)
            self.delay.append(optional(model, name + "_delay"))
            self.trips.append(optional(model, name + "_trips"))
        # Sim time each mover starts walking; None until the robot has moved.
        self.started = [0.0 if d is None else None for d in self.delay]
        for i, name in enumerate(self.names):
            if started and name in started:
                self.started[i] = started[name]
        self.robot_start = None

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
        for i, mocap in enumerate(self.mocap):
            legs = 0.0 if self.started[i] is None else max(0.0, t - self.started[i]) / self.duration[i]
            if self.trips[i] is not None:
                legs = min(legs, self.trips[i])
            phase = legs % 2.0
            share = phase if phase <= 1.0 else 2.0 - phase   # out, then back
            data.mocap_pos[mocap] = self.start[i] + share * (self.end[i] - self.start[i])


def numeric(model, name):
    i = model.numeric(name).id
    return float(model.numeric_data[model.numeric_adr[i]])


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
