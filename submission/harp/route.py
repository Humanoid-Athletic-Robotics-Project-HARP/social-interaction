"""A finite waypoint route for a course test: the points in order, how many laps,
and optional pass criteria for the whole route. Plain Python, so course.py (on the
host) and course_goal.py (in the image, Python 3.8) share it.

A route file (submission/routes/NAME.yaml):
  frame: odom                  # required: every point is in odom, from where the robot started
  waypoints:                   # [x m, y m, heading deg]; heading is optional (0)
    - [4.0, 0.0, 0.0]
    - [4.0, 2.0, 90.0]
  laps: 3                      # optional, 1 by default
  criteria:                    # optional, for the whole route (course.py's check)
    must_succeed: true
    max_contacts: 0

course.py hands it to the image as G1_TASK_COMMAND {"route": {...}}, the same mapping.
"""

import math

COUNT_CRITERIA = {"max_contacts", "max_recoveries"}
NUMBER_CRITERIA = {"max_time_s", "min_clearance_m", "max_end_error_m"}


class Route:
    """The points and laps, and where along them the robot is."""

    def __init__(self, waypoints, laps=1, criteria=None):
        self.waypoints = [tuple(p) for p in waypoints]
        self.laps = laps
        self.criteria = criteria
        self.index = 0             # the current point in this lap
        self.completed_laps = 0

    def current(self):
        """The point to go to now, or None once every lap is done."""
        return None if self.completed_laps >= self.laps else self.waypoints[self.index]

    def advance(self):
        """After the robot reached current(): the next point, or None."""
        if self.current() is not None:
            self.index += 1
            if self.index == len(self.waypoints):
                self.index, self.completed_laps = 0, self.completed_laps + 1
        return self.current()

    def data(self):
        """The mapping route_from_data reads, for route.yaml and G1_TASK_COMMAND."""
        out = {"frame": "odom", "waypoints": [list(p) for p in self.waypoints], "laps": self.laps}
        if self.criteria is not None:
            out["criteria"] = self.criteria
        return out


def finite(value, name):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f"{name} must be a finite number")
    return float(value)


def positive_int(value, name):
    if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
        raise ValueError(f"{name} must be a positive whole number")
    return value


def check_criteria(values):
    if values is None:
        return None
    if not isinstance(values, dict):
        raise ValueError("criteria must be a mapping")
    for name, value in values.items():
        if name == "must_succeed":
            ok = isinstance(value, bool)
        elif name in COUNT_CRITERIA:
            ok = isinstance(value, int) and not isinstance(value, bool) and value >= 0
        elif name in NUMBER_CRITERIA:
            ok = finite(value, f"criteria {name}") >= 0
        else:
            raise ValueError(f"unknown route criterion {name!r}")
        if not ok:
            raise ValueError(f"criteria {name} has the wrong type or is negative")
    return dict(values)


def route_from_data(data):
    """A Route from a route file's mapping; ValueError says what is wrong."""
    if not isinstance(data, dict):
        raise ValueError("a route must be a mapping")
    if data.get("frame") != "odom":
        raise ValueError("a route needs frame: odom")
    points = data.get("waypoints")
    if not isinstance(points, list) or not points:
        raise ValueError("a route needs a nonempty list of waypoints")
    waypoints = []
    for i, point in enumerate(points, 1):
        if not isinstance(point, list) or len(point) not in (2, 3):
            raise ValueError(f"waypoint {i} must be [X, Y] or [X, Y, HEADING]")
        x, y, *heading = [finite(v, f"waypoint {i}") for v in point]
        waypoints.append((x, y, heading[0] if heading else 0.0))
    return Route(waypoints, positive_int(data.get("laps", 1), "laps"),
                 check_criteria(data.get("criteria")))
