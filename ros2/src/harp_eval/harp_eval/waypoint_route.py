"""Plain Python route state for waypoint navigation."""

import math

import yaml


_DISTANCE_TIME_CRITERIA = {"max_time_s", "min_clearance_m", "max_end_error_m"}
_COUNT_CRITERIA = {"max_contacts", "max_recoveries"}


class WaypointRoute:
    """Store an ordered route and read its current target.

    Targets can be labels or poses. The navigation code interprets them.
    The route copies their order so changes to the input list do not alter it.
    """

    def __init__(self, waypoints, laps):
        if isinstance(laps, bool) or not isinstance(laps, int) or laps <= 0:
            raise ValueError("laps must be a positive integer")
        if isinstance(waypoints, (str, bytes)):
            raise TypeError("waypoints must be an iterable of targets")

        self.waypoints = tuple(waypoints)
        if not self.waypoints:
            raise ValueError("waypoints must not be empty")

        self.lap_limit = laps
        self.index = 0
        self.completed_laps = 0
        self.criteria = None

    def current(self):
        """Return the current target without advancing the route."""
        if self.completed_laps >= self.lap_limit:
            return None
        return self.waypoints[self.index]

    def advance(self):
        """Record successful arrival, then return the next target or None."""
        if self.completed_laps >= self.lap_limit:
            return None
        self.index += 1
        if self.index == len(self.waypoints):
            self.index = 0
            self.completed_laps += 1
        return self.current()


def _finite_number(value, name):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} must be a finite number")
    try:
        number = float(value)
    except OverflowError as exc:
        raise ValueError(f"{name} must be a finite number") from exc
    if not math.isfinite(number):
        raise ValueError(f"{name} must be a finite number")
    return number


def _criteria(values):
    if values is None:
        return None
    if not isinstance(values, dict):
        raise ValueError("criteria must be a mapping")
    checked = {}
    for name, value in values.items():
        if name == "must_succeed":
            if not isinstance(value, bool):
                raise ValueError("criteria.must_succeed must be a boolean")
        elif name in _COUNT_CRITERIA:
            if isinstance(value, bool) or not isinstance(value, int) or value < 0:
                raise ValueError(f"criteria.{name} must be a nonnegative integer")
        elif name in _DISTANCE_TIME_CRITERIA:
            value = _finite_number(value, f"criteria.{name}")
            if value < 0:
                raise ValueError(f"criteria.{name} must be nonnegative")
        else:
            raise ValueError(f"unknown route criterion: {name!r}")
        checked[name] = value
    return checked


def load_route(path):
    """Read odom-frame YAML with laps, waypoint pairs/triples and optional criteria.

    Waypoint values are meters and yaw degrees; omitted yaw defaults to zero.
    The returned route stores normalized float triples and criteria or None.
    """
    with open(path, encoding="utf-8") as source:
        try:
            data = yaml.safe_load(source)
        except yaml.YAMLError as exc:
            raise ValueError(f"invalid route YAML: {exc}") from exc
    if not isinstance(data, dict):
        raise ValueError("route must be a mapping")
    if data.get("frame") != "odom":
        raise ValueError("route frame must be 'odom'")
    points = data.get("waypoints")
    if not isinstance(points, (list, tuple)) or not points:
        raise ValueError("waypoints must be a nonempty list of positions")
    normalized = []
    for index, point in enumerate(points):
        if not isinstance(point, (list, tuple)) or len(point) not in (2, 3):
            raise ValueError(f"waypoint {index + 1} must contain X Y or X Y YAW_DEG")
        numbers = [_finite_number(value, f"waypoint {index + 1} value {column + 1}")
                   for column, value in enumerate(point)]
        normalized.append(tuple(numbers if len(numbers) == 3 else [*numbers, 0.0]))
    route = WaypointRoute(normalized, laps=data.get("laps"))
    route.criteria = _criteria(data.get("criteria"))
    return route
