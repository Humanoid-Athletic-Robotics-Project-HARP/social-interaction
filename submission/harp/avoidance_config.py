"""Select exactly one command filter from the existing task JSON, and check the task's
crowd (harp/movers.py), speed and robot options before anything starts.

On the robot (execution mode not sim) Nav2 runs only for a task that asks for it with
"robot": true and a route ({"route": ...}, points in odom from where the robot starts):
then Nav2, the filter and the route's goals run on the real lidar (entrypoint.sh). In the
sim, "robot" changes nothing, so the same task can be rehearsed in a world. "speed" (m/s)
lowers Nav2's top speed (v_linear_max) and the predictive filter's, for a Nav2 task."""
import json
import math
import os

from harp.movers import crowd_mode

MAX_SPEED = .5   # m/s: nav2_params.yaml's v_linear_max, which "speed" may only lower
MIN_SPEED = .1   # m/s: its v_linear_min


def task_speed(task):
    """The task's "speed", or None; ValueError unless a number from MIN_SPEED to MAX_SPEED."""
    speed = task.get("speed") if isinstance(task, dict) else None
    if speed is None:
        return None
    if isinstance(speed, bool) or not isinstance(speed, (int, float)) or not (
            math.isfinite(speed) and MIN_SPEED <= speed <= MAX_SPEED):
        raise ValueError(f"speed must be a number from {MIN_SPEED} to {MAX_SPEED} m/s")
    return float(speed)


def speed_of(task_text):
    """The "speed" of a task's JSON text, or None (also for text that isn't JSON)."""
    try:
        task = json.loads(task_text or "null")
    except ValueError:
        return None
    return task_speed(task)


def select(task_text, execution_mode="sim", has_world=True):
    try:
        task = json.loads(task_text or "null")
    except ValueError:
        task = None   # other task readers tolerate bad JSON too; keep the default filter
    if crowd_mode(task_text) == "aware" and (execution_mode != "sim" or not has_world):
        raise ValueError("an aware crowd needs a simulator world")
    mode = task.get("avoidance", "reactive") if isinstance(task, dict) else "reactive"
    if mode not in ("reactive", "predictive"):
        raise ValueError("avoidance must be reactive or predictive")
    robot = isinstance(task, dict) and task.get("robot", False)
    if not isinstance(robot, bool):
        raise ValueError('robot must be true or false')
    moves = isinstance(task, dict) and ("moves" in task or "to" in task)
    if task_speed(task) is not None and moves:
        raise ValueError("speed caps Nav2; planned moves set their own")
    if execution_mode != "sim":
        if robot:
            if "route" not in task or moves:
                raise ValueError('"robot": true needs a route, and no planned move')
            for key in ("behavior", "mapping", "map"):
                if task.get(key) not in (None, False):
                    raise ValueError(f'"robot": true runs a route only, not "{key}"')
        elif mode == "predictive":
            raise ValueError('predictive avoidance on the robot needs "robot": true and a route')
        elif isinstance(task, dict) and "route" in task:
            raise ValueError('a route on the robot needs "robot": true (it starts Nav2 there)')
    if mode == "predictive":
        if execution_mode == "sim" and not has_world:
            raise ValueError("predictive avoidance currently requires a simulator world")
        if moves:
            raise ValueError("predictive avoidance requires Nav2; use a waypoint route")
        if "behavior" in task:
            # It follows course_goal.py's goals only: with the behavior loop it stood the robot.
            raise ValueError("predictive avoidance doesn't follow the behavior loop's goals yet")
    elif isinstance(task, dict) and (task.get("mapping") or task.get("map") is not None):
        # harp/mapper.py maps from /harp/static_cloud, which only the predictive filter publishes.
        raise ValueError("a map (mapping or map) needs predictive avoidance")
    return "predictive_avoidance" if mode == "predictive" else "approach_stop"


def on_robot(task_text, execution_mode):
    """Whether entrypoint.sh starts Nav2 on the robot: a "robot": true task outside the sim
    (select() has checked it)."""
    try:
        task = json.loads(task_text or "null")
    except ValueError:
        return False
    return execution_mode != "sim" and isinstance(task, dict) and task.get("robot") is True


if __name__ == "__main__":
    # The filter, then "robot" when Nav2 runs on the robot (entrypoint.sh reads both).
    text, execution = os.environ.get("G1_TASK_COMMAND", ""), os.environ.get("G1_EXECUTION_MODE", "sim")
    selected = select(text, execution, os.path.isfile("/harp/model/world.xml"))
    print(selected + (" robot" if on_robot(text, execution) else ""))
