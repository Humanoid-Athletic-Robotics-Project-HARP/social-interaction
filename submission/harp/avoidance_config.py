"""Select exactly one command filter from the existing task JSON, and check the task's
crowd (harp/movers.py) before anything starts."""
import json
import os

from harp.movers import crowd_mode


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
    if mode == "predictive":
        if execution_mode != "sim" or not has_world:
            raise ValueError("predictive avoidance currently requires a simulator world")
        if "moves" in task or "to" in task:
            raise ValueError("predictive avoidance requires Nav2; use a waypoint route")
        if "behavior" in task:
            # It follows course_goal.py's goals only: with the behavior loop it stood the robot.
            raise ValueError("predictive avoidance doesn't follow the behavior loop's goals yet")
    elif isinstance(task, dict) and (task.get("mapping") or task.get("map") is not None):
        # harp/mapper.py maps from /harp/static_cloud, which only the predictive filter publishes.
        raise ValueError("a map (mapping or map) needs predictive avoidance")
    return "predictive_avoidance" if mode == "predictive" else "approach_stop"


if __name__ == "__main__":
    print(select(os.environ.get("G1_TASK_COMMAND", ""),
                 os.environ.get("G1_EXECUTION_MODE", "sim"),
                 os.path.isfile("/harp/model/world.xml")))
