"""Select exactly one command filter from the existing task JSON."""
import json
import os


def select(task_text, execution_mode="sim", has_world=True):
    task = json.loads(task_text or "null")
    mode = task.get("avoidance", "reactive") if isinstance(task, dict) else "reactive"
    if mode not in ("reactive", "predictive"):
        raise ValueError("avoidance must be reactive or predictive")
    if mode == "predictive":
        if execution_mode != "sim" or not has_world:
            raise ValueError("predictive avoidance currently requires a simulator world")
        if "moves" in task or "to" in task:
            raise ValueError("predictive avoidance requires Nav2; use a waypoint route")
    return "predictive_avoidance" if mode == "predictive" else "approach_stop"


if __name__ == "__main__":
    print(select(os.environ.get("G1_TASK_COMMAND", ""),
                 os.environ.get("G1_EXECUTION_MODE", "sim"),
                 os.path.isfile("/harp/model/world.xml")))
