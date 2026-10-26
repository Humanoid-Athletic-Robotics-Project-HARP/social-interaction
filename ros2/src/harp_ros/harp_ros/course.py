"""Run a recorded course test in the sim, once or several times:
  ros2 run harp_ros course slalom --tag rangecut --repeat 3 --note "baseline"
  ros2 run harp_ros course obstacle_course --extras nav --extras-reason "why it hesitates"
  ros2 run harp_ros course open_floor --goal 3 2 90 --no-rviz
  ros2 run harp_ros course one_box --replay --replay-reason "tune costmaps offline"

--replay records enough to play the run back into Nav2 later (replay.launch.py).
It is big (about 110 MB/min in the sim), so use it only when a person asks for
it, with a reason; it is never part of --extras.

Each run gets its own folder under runs/ and starts from a fresh sim (the
costmaps and leg odometry start where the policy starts). This command only
makes the folder, points ROS's logs into it (so it holds every program's
printed output), runs course.launch.py, and writes the summary afterwards.
Compare runs with: ros2 run harp_ros runs compare --world slalom
"""

import argparse
import datetime
import os
import shlex
import signal
import subprocess
import sys

from harp_control import runlog
from harp_ros import run_summary
from harp_ros.run_recorder import CONDITIONS, run_name

# Each program's lines in launch.log, by the launch's process name, go to these files.
LOG_FILES = {"run_sim": "sim.log", "walker_node": "walker.log", "run_recorder": "recorder.log",
             "course_goal": "goal.log", "rviz2": "rviz.log"}
NAV2 = ("controller_server", "planner_server", "behavior_server", "bt_navigator",
        "lifecycle_manager")


def split_launch_log(run):
    """launch.log has every program's output prefixed [name-N]; give each its own file."""
    logs = sorted(run.glob("ros_logs/*/launch.log"))
    if not logs:
        return
    files = {}
    for line in logs[-1].read_text(errors="replace").splitlines():
        parts = line.split(" ", 1)
        text = parts[1] if len(parts) == 2 and parts[0].replace(".", "").isdigit() else line
        if not text.startswith("["):
            continue
        name = text[1:text.find("]")].rsplit("-", 1)[0]
        if name.startswith(NAV2):
            out = "nav2.log"
        else:
            out = LOG_FILES.get(name)
        if out:
            files.setdefault(out, []).append(line)
    for out, lines in files.items():
        (run / out).write_text("\n".join(lines) + "\n")


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("world", help="a course in harp_sim/worlds: open_floor, one_box, slalom, obstacle_course")
    parser.add_argument("--tag", default="", help="what this run tests, used to group runs when comparing")
    parser.add_argument("--note", default="")
    parser.add_argument("--repeat", type=int, default=1)
    parser.add_argument("--extras", default="", help="extra streams from logging.yaml, e.g. nav,lidar")
    parser.add_argument("--extras-reason", default="")
    parser.add_argument("--goal", nargs="+", type=float, metavar="NUM",
                        help="X Y [YAW_DEG] in odom, instead of the course's goal")
    parser.add_argument("--timeout", type=float, default=240.0, help="s to wait for Nav2's result")
    parser.add_argument("--no-rviz", action="store_true")
    for name in CONDITIONS:
        parser.add_argument("--" + name.replace("_", "-"), default="", help="run condition, in run.yaml")
    parser.add_argument("--replay", action="store_true",
                        help="also record for replay.launch.py (big; only when asked for, with a reason)")
    parser.add_argument("--replay-reason", default="")
    parser.add_argument("--replay-dir", default="", help="where the replay recording goes "
                        "(default $HARP_REPLAY_DIR, else the run folder)")
    parser.add_argument("--lidar-fields", choices=["all", "xyz"], default="")
    parser.add_argument("--lidar-rate", type=float, default=0.0, help="Hz, replay lidar (default 10)")
    args = parser.parse_args()
    if args.extras and not args.extras_reason:
        parser.error("--extras needs --extras-reason (it goes in run.yaml)")
    if args.replay and not args.replay_reason:
        parser.error("--replay needs --replay-reason: it is only for when a person asks for it")
    if args.goal and len(args.goal) not in (2, 3):
        parser.error("--goal takes X Y or X Y YAW_DEG")

    command = " ".join(["ros2 run harp_ros course", *map(shlex.quote, sys.argv[1:])])
    runs = runlog.runs_dir()
    for i in range(args.repeat):
        name = run_name("sim", args.world, args.tag, datetime.datetime.now(datetime.timezone.utc))
        run = runs / name
        (run / "ros_logs").mkdir(parents=True)
        print(f"\n=== Run {i + 1}/{args.repeat}: {run}", flush=True)
        env = dict(os.environ, ROS_LOG_DIR=str(run / "ros_logs"), HARP_RUN_DIR=str(run))
        values = {"world": args.world, "run_dir": run, "tag": args.tag, "note": args.note,
                  "extras": args.extras, "extras_reason": args.extras_reason, "command": command,
                  "timeout": args.timeout, "rviz": "false" if args.no_rviz else "true",
                  "goal": " ".join(str(v) for v in args.goal or []),
                  "replay": "true" if args.replay else "", "replay_reason": args.replay_reason,
                  "replay_dir": args.replay_dir, "lidar_fields": args.lidar_fields,
                  "lidar_rate": args.lidar_rate or "",
                  **{name: getattr(args, name) for name in CONDITIONS}}
        # Launch arguments can't be empty, so unset ones keep the launch file's defaults.
        launch = ["ros2", "launch", "harp_ros", "course.launch.py",
                  *(f"{k}:={v}" for k, v in values.items() if v != "")]
        # Ctrl-C reaches the launch too (same terminal); let it shut down, then stop repeating.
        interrupted = []
        previous = signal.signal(signal.SIGINT, lambda *_: interrupted.append(True))
        try:
            code = subprocess.call(launch, env=env)
        finally:
            signal.signal(signal.SIGINT, previous)
        split_launch_log(run)
        if (run / "run.yaml").exists():
            print(run_summary.one_screen(run_summary.summarize(run)), flush=True)
        else:
            print(f"No run.yaml in {run}: the recorder didn't start (launch exit code {code}).")
        if interrupted:
            print("Interrupted: no more repeats.")
            break
    print(f"\nCompare: ros2 run harp_ros runs compare --world {args.world}"
          + (f" --tag {args.tag}" if args.tag else ""))


if __name__ == "__main__":
    main()
