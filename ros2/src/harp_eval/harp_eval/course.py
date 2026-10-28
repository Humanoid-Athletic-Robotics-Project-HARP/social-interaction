"""Run a recorded course test in the sim, once or several times:
  ros2 run harp_eval course slalom --tag rangecut --repeat 3 --note "baseline"
  ros2 run harp_eval course obstacle_course --extras nav --extras-reason "why it hesitates"
  ros2 run harp_eval course open_floor --goal 3 2 90 --no-rviz
  ros2 run harp_eval course one_box --replay --replay-reason "tune costmaps offline"

--replay records enough to play the run back into Nav2 later (replay.launch.py).
It is big (about 110 MB/min in the sim), so use it only when a person asks for
it, with a reason; it is never part of --extras.

Each run gets its own folder under runs/ and starts from a fresh sim (the
costmaps and leg odometry start where the policy starts). Each run also
claims its own pair of DDS domains (the sim's robot topics, and ROS 2), so
several course commands can run at once, in different terminals; they start up
together and then walk together. This command only
makes the folder, points ROS's logs into it (so it holds every program's
printed output), runs course.launch.py, and writes the summary afterwards.
Compare runs with: ros2 run harp_eval runs compare --world slalom
"""

import argparse
import datetime
import fcntl
import math
import os
import random
import shlex
import shutil
import signal
import subprocess
import sys
import tempfile
import time
from pathlib import Path

import yaml

from harp_control import runlog
from harp_eval import run_summary
from harp_eval.run_recorder import CONDITIONS, run_name
from harp_eval.waypoint_route import load_route, route_from_data

# Each program's lines in launch.log, by the launch's process name, go to these files.
LOG_FILES = {"run_sim": "sim.log", "walker_node": "walker.log", "lidar_relay": "lidar_relay.log",
             "run_recorder": "recorder.log",
             "course_goal": "goal.log", "rviz2": "rviz.log"}
NAV2 = ("controller_server", "planner_server", "behavior_server", "bt_navigator",
        "lifecycle_manager")
# DDS domains for runs: pairs (sim robot topics, ROS 2) from 10/11 up to 98/99. A
# process can't use one domain for both (CycloneDDS), the robot uses 0 and a sim
# started by hand 1 and 2. Domain ids map to UDP ports (7400 + 250 per domain),
# and above about 101 they reach the ports Linux hands out to programs.
FIRST_DOMAIN, SLOTS = 10, 45
LOCKS = Path(tempfile.gettempdir()) / "harp-dds-domains"
# Runs start up together, then walk together. A run's busiest part is its
# start-up (the windows open, then Nav2 and rviz2 start; with STVL its Nav2 takes
# about 18 s alone), and a start-up while another robot walked lagged that sim up
# to 300 ms and made the robot fall. Lag while the band holds the robot is harmless.
# So the lock's state goes waiting -> starting -> ready -> walking: a run waits to
# start while any other walks; once its Nav2 is active, course_goal marks it ready
# and asks for the band's release only when no other run is still starting (for at
# most MAX_START s) and its own sim has stopped lagging (see course_goal.py).
MAX_START = 120.0


def positive_int(value):
    try:
        number = int(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("must be a positive integer") from exc
    if number <= 0:
        raise argparse.ArgumentTypeError("must be a positive integer")
    return number


def route_file(value):
    """A user file or a route bundled in harp_eval/config, validated before startup."""
    path = Path(value).expanduser()
    if not path.is_file() and path.parent == Path("."):
        from ament_index_python.packages import get_package_share_directory
        name = path.name if path.suffix else path.name + ".yaml"
        path = Path(get_package_share_directory("harp_eval")) / "config" / name
    try:
        path = path.resolve(strict=True)
        load_route(path)
    except (OSError, ValueError, TypeError) as exc:
        raise argparse.ArgumentTypeError(str(exc)) from exc
    return path


def read_lock(lock):
    """(pid, claimed, state, began) from a slot's lock file; None if it's free."""
    try:
        pid, claimed, state, began = lock.read_text().split()
        os.kill(int(pid), 0)
        return int(pid), float(claimed), state, float(began)
    except (FileNotFoundError, ValueError):
        return None
    except ProcessLookupError:
        lock.unlink(missing_ok=True)   # left by a process that died
        return None


def write_lock(lock, claimed, state, began=0.0):
    tmp = lock.with_suffix(".tmp")
    tmp.write_text(f"{os.getpid()} {claimed} {state} {began}")
    os.replace(tmp, lock)


def set_state(lock, state):
    """Changes the state only (course_goal, another process, uses it too)."""
    pid, claimed, _, began = lock.read_text().split()
    tmp = lock.with_suffix(".tmp")
    tmp.write_text(f"{pid} {claimed} {state} {began}")
    os.replace(tmp, lock)


def others(lock):
    """The other course runs' (pid, claimed, state, began)."""
    return [o for o in (read_lock(p) for p in LOCKS.glob("slot-*") if p != lock) if o]


def starting(lock):
    """The other runs still starting up (course_goal waits for them before the release)."""
    return [o for o in others(lock) if o[2] == "starting" and time.time() - o[3] < MAX_START]


def claim_domains():
    """A free (sim, ROS 2) domain pair, held by a lock file with our pid."""
    LOCKS.mkdir(exist_ok=True)
    with open(LOCKS / "schedule", "a") as schedule:
        fcntl.flock(schedule, fcntl.LOCK_EX)   # one course at a time picks a slot
        free = [slot for slot in range(SLOTS) if read_lock(LOCKS / f"slot-{slot}") is None]
        if not free:
            raise SystemExit(f"No free DDS domains: {SLOTS} course runs at once? Locks are in {LOCKS}.")
        slot = random.choice(free)
        lock = LOCKS / f"slot-{slot}"
        write_lock(lock, time.time(), "waiting")
    return lock, FIRST_DOMAIN + 2 * slot, FIRST_DOMAIN + 2 * slot + 1


def wait_while_walking(lock, then):
    """Waits while another run's robot walks, so neither a start-up nor a summary
    lags it; then (under the schedule lock) sets our state to then, if given."""
    said = False
    while True:
        with open(LOCKS / "schedule", "a") as schedule:
            fcntl.flock(schedule, fcntl.LOCK_EX)
            if not any(o[2] == "walking" for o in others(lock)):
                if then:
                    write_lock(lock, read_lock(lock)[1], then, time.time())
                return
        if not said:
            print("    Waiting for the other course runs' robots to finish walking.", flush=True)
            said = True
        time.sleep(2)


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
    parser.add_argument("--repeat", type=positive_int, default=1)
    parser.add_argument("--extras", default="", help="extra streams from logging.yaml, e.g. nav,lidar")
    parser.add_argument("--extras-reason", default="")
    destination = parser.add_mutually_exclusive_group()
    destination.add_argument("--goal", nargs="+", type=float, metavar="NUM",
                             help="X Y [YAW_DEG] in odom, instead of the course's goal")
    destination.add_argument("--route", type=route_file, metavar="FILE_OR_NAME",
                             help="waypoint YAML file or bundled route name, e.g. rectangle")
    destination.add_argument("--waypoint", action="append", nargs="+", type=float, metavar="NUM",
                             help="X Y [YAW_DEG] in odom; repeat this argument for each point")
    parser.add_argument("--laps", type=positive_int,
                        help="number of route laps; overrides YAML, defaults to 1 for inline points")
    parser.add_argument("--timeout", type=float, default=240.0, help="s to wait for Nav2's result")
    parser.add_argument("--no-rviz", action="store_true")
    parser.add_argument("--params", default="", metavar="FILE",
                        help="Nav2 parameters instead of harp_ros's nav2_params.yaml (copied into the run)")
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
    if args.goal and not all(math.isfinite(v) for v in args.goal):
        parser.error("--goal values must be finite")
    if not math.isfinite(args.timeout) or args.timeout <= 0:
        parser.error("--timeout must be positive and finite")
    if args.laps is not None and not (args.route or args.waypoint):
        parser.error("--laps requires --route or --waypoint")
    route = None
    if args.route or args.waypoint:
        try:
            route = (load_route(args.route) if args.route else route_from_data(
                {"frame": "odom", "waypoints": args.waypoint, "laps": args.laps or 1}))
        except (OSError, ValueError, TypeError) as exc:
            parser.error(str(exc))
        if args.laps is not None:
            route.lap_limit = args.laps

    command = " ".join(["ros2 run harp_eval course", *map(shlex.quote, sys.argv[1:])])
    runs = runlog.runs_dir()
    failed = False
    for i in range(args.repeat):
        when = datetime.datetime.now(datetime.timezone.utc)
        while (runs / run_name("sim", args.world, args.tag, when)).exists():
            when += datetime.timedelta(seconds=1)   # another course started this second
        run = runs / run_name("sim", args.world, args.tag, when)
        (run / "ros_logs").mkdir(parents=True)
        if route:
            if args.route and args.laps is None:
                shutil.copyfile(args.route, run / "route.yaml")
            else:
                snapshot = {"frame": "odom", "laps": route.lap_limit,
                            "waypoints": [list(point) for point in route.waypoints]}
                if route.criteria is not None:
                    snapshot["criteria"] = route.criteria
                (run / "route.yaml").write_text(yaml.safe_dump(snapshot, sort_keys=False),
                                               encoding="utf-8")
        lock, sim_domain, ros_domain = claim_domains()
        print(f"\n=== Run {i + 1}/{args.repeat}: {run}\n    DDS domains: sim {sim_domain}, "
              f"ROS 2 {ros_domain}", flush=True)
        wait_while_walking(lock, "starting")
        env = dict(os.environ, ROS_LOG_DIR=str(run / "ros_logs"), HARP_RUN_DIR=str(run),
                   ROS_DOMAIN_ID=str(ros_domain), HARP_DDS_LOCK=str(lock))
        values = {"world": args.world, "run_dir": run, "tag": args.tag, "note": args.note,
                  "extras": args.extras, "extras_reason": args.extras_reason, "command": command,
                  "timeout": args.timeout, "rviz": "false" if args.no_rviz else "true",
                  "goal": " ".join(str(v) for v in args.goal or []),
                  "route": str(run / "route.yaml") if route else "",
                  "replay": "true" if args.replay else "", "replay_reason": args.replay_reason,
                  "replay_dir": args.replay_dir, "lidar_fields": args.lidar_fields,
                  "lidar_rate": args.lidar_rate or "",
                  "nav2_params_file": os.path.abspath(args.params) if args.params else "",
                  "sim_domain": sim_domain,
                  **{name: getattr(args, name) for name in CONDITIONS}}
        # Launch arguments can't be empty, so unset ones keep the launch file's defaults.
        launch = ["ros2", "launch", "harp_eval", "course.launch.py",
                  *(f"{k}:={v}" for k, v in values.items() if v != "")]
        # Ctrl-C reaches the launch too (same terminal); let it shut down, then stop repeating.
        interrupted = []
        previous = signal.signal(signal.SIGINT, lambda *_: interrupted.append(True))
        try:
            code = subprocess.call(launch, env=env)
        finally:
            signal.signal(signal.SIGINT, previous)
            lock.unlink(missing_ok=True)
        wait_while_walking(lock, None)
        split_launch_log(run)
        if (run / "run.yaml").exists():
            summary = run_summary.summarize(run)
            print(run_summary.one_screen(summary), flush=True)
            failed |= summary["outcome"] != "succeeded" or summary.get("passed") is False
        else:
            failed = True
            print(f"No run.yaml in {run}: the recorder didn't start (launch exit code {code}).")
        if interrupted:
            print("Interrupted: no more repeats.")
            return 130
        failed |= code != 0
    print(f"\nCompare: ros2 run harp_eval runs compare --world {args.world}"
          + (f" --tag {args.tag}" if args.tag else ""))
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
