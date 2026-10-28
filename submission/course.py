"""Run a course test in HIM's simulator and score it:
  python3 submission/course.py one_box
  python3 submission/course.py slalom --tag rpp --repeat 3 --view
  python3 submission/course.py one_box --route rectangle --laps 3
  python3 submission/course.py open_floor --waypoint 2 0 --waypoint 2 2 90 --waypoint 0 0
  python3 submission/course.py --score runs/2026-10-28_05-00-00Z_sim_one_box
  python3 submission/course.py --prune
  python3 submission/course.py --compare slalom

Each run gets a folder in runs/ (HARP_RUNS_DIR to change it) holding our run.json
(world, tag, start, git commit, whether the tree was dirty; git.diff then holds the
uncommitted changes), HIM's
report.json, trace.npz (the true pose every 20 ms) and policy.log (our image's
output: events and the once-a-second "odom" lines), and our summary.json:
  outcome    succeeded / failed / timed_out / error / violation (HIM stopped the policy) /
             inconclusive (HIM stopped it while the host had paused its sim: host_stall)
  passed     against the world's criteria, submission/worlds/NAME.yaml
  seconds, recoveries   from course_goal's goal_result
  path_m, end_error_m   from the true pose (end: distance from the goal site)
  closest_m, contacts   true distance from the robot's body to every obstacle,
                        movers included (replayed from their mover_start events;
                        HIM's physics never sees movers, so these are the only contacts)
  drift_m    leg odometry against the truth at the last "odom" line
A route (--route NAME_OR_FILE from submission/routes, or --waypoint X Y [HEADING] for
each point; --laps N, harp/route.py) replaces the world's goal: the run folder keeps
it as route.yaml, the end error is from its last point, passed uses its criteria (none:
no verdict) instead of the world's, and summary.json adds route (planned and completed
waypoints and laps). The sim runs for the route's max_time_s, or ROUTE_SPEED and
ROUTE_POINT_TIME per point, plus STARTUP.
It exits 1 if any run didn't succeed or failed its criteria.
--compare [WORLD] lists every scored run by world and tag: how many succeeded and
passed, time (mean, min-max), closest, end error, recoveries and drift.
Runs never use HIM's live viewer (its load stopped about 1 in 4 watched runs on a
deadline). --view instead replays each run once it is scored, at the URL it prints,
for the run's length, before starting the next. After that, HIM's viewer output
(view/, tens of MB) is deleted: HIM's ./desktop_sim.sh replay RUN/trace.npz rebuilds it. --prune does
the same for every run folder course.py made (those with a run.json).

Needs MuJoCo, so it runs under HIM's .venv-policy-safety (it re-executes
itself there). Build the image first: submission/build.sh.
"""

import argparse
import datetime
import json
import math
import os
import shutil
import signal
import subprocess
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
REPO = HERE.parent
HIM = Path(os.environ.get("HIM_REPO", REPO.parent / "him-sim2real-management"))
VENV_PYTHON = HIM / ".venv-policy-safety/bin/python"
RUNS = Path(os.environ.get("HARP_RUNS_DIR", REPO / "runs"))
STARTUP = 30.0          # s added to the world's max_time_s: HIM's 5 s balance, Nav2, settling
CLEARANCE_MAX = 2.0     # m: farther than this counts as this
CLEARANCE_EVERY = 5     # trace frames between clearance checks (0.1 s)
REPLAY_SLACK = 10.0     # s the --view replay stays up beyond the run's length
VIEWER_OUTPUT = ("view", "live.jsonl")   # HIM's viewer files, rebuilt from trace.npz by replay
ROUTES = HERE / "routes"
ROUTE_SPEED = 0.15      # m/s along a route's straight legs, for its run length (Nav2's mean is about this)
ROUTE_POINT_TIME = 10.0 # s more per point: turning to it and settling

try:
    import mujoco
    import numpy as np
    import yaml
except ImportError:
    if Path(sys.prefix).resolve() == VENV_PYTHON.parents[1].resolve():   # already in it
        raise
    os.execv(str(VENV_PYTHON), [str(VENV_PYTHON), __file__, *sys.argv[1:]])

sys.path.insert(0, str(HERE))
import stage_world  # noqa: E402
from harp.movers import Movers, obstacle_geoms  # noqa: E402
from harp.route import route_from_data  # noqa: E402

SCENES = HIM / ".cache/harp/scenes"
MODEL = HERE / ".build/him/environment"


def read_log(run):
    """Our events (JSON lines in policy.log, after compose's "submission-1  | " prefix)."""
    events, odom = [], []
    for line in (run / "policy.log").read_text(errors="replace").splitlines():
        text = line.split("| ", 1)[-1].strip()
        if not text.startswith("{"):
            continue
        try:
            record = json.loads(text)
        except ValueError:
            continue
        (odom if record.get("event") == "odom" else events).append(record)
    return events, odom


def clearance(world, trace, movers_started):
    """Closest robot-to-obstacle distance and how many separate contacts, from the truth."""
    model = mujoco.MjModel.from_xml_path(str(world))
    data = mujoco.MjData(model)
    movers = Movers(model, started=movers_started)
    obstacles = obstacle_geoms(model)
    body = [g for g in range(model.ngeom)
            if model.geom_bodyid[g] > 0 and model.body_mocapid[model.geom_bodyid[g]] < 0
            and model.geom_contype[g]]
    closest, contacts, touching = CLEARANCE_MAX, 0, set()
    for t, qpos in zip(trace["time"][::CLEARANCE_EVERY], trace["qpos"][::CLEARANCE_EVERY]):
        data.qpos[:] = qpos
        movers.apply(data, t)
        mujoco.mj_kinematics(model, data)
        now = set()
        for o in obstacles:
            d = min(mujoco.mj_geomDistance(model, data, g, o, CLEARANCE_MAX, None) for g in body)
            closest = min(closest, d)
            if d <= 0.0:
                now.add(o)
        contacts += len(now - touching)
        touching = now
    return round(float(closest), 3), contacts


def score(run):
    run = Path(run)
    meta = json.loads((run / "run.json").read_text())
    name = meta["world"]
    report = json.loads((run / "report.json").read_text())
    trace = np.load(run / "trace.npz")
    events, odom = read_log(run)
    result = next((e for e in events if e.get("event") == "goal_result"), None)
    started = {e["mover"]: e["starts_at"] for e in events if e.get("event") == "mover_start"}
    route = load_route(run / "route.yaml") if (run / "route.yaml").is_file() else None

    stage_world.main(name, MODEL, SCENES)
    world = SCENES / (name + "_all.xml")
    goal = (np.array(route.waypoints[-1][:2]) if route
            else mujoco.MjModel.from_xml_path(str(world)).site("goal").pos[:2])
    xy = trace["qpos"][:, :2]
    path = float(np.linalg.norm(np.diff(xy, axis=0), axis=1).sum())
    closest, contacts = clearance(world, trace, started)
    drift = (round(float(np.hypot(*(np.array(odom[-1]["legs"][:2]) - odom[-1]["truth"][:2]))), 3)
             if odom else None)

    if report["status"] not in ("within_limits", "completed") and report.get("host_stall"):
        outcome = "inconclusive"    # HIM's sim was paused by the host: rerun
    elif report["status"] not in ("within_limits", "completed"):
        outcome = "violation"
    elif result is None:
        outcome = "error"
    else:
        outcome = {"SUCCEEDED": "succeeded", "TIMEOUT": "timed_out",
                   "NAV2_NOT_ACTIVE": "error"}.get(result["status"], "failed")
    summary = {
        "world": name, "tag": meta.get("tag"), "outcome": outcome,
        "him_status": report["status"], "him_reason": report.get("reason"),
        "seconds": result and result["seconds"], "recoveries": result and result["recoveries"],
        "path_m": round(path, 2), "end_error_m": round(float(np.linalg.norm(xy[-1] - goal)), 3),
        "closest_m": closest, "contacts": contacts, "drift_m": drift,
    }
    if route:
        summary["route"] = {"waypoints": len(route.waypoints) * route.laps, "laps": route.laps,
                            "completed_waypoints": result and result.get("completed_waypoints"),
                            "completed_laps": result and result.get("completed_laps")}
        criteria = route.criteria
    else:
        criteria = world_criteria(name)
    summary["passed"], summary["failed_checks"] = check(summary, criteria)
    (run / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    return summary


def world_criteria(name):
    """The world's criteria (submission/worlds/NAME.yaml), or None."""
    path = HERE / "worlds" / (name + ".yaml")
    return yaml.safe_load(path.read_text()) if path.is_file() else None


def load_route(value):
    """A route from a file, or by name from submission/routes; ValueError if bad."""
    path = Path(value).expanduser()
    if not path.is_file():
        path = ROUTES / (value if value.endswith(".yaml") else value + ".yaml")
    try:
        data = yaml.safe_load(path.read_text())
    except (OSError, yaml.YAMLError) as e:
        raise ValueError(f"can't read route {value}: {e}")
    return route_from_data(data)


def route_time(route):
    """s the sim runs for a route, before STARTUP: its max_time_s, or an estimate."""
    if route.criteria and "max_time_s" in route.criteria:
        return float(route.criteria["max_time_s"])
    points = [(0.0, 0.0)] + [p[:2] for p in route.waypoints] * route.laps
    length = sum(math.dist(a, b) for a, b in zip(points, points[1:]))
    return length / ROUTE_SPEED + ROUTE_POINT_TIME * (len(points) - 1)


def check(summary, c):
    """True, [] if every criterion in c holds; None, [] without criteria."""
    if not c:
        return None, []
    failed = []
    if c.get("must_succeed") and summary["outcome"] != "succeeded":
        failed.append("must_succeed")
    for key, field, worse in (("max_time_s", "seconds", float.__gt__),
                              ("min_clearance_m", "closest_m", float.__lt__),
                              ("max_contacts", "contacts", float.__gt__),
                              ("max_recoveries", "recoveries", float.__gt__),
                              ("max_end_error_m", "end_error_m", float.__gt__)):
        value = summary[field]
        if key in c and (value is None or worse(float(value), float(c[key]))):
            failed.append(key)
    return not failed, failed


def prune(run):
    """Delete HIM's viewer output from a run folder; trace.npz and the logs stay."""
    for name in VIEWER_OUTPUT:
        path = run / name
        if path.is_dir():
            shutil.rmtree(path)
        elif path.exists():
            path.unlink()


def git_state():
    """This repo's commit, and its uncommitted changes to tracked files ("" if none)."""
    def git(*args):
        return subprocess.run(["git", "-C", str(REPO), *args], capture_output=True, text=True).stdout
    return git("rev-parse", "--short", "HEAD").strip() or None, git("diff", "HEAD")


def compare(world=None):
    """One line per world and tag over every scored run in runs/."""
    groups = {}
    for path in sorted(RUNS.glob("*/summary.json")):
        if not (path.parent / "run.json").is_file():    # the old stack's runs
            continue
        s = json.loads(path.read_text())
        if world is None or s["world"] == world:
            groups.setdefault((s["world"], s["tag"] or "-"), []).append(s)
    print(f"{'world':16} {'tag':14} {'runs':>4} {'ok':>3} {'pass':>4}  {'time s (min-max)':18} "
          f"{'closest':>7} {'end m':>6} {'recov':>5} {'drift':>5}")
    for (w, tag), runs in sorted(groups.items()):
        done = [s for s in runs if s["outcome"] == "succeeded"]
        times = [s["seconds"] for s in done if s["seconds"] is not None]
        span = f"{np.mean(times):5.1f} ({min(times):.0f}-{max(times):.0f})" if times else "-"
        mean = lambda key, rs=done: f"{np.mean([s[key] for s in rs if s[key] is not None]):.2f}" \
            if any(s[key] is not None for s in rs) else "-"
        closest = f"{min(s['closest_m'] for s in done):.2f}" if done else "-"
        drift = f"{max(s['drift_m'] or 0 for s in done):.3f}" if done else "-"
        passed = sum(1 for s in runs if s["passed"]) if any(s["passed"] is not None for s in runs) else "-"
        print(f"{w:16} {tag:14} {len(runs):>4} {len(done):>3} {passed:>4}  {span:18} "
              f"{closest:>7} {mean('end_error_m'):>6} {mean('recoveries'):>5} {drift:>5}")


def run_once(name, tag, route=None):
    max_time = route_time(route) if route else (world_criteria(name) or {}).get("max_time_s", 60)
    stamp = datetime.datetime.now(datetime.timezone.utc).strftime("%Y-%m-%d_%H-%M-%SZ")
    run = RUNS / "_".join(filter(None, (stamp, "sim", name, tag)))
    run.mkdir(parents=True)
    env = dict(os.environ)
    if route:
        (run / "route.yaml").write_text(yaml.safe_dump(route.data(), sort_keys=False))
        env["G1_TASK_COMMAND"] = json.dumps({"route": route.data()})   # for harp/course_goal.py
    commit, diff = git_state()
    if diff:
        (run / "git.diff").write_text(diff)
    (run / "run.json").write_text(json.dumps({"world": name, "tag": tag, "started": stamp,
                                              "commit": commit, "dirty": bool(diff)}) + "\n")
    # Never HIM's live viewer: its load delayed HIM's states past the 0.2 s deadlines and
    # stopped about 1 in 4 watched runs. --view replays the run afterwards (replay()).
    command = [str(HERE / "sim.sh"), "--world", name, "--seconds", str(int(max_time + STARTUP)),
               "--output", str(run), "--no-view"]
    with open(run / "sim.log", "w") as log:
        subprocess.run(command, stdout=log, stderr=subprocess.STDOUT, env=env)
    if not (run / "report.json").is_file():
        print(f"{run.name}: no report; see {run / 'sim.log'}")
        return None
    return run


def replay(run):
    """Serve HIM's replay of a run until it has had time to play once, then close it."""
    with open(run / "replay.log", "w") as log:
        server = subprocess.Popen([str(HIM / "desktop_sim.sh"), "replay", str(run / "trace.npz")],
                                  stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        while server.poll() is None and "Open http" not in (run / "replay.log").read_text(errors="replace"):
            time.sleep(0.5)
        url = next((line.split()[-1] for line in (run / "replay.log").read_text().splitlines()
                    if line.startswith("Open http")), None)
        if url:
            length = float(np.load(run / "trace.npz", allow_pickle=True)["time"][-1])
            print(f"{run.name}: replay at {url} for {length + REPLAY_SLACK:.0f} s", flush=True)
            time.sleep(length + REPLAY_SLACK)
        else:
            print(f"{run.name}: no replay; see {run / 'replay.log'}", flush=True)
        stop(server, run.name)
    (run / "replay.log").unlink()


def stop(process, name):
    """Ctrl-C a process group, then SIGTERM, then SIGKILL, until it exits."""
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGKILL):
        if process.poll() is not None:
            return
        os.killpg(process.pid, sig)
        try:
            process.wait(timeout=15)
        except subprocess.TimeoutExpired:
            print(f"{name}: the replay didn't close on {sig.name}", flush=True)


def positive(text):
    if not text.isdigit() or int(text) == 0:
        raise argparse.ArgumentTypeError("must be a positive whole number")
    return int(text)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("world", nargs="?", help="a world in submission/worlds")
    parser.add_argument("--tag", help="added to the run folder's name")
    parser.add_argument("--repeat", type=positive, default=1)
    where = parser.add_mutually_exclusive_group()
    where.add_argument("--route", metavar="NAME_OR_FILE", help="a route in submission/routes, or a file")
    where.add_argument("--waypoint", action="append", nargs="+", type=float, metavar="NUM",
                       help="X Y [HEADING] in odom; one per point, in order")
    parser.add_argument("--laps", type=positive, help="laps of the route (default: its file's, or 1)")
    parser.add_argument("--view", action="store_true",
                        help="after each run, replay it in HIM's browser viewer before the next")
    parser.add_argument("--score", metavar="RUN", help="only (re)score this run folder")
    parser.add_argument("--compare", nargs="?", const="", metavar="WORLD",
                        help="only list the scored runs in runs/ by world and tag")
    parser.add_argument("--prune", action="store_true",
                        help="only delete HIM's viewer output from every course run in runs/")
    args = parser.parse_args()
    if args.laps and not (args.route or args.waypoint):
        parser.error("--laps needs --route or --waypoint")
    route = None
    if args.route or args.waypoint:
        try:
            route = load_route(args.route) if args.route else route_from_data(
                {"frame": "odom", "waypoints": args.waypoint})
        except ValueError as e:
            parser.error(str(e))
        route.laps = args.laps or route.laps
    if args.compare is not None:
        compare(args.compare or None)
        return 0
    if args.prune:
        for run in sorted(p.parent for p in RUNS.glob("*/run.json")):
            prune(run)
        return 0
    runs = [Path(args.score)] if args.score else range(args.repeat)
    failed = False
    for run in runs:
        run = run if args.score else run_once(args.world, args.tag, route)
        if run is None:
            failed = True
            continue
        s = score(run)
        verdict = {True: "PASS", False: "FAIL " + ",".join(s["failed_checks"]), None: "-"}[s["passed"]]
        print(f"{run.name}: {s['outcome']} ({s['him_status']}) {verdict}  "
              f"{s['seconds']} s, path {s['path_m']} m, end {s['end_error_m']} m, "
              f"closest {s['closest_m']} m, contacts {s['contacts']}, "
              f"recoveries {s['recoveries']}, drift {s['drift_m']} m"
              + (f", waypoints {s['route']['completed_waypoints']}/{s['route']['waypoints']}"
                 if "route" in s else ""))
        failed |= s["outcome"] != "succeeded" or s["passed"] is False
        if args.view and not args.score:
            replay(run)    # before the next run, so its load can't delay that run's states
        prune(run)
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
