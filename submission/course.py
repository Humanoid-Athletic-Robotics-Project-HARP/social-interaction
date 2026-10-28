"""Run a course test in HIM's simulator and score it:
  python3 submission/course.py one_box
  python3 submission/course.py slalom --tag rpp --repeat 3 --view
  python3 submission/course.py --score runs/2026-10-28_05-00-00Z_sim_one_box
  python3 submission/course.py --prune

Each run gets a folder in runs/ (HARP_RUNS_DIR to change it) holding HIM's
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
After scoring, HIM's viewer output (view/ and live.jsonl, tens of MB a watched run)
is deleted: HIM's ./desktop_sim.sh replay RUN/trace.npz rebuilds it. --prune does
the same for every run folder course.py made (those with a run.json).

Needs MuJoCo, so it runs under HIM's .venv-policy-safety (it re-executes
itself there). Build the image first: submission/build.sh.
"""

import argparse
import datetime
import json
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
VIEW_LINGER = 5.0       # s the live viewer stays up after the report, with --view
VIEWER_OUTPUT = ("view", "live.jsonl")   # HIM's viewer files, rebuilt from trace.npz by replay

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

    stage_world.main(name, MODEL, SCENES)
    world = SCENES / (name + "_all.xml")
    goal = mujoco.MjModel.from_xml_path(str(world)).site("goal").pos[:2]
    xy = trace["qpos"][:, :2]
    path = float(np.linalg.norm(np.diff(xy, axis=0), axis=1).sum())
    closest, contacts = clearance(world, trace, started)
    drift = (round(float(np.hypot(*(np.array(odom[-1]["legs"][:2]) - odom[-1]["truth"][:2]))), 3)
             if odom else None)

    if report["status"] not in ("within_limits", "completed") and report.get("host_stall"):
        outcome = "inconclusive"    # HIM's sim was paused by the host: rerun, maybe with --no-view
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
    summary["passed"], summary["failed_checks"] = check(summary, HERE / "worlds" / (name + ".yaml"))
    (run / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    return summary


def check(summary, criteria_file):
    """True, [] if every criterion holds; None, [] without criteria."""
    if not criteria_file.is_file():
        return None, []
    c = yaml.safe_load(criteria_file.read_text())
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


def run_once(name, tag, view):
    criteria = HERE / "worlds" / (name + ".yaml")
    max_time = yaml.safe_load(criteria.read_text()).get("max_time_s", 60) if criteria.is_file() else 60
    stamp = datetime.datetime.now(datetime.timezone.utc).strftime("%Y-%m-%d_%H-%M-%SZ")
    run = RUNS / "_".join(filter(None, (stamp, "sim", name, tag)))
    run.mkdir(parents=True)
    (run / "run.json").write_text(json.dumps({"world": name, "tag": tag, "started": stamp}) + "\n")
    command = [str(HERE / "sim.sh"), "--world", name, "--seconds", str(int(max_time + STARTUP)),
               "--output", str(run)] + ([] if view else ["--no-view"])
    with open(run / "sim.log", "w") as log:
        sim = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        if view:
            print(f"{run.name}: press Start at http://127.0.0.1:8765", flush=True)
            # HIM's viewer keeps serving after the run; close it once the report is written.
            while sim.poll() is None and not (run / "report.json").is_file():
                time.sleep(1)
            time.sleep(VIEW_LINGER)
            if sim.poll() is None:
                os.killpg(sim.pid, signal.SIGINT)
        sim.wait()
    if not (run / "report.json").is_file():
        print(f"{run.name}: no report; see {run / 'sim.log'}")
        return None
    return run


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("world", nargs="?", help="a world in submission/worlds")
    parser.add_argument("--tag", help="added to the run folder's name")
    parser.add_argument("--repeat", type=int, default=1)
    parser.add_argument("--view", action="store_true", help="watch in HIM's browser viewer")
    parser.add_argument("--score", metavar="RUN", help="only (re)score this run folder")
    parser.add_argument("--prune", action="store_true",
                        help="only delete HIM's viewer output from every course run in runs/")
    args = parser.parse_args()
    if args.prune:
        for run in sorted(p.parent for p in RUNS.glob("*/run.json")):
            prune(run)
        return 0
    runs = [Path(args.score)] if args.score else [
        run_once(args.world, args.tag, args.view) for _ in range(args.repeat)]
    for run in filter(None, runs):
        s = score(run)
        prune(run)
        verdict = {True: "PASS", False: "FAIL " + ",".join(s["failed_checks"]), None: "-"}[s["passed"]]
        print(f"{run.name}: {s['outcome']} ({s['him_status']}) {verdict}  "
              f"{s['seconds']} s, path {s['path_m']} m, end {s['end_error_m']} m, "
              f"closest {s['closest_m']} m, contacts {s['contacts']}, "
              f"recoveries {s['recoveries']}, drift {s['drift_m']} m")
    return 0


if __name__ == "__main__":
    sys.exit(main())
