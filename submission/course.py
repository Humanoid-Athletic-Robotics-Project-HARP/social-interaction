"""Run a course test in HIM's simulator and score it:
  python3 submission/course.py one_box
  python3 submission/course.py slalom --tag rpp --repeat 3 --view
  python3 submission/course.py one_box --route rectangle --laps 3
  python3 submission/course.py open_floor --waypoint 2 0 --waypoint 2 2 90 --waypoint 0 0
  python3 submission/course.py --score runs/2026-10-28_05-00-00Z_sim_one_box
  python3 submission/course.py --prune
  python3 submission/course.py --replay runs/2026-10-28_05-00-00Z_sim_one_box
  python3 submission/course.py --compare slalom
  python3 submission/course.py --events runs/2026-10-28_05-00-00Z_sim_one_box

Each run gets a folder in the main checkout's runs/, shared by every git worktree
(HARP_RUNS_DIR to change it; checkout.py), holding our run.json (world, tag, start, git
commit and branch, whether the tree was dirty; git.diff then holds the uncommitted
changes; and from sim.sh, "sim": its arguments, image, G1_TASK_COMMAND and
HIM's commit), the scene HIM loaded without HIM's G1 (scene/, with the world as our lidar saw it,
movers included: MOVERS_WORLD), HIM's
report.json, trace.npz (the true pose every 20 ms) and policy.log (our image's
output), our events.jsonl (our records from policy.log, one JSON object per line:
harp/events.py; scoring reads only this) and view.jsonl (harp/view_feed.py's frames,
moved out of policy.log: submission/lidar_view.py --run shows them), and our summary.json:
  outcome    succeeded / failed / timed_out / error / violation (HIM stopped the policy) /
             inconclusive (HIM stopped it while the host had paused its sim: host_stall)
  passed     against the world's criteria, submission/worlds/NAME.yaml
  seconds, recoveries   from course_goal's goal_result
  path_m, end_error_m   from the true pose (end: distance from the goal site)
  closest_m, contacts   true distance from the robot's body to every obstacle,
                        movers included (replayed from their mover_start events, or for
                        an aware crowd from the paths the lidar logged; HIM's physics
                        never sees movers, so these are the only contacts)
  speed_near_people_mps  the robot's fastest true speed while a mover within PEOPLE_NEAR
                        was coming closer (harp/approach_stop.py should have it standing)
  drift_m    leg odometry against the truth at the last "odom" line
  crowd      scripted, or aware: --crowd aware has the world's movers give way to the
             robot (harp/movers.py AwareMovers) instead of keeping to their routes
A world whose criteria give a task (people.yaml: the behavior loop, harp/behavior.py) runs
that as G1_TASK_COMMAND instead of its goal. Then outcome is succeeded if the loop started
and didn't stop on an error, seconds are from its start to its first talk, there's no end
error, and summary.json adds loop: each talk's true distance to that person and how far
the robot faced from them (medians over its start) and how it ended, how many wander goals and people
noticed, who it found not a person (not_people: candidates the camera never confirmed),
and the farthest the robot got from its start.
A route (--route NAME_OR_FILE from submission/routes, or --waypoint X Y [HEADING] for
each point; --laps N, harp/route.py) replaces the world's goal: the run folder keeps
it as route.yaml, the end error is from its last point, passed uses its criteria (none:
no verdict) instead of the world's, and summary.json adds route (planned and completed
waypoints and laps). The sim runs for the route's max_time_s, or ROUTE_SPEED and
ROUTE_POINT_TIME per point, plus STARTUP.
It exits 1 if any run didn't succeed or failed its criteria.
--events RUN prints the run's events in order, with their sim time (estimated from the
wall clock, "~", for programs that don't read HIM's state), the seconds whose
latest state was LATE s or more old (harp_node.py's "timing" lines; each such state is a
"late_state" event), and the seconds the host paged in PAGED_IN MB or more (macOS:
decompressed; run.json's host_memory "seconds").
--compare [WORLD] lists every scored run by world and tag: how many succeeded and
passed, time (mean, min-max), closest, end error, recoveries and drift.
--view watches each run in HIM's live viewer, at the URL it prints: the run starts at once
(sim.sh), and Start on the page plays it from the first frame; it closes the viewer
VIEW_LINGER s after HIM's report, before the next run. HIM's
deadlines stop runs when the host is short of memory, and a viewer holds about 700 MB, so
before each run it warns about HIM viewers still up and macOS memory pressure, and run.json
keeps the host's memory during the run (host_memory: free, compressed and decompressed MB).
Pruning, after every run (sim.sh's with --output too) or with --prune [RUN]: our lines are
split out of policy.log (above), HIM's viewer output (view/, tens of MB) is deleted, and every trace.npz but the newest KEEP_FULL of each
world loses STRIPPED (about 52 of its 53 MB), once the scene rebuilt from run.json and
scene/ (HIM's G1 read at HIM's commit) matches the one in the trace. Runs from before
run.json kept a scene get theirs filled in the same way. --replay RUN serves HIM's replay
with the scene put back, from a copy of the trace; a stripped run's thrash panel is empty.
HIM's live viewer never shows movers (only our lidar sees them); the replay does, walking
where scoring placed them (an aware crowd: where the lidar logged them).
Each run, with its scoring and pruning, and --prune and --score hold checkout.py's
machine-wide lock: one build or sim at a time, waiting for whoever holds it.

Needs MuJoCo, so it runs under HIM's .venv-policy-safety (it re-executes
itself there). Build the image first: submission/build.sh.
"""

import argparse
import datetime
import json
import math
import os
import re
import shutil
import signal
import subprocess
import sys
import tempfile
import threading
import time
import urllib.request
import zipfile
from pathlib import Path

import checkout  # beside this file
from host_health import host_memory, memory_warnings  # beside this file too

HERE = Path(__file__).resolve().parent
REPO = HERE.parent
HIM = checkout.HIM
VENV_PYTHON = HIM / ".venv-policy-safety/bin/python"
RUNS = checkout.RUNS
STARTUP = 12.0          # s added to the world's max_time_s: Nav2 sends the goal about 6 s in, plus slack
CLEARANCE_MAX = 2.0     # m: farther than this counts as this
CLEARANCE_EVERY = 5     # trace frames between clearance checks (0.1 s)
TALK_SKIP = 0.5         # s after a talk is logged before it is scored (the robot settling)
TALK_SCORED = 3.0       # s of each talk scored from then: where the robot stopped, before anyone leaves
PEOPLE_NEAR = 1.5       # m, robot to a mover's centre, inside which the robot should stand
SPEED_WINDOW = 5        # clearance checks the robot's speed is taken over (0.5 s: GR00T sways)
VIEW_LINGER = 10.0      # s the --view live viewer stays up after HIM's report
VIEWER_OUTPUT = ("view", "live.jsonl")   # HIM's viewer files, rebuilt from trace.npz by replay
KEEP_FULL = 2           # newest runs of each world whose trace.npz keeps everything (tags are
                        # mostly one-off experiments: per world and tag kept 60 of 69 whole)
STRIPPED = ("model", "thrash")   # the rest lose these: the compiled scene (about 47 of a trace's
                                 # 53 MB; rebuilt from run.json) and HIM's thrash telemetry (5 MB)
G1 = "packages/policy_safety/src/policy_safety/robots/g1"   # HIM's G1 model, in its checkout
ROUTES = HERE / "routes"
ROUTE_SPEED = 0.15      # m/s along a route's straight legs, for its run length (Nav2's mean is about this)
ROUTE_POINT_TIME = 10.0 # s more per point: turning to it and settling
EVENTS = "events.jsonl" # our records from policy.log, in the run folder
VIEW = "view.jsonl"     # harp/view_feed.py's frames (source "view"), moved out of policy.log
MOVERS_WORLD = "image_world.xml"   # in scene/: the world with its movers (our lidar's)
PAGED_IN = 50           # MB in one second: --events shows that second of host memory
LATE = 0.04             # s: --events shows the seconds whose worst state was this old

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
from harp.leg_odometry import wrap_angle, yaw_of  # noqa: E402
from harp.movers import CROWDS, Movers, logged_path, obstacle_geoms  # noqa: E402
from harp.route import route_from_data  # noqa: E402

SCENES = checkout.STAGE / "scenes"
MODEL = HERE / ".build/him/environment"


def our_record(line):
    """Our JSON record in a line of the container's output (after compose's
    "submission-1  | " prefix), or None."""
    text = line.split("| ", 1)[-1].strip()
    if not text.startswith("{"):
        return None
    try:
        record = json.loads(text)
    except ValueError:
        return None
    return record if isinstance(record, dict) and "event" in record else None


def split_log(run):
    """Copy our records from policy.log into EVENTS, and move the viewer's frames out of
    it into VIEW (a few MB a run, otherwise unreadable there). Done once per run."""
    log = run / "policy.log"
    if (run / EVENTS).is_file() or not log.is_file():
        return
    events, view, rest = [], [], []
    for line in log.read_text(errors="replace").splitlines():
        record = our_record(line)
        if record is not None and record.get("source") == "view":
            view.append(json.dumps(record))
            continue
        rest.append(line)
        if record is not None:
            events.append(json.dumps(record))
    if view:
        (run / VIEW).write_text("\n".join(view) + "\n")
        part = run / "policy.part.log"
        part.write_text("\n".join(rest) + "\n")
        os.replace(part, log)
    part = run / (EVENTS + ".part")
    part.write_text("".join(e + "\n" for e in events))
    os.replace(part, run / EVENTS)


def read_log(run):
    """Our records from the run's EVENTS: the events, and the "odom" lines."""
    split_log(run)
    events, odom = [], []
    for line in (run / EVENTS).read_text().splitlines():
        record = json.loads(line)
        (odom if record.get("event") == "odom" else events).append(record)
    return events, odom


def sim_time_of(event, odom):
    """The sim time of an event without one of its own, from its wall time and the last
    "odom" record (both clocks) logged before it; None without one."""
    before = [o for o in odom if "wall" in o and o["wall"] <= event.get("wall", -math.inf)]
    return before[-1]["sim_time"] + event["wall"] - before[-1]["wall"] if before else None


def show_events(run):
    """Print the run's events with their sim time; see --events."""
    events, _ = read_log(run)
    host = (json.loads((run / "run.json").read_text()).get("host_memory") or {}).get("seconds", [])
    events += [{"source": "host", "event": "memory", "wall": wall, "free_mb": free,
                "paged_in_mb": paged} for wall, free, paged in host if paged >= PAGED_IN]
    timed = sorted((e for e in events if "wall" in e), key=lambda e: e["wall"])
    events = [e for e in events if "wall" not in e] + timed
    clock = None     # (wall, sim time) of the latest record that has both
    for e in events:
        if "wall" in e and "sim_time" in e:
            clock = (e["wall"], e["sim_time"])
        if e["event"] == "timing" and e.get("age", 0.0) < LATE:
            continue
        if "sim_time" in e:
            when = f"{e['sim_time']:8.2f}"
        elif "wall" in e and clock:
            when = f"~{clock[1] + e['wall'] - clock[0]:7.2f}"
        else:
            when = " " * 8
        fields = " ".join(f"{k}={json.dumps(v)}" for k, v in e.items()
                          if k not in ("source", "event", "wall", "sim_time"))
        print(f"{when}  {e.get('source', '-'):10} {e['event']:16} {fields}")


def geom_distance(model, data, g, o, limit):
    """mj_geomDistance from geom g to geom o, checked. For a capsule and a cylinder MuJoCo 3.3
    now and then returns 0 with nearest points far from both geoms (HIM atrium runs, Nov 3:
    8 of 3.1 million robot-to-fixed pairs, each a pillar "contact" with the foot 0.66 m away;
    once its nearest points were on both geoms but 0.62 m apart, a wheelchair "contact").
    MuJoCo 3.2.3 (our image's, where the tests run) also returns too small a positive
    distance: 0.39 m for a foot 0.57 m from a pillar, its nearest point 0.4 m off the foot.
    Any result under limit not borne out by its own nearest points (off geom g, or not as
    far apart as it says) is replaced by the exact distance when g is a sphere or capsule
    and o a cylinder (the cases seen); any other is kept as it came."""
    fromto = np.zeros(6)
    d = mujoco.mj_geomDistance(model, data, g, o, limit, fromto)
    if d >= limit or (abs(np.linalg.norm(fromto[3:] - fromto[:3]) - abs(d)) <= 1e-3
                      and np.linalg.norm(fromto[:3] - data.geom_xpos[g]) <= model.geom_rbound[g] + 1e-6):
        return d
    exact = rounded_to_cylinder(model, data, g, o)
    return d if exact is None else min(exact, limit)


def rounded_to_cylinder(model, data, g, o):
    """Signed distance from a sphere or capsule g to a cylinder o (None for other shapes):
    its axis sampled every few mm, each point measured to the cylinder exactly."""
    sphere, capsule, cylinder = (int(t) for t in (
        mujoco.mjtGeom.mjGEOM_SPHERE, mujoco.mjtGeom.mjGEOM_CAPSULE, mujoco.mjtGeom.mjGEOM_CYLINDER))
    if int(model.geom_type[o]) != cylinder or int(model.geom_type[g]) not in (sphere, capsule):
        return None
    half = model.geom_size[g][1] if int(model.geom_type[g]) == capsule else 0.0
    axis = data.geom_xmat[g].reshape(3, 3)[:, 2]
    points = data.geom_xpos[g] + np.linspace(-half, half, 65)[:, None] * axis
    local = (points - data.geom_xpos[o]) @ data.geom_xmat[o].reshape(3, 3)
    radius, height = model.geom_size[o][:2]
    across = np.hypot(local[:, 0], local[:, 1]) - radius
    along = np.abs(local[:, 2]) - height
    outside = np.hypot(np.maximum(across, 0), np.maximum(along, 0))
    inside = np.minimum(np.maximum(across, along), 0)
    return float((outside + inside).min() - model.geom_size[g][0])


def clearance(world, trace, movers_started, path=None):
    """From the truth: the closest robot-to-obstacle distance, how many separate contacts,
    and the robot's fastest speed (over SPEED_WINDOW samples) while a mover nearer than
    PEOPLE_NEAR was coming closer (None without movers). path: an aware crowd's logged
    paths (logged_path), which the movers then follow."""
    model = mujoco.MjModel.from_xml_path(str(world))
    data = mujoco.MjData(model)
    movers = Movers(model, started=movers_started, path=path)
    obstacles = obstacle_geoms(model)
    body = [g for g in range(model.ngeom)
            if model.geom_bodyid[g] > 0 and model.body_mocapid[model.geom_bodyid[g]] < 0
            and model.geom_contype[g]]
    closest, contacts, touching = CLEARANCE_MAX, 0, set()
    near_speed = 0.0 if movers.mocap else None
    seen, apart = [], None   # (t, robot xy) per sample; distance to each mover
    for t, qpos in zip(trace["time"][::CLEARANCE_EVERY], trace["qpos"][::CLEARANCE_EVERY]):
        data.qpos[:] = qpos
        movers.apply(data, t)
        mujoco.mj_kinematics(model, data)
        seen = (seen + [(t, qpos[:2].copy())])[-(SPEED_WINDOW + 1):]
        now_apart = [float(np.linalg.norm(data.mocap_pos[m][:2] - qpos[:2])) for m in movers.mocap]
        if apart and len(seen) > SPEED_WINDOW and any(
                d < PEOPLE_NEAR and d < before for d, before in zip(now_apart, apart)):
            (t0, xy0), (t1, xy1) = seen[0], seen[-1]
            near_speed = max(near_speed, float(np.linalg.norm(xy1 - xy0)) / (t1 - t0))
        apart = now_apart
        now = set()
        for o in obstacles:
            d = min(geom_distance(model, data, g, o, CLEARANCE_MAX) for g in body)
            closest = min(closest, d)
            if d <= 0.0:
                now.add(o)
        contacts += len(now - touching)
        touching = now
    return round(float(closest), 3), contacts, near_speed and round(near_speed, 3)


def talk_scores(world, trace, movers_started, talks, ends):
    """For each talk event: medians over TALK_SCORED s of the talk (from TALK_SKIP s after its
    sim time) of the true distance from the robot to that person's centre, and of how far
    (deg) the robot's heading is from facing them; and from its talk_done (ends: the same
    person's next one, or None), how it ended and how long it took."""
    model = mujoco.MjModel.from_xml_path(str(world))
    data = mujoco.MjData(model)
    movers = Movers(model, started=movers_started)
    scores = []
    for talk, end in zip(talks, ends):
        if talk["person"] not in movers.names:    # a talk with something that isn't a person
            scores.append({"person": talk["person"], "distance_m": None, "facing_deg": None,
                           "ended": end and end["ended"], "seconds": end and end["seconds"]})
            continue
        mocap = movers.mocap[movers.names.index(talk["person"])]
        start = (talk["sim_time"] if talk["sim_time"] is not None else math.inf) + TALK_SKIP
        during = (trace["time"] >= start) & (trace["time"] <= start + TALK_SCORED)
        distance, facing = [], []
        for t, qpos in zip(trace["time"][during], trace["qpos"][during]):
            movers.apply(data, t)
            dx, dy = data.mocap_pos[mocap][:2] - qpos[:2]
            distance.append(math.hypot(dx, dy))
            facing.append(abs(math.degrees(wrap_angle(math.atan2(dy, dx) - yaw_of(qpos[3:7])))))
        scores.append({"person": talk["person"],
                       "distance_m": round(float(np.median(distance)), 2) if distance else None,
                       "facing_deg": round(float(np.median(facing)), 1) if facing else None,
                       "ended": end and end["ended"], "seconds": end and end["seconds"]})
    return scores


def score(run):
    run = Path(run)
    meta = json.loads((run / "run.json").read_text())
    name = meta["world"]
    report = json.loads((run / "report.json").read_text())
    trace = np.load(run / "trace.npz")
    events, odom = read_log(run)
    result = next((e for e in events if e.get("event") == "goal_result"), None)
    started = {e["mover"]: e["starts_at"] for e in events if e.get("event") == "mover_start"}
    logged = logged_path(events)
    route = load_route(run / "route.yaml") if (run / "route.yaml").is_file() else None
    behavior = '"behavior"' in ((meta.get("sim") or {}).get("task_command") or "")
    loop = [dict(e, sim_time=sim_time_of(e, odom)) for e in events if e.get("source") == "behavior"]

    stage_world.main(name, MODEL, SCENES)
    world = SCENES / (name + "_all.xml")
    goal = (np.array(route.waypoints[-1][:2]) if route else None if behavior
            else mujoco.MjModel.from_xml_path(str(world)).site("goal").pos[:2])
    xy = trace["qpos"][:, :2]
    path = float(np.linalg.norm(np.diff(xy, axis=0), axis=1).sum())
    closest, contacts, near_speed = clearance(world, trace, started, logged)
    drift = (round(float(np.hypot(*(np.array(odom[-1]["legs"][:2]) - odom[-1]["truth"][:2]))), 3)
             if odom else None)

    if report["status"] not in ("within_limits", "completed") and report.get("host_stall"):
        outcome = "inconclusive"    # HIM's sim was paused by the host: rerun
    elif report["status"] not in ("within_limits", "completed"):
        outcome = "violation"
    elif behavior:
        ok = any(e["event"] == "loop_start" for e in loop) and not any(e["event"] == "error" for e in loop)
        outcome = "succeeded" if ok else "error"
    elif result is None:
        outcome = "error"
    else:
        outcome = {"SUCCEEDED": "succeeded", "TIMEOUT": "timed_out",
                   "NAV2_NOT_ACTIVE": "error"}.get(result["status"], "failed")
    if behavior:
        talks = [e for e in loop if e["event"] == "talk"]
        begun = next((e["sim_time"] for e in loop if e["event"] == "loop_start"), None)
        first = talks[0]["sim_time"] if talks else None
        result = {"seconds": round(first - begun, 1) if None not in (first, begun) else None,
                  "recoveries": None}
    summary = {
        "world": name, "tag": meta.get("tag"), "outcome": outcome,
        "him_status": report["status"], "him_reason": report.get("reason"),
        "seconds": result and result["seconds"], "recoveries": result and result["recoveries"],
        "path_m": round(path, 2),
        "end_error_m": None if goal is None else round(float(np.linalg.norm(xy[-1] - goal)), 3),
        "closest_m": closest, "contacts": contacts, "speed_near_people_mps": near_speed,
        "drift_m": drift, "crowd": "scripted" if logged is None else "aware",
    }
    if behavior:
        summary["loop"] = {
            "talks": talk_scores(world, trace, started, talks, [
                next((e for e in loop[loop.index(t):] if e["event"] == "talk_done"
                      and e["person"] == t["person"]), None) for t in talks]),
            "wander_goals": sum(1 for e in loop if e["event"] == "wander_goal"),
            "noticed": sum(1 for e in loop if e["event"] == "noticed"),
            "not_people": sorted({e["person"] for e in loop if e["event"] == "not_a_person"}),
            "farthest_m": round(float(np.linalg.norm(xy - xy[0], axis=1).max()), 2)}
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
                              ("max_first_talk_s", "seconds", float.__gt__),
                              ("min_clearance_m", "closest_m", float.__lt__),
                              ("max_contacts", "contacts", float.__gt__),
                              ("max_speed_near_people_mps", "speed_near_people_mps", float.__gt__),
                              ("max_recoveries", "recoveries", float.__gt__),
                              ("max_end_error_m", "end_error_m", float.__gt__)):
        value = summary[field]
        if key in c and (value is None or not math.isfinite(float(value))
                         or not math.isfinite(float(c[key])) or worse(float(value), float(c[key]))):
            failed.append(key)
    talks = (summary.get("loop") or {}).get("talks", [])
    if "min_talks" in c and len(talks) < c["min_talks"]:
        failed.append("min_talks")
    if "first_talk" in c and (not talks or talks[0]["person"] != c["first_talk"]):
        failed.append("first_talk")
    found = (summary.get("loop") or {}).get("not_people", [])
    if any(p not in found or p in {t["person"] for t in talks} for p in c.get("not_people", [])):
        failed.append("not_people")
    low, high = c.get("talk_distance_m", (-math.inf, math.inf))
    if any(t["distance_m"] is None or not low <= t["distance_m"] <= high for t in talks):
        failed.append("talk_distance_m")
    if "max_talk_facing_deg" in c and any(
            t["facing_deg"] is None or t["facing_deg"] > c["max_talk_facing_deg"] for t in talks):
        failed.append("max_talk_facing_deg")
    return not failed, failed


def prune(run):
    """Split our lines out of policy.log and delete HIM's viewer output from a run folder;
    trace.npz and the logs stay."""
    split_log(run)
    for name in VIEWER_OUTPUT:
        path = run / name
        if path.is_dir():
            shutil.rmtree(path)
        elif path.exists():
            path.unlink()


def git_state():
    """This checkout's commit, branch, and uncommitted changes to tracked files ("" if none)."""
    def git(*args):
        return subprocess.run(["git", "-C", str(REPO), *args], capture_output=True, text=True).stdout
    return (git("rev-parse", "--short", "HEAD").strip() or None, checkout.branch(),
            git("diff", "HEAD"))


def him_git(*args, **kwargs):
    """Read-only git in HIM's checkout."""
    return subprocess.run(["git", "-C", str(HIM), *args], capture_output=True, **kwargs)


def stamp_now():
    return datetime.datetime.now(datetime.timezone.utc).strftime("%Y-%m-%d_%H-%M-%SZ")


def new_run_json(run, started=None):
    """run.json's common part, and git.diff when the tree has uncommitted changes."""
    commit, branch, diff = git_state()
    if diff:
        (run / "git.diff").write_text(diff)
    return {"world": None, "tag": None, "started": started or stamp_now(), "commit": commit,
            "branch": branch, "dirty": bool(diff)}


def scene_files(scene):
    """The scene file and the files it includes, other than HIM's G1 (in MODEL)."""
    scene = Path(scene)
    files, todo = [], [scene]
    while todo:
        path = todo.pop()
        files.append(path)
        for name in re.findall(r'<include\s+file="([^"]+)"', path.read_text()):
            child = path.parent / name
            if not (MODEL / name).exists() and child not in files:
                todo.append(child)
    return files


def save_scene(run, scene):
    """Copy the scene (HIM's flat one without --scene) into run/scene/; its file name."""
    folder = run / "scene"
    folder.mkdir(exist_ok=True)
    files = scene_files(scene) if scene else [MODEL / "scene.xml"]
    for path in files:
        shutil.copy(path, folder / path.relative_to(files[0].parent))
    world = Path(scene).parent / "image" / Path(scene).stem / "world.xml" if scene else None
    if world and world.is_file():     # stage_world.py's world for our image, movers included
        shutil.copy(world, folder / MOVERS_WORLD)
    return files[0].name


def record(run, scene, image, args):
    """How sim.sh ran, in run.json (made here for a plain sim.sh run), and its scene."""
    run = Path(run)
    run.mkdir(parents=True, exist_ok=True)
    path = run / "run.json"
    meta = json.loads(path.read_text()) if path.is_file() else new_run_json(run)
    image_id = subprocess.run(["docker", "image", "inspect", "-f", "{{.Id}}", image],
                              capture_output=True, text=True).stdout.strip()
    meta["sim"] = {
        "args": args, "image": image, "image_id": image_id or None,
        "task_command": os.environ.get("G1_TASK_COMMAND") or None,
        "scene": save_scene(run, scene),
        "him_commit": him_git("rev-parse", "HEAD", text=True).stdout.strip() or None,
        "him_g1_dirty": bool(him_git("status", "--porcelain", "--", G1, text=True).stdout.strip()),
    }
    path.write_text(json.dumps(meta) + "\n")


def build_model(scene_dir, scene, him_commit):
    """Compile a saved scene with HIM's G1 as it was at him_commit (read with git archive)."""
    with tempfile.TemporaryDirectory() as tmp:
        archive = him_git("archive", him_commit, G1)
        if archive.returncode:
            raise ValueError(f"HIM's checkout has no commit {him_commit}")
        subprocess.run(["tar", "-x", "-C", tmp], input=archive.stdout, check=True)
        g1 = Path(tmp) / G1
        shutil.copytree(scene_dir, g1, dirs_exist_ok=True)
        return mujoco.MjModel.from_xml_path(str(g1 / scene))


def rebuild_model(run):
    """The run's compiled scene, from run.json and run/scene (what a stripped trace lacks)."""
    sim = json.loads((run / "run.json").read_text()).get("sim") or {}
    if not sim.get("scene") or not sim.get("him_commit"):
        raise ValueError(f"{run.name} has no saved scene")
    return build_model(run / "scene", sim["scene"], sim["him_commit"])


def same_model(a, b):
    """True if two compiled scenes have the same joints, bodies and geoms."""
    names = lambda m: [mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, i) for i in range(m.ngeom)]
    return (a.nq == b.nq and a.nbody == b.nbody and a.ngeom == b.ngeom and a.nmesh == b.nmesh
            and names(a) == names(b) and np.array_equal(a.geom_type, b.geom_type)
            and all(np.allclose(getattr(a, k), getattr(b, k))
                    for k in ("geom_size", "geom_pos", "geom_quat", "body_pos", "body_quat", "mesh_vert")))


def saved_model(run):
    """The compiled scene HIM saved in the run's trace, or None once stripped (or if the
    trace can't be read: HIM's sim stopped while writing it, say; then it is left as it is)."""
    try:
        trace = np.load(run / "trace.npz")
    except (OSError, ValueError, zipfile.BadZipFile) as e:
        print(f"{run.name}: can't read trace.npz ({e}); leaving it as it is", flush=True)
        return None
    with trace:
        if "model" not in trace.files:
            return None
        return mujoco.MjModel.from_binary_path("scene.mjb", {"scene.mjb": trace["model"].tobytes()})


def backfill(run):
    """For a run from before run.json kept its scene: save the scene it must have used (its
    world at its commit, or HIM's flat scene) if that rebuilds exactly the trace's model."""
    path = run / "run.json"
    meta = json.loads(path.read_text()) if path.is_file() else None
    if meta and meta.get("sim"):
        return
    saved = saved_model(run)
    if saved is None:
        return
    him = him_git("rev-parse", "HEAD", text=True).stdout.strip()
    with tempfile.TemporaryDirectory() as tmp:
        tmp = Path(tmp)
        if meta and meta.get("world"):
            rev = meta.get("commit") or "HEAD"    # runs from before run.json had it: try today's
            world = subprocess.run(["git", "-C", str(REPO), "show",
                                    f"{rev}:submission/worlds/{meta['world']}.xml"], capture_output=True)
            if world.returncode:
                return
            (tmp / "world.xml").write_bytes(world.stdout)
            scene = stage_world.main(meta["world"], MODEL, tmp / "staged", tmp / "world.xml")
            files = scene_files(scene)
        else:
            files = [MODEL / "scene.xml"]
        (tmp / "scene").mkdir()
        for f in files:
            shutil.copy(f, tmp / "scene" / f.name)
        try:
            ok = same_model(saved, build_model(tmp / "scene", files[0].name, him))
        except (ValueError, OSError):
            ok = False
        if not ok:
            return
        shutil.copytree(tmp / "scene", run / "scene", dirs_exist_ok=True)
    # A plain sim.sh run from before then: when it ran, but not from which commit.
    meta = meta or {"world": None, "tag": None, "started": run.name[:20], "commit": None, "dirty": None}
    meta["sim"] = {"args": None, "image": None, "image_id": None, "task_command": None,
                   "scene": files[0].name, "him_commit": him, "him_g1_dirty": False,
                   "backfilled": True}     # rebuilt and checked against the trace, afterwards
    path.write_text(json.dumps(meta) + "\n")


def strip(run):
    """Drop STRIPPED from the run's trace once a rebuilt scene matches the saved one."""
    saved = saved_model(run)
    if saved is None:
        return True
    try:
        if not same_model(saved, rebuild_model(run)):
            print(f"{run.name}: rebuilt scene differs from the trace's; keeping it whole", flush=True)
            return False
    except (ValueError, OSError):
        return False
    with np.load(run / "trace.npz") as trace:
        kept = {k: trace[k] for k in trace.files if k not in STRIPPED}
    part = run / "trace.part.npz"
    np.savez_compressed(part, **kept)
    os.replace(part, run / "trace.npz")
    return True


def run_folders():
    """Every run folder in runs/ with a trace, oldest first."""
    return sorted(p.parent for p in RUNS.glob("*/trace.npz"))


def slim(runs):
    """Strip every trace but the newest KEEP_FULL of each world (plain sim.sh runs count as
    one), filling in old runs' scenes first."""
    groups = {}
    for run in runs:
        backfill(run)
        path = run / "run.json"
        meta = json.loads(path.read_text()) if path.is_file() else {}
        groups.setdefault(meta.get("world"), []).append(run)
    for group in groups.values():
        for run in sorted(group)[:-KEEP_FULL]:
            strip(run)


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


class MemoryWatch(threading.Thread):
    """host_memory() each second during a run; finish() sums it up for run.json, with each
    second's sample ("seconds": wall time, free MB and, on macOS, MB decompressed since the
    sample before), so --events can line late states up with the host's memory."""

    def __init__(self):
        super().__init__(daemon=True)
        self.samples = [self.sample()]
        self.done = threading.Event()

    def run(self):
        while not self.done.wait(1.0):
            self.samples.append(self.sample())

    def sample(self):
        found = host_memory()
        return found and dict(found, wall=round(time.time(), 2))

    def finish(self):
        self.done.set()
        self.join()
        samples = [s for s in self.samples if s]
        if not samples:
            return None
        result = {"before": samples[0], "min_free_mb": min(s["free_mb"] for s in samples)}
        if "compressed_mb" in samples[0]:
            result["max_compressed_mb"] = max(s["compressed_mb"] for s in samples)
        for key in ("decompressed_mb", "swapped_in_mb"):
            if key in samples[0]:
                result[key] = samples[-1][key] - samples[0][key]
                result[f"most_{key}_in_1s"] = max(
                    (b[key] - a[key] for a, b in zip(samples, samples[1:])), default=0)
        change = "decompressed_mb" if "decompressed_mb" in samples[0] else "swapped_in_mb"
        result["seconds"] = [[b["wall"], b["free_mb"], b[change] - a[change]]
                             for a, b in zip(samples, samples[1:])]
        return result


def watch_live(process, run):
    """Print HIM's live viewer's URL once it is up (sim.sh starts the run at once), and close
    it (with sim.sh's Ctrl-C) VIEW_LINGER s after HIM's report."""
    log_path = run / "sim.log"

    def line(prefix):
        while process.poll() is None:
            found = next((x for x in log_path.read_text(errors="replace").splitlines()
                          if x.startswith(prefix)), None)
            if found:
                return found
            time.sleep(0.5)
        return None

    url = line("Open http")
    if not url:
        return
    # The page fetches frames only once its own Start is pressed, from the first frame.
    print(f"{run.name}: live at {url.split()[-1]} (running; press Start there to watch it "
          "from the beginning)", flush=True)
    if line("Viewer is still at"):
        # Start pressed on a finished run's page deletes HIM's report: put it back.
        report = run / "report.json"
        kept = report.read_bytes() if report.is_file() else None
        time.sleep(VIEW_LINGER)
        stop(process, run.name, wait=120)   # sim.sh prunes the run once the viewer closes
        if kept and not report.exists():
            report.write_bytes(kept)


def course_task(existing, route=None, avoidance=None, crowd=None):
    """Preserve the task's independent options when adding a route, avoidance mode or crowd."""
    from harp.avoidance_config import select
    task = json.loads(existing or "{}")
    if route is not None or avoidance is not None or crowd is not None:
        if not isinstance(task, dict):
            task = {}
        if route is not None:
            # An explicit route replaces a prior planned move, as it did before
            # avoidance options existed. Otherwise entrypoint would skip Nav2.
            task.pop("to", None)
            task.pop("moves", None)
            task["route"] = route.data()
        if avoidance is not None:
            task["avoidance"] = avoidance
        if crowd is not None:
            task["crowd"] = crowd
    text = json.dumps(task)
    select(text)
    return text


def run_once(name, tag, route=None, view=False, avoidance=None, crowd=None):
    criteria = world_criteria(name) or {}
    max_time = route_time(route) if route else criteria.get("max_time_s", 60)
    stamp = datetime.datetime.now(datetime.timezone.utc).strftime("%Y-%m-%d_%H-%M-%SZ")
    run = RUNS / "_".join(filter(None, (stamp, "sim", name, tag)))
    run.mkdir(parents=True)
    env = dict(os.environ)
    if route:
        (run / "route.yaml").write_text(yaml.safe_dump(route.data(), sort_keys=False))
    existing = env.get("G1_TASK_COMMAND")
    if not route and "task" in criteria:
        existing = json.dumps(criteria["task"])   # e.g. harp/behavior.py's loop
    env["G1_TASK_COMMAND"] = course_task(existing, route, avoidance, crowd)
    (run / "run.json").write_text(json.dumps(dict(new_run_json(run, stamp), world=name,
                                                  tag=tag)) + "\n")
    for note in memory_warnings():
        print(f"{run.name}: warning: {note}", flush=True)
    command = [str(HERE / "sim.sh"), "--world", name, "--seconds", str(int(max_time + STARTUP)),
               "--output", str(run)] + ([] if view else ["--no-view"])
    memory = MemoryWatch()
    memory.start()
    with open(run / "sim.log", "w") as log:
        # Its own session with the viewer, so stop() can Ctrl-C HIM's viewer without us.
        process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, env=env,
                                   start_new_session=view)
        try:
            if view:
                watch_live(process, run)
            process.wait()
        finally:
            if view:
                stop(process, run.name, wait=120)
    host = memory.finish()
    if host:
        info = json.loads((run / "run.json").read_text())
        (run / "run.json").write_text(json.dumps(dict(info, host_memory=host)) + "\n")
        print(f"{run.name}: host memory: " + ", ".join(f"{k} {v}" for k, v in host.items()
                                                      if k not in ("before", "seconds")), flush=True)
    if not (run / "report.json").is_file():
        print(f"{run.name}: no report; see {run / 'sim.log'}")
        return None
    return run


def replay_movers(run, qpos, times):
    """(model, qpos) with the run's movers walking, for HIM's replay viewer, or None for a
    world without movers. HIM's sim never had them and its viewer draws joint positions, so
    the model is the world's NAME_replay.xml (each mover a free body after the G1's joints)
    and each pose gets where every mover was at that sim time, as scoring places them."""
    name = json.loads((run / "run.json").read_text()).get("world")
    if not name:
        return None
    stage_world.main(name, MODEL, SCENES)
    try:
        events, _ = read_log(run)
    except FileNotFoundError:
        events = []
    started = {e["mover"]: e["starts_at"] for e in events if e.get("event") == "mover_start"}
    movers_model = mujoco.MjModel.from_xml_path(str(SCENES / (name + "_all.xml")))
    movers = Movers(movers_model, started=started, path=logged_path(events))
    if not movers.mocap:
        return None
    model = mujoco.MjModel.from_xml_path(str(SCENES / (name + "_replay.xml")))
    starts = [int(model.jnt_qposadr[model.joint(n + "_replay").id]) for n in movers.names]
    if model.nq != qpos.shape[1] + 7 * len(starts) or min(starts) < qpos.shape[1]:
        raise ValueError(f"{run.name}: the replay scene's joints don't extend the trace's")
    data = mujoco.MjData(movers_model)
    poses = np.zeros((len(qpos), model.nq))
    poses[:, :qpos.shape[1]] = qpos
    for row, t in zip(poses, times):
        movers.apply(data, t)
        for start, mocap in zip(starts, movers.mocap):
            row[start:start + 3] = data.mocap_pos[mocap]
            row[start + 3] = 1.0   # upright
    return model, poses


def replay(run, seconds=None):
    """Serve HIM's replay of a run, with its movers walking and its scene put back if the
    trace was stripped, from a copy of the trace (HIM writes view/ beside it): for seconds,
    or until Ctrl-C."""
    with tempfile.TemporaryDirectory() as tmp:
        tmp = Path(tmp)
        with np.load(run / "trace.npz") as trace:
            parts = {k: trace[k] for k in trace.files}
        people = replay_movers(run, parts["qpos"], parts["time"])
        model = None
        if people:
            model, parts["qpos"] = people
        elif "model" not in parts:
            model = rebuild_model(run)
        if model is not None:
            parts["model"] = np.empty(mujoco.mj_sizeModel(model), dtype=np.uint8)
            mujoco.mj_saveModel(model, None, parts["model"])
        np.savez(tmp / "trace.npz", **parts)
        shutil.copy(run / "report.json", tmp / "report.json")
        log_path = tmp / "replay.log"
        with open(log_path, "w") as log:
            server = subprocess.Popen([str(HIM / "desktop_sim.sh"), "replay", str(tmp / "trace.npz")],
                                      stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            while server.poll() is None and "Open http" not in log_path.read_text(errors="replace"):
                time.sleep(0.5)
            url = next((line.split()[-1] for line in log_path.read_text().splitlines()
                        if line.startswith("Open http")), None)
            if url and people:
                overview_camera(tmp / "view" / "index.html", people[0], run.name)
            if not url:
                print(f"{run.name}: no replay:\n{log_path.read_text()}", flush=True)
            elif seconds is None:
                print(f"{run.name}: replay at {url}; Ctrl-C to stop", flush=True)
                try:
                    server.wait()
                except KeyboardInterrupt:
                    pass
            else:
                print(f"{run.name}: replay at {url} for {seconds:.0f} s", flush=True)
                time.sleep(seconds)
            stop(server, run.name)


VIEWER_CAMERA = ("camera.position.set(2.2, -2.2, 1.4);", "controls.target.set(0, 0, 0.8);")


def overview_camera(page, model, name):
    """Start HIM's replay page (our temporary copy, never HIM's files) looking over the
    whole world, its goal included, instead of from beside the start: from there walkers
    near the start filled the view and the robot walked out of sight."""
    fixed = [g for g in range(model.ngeom) if model.geom_bodyid[g] == 0
             and model.geom_type[g] != mujoco.mjtGeom.mjGEOM_PLANE]
    if not fixed or not page.is_file():
        return
    reach = np.abs(model.geom_size[fixed, :2]).max(axis=1, keepdims=True)
    low = (model.geom_pos[fixed, :2] - reach).min(axis=0)
    high = (model.geom_pos[fixed, :2] + reach).max(axis=0)
    (cx, cy), size = (low + high) / 2, float(max(high - low))
    text = page.read_text()
    if not all(line in text for line in VIEWER_CAMERA):
        print(f"{name}: HIM's replay page changed; kept its own camera", flush=True)
        return
    text = text.replace(VIEWER_CAMERA[0], f"camera.position.set({cx:.2f}, {cy - .9 * size:.2f}, {.9 * size:.2f});")
    page.write_text(text.replace(VIEWER_CAMERA[1], f"controls.target.set({cx:.2f}, {cy:.2f}, 0);"))


def stop(process, name, wait=15):
    """Ctrl-C a process group, then SIGTERM, then SIGKILL, until it exits (wait s each)."""
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGKILL):
        if process.poll() is not None:
            return
        os.killpg(process.pid, sig)
        try:
            process.wait(timeout=wait)
        except subprocess.TimeoutExpired:
            print(f"{name}: the viewer didn't close on {sig.name}", flush=True)


def positive(text):
    if not text.isdigit() or int(text) == 0:
        raise argparse.ArgumentTypeError("must be a positive whole number")
    return int(text)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("world", nargs="?", help="a world in submission/worlds")
    parser.add_argument("--tag", help="added to the run folder's name")
    parser.add_argument("--repeat", type=positive, default=1)
    parser.add_argument("--avoidance", choices=("reactive", "predictive"),
                        help="Nav2 command filter (default: existing reactive stop)")
    parser.add_argument("--crowd", choices=CROWDS,
                        help="the world's movers: scripted routes (default), or aware: they "
                             "give way to the robot")
    where = parser.add_mutually_exclusive_group()
    where.add_argument("--route", metavar="NAME_OR_FILE", help="a route in submission/routes, or a file")
    where.add_argument("--waypoint", action="append", nargs="+", type=float, metavar="NUM",
                       help="X Y [HEADING] in odom; one per point, in order")
    parser.add_argument("--laps", type=positive, help="laps of the route (default: its file's, or 1)")
    parser.add_argument("--view", action="store_true",
                        help="watch each run in HIM's live viewer (closed before the next)")
    parser.add_argument("--score", metavar="RUN", help="only (re)score this run folder")
    parser.add_argument("--compare", nargs="?", const="", metavar="WORLD",
                        help="only list the scored runs in runs/ by world and tag")
    parser.add_argument("--prune", nargs="?", const="", metavar="RUN",
                        help="only prune this run folder, or every one in runs/: delete HIM's viewer "
                             "output, and strip all but the newest traces of each world")
    parser.add_argument("--replay", metavar="RUN", help="only replay this run folder, until Ctrl-C")
    parser.add_argument("--events", metavar="RUN", help="only print this run folder's events")
    parser.add_argument("--record", metavar="RUN", help=argparse.SUPPRESS)   # for sim.sh
    parser.add_argument("--scene", default="", help=argparse.SUPPRESS)       # with --record
    parser.add_argument("--image", default="", help=argparse.SUPPRESS)       # with --record
    parser.add_argument("--sim-args", nargs=argparse.REMAINDER, default=[], help=argparse.SUPPRESS)
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
    if args.record:
        record(args.record, args.scene or None, args.image, args.sim_args)
        return 0
    if args.replay:
        replay(Path(args.replay))
        return 0
    if args.events:
        show_events(Path(args.events))
        return 0
    if args.prune is not None:
        with checkout.Hold("course.py --prune"):   # slim() strips traces in the shared runs/
            runs = [Path(args.prune)] if args.prune else run_folders()
            for run in runs:
                prune(run)
            slim(run_folders())
        return 0
    runs = [Path(args.score)] if args.score else range(args.repeat)
    failed = False
    for run in runs:
        with checkout.Hold(f"course.py {args.world or args.score}"):
            failed |= not run_and_score(run, args, route)
    return 1 if failed else 0


def run_and_score(run, args, route):
    """Run (unless --score) and score one run, print its line, and prune; False if it
    didn't succeed or failed its criteria."""
    run = run if args.score else run_once(args.world, args.tag, route, args.view, args.avoidance,
                                          args.crowd)
    if run is None:
        return False
    try:
        np.load(run / "trace.npz").close()
    except (OSError, ValueError, zipfile.BadZipFile) as e:
        # HIM writes the trace last, under its report deadline: a Mac short of memory or a
        # stopped plant can leave it cut off ([[sim-deadline-stops]]), with nothing to score.
        print(f"{run.name}: not scored, HIM's trace.npz can't be read ({e})", flush=True)
        return False
    s = score(run)
    verdict = {True: "PASS", False: "FAIL " + ",".join(s["failed_checks"]), None: "-"}[s["passed"]]
    print(f"{run.name}: {s['outcome']} ({s['him_status']}) {verdict}  "
          f"{s['seconds']} s, path {s['path_m']} m, end {s['end_error_m']} m, "
          f"closest {s['closest_m']} m, contacts {s['contacts']}, "
          f"recoveries {s['recoveries']}, drift {s['drift_m']} m"
          + (f", speed near people {s['speed_near_people_mps']} m/s"
             if s["speed_near_people_mps"] is not None else "")
          + (f", waypoints {s['route']['completed_waypoints']}/{s['route']['waypoints']}"
             if "route" in s else "")
          + (f", talks {len(s['loop']['talks'])} ("
             + ", ".join(f"{t['person']} {t['distance_m']} m {t['facing_deg']} deg {t['ended']}"
                         for t in s["loop"]["talks"])
             + f"), not people {s['loop'].get('not_people') or 'none'}"
             + f", wander goals {s['loop']['wander_goals']}, farthest {s['loop']['farthest_m']} m"
             if "loop" in s else ""))
    prune(run)
    slim(run_folders())
    return s["outcome"] == "succeeded" and s["passed"] is not False


if __name__ == "__main__":
    sys.exit(main())
