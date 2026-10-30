"""Run a course test in HIM's simulator and score it:
  python3 submission/course.py one_box
  python3 submission/course.py slalom --tag rpp --repeat 3 --view
  python3 submission/course.py one_box --route rectangle --laps 3
  python3 submission/course.py open_floor --waypoint 2 0 --waypoint 2 2 90 --waypoint 0 0
  python3 submission/course.py --score runs/2026-10-28_05-00-00Z_sim_one_box
  python3 submission/course.py --prune
  python3 submission/course.py --replay runs/2026-10-28_05-00-00Z_sim_one_box
  python3 submission/course.py --compare slalom

Each run gets a folder in runs/ (HARP_RUNS_DIR to change it) holding our run.json
(world, tag, start, git commit, whether the tree was dirty; git.diff then holds the
uncommitted changes; and from sim.sh, "sim": its arguments, image, G1_TASK_COMMAND and
HIM's commit), the scene HIM loaded without HIM's G1 (scene/), HIM's
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
--view watches each run in HIM's live viewer, at the URL it prints: press Start there
(after START_WAIT s it starts the run itself, unseen: the page shows only runs started
from it), and it closes the viewer VIEW_LINGER s after HIM's report, before the next run. HIM's
deadlines stop runs when the host is short of memory, and a viewer holds about 700 MB, so
before each run it warns about HIM viewers still up and macOS memory pressure, and run.json
keeps the host's memory during the run (host_memory: free, compressed and decompressed MB).
Pruning, after every run (sim.sh's with --output too) or with --prune [RUN]: HIM's viewer
output (view/, tens of MB) is deleted, and every trace.npz but the newest KEEP_FULL of each
world loses STRIPPED (about 52 of its 53 MB), once the scene rebuilt from run.json and
scene/ (HIM's G1 read at HIM's commit) matches the one in the trace. Runs from before
run.json kept a scene get theirs filled in the same way. --replay RUN serves HIM's replay
with the scene put back, from a copy of the trace; a stripped run's thrash panel is empty.

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
from pathlib import Path

HERE = Path(__file__).resolve().parent
REPO = HERE.parent
HIM = Path(os.environ.get("HIM_REPO", REPO.parent / "him-sim2real-management"))
VENV_PYTHON = HIM / ".venv-policy-safety/bin/python"
RUNS = Path(os.environ.get("HARP_RUNS_DIR", REPO / "runs"))
STARTUP = 12.0          # s added to the world's max_time_s: Nav2 sends the goal about 6 s in, plus slack
CLEARANCE_MAX = 2.0     # m: farther than this counts as this
CLEARANCE_EVERY = 5     # trace frames between clearance checks (0.1 s)
VIEW_LINGER = 10.0      # s the --view live viewer stays up after HIM's report
START_WAIT = 120.0      # s --view waits for Start to be pressed before starting the run unseen
VIEWER_OUTPUT = ("view", "live.jsonl")   # HIM's viewer files, rebuilt from trace.npz by replay
KEEP_FULL = 2           # newest runs of each world whose trace.npz keeps everything (tags are
                        # mostly one-off experiments: per world and tag kept 60 of 69 whole)
STRIPPED = ("model", "thrash")   # the rest lose these: the compiled scene (about 47 of a trace's
                                 # 53 MB; rebuilt from run.json) and HIM's thrash telemetry (5 MB)
G1 = "packages/policy_safety/src/policy_safety/robots/g1"   # HIM's G1 model, in its checkout
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


def him_git(*args, **kwargs):
    """Read-only git in HIM's checkout."""
    return subprocess.run(["git", "-C", str(HIM), *args], capture_output=True, **kwargs)


def stamp_now():
    return datetime.datetime.now(datetime.timezone.utc).strftime("%Y-%m-%d_%H-%M-%SZ")


def new_run_json(run, started=None):
    """run.json's common part, and git.diff when the tree has uncommitted changes."""
    commit, diff = git_state()
    if diff:
        (run / "git.diff").write_text(diff)
    return {"world": None, "tag": None, "started": started or stamp_now(), "commit": commit,
            "dirty": bool(diff)}


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
    """The compiled scene HIM saved in the run's trace, or None once stripped."""
    with np.load(run / "trace.npz") as trace:
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


def host_memory():
    """The host's memory now in MB, or None: free, compressed and decompressed so far on
    macOS (vm_stat), available and swapped in so far on Linux (/proc)."""
    try:
        if sys.platform == "darwin":
            text = subprocess.run(["vm_stat"], capture_output=True, text=True, timeout=5).stdout
            mb = int(re.search(r"page size of (\d+)", text).group(1)) / 2**20
            pages = {k.strip('"'): int(v.strip(" .")) for k, v in
                     (line.split(":") for line in text.splitlines()[1:] if ":" in line)}
            return {"free_mb": round((pages["Pages free"] + pages["Pages speculative"]) * mb),
                    "compressed_mb": round(pages["Pages occupied by compressor"] * mb),
                    "decompressed_mb": round(pages["Decompressions"] * mb)}
        if sys.platform.startswith("linux"):
            info = dict(line.split(":") for line in Path("/proc/meminfo").read_text().splitlines())
            vmstat = dict(line.split() for line in Path("/proc/vmstat").read_text().splitlines())
            return {"free_mb": round(int(info["MemAvailable"].split()[0]) / 1024),
                    "swapped_in_mb": round(int(vmstat["pswpin"]) * os.sysconf("SC_PAGE_SIZE") / 2**20)}
    except (OSError, ValueError, KeyError, AttributeError, subprocess.SubprocessError):
        pass
    return None


class MemoryWatch(threading.Thread):
    """host_memory() each second during a run; finish() sums it up for run.json."""

    def __init__(self):
        super().__init__(daemon=True)
        self.samples = [host_memory()]
        self.done = threading.Event()

    def run(self):
        while not self.done.wait(1.0):
            self.samples.append(host_memory())

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
        return result


def memory_warnings():
    """What to warn about before a run: HIM viewers still up, macOS short of memory."""
    notes = []
    try:
        viewers = subprocess.run(["pgrep", "-f", "containers/sim/live.py"],
                                 capture_output=True, text=True).stdout.split()
    except OSError:
        viewers = []
    if viewers:
        notes.append(f"{len(viewers)} HIM viewer(s) still up (pid {', '.join(viewers)}), about "
                     "700 MB each: Ctrl-C them where they run")
    if sys.platform == "darwin":
        level = subprocess.run(["sysctl", "-n", "kern.memorystatus_vm_pressure_level"],
                               capture_output=True, text=True).stdout.strip()
        if level in ("2", "4"):
            notes.append(f"macOS reports {'warning' if level == '2' else 'critical'} memory "
                         "pressure: close apps first")
    return notes


def watch_live(process, run):
    """Print HIM's live viewer's URL once it is up, wait for its Start (pressing it after
    START_WAIT s), and close it (with sim.sh's Ctrl-C) VIEW_LINGER s after HIM's report."""
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
    url = url.split()[-1]
    # The page shows the run only once its own Start is pressed (it then fetches frames),
    # so wait for that, and start the run unseen after START_WAIT s.
    print(f"{run.name}: live at {url}: press Start there (unseen after {START_WAIT:.0f} s)",
          flush=True)
    started = run / "view" / "start"   # sim.sh's flag, written by the viewer's /start
    deadline = time.monotonic() + START_WAIT
    while not started.exists() and process.poll() is None:
        if time.monotonic() > deadline:
            try:
                urllib.request.urlopen(urllib.request.Request(url + "/start", method="POST"),
                                       timeout=5)
                print(f"{run.name}: started unseen", flush=True)
            except OSError as e:
                print(f"{run.name}: couldn't start it ({e})", flush=True)
                deadline = time.monotonic() + 5
                continue
            break
        time.sleep(0.5)
    if line("Viewer is still at"):
        # Start pressed on a finished run's page deletes HIM's report: put it back.
        report = run / "report.json"
        kept = report.read_bytes() if report.is_file() else None
        time.sleep(VIEW_LINGER)
        stop(process, run.name, wait=120)   # sim.sh prunes the run once the viewer closes
        if kept and not report.exists():
            report.write_bytes(kept)


def run_once(name, tag, route=None, view=False):
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
                                                      if k != "before"), flush=True)
    if not (run / "report.json").is_file():
        print(f"{run.name}: no report; see {run / 'sim.log'}")
        return None
    return run


def replay(run, seconds=None):
    """Serve HIM's replay of a run, with its scene put back if the trace was stripped, from
    a copy of the trace (HIM writes view/ beside it): for seconds, or until Ctrl-C."""
    with tempfile.TemporaryDirectory() as tmp:
        tmp = Path(tmp)
        with np.load(run / "trace.npz") as trace:
            parts = {k: trace[k] for k in trace.files}
        if "model" not in parts:
            model = rebuild_model(run)
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
    if args.prune is not None:
        runs = [Path(args.prune)] if args.prune else run_folders()
        for run in runs:
            prune(run)
        slim(run_folders())
        return 0
    runs = [Path(args.score)] if args.score else range(args.repeat)
    failed = False
    for run in runs:
        run = run if args.score else run_once(args.world, args.tag, route, args.view)
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
        prune(run)
        slim(run_folders())
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
