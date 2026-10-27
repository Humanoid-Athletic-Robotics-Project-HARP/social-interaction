"""What a run folder adds up to (summary.json), the list of all runs
(runs/index.csv), pass/fail against each course's criteria, and how long run
data is kept. Used by run_recorder, course and the runs command. Each metric
is a small function of the run folder, so a new one is added here and
`runs resummarize --all` fills it in for old runs.
"""

import csv
import json
import os
import shutil
from pathlib import Path

import numpy as np
import yaml

from harp_control import runlog

SUMMARY_VERSION = 2
FOREVER = {"run.yaml", "events.csv", "summary.json"}   # never pruned
NEAR = 0.3                # m: "time within NEAR of an obstacle"
CLEARANCE_MAX = 2.0       # m: clearances beyond this aren't computed
CLEARANCE_PERIOD = 0.1    # s between poses checked for clearance
SETTLE = 3.0              # s after the end event still counted (the robot coming to rest)
STOPPED = 0.01            # m/s and rad/s: a /cmd_vel below this is "stopped"
MOVING_V, MOVING_W = 0.05, 0.1   # m/s, rad/s: commands above these count for speed tracking
RESPONSE_LAG = 0.3        # s: achieved speed is compared this long after the command
LIDAR_GAP = 0.3           # s between scans that counts as a gap (3 periods at 10 Hz)
INDEX_FIELDS = ["run", "started", "kind", "world", "tag", "outcome", "passed", "duration_s",
                "path_m", "end_error_m", "closest_m", "contacts", "recoveries", "pinned", "note"]


def read_meta(run):
    return yaml.safe_load((Path(run) / "run.yaml").read_text())


def write_meta(run, meta):
    (Path(run) / "run.yaml").write_text(yaml.safe_dump(meta, sort_keys=False))


def merge_events(run):
    """Fold events-<program>.csv (from programs without ROS) into events.csv, in time order."""
    run = Path(run)
    rows = []
    for path in [run / "events.csv", *sorted(run.glob("events-*.csv"))]:
        if path.exists():
            with open(path, newline="") as f:
                rows += list(csv.DictReader(f))
    rows.sort(key=lambda r: float(r["t"]))
    with open(run / "events.csv", "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=runlog.EVENT_FIELDS)
        writer.writeheader()
        writer.writerows(rows)
    for path in run.glob("events-*.csv"):
        path.unlink()
    return [{**r, "t": float(r["t"]), "details": json.loads(r["details"] or "{}")} for r in rows]


def first(events, source, name):
    return next((e for e in events if e["source"] == source and e["event"] == name), None)


def outcome(events):
    """succeeded / failed / timed_out / fell / error / stopped, and why."""
    goal = first(events, "goal", "goal_result")
    fall = first(events, "walker", "fall")
    if fall and (goal is None or fall["t"] < goal["t"]):
        return "fell", "the walker detected a fall"
    if goal:
        status = goal["details"].get("status", "")
        return {"SUCCEEDED": "succeeded", "TIMEOUT": "timed_out",
                "NAV2_NOT_ACTIVE": "error"}.get(status, "failed"), status
    if first(events, "move_distance", "all_moves_done"):
        return "succeeded", "all moves done"
    error = first(events, "move_distance", "error") or first(events, "walker", "error")
    if error:
        return "error", error["details"].get("message")
    return "stopped", "ended before a goal or moves finished"


def window(events):
    """Wall-clock start and end of the part that counts: goal sent to result, or the moves."""
    start = (first(events, "goal", "goal_sent") or first(events, "move_distance", "move_start")
             or first(events, "walker", "policy_on"))
    end = (first(events, "goal", "goal_result") or first(events, "move_distance", "all_moves_done")
           or first(events, "walker", "fall") or first(events, "move_distance", "error"))
    return (start["t"] if start else None), (end["t"] if end else None)


def load_truth(run):
    path = Path(run) / "truth.npz"
    if not path.exists():
        return None
    with np.load(path) as f:
        return {k: f[k] for k in f.files}


def read_csv(run, name):
    path = Path(run) / name
    if not path.exists():
        return None
    data = np.genfromtxt(path, delimiter=",", names=True)
    return data if data.size > 1 else None


def read_bag(path, topics):
    """(topic, wall time s, message) from a run's bag, for the given topics."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    if not Path(path).exists():
        return []
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(path), storage_id="mcap"),
                rosbag2_py.ConverterOptions("cdr", "cdr"))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    out = []
    while reader.has_next():
        topic, data, stamp = reader.read_next()
        if topic in topics:
            out.append((topic, stamp / 1e9, deserialize_message(data, get_message(types[topic]))))
    return out


def smoothed_xy(qpos, n=20):
    """1 s moving average of the pelvis, so each step's side-to-side sway isn't path length."""
    if len(qpos) < n:
        return qpos[:, :2].copy()
    kernel = np.ones(n) / n
    return np.stack([np.convolve(qpos[:, k], kernel, "valid") for k in (0, 1)], axis=1)


def yaw_of(qpos):
    w, x, y, z = qpos[..., 3], qpos[..., 4], qpos[..., 5], qpos[..., 6]
    return np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


# --- metrics: each takes what it needs and returns a dict for summary.json ---

def nav_tree(run):
    """From the behaviour tree log (standard bag): recoveries by type, replans, failures."""
    log = read_bag(Path(run) / "standard_bag", {"/behavior_tree_log"})
    if not log:
        return {}
    counts = {"replans": 0, "planner_failures": 0, "controller_failures": 0}
    recoveries = {}
    for _, _, msg in log:
        for change in msg.event_log:
            name, now = change.node_name, change.current_status
            if name == "ComputePathToPose" and now == "SUCCESS":
                counts["replans"] += 1
            elif name == "ComputePathToPose" and now == "FAILURE":
                counts["planner_failures"] += 1
            elif name == "FollowPath" and now == "FAILURE":
                counts["controller_failures"] += 1
            elif name in ("Wait", "Spin", "BackUp", "ClearEntireCostmap") and now == "RUNNING" \
                    and change.previous_status != "RUNNING":
                recoveries[name] = recoveries.get(name, 0) + 1
    return {"recoveries": sum(recoveries.values()), "recovery_types": recoveries, **counts,
            "nav_source": "behavior_tree_log"}


def alignment(truth, events, odom):
    """The odom frame in the world: odom starts where the policy started, so the true
    pelvis pose at policy_on, less odom's own yaw then, maps odom into the world."""
    on = first(events, "walker", "policy_on")
    if truth is None or on is None or odom is None:
        return None
    i = min(int(np.searchsorted(truth["t"], on["t"])), len(truth["t"]) - 1)
    origin = truth["qpos"][i, :2]
    odom_yaw0 = float(np.interp(on["t"], odom["t"], odom["yaw"]))
    dyaw = float(yaw_of(truth["qpos"][i])) - odom_yaw0
    c, s = np.cos(dyaw), np.sin(dyaw)

    def to_world(x, y):
        x, y = np.asarray(x, float), np.asarray(y, float)
        return origin[0] + c * x - s * y, origin[1] + s * x + c * y
    return {"to_world": to_world, "origin": origin, "dyaw": dyaw,
            "text": f"odom mapped to world by the true pelvis pose at policy_on "
                    f"(origin {origin[0]:+.3f}, {origin[1]:+.3f} m, rotation {np.degrees(dyaw):+.2f} deg)"}


def odometry(truth, odom, align, end_t):
    """Odometry against the truth, both in the world frame."""
    keep = odom["t"] <= end_t
    if not keep.any():
        return {}
    ox, oy = align["to_world"](odom["x"][keep], odom["y"][keep])
    tx = np.interp(odom["t"][keep], truth["t"], truth["qpos"][:, 0])
    ty = np.interp(odom["t"][keep], truth["t"], truth["qpos"][:, 1])
    err = np.hypot(ox - tx, oy - ty)
    return {"odom_end_world": [round(float(ox[-1]), 3), round(float(oy[-1]), 3)],
            "odom_error_end_m": round(float(err[-1]), 3), "odom_error_max_m": round(float(err.max()), 3)}


def ratio_stats(achieved, commanded):
    r = achieved / commanded
    r = r[np.isfinite(r)]
    if len(r) < 5:
        return None
    return {"mean": round(float(np.mean(r)), 3), "sd": round(float(np.std(r)), 3), "n": int(len(r))}


def speed_tracking(cmd, odom, truth, t0, t1):
    """How closely achieved speed follows the command, forward and turning, while moving.
    Achieved = /odom's body velocity (sim and robot) and the true pose (sim)."""
    if cmd is None or t0 is None:
        return {}
    inside = (cmd["t"] >= t0) & (cmd["t"] <= (t1 or cmd["t"][-1]))
    t = cmd["t"][inside] + RESPONSE_LAG
    vx, wz = cmd["vx"][inside], cmd["wz"][inside]
    fwd, turn = np.abs(vx) > MOVING_V, np.abs(wz) > MOVING_W
    out = {}
    if odom is not None and "vx" in odom.dtype.names:
        ovx, owz = np.interp(t, odom["t"], odom["vx"]), np.interp(t, odom["t"], odom["wz"])
        out["speed_forward_ratio_odom"] = ratio_stats(ovx[fwd], vx[fwd])
        out["speed_turn_ratio_odom"] = ratio_stats(owz[turn], wz[turn])
    if truth is not None and len(truth["t"]) > 20:
        n = 10                                      # 0.5 s at 20 Hz
        k = np.ones(n) / n
        tt = np.convolve(truth["t"], k, "valid")
        x, y = (np.convolve(truth["qpos"][:, j], k, "valid") for j in (0, 1))
        yaw = np.unwrap(yaw_of(truth["qpos"]))
        yaw = np.convolve(yaw, k, "valid")
        dt = np.gradient(tt)
        v = (np.gradient(x) * np.cos(yaw) + np.gradient(y) * np.sin(yaw)) / dt
        w = np.gradient(yaw) / dt
        tv, tw = np.interp(t, tt, v), np.interp(t, tt, w)
        out["speed_forward_ratio_true"] = ratio_stats(tv[fwd], vx[fwd])
        out["speed_turn_ratio_true"] = ratio_stats(tw[turn], wz[turn])
    out["speed_lag_s"] = RESPONSE_LAG
    return out


def lidar_quality(stats):
    """From lidar_stats.csv: points per scan, scan rate, gaps."""
    if stats is None or len(stats) < 2:
        return {}
    dt = np.diff(stats["t"])
    return {"lidar_points_mean": round(float(np.mean(stats["points"])), 0),
            "lidar_points_min": int(np.min(stats["points"])),
            "lidar_scan_rate_hz": round(float(len(stats) / (stats["t"][-1] - stats["t"][0])), 2),
            "lidar_gaps": int((dt > LIDAR_GAP).sum()), "lidar_gap_max_s": round(float(dt.max()), 2)}


def clearances(meta, qpos, sim_t=None, events=()):
    """Closest true distance from the robot's collision geoms to each obstacle (sim only).
    Moving obstacles are placed where they were at each sim time, from the sim's
    mover_start events."""
    import mujoco
    from harp_sim.movers import Movers, obstacle_geoms
    from harp_sim.run_sim import load_model
    sim = meta.get("sim") or {}
    if not sim.get("scene"):
        return None
    boxes = [tuple(b) for b in sim.get("boxes") or []]
    model = load_model(sim["scene"], boxes, sim.get("world"))
    data = mujoco.MjData(model)
    movers = Movers(model, {e["details"]["mover"]: e["details"]["starts_at"] for e in events
                            if e["source"] == "sim" and e["event"] == "mover_start"})
    if movers.names and sim_t is None:
        return None   # can't tell where the movers were
    obstacles = obstacle_geoms(model)
    robot = [g for g in range(model.ngeom) if model.geom_bodyid[g] != 0 and model.geom_contype[g]
             and g not in obstacles]
    if not obstacles:
        return {"closest_m": None, "near_share": 0.0, "per_obstacle": {}}
    best = {g: CLEARANCE_MAX for g in obstacles}
    near = []
    fromto = np.zeros(6)
    for i, q in enumerate(qpos):
        data.qpos[:] = q
        if sim_t is not None:
            movers.apply(data, sim_t[i])
        mujoco.mj_kinematics(model, data)
        closest = CLEARANCE_MAX
        for o in obstacles:
            for r in robot:
                d = mujoco.mj_geomDistance(model, data, r, o, CLEARANCE_MAX, fromto)
                best[o] = min(best[o], d)
                closest = min(closest, d)
        near.append(closest < NEAR)
    names = {g: model.geom(g).name.removeprefix("course/") or f"geom {g}" for g in obstacles}
    per = {names[g]: round(float(d), 3) for g, d in sorted(best.items(), key=lambda kv: kv[1])
           if d < CLEARANCE_MAX}
    return {"closest_m": round(float(min(best.values())), 3),
            "near_share": round(float(np.mean(near)), 3), "per_obstacle": per}


def criteria_for(world):
    """Pass criteria for a course: harp_sim/worlds/<world>.yaml, or None."""
    if not world:
        return None, None
    from ament_index_python.packages import get_package_share_directory
    path = Path(get_package_share_directory("harp_sim")) / "worlds" / f"{world}.yaml"
    return (yaml.safe_load(path.read_text()), path.name) if path.exists() else (None, None)


def check(s):
    """Pass/fail per criterion and overall, for course runs (started by course, or a goal
    was sent). A run that ended before its goal fails: its values are missing."""
    criteria, name = criteria_for(s.get("world"))
    if not criteria or not s.get("course_run"):
        return None
    rules = {"must_succeed": (lambda v, lim: v == "succeeded" or not lim, "outcome"),
             "max_time_s": (lambda v, lim: v <= lim, "duration_s"),
             "min_clearance_m": (lambda v, lim: v >= lim, "closest_m"),
             "max_contacts": (lambda v, lim: v <= lim, "contacts"),
             "max_recoveries": (lambda v, lim: v <= lim, "recoveries"),
             "max_end_error_m": (lambda v, lim: v <= lim, "end_error_m")}
    results = {}
    for key, limit in criteria.items():
        if key not in rules or limit is None:
            continue
        test, field = rules[key]
        value = s.get(field)
        results[key] = {"limit": limit, "value": value,
                        "pass": bool(test(value, limit)) if value is not None else False}
    return {"criteria": name, "results": results,
            "failed": [k for k, r in results.items() if not r["pass"]],
            "passed": all(r["pass"] for r in results.values())}


def summarize(run):
    """Compute summary.json for one run folder, put the result in run.yaml, update the index."""
    run = Path(run)
    meta = read_meta(run)
    events = merge_events(run)
    result, reason = outcome(events)
    t0, t1 = window(events)
    s = {"summary_version": SUMMARY_VERSION, "run": run.name, "kind": meta.get("kind"),
         "world": meta.get("world"), "tag": meta.get("tag"), "started": meta.get("started"),
         # Runs recorded before course moved to harp_eval say "harp_ros course".
         "course_run": any(f"{p} course" in (meta.get("command") or "") for p in ("harp_eval", "harp_ros"))
                       or first(events, "goal", "goal_sent") is not None,
         "outcome": result, "reason": reason,
         "duration_s": round(t1 - t0, 1) if t0 and t1 else None}

    contacts = [e for e in events if e["event"] == "contact"]
    s["contacts"] = len(contacts)
    s["contact_list"] = [f'{e["details"].get("body")} touched {e["details"].get("obstacle")}'
                         for e in contacts]
    goal = first(events, "goal", "goal_result")
    s["recoveries_feedback"] = goal["details"].get("recoveries") if goal else None
    s["recoveries"] = s["recoveries_feedback"]   # replaced by the tree log's count when there is one
    s["nav_source"] = "goal feedback (no behavior_tree_log)" if goal else None
    s.update(nav_tree(run))
    lags = [e["details"].get("ms", 0) for e in events if e["event"] == "lag"]
    s["sim_lag_warnings"], s["sim_lag_max_ms"] = len(lags), max(lags, default=0)
    moves = [e["details"] for e in events if e["event"] == "move_done"]
    if moves:
        s["moves"] = moves

    truth = load_truth(run)
    # /odom from walker_node, or leg_odom.csv that move_distance writes without ROS.
    odom = read_csv(run, "odom.csv")
    if odom is None:
        odom = read_csv(run, "leg_odom.csv")
    cmd = read_csv(run, "cmd_vel.csv")
    align = alignment(truth, events, odom)
    s["frame_alignment"] = align["text"] if align else None
    sent = first(events, "goal", "goal_sent")
    if sent and align:
        gx, gy = align["to_world"](sent["details"]["x"], sent["details"]["y"])
        s["goal_world"] = [round(float(gx), 3), round(float(gy), 3)]

    if truth is not None and len(truth["t"]) > 1 and t0:
        end = (t1 or truth["t"][-1]) + SETTLE
        # Not past the walker damping or falling: then the robot sinks to the floor.
        for name in ("damped", "fall"):
            e = first(events, "walker", name)
            if e and e["t"] > t0:
                end = min(end, e["t"])
        inside = (truth["t"] >= t0) & (truth["t"] <= end)
        qpos = truth["qpos"][inside]
        if len(qpos) > 1:
            xy = smoothed_xy(qpos)
            s["path_m"] = round(float(np.linalg.norm(np.diff(xy, axis=0), axis=1).sum()), 2)
            s["end_true"] = [round(float(v), 3) for v in qpos[-1, :2]]
            if s["duration_s"]:
                s["mean_speed_mps"] = round(s["path_m"] / s["duration_s"], 3)
            if s.get("goal_world"):
                s["end_error_m"] = round(float(np.linalg.norm(qpos[-1, :2] - s["goal_world"])), 3)
            if align:
                s.update(odometry(truth, odom, align, end))
            step = max(1, int(round(CLEARANCE_PERIOD / np.median(np.diff(truth["t"])))))
            sim_t = truth["sim_t"][inside][::step] if "sim_t" in truth else None
            c = clearances(meta, qpos[::step], sim_t, events)
            if c:
                s.update(c)
    measured = (meta.get("manual") or {}).get("end_position_measured")
    if measured and odom is not None:
        # Robot runs: odom's end against the tape-measured end, both in odom.
        s["odom_error_vs_measured_m"] = round(float(np.hypot(odom["x"][-1] - measured[0],
                                                             odom["y"][-1] - measured[1])), 3)
        if s.get("end_error_m") is None and sent:
            s["end_error_m"] = round(float(np.hypot(measured[0] - sent["details"]["x"],
                                                    measured[1] - sent["details"]["y"])), 3)
    if cmd is not None and t0:
        inside = (cmd["t"] >= t0) & (cmd["t"] <= (t1 or cmd["t"][-1]))
        if inside.any():
            stopped = (np.hypot(cmd["vx"][inside], cmd["vy"][inside]) < STOPPED) & \
                      (np.abs(cmd["wz"][inside]) < STOPPED)
            s["cmd_stopped_share"] = round(float(stopped.mean()), 3)
    s.update(speed_tracking(cmd, odom, truth, t0, t1))
    s.update(lidar_quality(read_csv(run, "lidar_stats.csv")))
    s["check"] = check(s)
    s["passed"] = s["check"]["passed"] if s["check"] else None

    (run / "summary.json").write_text(json.dumps(s, indent=2))
    meta["result"] = {k: s.get(k) for k in ("outcome", "reason", "passed", "duration_s", "path_m",
                                            "end_error_m", "closest_m", "contacts", "recoveries")}
    write_meta(run, meta)
    rebuild_index(run.parent)
    return s


def one_screen(s):
    """The summary as a few lines, printed at the end of a run."""
    def get(key, fmt="{}", unit=""):
        v = s.get(key)
        return "-" if v is None else fmt.format(v) + unit
    verdict = "" if s.get("passed") is None else (" PASS" if s["passed"] else
                                                  " FAIL: " + ", ".join(s["check"]["failed"]))
    lines = [f"Run {s['run']}: {s['outcome'].upper()} ({s.get('reason')}){verdict}",
             f"  time {get('duration_s', '{:.0f}', ' s')}, path {get('path_m', '{:.2f}', ' m')}, "
             f"end error {get('end_error_m', '{:.2f}', ' m')}, recoveries {get('recoveries')} "
             f"{s.get('recovery_types') or ''}",
             f"  contacts {s['contacts']}, closest {get('closest_m', '{:.2f}', ' m')}, "
             f"odometry error at the end {get('odom_error_end_m', '{:.2f}', ' m')}"]
    fwd = s.get("speed_forward_ratio_odom")
    if fwd:
        lines.append(f"  achieved/commanded forward speed {fwd['mean']:.2f} ± {fwd['sd']:.2f} (odom)"
                     + (f", {s['speed_forward_ratio_true']['mean']:.2f} (true)"
                        if s.get("speed_forward_ratio_true") else ""))
    if s.get("lidar_points_mean") is not None:
        lines.append(f"  lidar {s['lidar_points_mean']:.0f} points/scan at {s['lidar_scan_rate_hz']} Hz, "
                     f"{s['lidar_gaps']} gaps")
    if s.get("per_obstacle"):
        nearest = list(s["per_obstacle"].items())[:3]
        lines.append("  nearest: " + ", ".join(f"{k} {v:.2f} m" for k, v in nearest))
    if s.get("contact_list"):
        lines.append("  touched: " + "; ".join(s["contact_list"][:5]))
    return "\n".join(lines)


def run_dirs(runs):
    """Every run folder, oldest first (names start with the date and time)."""
    return sorted(p for p in Path(runs).iterdir() if p.is_dir() and (p / "run.yaml").exists())


def rebuild_index(runs):
    rows = []
    for run in run_dirs(runs):
        meta = read_meta(run)
        path = run / "summary.json"
        s = json.loads(path.read_text()) if path.exists() else {}
        rows.append({"run": run.name, "started": meta.get("started"), "kind": meta.get("kind"),
                     "world": meta.get("world"), "tag": meta.get("tag"),
                     "outcome": s.get("outcome"), "passed": s.get("passed"),
                     "duration_s": s.get("duration_s"), "path_m": s.get("path_m"),
                     "end_error_m": s.get("end_error_m"), "closest_m": s.get("closest_m"),
                     "contacts": s.get("contacts"), "recoveries": s.get("recoveries"),
                     "pinned": meta.get("pinned"), "note": meta.get("note")})
    # Written whole and swapped in, so runs finishing at the same time can't interleave it.
    tmp = Path(runs) / f".index.csv.{os.getpid()}"
    with open(tmp, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=INDEX_FIELDS)
        writer.writeheader()
        writer.writerows(rows)
    os.replace(tmp, Path(runs) / "index.csv")


def size(path):
    return sum(f.stat().st_size for f in Path(path).rglob("*") if f.is_file())


def prune(runs, retention, replay=None, dry_run=False):
    """Apply the retention limits in logging.yaml; returns what was (or would be) removed.
    Replay recordings have their own pool (replay: total_cap_gb)."""
    runs = Path(runs)
    if not runs.exists():
        return []
    active = (runs / runlog.ACTIVE).read_text().split()[0] if (runs / runlog.ACTIVE).exists() else None
    newest_first = [r for r in reversed(run_dirs(runs)) if r.name != active]
    keep = [r for r in newest_first if not read_meta(r).get("pinned")]
    actions = []
    for run in keep[retention["baseline_keep_runs"]:]:
        extra = [p for p in run.iterdir() if p.name not in FOREVER]
        if extra:
            actions.append((run, "keep only run.yaml, events.csv, summary.json", extra))
    with_bags = [r for r in keep[:retention["baseline_keep_runs"]] if (r / "bag").exists()]
    total = 0.0
    for i, run in enumerate(with_bags):
        total += size(run / "bag") / 1e9
        if i >= retention["extras_keep_runs"] or total > retention["extras_max_gb"]:
            actions.append((run, "remove bag/", [run / "bag"]))
    if replay:
        total = 0.0
        for run in keep:
            path = Path(((read_meta(run).get("replay") or {}).get("path")) or run / "replay")
            if path.exists():
                total += size(path) / 1e9
                if total > replay["total_cap_gb"]:
                    actions.append((run, "remove the replay recording", [path]))
    if not dry_run:
        for _, _, paths in actions:
            for p in paths:
                if p.exists():
                    shutil.rmtree(p) if p.is_dir() else p.unlink()
    return actions
