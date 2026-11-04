"""Score wave runs (G1_TASK_COMMAND='{"wave": {}}', submission/sim.sh --output DIR) against
the wave's pass rules (context/wave.md):
  python3 submission/wave_check.py RUN_DIR [RUN_DIR ...]

One JSON line per run, its metrics and "failed" (the checks it failed); exit 1 if any run
failed. The window runs from wave_start to AFTER s past wave_done, on the sim's clock.
  status           report.json says within_limits
  thrash_mode      HIM's back-and-forth check was enforced, as on the robot
  weighted_rms     that check's peak score in the window (missing: thrash_telemetry)
  pelvis_drift_m   how far the pelvis moved over the floor
  tilt_change_deg  how far the pelvis tipped from its tilt at the window's start
  foot_move_m      how far either foot moved (forward kinematics on HIM's G1 model)
  reply_s          our node's worst reply time (its timing events)
  wave_done        the wave finished
Reported, not checked: arm_error_rad (the right arm's worst gap from the wave's targets),
arm_warnings (HIM's right-arm joint warnings) and late_states.

Needs MuJoCo, so it runs under HIM's .venv-policy-safety (it re-executes itself there).
"""

import json
import math
import os
import sys
from pathlib import Path

import checkout  # beside this file

HERE = Path(__file__).resolve().parent
VENV_PYTHON = checkout.HIM / ".venv-policy-safety/bin/python"
G1_MODEL = checkout.HIM / "packages/policy_safety/src/policy_safety/robots/g1/g1_robot.xml"
LIMITS = {"weighted_rms": 0.75, "pelvis_drift_m": 0.03, "tilt_change_deg": 5.0,
          "foot_move_m": 0.02, "reply_s": 0.02}
AFTER = 1.0                                          # s of standing after wave_done in the window
ARM = ("right_shoulder", "right_elbow", "right_wrist")   # HIM's warning keys are joint names

try:
    import mujoco
    import numpy as np
except ImportError:
    if Path(sys.prefix).resolve() == VENV_PYTHON.parents[1].resolve():   # already in it
        raise
    os.execv(str(VENV_PYTHON), [str(VENV_PYTHON), __file__, *sys.argv[1:]])

sys.path.insert(0, str(HERE))
from harp import wave  # noqa: E402


def tilt_deg(quat):
    """Degrees between a body's z axis and the world's, from its (w, x, y, z) quaternion."""
    w, x, y, z = quat
    up = 1 - 2 * (x * x + y * y) / (w * w + x * x + y * y + z * z)
    return math.degrees(math.acos(max(-1.0, min(1.0, up))))


def failures(m):
    """The names of the checks a run's metrics fail."""
    out = [name for name, ok in (("status", m.get("status") == "within_limits"),
                                 ("thrash_mode", m.get("thrash_mode") == "enforce"),
                                 ("thrash_telemetry", m.get("weighted_rms") is not None)) if not ok]
    out += [key for key, limit in LIMITS.items() if m.get(key) is not None and m[key] > limit]
    return out + ([] if m.get("wave_done") else ["wave_done"])


def events(run):
    """Our records: events.jsonl if course.py made one, else the JSON lines of policy.log."""
    path = run / "events.jsonl"
    lines = (path.read_text().splitlines() if path.exists() else
             [line.split("| ", 1)[-1] for line in (run / "policy.log").read_text(errors="replace").splitlines()])
    out = []
    for line in lines:
        try:
            record = json.loads(line.strip()) if line.strip().startswith("{") else None
        except ValueError:
            continue
        if isinstance(record, dict) and record.get("source") == "policy" and "event" in record:
            out.append(record)
    return out


def foot_move(qpos, rows):
    """The farthest either foot moved from where it was at rows[0]."""
    model = mujoco.MjModel.from_xml_path(str(G1_MODEL))
    if qpos.shape[1] != model.nq:
        raise ValueError(f"trace has {qpos.shape[1]} positions, the G1 {model.nq}: not the plain scene")
    data = mujoco.MjData(model)
    sites = [model.site(name).id for name in ("left_foot", "right_foot")]

    def feet(row):
        data.qpos[:] = qpos[row]
        mujoco.mj_kinematics(model, data)
        return data.site_xpos[sites].copy()

    start = feet(rows[0])
    return max(float(np.max(np.linalg.norm(feet(row) - start, axis=1))) for row in rows)


def measure(run):
    report = json.loads((run / "report.json").read_text())
    m = {"run": str(run), "status": report.get("status"),
         "thrash_mode": (report.get("thrash") or {}).get("mode"),
         "arm_warnings": {k: v for k, v in (report.get("warnings") or {}).items() if k.startswith(ARM)}}
    records = events(run)
    start = next((e for e in records if e["event"] == "wave_start"), None)
    done = next((e for e in records if e["event"] == "wave_done"), None)
    m["wave_done"] = done is not None
    if start is None or start.get("sim_time") is None:
        return m
    t0 = start["sim_time"]
    motion = wave.Wave(t0, start["pose"])
    t1 = (done["sim_time"] if done else t0 + motion.duration) + AFTER
    with np.load(run / "trace.npz") as trace:
        time, qpos = trace["time"], trace["qpos"]
        thrash = json.loads(str(trace["thrash"])) if "thrash" in trace.files else None
    rows = np.flatnonzero((time >= t0) & (time <= t1))
    if rows.size == 0:
        return m
    first = rows[0]
    m["pelvis_drift_m"] = round(float(np.max(np.linalg.norm(qpos[rows, 0:2] - qpos[first, 0:2], axis=1))), 4)
    tilt = tilt_deg(qpos[first, 3:7])
    m["tilt_change_deg"] = round(max(abs(tilt_deg(qpos[row, 3:7]) - tilt) for row in rows), 3)
    m["foot_move_m"] = round(foot_move(qpos, rows), 4)
    scores = [s["metrics"]["weighted_rms"] for s in (thrash[row] for row in rows) if thrash
              if s and (s.get("metrics") or {}).get("weighted_rms") is not None]
    m["weighted_rms"] = round(max(scores), 3) if scores else None
    m["arm_error_rad"] = round(max(float(np.max(np.abs(qpos[row, 29:36] - motion.at(float(time[row]))[0])))
                                   for row in rows if time[row] <= t0 + motion.duration), 3)
    timing = [e["reply"] for e in records if e["event"] == "timing" and e.get("sim_time") is not None
              and t0 <= e["sim_time"] <= t1 + 1.0]    # a timing line covers the second before it
    m["reply_s"] = max(timing) if timing else None
    m["late_states"] = sum(1 for e in records if e["event"] == "late_state"
                           and e.get("sim_time") is not None and t0 <= e["sim_time"] <= t1)
    return m


def main(argv):
    if not argv:
        print(__doc__, file=sys.stderr)
        return 2
    failed = False
    for arg in argv:
        m = measure(Path(arg))
        m["failed"] = failures(m)
        failed = failed or bool(m["failed"])
        print(json.dumps(m), flush=True)
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
