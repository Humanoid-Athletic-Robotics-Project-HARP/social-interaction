#!/usr/bin/env python3
"""Closed-loop planar validation with MuJoCo raycast observations.

Run from the repo: PYTHONPATH=submission python3 submission/test_predictive_sim.py
Requires numpy and mujoco, as supplied by the submission image. This is a
component simulation: ideal odometry, circular bodies and an acceleration-limited
planar plant. It does not run GR00T, Foxy, Nav2 or HIM, and does not validate G1
balance or physical stopping. Object truth is used only to place geometry and
score separation; the controller receives tracks made from ray hits.
"""
import argparse
from datetime import datetime, timezone
from collections import Counter
import json
import math
from pathlib import Path
import time

import mujoco
import numpy as np

from harp.obstacle_tracker import Tracker
from harp.predictive_mpc import Planner

DT = 0.1
ROBOT_RADIUS = 0.4
ACCELERATION = 0.4


def scenarios():
    # (initial xy, velocity xy, radius); units are metres and seconds.
    return {
        "open": {"duration": 14., "objects": [], "goal": (3., 0.)},
        "crossing": {"duration": 18., "objects": [((1.3, -1.5), (0., .4), .18)],
                     "goal": (3., 0.)},
        "head_on": {"duration": 20., "objects": [((3., 0.), (-.5, 0.), .18)],
                    "goal": (3., 0.)},
        # Stop and signal before an unavoidable impact: do not call standing safe.
        "trapped": {"duration": 1.5, "objects": [
            ((1.5 * math.cos(a), 1.5 * math.sin(a)),
             (-.3 * math.cos(a), -.3 * math.sin(a)), .18)
            for a in np.linspace(0, 2 * np.pi, 12, endpoint=False)],
                    "goal": (3., 0.)},
    }


class World:
    """MuJoCo geometry and a 360-degree horizontal ray sensor."""
    def __init__(self, objects):
        body = '<body name="robot" mocap="true"><geom type="cylinder" size=".4 .5"/></body>'
        for i, (_, _, radius) in enumerate(objects):
            body += ('<body name="object%d" mocap="true">'
                     '<geom type="cylinder" size="%s .7"/></body>' % (i, radius))
        self.model = mujoco.MjModel.from_xml_string(
            '<mujoco><option gravity="0 0 0"/><worldbody>' + body + '</worldbody></mujoco>')
        self.data = mujoco.MjData(self.model)
        self.objects = objects
        self.robot_body = self.model.body("robot").id
        angle = np.linspace(-math.pi, math.pi, 720, endpoint=False)
        self.directions = np.column_stack((np.cos(angle), np.sin(angle), np.zeros(len(angle))))

    def centers(self, stamp):
        return [np.asarray(p) + stamp * np.asarray(v) for p, v, _ in self.objects]

    def scan(self, pose, stamp):
        self.data.mocap_pos[0] = (pose[0], pose[1], .5)
        for i, center in enumerate(self.centers(stamp), 1):
            self.data.mocap_pos[i] = (center[0], center[1], .5)
        mujoco.mj_forward(self.model, self.data)
        origin = np.array([pose[0], pose[1], .6], dtype=float)
        geom_id = np.zeros(1, dtype=np.int32)
        hits = []
        # mj_ray is shared by MuJoCo 3.2.3 (Foxy) and the native test environment.
        for direction in self.directions:
            distance = mujoco.mj_ray(self.model, self.data, origin, direction,
                                     None, 1, self.robot_body, geom_id)
            if 0.0 <= distance <= 6.0:
                hits.append((origin + distance * direction)[:2])
        return np.asarray(hits, dtype=float).reshape(-1, 2)


def separation_segment(robot_start, robot_end, object_start, object_end, radius):
    """Minimum separation over the whole step, not just its two endpoints."""
    r = np.asarray(robot_start) - np.asarray(object_start)
    dr = (np.asarray(robot_end) - robot_start) - (np.asarray(object_end) - object_start)
    divisor = float(dr @ dr)
    t = 0.0 if divisor < 1e-12 else float(np.clip(-(r @ dr) / divisor, 0., 1.))
    return float(np.linalg.norm(r + t * dr)) - ROBOT_RADIUS - radius


def run(name, spec, observer=None):
    world, tracker, planner = World(spec["objects"]), Tracker(), Planner()
    pose, velocity, queued = np.zeros(3), np.zeros(3), np.zeros(3)
    goal = np.asarray(spec["goal"], dtype=float)
    modes, latencies, trace = Counter(), [], []
    minimum, contacts, alerts, max_speed = float("inf"), 0, 0, 0.
    have_yielded, resumed = False, False
    for step in range(int(round(spec["duration"] / DT))):
        stamp = round(step * DT, 8)
        points = world.scan(pose, stamp)
        tracks = tracker.update(points, stamp)
        error = goal - pose[:2]
        speed = min(.25, float(np.linalg.norm(error)))
        world_desired = error / max(float(np.linalg.norm(error)), 1e-9) * speed
        c, s = math.cos(pose[2]), math.sin(pose[2])
        desired = (c * world_desired[0] + s * world_desired[1],
                   -s * world_desired[0] + c * world_desired[1], 0.)
        if step % 2 == 0:  # Match the ROS node: 5 Hz planning, 10 Hz sensing.
            before = time.perf_counter()
            result = planner.plan(tuple(pose), tuple(velocity), desired, tracks, stamp)
            latencies.append(time.perf_counter() - before)
        command = np.asarray(result.command, dtype=float)
        if observer is not None:
            observer(world, pose.copy(), stamp, points, tracks, result)
        if not np.isfinite(command).all():
            raise AssertionError("Non-finite command")
        if np.linalg.norm(command[:2]) > .300001 or abs(command[2]) > .600001:
            raise AssertionError("Command exceeds configured caps")
        modes[result.mode] += 1
        alerts += int(result.alert)
        have_yielded |= result.mode in ("yield", "blocked", "evade")
        resumed |= have_yielded and result.mode == "follow" and np.linalg.norm(command[:2]) > .15
        start = pose[:2].copy()
        # One cycle of transport delay, followed by finite acceleration.
        delta = queued[:2] - velocity[:2]
        velocity[:2] += delta * min(1., ACCELERATION * DT / max(np.linalg.norm(delta), 1e-9))
        velocity[2] += float(np.clip(queued[2] - velocity[2], -.8 * DT, .8 * DT))
        pose[:2] += DT * np.array([c * velocity[0] - s * velocity[1],
                                  s * velocity[0] + c * velocity[1]])
        pose[2] += DT * velocity[2]
        queued = command
        max_speed = max(max_speed, float(np.linalg.norm(velocity[:2])))
        for i, (_, _, radius) in enumerate(spec["objects"]):
            clearance = separation_segment(start, pose[:2], world.centers(stamp)[i],
                                           world.centers(stamp + DT)[i], radius)
            minimum = min(minimum, clearance)
            contacts += int(clearance < 0.)
        trace.append({"t": stamp, "pose": pose.tolist(), "command": command.tolist(),
                      "mode": result.mode, "alert": bool(result.alert), "tracks": len(tracks),
                      "solver": result.solver, "predicted_clearance": None if not math.isfinite(result.clearance) else result.clearance,
                      "objects": [[float(t.x), float(t.y), float(t.vx), float(t.vy),
                                   float(t.radius), float(t.uncertainty)] for t in tracks]})
    report = {
        "scenario": name, "simulation_seconds": spec["duration"], "colliding_steps": contacts,
        "minimum_surface_separation_m": None if math.isinf(minimum) else round(minimum, 4),
        "goal_error_m": round(float(np.linalg.norm(goal - pose[:2])), 4),
        "maximum_speed_mps": round(max_speed, 4), "modes": dict(modes),
        "alert_steps": alerts, "resumed": bool(resumed),
        "max_lateral_displacement_m": round(max(abs(t["pose"][1]) for t in trace), 4),
        "planner_ms_p50": round(float(np.percentile(latencies, 50)) * 1000, 3),
        "planner_ms_p95": round(float(np.percentile(latencies, 95)) * 1000, 3),
        "planner_ms_max": round(max(latencies) * 1000, 3),
    }
    if name == "trapped":
        passed = contacts == 0 and alerts > 0 and "blocked" in modes
    else:
        passed = contacts == 0 and report["goal_error_m"] < .1
        if name == "crossing":
            passed &= have_yielded and resumed
        if name == "head_on":
            passed &= "evade" in modes and report["max_lateral_displacement_m"] > .3
    report["passed"] = bool(passed)
    return report, trace


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scenario", choices=["all"] + list(scenarios()), default="all")
    parser.add_argument("--output", type=Path, help="Write summary.json and per-scenario traces here")
    args = parser.parse_args()
    reports = []
    if args.output:
        args.output.mkdir(parents=True, exist_ok=True)
    for name, spec in scenarios().items():
        if args.scenario not in ("all", name):
            continue
        report, trace = run(name, spec)
        reports.append(report)
        print(json.dumps(report, sort_keys=True), flush=True)
        if args.output:
            (args.output / (name + ".json")).write_text(json.dumps(trace, indent=2) + "\n")
    summary = {"simulator": "MuJoCo raycast geometry + delayed acceleration-limited planar plant",
               "mujoco_version": mujoco.__version__, "humanoid_validation": False,
               "recorded_utc": datetime.now(timezone.utc).isoformat(),
               "sensor_period_s": DT, "planner_period_s": 2 * DT,
               "robot_radius_m": ROBOT_RADIUS, "plant_delay_s": DT,
               "plant_acceleration_mps2": ACCELERATION,
               "passed": all(r["passed"] for r in reports), "scenarios": reports}
    if args.output:
        (args.output / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    return 0 if summary["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
