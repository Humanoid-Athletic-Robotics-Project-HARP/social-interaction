---
name: sim-vs-real
description: Read before trusting a sim result for anything ahead of the robot, or before changing the sim lidar, adding a sim camera, Nav2's sensors, approach_stop or person detection — where the sim's sensors differ from the real G1's and what each gap breaks
metadata:
  type: project
  updated: 2026-11-01T22:25:38Z
  covers: [submission/harp/lidar.py, submission/harp/approach_stop.py, submission/nav2/nav2_params.yaml]
---

Measured on the robot 2026-11-01 ([[g1-lidar-measured]], [[g1-head-camera]],
[[g1-sensor-fusion]]) against `harp/lidar.py` and HIM's G1 model.

**Gaps** (sim → robot):
- **Lidar sees through the head.** The head's geoms belong to `torso_link`, which
  `lidar.py` excludes from its rays, so the sim sees all around; the robot is nearly
  blind ±50° around the front (0.3% of returns within ±30°, about 1.5% at 30–60°).
- **No camera in the sim** (HIM's model has none); the robot's depth camera is the
  only sensor ahead, and only near and low.
- Lidar data: the sim publishes `/utlidar/cloud_livox_mid360` itself; the robot sends
  raw Livox UDP to PC2 and has no such topic (`harp/lidar_relay.py` gets nothing).
- Smaller: the sim drops 5% of rays, the robot 43% (no echo); sim rays are an even
  grid at one instant, the robot's a wandering pattern over 100 ms (smears when
  turning). The mount, height and floor ring (seen from about 0.8–1.0 m) match.

**What breaks on the robot, though it passes in the sim:**
- `approach_stop.py`: its strip (±0.7 m, out to 6 m, 0.3–1.8 m up) is inside the
  blind cone beyond 0.6 m and inside `BODY` nearer, so it never sees someone walking
  straight at the robot.
- Nav2: obstacles straight ahead aren't marked until they are about 50° off the nose
  (STVL then remembers them 15 s); course scores in the sim overstate the robot.
- The README's person-detection milestone ("on the D435i, out to 4 m"): the camera
  sees nothing past 2.7 m and only legs nearer (up to 0.79 m at 1 m, 0.33 m at 2 m).

**In the sim now** (2026-11-01): the lidar's blind cone and real no-echo rate
(`harp/lidar.py`, `BLIND`); a depth camera cast from the same state at the URDF pose
(`harp/depth_camera.py`, `/camera/depth/points`, optical frame, live TF from
`harp/state.py`); both in Nav2's STVL costmaps; and `approach_stop.py` taking the
nearer of the lidar's and the camera's point in its strip. Results: [[nav2-sim]].
Still sim-only: the camera at 10 Hz on an even grid, no colour.

**Robot side to match:** a Livox UDP reader into ROS 2 (in place of `lidar_relay.py`),
and the depth camera into `/camera/depth/points` in the optical frame
([[seeing-ahead-research]]).
