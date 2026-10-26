---
name: move-distance
description: Read when changing or debugging move_distance, curves (--to, smooth_control.py) or leg odometry, or when you need its accuracy in the sim
metadata:
  type: reference
  updated: 2026-10-26T22:40:10Z
  covers: [ros2/src/harp_control/harp_control/move_distance.py, ros2/src/harp_control/harp_control/smooth_control.py, ros2/src/harp_control/harp_control/leg_odometry.py]
---

Usage and flags: README "Planned moves" and `--help`. Speeds are capped at 0.4 m/s and 30°/s; moves run in order with 1.5 s of balancing between them.

**Running:** "forward" is wherever the robot faces when `g` is pressed, so wait a few seconds after the band release. After `o` the robot lies on the floor with the band anchored above the start: restart both sim and controller. When run_recorder is running, moves and leg odometry (`leg_odom.csv`) are logged into the run through `runlog.py` ([[run-logging]]).
- Scripting it: `TerminalKeys` needs a real terminal, so run the controller in a pty (`pty.fork`) with its flags as separate arguments, beside `run_sim --auto-release 9 --log` (shorter can drop the band before `]`). Send `]` once the logged pelvis z is below 0.81, `g` about 6 s later, `o` after "All moves done". If the controller exits, stop the sim before the band drops a limp robot.

**Aborts and damps** if a move takes much longer than expected, a turn goes over 10° the wrong way, the pelvis tilts past 60°, or robot state stops. A walk that times out stepping in place means the band was still on.

**Leg odometry** (`leg_odometry.py`; Unitree's needs its controller): heading from the pelvis IMU; position from MuJoCo forward kinematics, taking the lower foot as planted. On the step where the planted foot switches, it uses the previous planted foot; otherwise each switch drops that step's motion (about 7% short over a walk). Straight walks steer to hold their starting heading (within about 2°, against 5–10° drift without).

**Curves (`--to`)** use Park & Kuipers' smooth control law (ICRA 2011), the same law and gains as Nav2's Graceful Controller. One forward curve ending at the target heading, slowing on sharp curves and near the point, with a short turn in place if still over 5° off. Never steps sideways: a close point to the side takes a small loop, a point behind a wide curve. Points closer than 0.16 m are refused. Stops within 8 cm and 5° by its own estimate.

**Accuracy in the sim** (true pose from `run_sim --log`):
- 90° turn: 90.1° estimated, 89.9° true. 2 m walk: 2.03 / 2.03 m. 1 m backward: 1.11 / 1.12 m.
- The robot drifts about 0.1 m after stopping, so walks end slightly long. Each move only knows about itself, so errors carry into the next move's start.
- Curves end within about 8 cm and 5° of target, estimate and truth agreeing within 2 cm and 1°: e.g. `--to 2 1` ended at (1.97, 0.92, 32°) for a target of (2.00, 1.00, 27°); `--to 1 0 180` at (1.07, 0.00, 175°). The weave `--to 2 1 0 --to 2 -1 0` ends 3.93 m ahead, 0.01 m left, facing 7° left.
