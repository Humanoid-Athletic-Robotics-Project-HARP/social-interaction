---
name: planned-moves
description: Read when changing or debugging planned moves (harp/moves.py, G1_TASK_COMMAND), the curve law or leg odometry, or when you need their accuracy
metadata:
  type: reference
  updated: 2026-10-31T09:37:20Z
  covers: [submission/harp/moves.py, submission/harp/smooth_control.py, submission/harp/leg_odometry.py, submission/tests/test_moves.py]
---

Usage: README "Run it in the sim" and `harp/moves.py`'s docstring. Moves run in order, 2 s apart, each from where the last ended (errors carry over); speeds are capped at 0.4 m/s and 30°/s.

**Walks** hold the starting heading and ramp speed requests by `WALK_ACCEL / RATE` per update. Their approach speed follows the braking envelope `sqrt(2 * WALK_DECEL * remaining)`, with a walking floor bounded by the requested speed, so GR00T does not switch to Balance short of the target. Once there, or on timeout, zero is immediate; smoothing never delays a fault stop. Constants live in `harp/moves.py`. Ramp-up briefly passes through the policy's deadband; requests entirely below that deadband can still time out. The deadline remains finite. **Turns** stop on angle, and stop everything if they go 10° the wrong way. **Curves** (`"to"`) use Park & Kuipers' smooth control law (ICRA 2011), the same law and gains as Nav2's Graceful Controller: one forward curve ending at the target heading, slowing on sharp curves and near the point (at least 0.1 m/s, above GR00T's 0.05 Balance switch), then a turn in place if over 5° off. Never sideways: a close point to the side takes a small loop. Points under 0.16 m are refused. A curve counts as there within 8 cm, so use `"walk"` for straight distance. A move that takes far longer than expected stops the rest (`move_timeout`).

**Leg odometry** (`harp/leg_odometry.py`, our `/odom` in the sim and on the robot): heading from the pelvis IMU; position from MuJoCo forward kinematics of HIM's G1, taking the lower foot as planted. On the step where the planted foot switches it uses the previous planted foot; otherwise each switch dropped that step's motion (about 7% short over a walk).

**Speed-profile validation** (macOS, 2026-10-31): eight focused tests cover acceleration/deceleration bounds, forward/backward requests, requested caps, heading correction, target/timeout zeroing, and the next move's ramp. A synthetic point-mass model reaches 0.01/0.5/1/2 m targets in both directions; this is not GR00T simulation. Docker and HIM were unavailable here. The new profile still needs stopping-distance and timing measurements in HIM's sim; older straight-walk accuracy numbers do not validate it.

**Turn/curve accuracy in HIM's sim** (one run each, 2026-10-28, `within_limits`): turn 90° ended 90.2°; curve to (1, 1, 45°) ended at (0.96–0.98, 0.94, 49–50°), 6–7 cm and 4–5° off. Curve to (0.5, 0.5), no heading (2026-10-28, `runs/2026-10-28_18-43-57Z_sim_goto_0.5_0.5`): done 10.6 s after it started, at (0.475, 0.445, 47.8°) true, 6 cm off; leg odometry said (0.47, 0.448). Nav2 odometry results are in [[nav2-sim]].
