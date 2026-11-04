---
name: planned-moves
description: Read when changing or debugging planned moves (harp/moves.py, G1_TASK_COMMAND), the curve law or leg odometry, or when you need their accuracy
metadata:
  type: reference
  updated: 2026-11-04T16:58:53Z
  covers: [submission/harp/moves.py, submission/harp/smooth_control.py, submission/harp/leg_odometry.py, submission/tests/test_moves.py]
---

Usage: README "Run it in the sim" and `harp/moves.py`'s docstring. Moves run in order, 2 s apart, each from where the last ended (errors carry over); speeds are capped at 0.4 m/s and 30°/s.

**Walks** measure signed displacement along the starting heading; sideways travel does not count. Crossing `WALK_MAX_CROSS_TRACK` or `WALK_WRONG_WAY_LIMIT` ends the plan with immediate zero (`walk_drift`/`wrong_way`), even if the forward target is reached. These initial limits need HIM validation. Walks hold the starting heading and ramp speed requests by `WALK_ACCEL / RATE` per update. Their approach speed follows the braking envelope `sqrt(2 * WALK_DECEL * remaining)`, with a walking floor bounded by the requested speed, so GR00T does not switch to Balance short of the target. Once there, or on timeout, zero is immediate; smoothing never delays a fault stop. Constants live in `harp/moves.py`. Ramp-up briefly passes through the policy's deadband; requests entirely below that deadband can still time out. The deadline remains finite. **Turns** ramp signed angular velocity with `TURN_ACCEL`/`TURN_DECEL`, brake near the angle, and keep a floor bounded by the requested rate. The same helper handles final curve alignment, including an opposite incoming turn rate. Target/fault zero remains immediate; turns stop everything if they go 10° the wrong way. **Curves** (`"to"`) use Park & Kuipers' smooth control law (ICRA 2011), the same law and curve gains as Nav2's Graceful Controller (slow-down radius 0.6 m here, 0.8 m in our Nav2 settings): one forward curve ending at the target heading, slowing on sharp curves and near the point (at least 0.1 m/s, above GR00T's 0.05 Balance switch, unless a lower speed was requested), then a turn in place if over 5° off. Never sideways: a close point to the side takes a small loop, and a close point behind is turned to and walked to (counted as passed only once it has been in front; before Nov 4 one within 0.3 m behind ended the move at once, facing its heading, without walking). Points under 0.16 m are refused. A curve counts as there within 8 cm, so use `"walk"` for straight distance. A move that takes far longer than expected stops the rest (`move_timeout`).

**Exit handling:** success exits 0, interrupted input exits 130, and an incomplete/faulted plan exits 1. Callback exceptions still request zero and release ROS resources. Malformed/non-object shared events are ignored.

**Feedback guard:** planned moves wait for a valid pose. A gap over `ODOM_TIMEOUT` in valid pose receipts ends an active/settling plan with immediate zero and `odom_timeout`; later feedback does not resume it. Invalid positions or yaw quaternions do not refresh receipt time. This is an arrival watchdog, not a check of sensor acquisition age.

**Leg odometry** (`harp/leg_odometry.py`, our `/odom` in the sim and on the robot): heading from the pelvis IMU; position from MuJoCo forward kinematics of HIM's G1, taking the lower foot as planted. On the step where the planted foot switches it uses the previous planted foot; otherwise each switch dropped that step's motion (about 7% short over a walk).

**Validation:** unit tests (2026-10-31) cover the guards, faults, ramps, caps and final alignment against synthetic models, not GR00T. The feedback and drift limits and the speed profiles still need stopping-distance, angle and timing measurements in HIM's sim.

**Last sim accuracy, before the speed ramps** (2026-10-28, one run each): turn 90° ended 90.2°; curve to (1, 1, 45°) 6–7 cm and 4–5° off; curve to (0.5, 0.5) 6 cm off, leg odometry within 1 cm of the truth (`runs/2026-10-28_18-43-57Z_sim_goto_0.5_0.5`).
