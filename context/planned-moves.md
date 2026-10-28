---
name: planned-moves
description: Read when changing or debugging planned moves (harp/moves.py, G1_TASK_COMMAND), the curve law or leg odometry, or when you need their accuracy
metadata:
  type: reference
  updated: 2026-10-28T18:46:58Z
  covers: [submission/harp/moves.py, submission/harp/smooth_control.py, submission/harp/leg_odometry.py]
---

Usage: README "Run it in the sim" and `harp/moves.py`'s docstring. Moves run in order, 2 s apart, each from where the last ended (errors carry over); speeds are capped at 0.4 m/s and 30°/s.

**Walks** hold the heading they started with (gain 1.5, at most 0.3 rad/s) and stop on distance. **Turns** stop on angle, and stop everything if they go 10° the wrong way. **Curves** (`"to"`) use Park & Kuipers' smooth control law (ICRA 2011), the same law and gains as Nav2's Graceful Controller: one forward curve ending at the target heading, slowing on sharp curves and near the point (at least 0.1 m/s, above GR00T's 0.05 Balance switch), then a turn in place if over 5° off. Never sideways: a close point to the side takes a small loop. Points under 0.16 m are refused. A curve counts as there within 8 cm, so for a straight line use `"walk"` (a 0.5 m `"to"` ended at 0.43 m). A move that takes far longer than expected stops the rest (`move_timeout`).

**Leg odometry** (`harp/leg_odometry.py`, our `/odom` in the sim and on the robot): heading from the pelvis IMU; position from MuJoCo forward kinematics of HIM's G1, taking the lower foot as planted. On the step where the planted foot switches it uses the previous planted foot; otherwise each switch dropped that step's motion (about 7% short over a walk).

**Accuracy in HIM's sim** (one run each, 2026-10-28, `within_limits`): walk 1 m ended 1.02 m, 0.5 m ended 0.51 m; turn 90° ended 90.2°; curve to (1, 1, 45°) ended at (0.96–0.98, 0.94, 49–50°), 6–7 cm and 4–5° off. Curve to (0.5, 0.5), no heading (2026-10-28, `runs/2026-10-28_18-43-57Z_sim_goto_0.5_0.5`): done 10.6 s after it started, at (0.475, 0.445, 47.8°) true, 6 cm off; leg odometry said (0.47, 0.448). Leg odometry agreed with the truth within 1 cm each time, and within 5 cm over every Nav2 course ([[nav2-sim]]).
