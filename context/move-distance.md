---
name: move-distance
description: move_distance (harp_control) in detail: flags and limits, how moves run and abort, curve (--to) behavior, leg odometry method, and the measured sim results behind the README's examples
metadata:
  type: reference
  updated: 2026-10-26T21:32:46Z
  covers: [ros2/src/harp_control/harp_control/move_distance.py, ros2/src/harp_control/harp_control/smooth_control.py, ros2/src/harp_control/harp_control/leg_odometry.py]
---

`ros2 run harp_control move_distance` drives GR00T ([[gr00t-wbc]]) through a list of moves in the sim ([[mujoco-sim]]) or on the robot. The README only lists example commands; this note has the detail.


Runs are recorded when run_recorder is running ([[run-logging]]): move_distance writes start, `move_start`, `move_done` (target and estimate), timeouts, wrong-way turns and errors, plus leg odometry and the command at 10 Hz (`leg_odom.csv`), into the active run folder through `harp_control/runlog.py`. That needs no ROS.

**Flags**
- `--distance M`: meters, negative walks backward. `--angle DEG`: positive turns left (counterclockwise). `--to X Y [HEADING]`: curve to X m forward, Y m left of where the move starts, ending facing HEADING (positive left), or without it, facing away from the start.
- `--distance`, `--angle` and `--to` repeat and mix; moves run in the given order, with 1.5 s of balancing in place between them.
- `--speed`: m/s for every walk (the top speed for `--to`), default 0.2, capped at 0.4. `--turn-speed`: deg/s, default 20, capped at 30; for `--to` it's the fastest turn while walking.
- `--real --interface IF` for the robot; `--scene` for the G1 model (default `$HARP_G1_SCENE`).

**How a run goes:** stand-up as for `walk` (feet down, `]`, release the band), then `g` starts the moves. "Forward" is wherever the robot faces when you press `g`; give it a few seconds after the release, since it often steps and turns a little while catching itself. Each move prints its estimate; after the last one it keeps balancing until `o`. After `o` it lies on the floor and the band is anchored above the start, so restart both the sim and the controller to run again.

**Aborts (and damps)** if a move takes much longer than expected, a turn goes more than 10° the wrong way, the pelvis tilts past 60°, or robot state stops arriving (0.1 s on the robot, 0.5 s in the sim). A walk that times out with the robot stepping in place means the band was still on.

**Leg odometry** (`leg_odometry.py`), since Unitree's odometry needs its controller: heading from the pelvis IMU; position from leg kinematics (MuJoCo forward kinematics of the G1 model), taking the lower foot as planted, so the pelvis moves by minus that foot's motion. Each straight walk steers to hold the heading it started with (without that, a 2 m walk drifted 5–10°; with it, within about 2°). History of the 7% fix: [[gr00t-wbc]].

**Measured in the sim (2026-10-25), true positions from `run_sim --log`:**
- 90° turn: estimated 90.1°, true 89.9°. 2 m walk: estimated 2.03 m, true 2.03 m. 1 m backward: estimated 1.11 m, true 1.12 m.
- The robot drifts about 0.1 m after the stop command, so walks end a little long. GR00T walks at about 85% of the commanded speed.
- `--distance 2 --angle 90 --distance 2 --speed 0.3`: estimated 2.00 m, +89.4°, 2.02 m; truly 2.01 m, 88.8°, 2.01 m, ending 2.24 m forward and 1.88 m left of the start. Each move only knows about itself, so the 1° short turn and the 0.1 m drift carry into the end position; `--to` moves have the same limit.

**Curves (`--to`)**, Park & Kuipers' control law ("A Smooth Control Law for Graceful Motion of Differential Wheeled Mobile Robots in 2D Environments", ICRA 2011) in `smooth_control.py`, the same law and default gains as Nav2's Graceful Controller:
- Walks forward along one smooth curve that ends at the point already facing the final heading; slows on sharp curves and near the point; if still more than 5° off on arrival, finishes with a short turn in place.
- Never steps sideways, so a close point off to one side takes a small loop; a point behind is reached in a wider curve rather than turning around first; points closer than 0.16 m are refused (use `--angle`).
- Stops within 8 cm and 5° of the target by its own estimate; moves took 13–21 s including the 1.5 s settle.

| Move (`--speed 0.3`) | Target (x, y, heading) | Really ended | Estimated |
|---|---|---|---|
| `--to 2 1` | 2.00, 1.00, 27° | 1.97, 0.92, 32° | 1.96, 0.94, 32° |
| `--to 1.5 -1 -90` | 1.50, −1.00, −90° | 1.45, −0.94, −85° | 1.45, −0.93, −85° |
| `--to 1 0 180` | 1.00, 0.00, 180° | 1.07, 0.00, 175° | 1.08, 0.01, 175° |
| `--to -1 0.5` (behind) | −1.00, 0.50, 153° | −0.93, 0.54, 156° | −0.93, 0.54, 156° |
| `--to 1 1 90` after a walk and a turn | 1.00, 1.00, 90° | 1.03, 0.93, 94° | 1.04, 0.94, 94° |

- Weave, `--to 2 1 0 --to 2 -1 0 --speed 0.3`: estimated (+1.94, +0.96) facing +3.2°, then (+1.93, −1.03) facing +4.8°; truly ended 3.93 m forward, 0.01 m left, facing 7° left; about 15 s per move. To approach someone, give the final heading toward them: `--to 2 1 90` ends 2 m ahead and 1 m left, facing left.
- After the package move (2026-10-25), `--to 2 1` via `ros2 run` estimated (1.96, 0.94) facing 30°, matching the table.
