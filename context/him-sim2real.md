---
name: him-sim2real
description: Read before any work on the simulation-migration branch, or anything that touches HARP's safety harness/simulator (him-sim2real-management)
metadata:
  type: project
  updated: 2026-10-28T05:20:46Z
  covers: []
---

**Decision (the maintainer's call, 2026-10-27; ask before changing it):** our code is being migrated (branch `simulation-migration`) to run inside HIMRobotics' `him-sim2real-management` (github.com/HIMRobotics/him-sim2real-management, private), which is both HARP's safety harness and its MuJoCo simulator. It is an **external dependency**: never edit it, never copy its files into this repo; keep only our code on top of it. Where it and our code overlap, its version wins (walker, stand-up, stops, sim). Where it provides nothing, our code fills the gap.

**Odometry (the maintainer's call):** `/odom` is always our leg odometry, in the sim too, so the sim drifts like the robot (which usually has no HIM base position). HIM's sim `position` is the exact MuJoCo pose: ground truth only, for drift checks and course scoring, never for Nav2 or `move_distance`.

**Contract** (its `containers/INSTRUCTIONS.md`): a submission is one `linux/arm64` Docker image (`docker save` → `submission.tar`) on `ros:foxy-ros-base` (ROS 2 Foxy, Python 3.8, Fast DDS), with its `ros2/him_safety_interfaces` built in at build time (from a checkout, not our repo) and no network at run time. It reads `/safety/policy_state` (JSON, 50 Hz: 29 `q`/`dq`, IMU `quat`/`gyro`/`accel`, `observation_id`, `control_ready`; `position`/`base_velocity` always in sim, on the robot only if a `SportModeState_` source is configured), arms through `/safety/checker/arm` (plus `controller_stop/ready`, `motor_gate/ready`, `motor_gate/arm` on the robot), answers each state with a full 29-joint `JointCommand` on `/safety/precheck` within 0.2 s, and exits on `/safety/abort`. Never publish `rt/lowcmd`. The host replaces our gains with its joint profile. Test with `./desktop_sim.sh` (Mac, Docker Desktop) and `--scene` for our worlds; pass is `report.json` `status: within_limits`.

**What it owns:** stand-up (GR00T Balance for 5 s in sim; on the robot until D-pad Left + B hands over), stops (double Start = damping, double F3 = Holosoma recovery, 42° tilt, pelvis height, torque, speed, thrash, deadlines), the G1 model (`g1_29dof_rev_1_0`, Menagerie-based), recording. Its walker example (`NvidiaWalker` in `packages/policy_safety/.../adapter.py`) is the same GR00T Walk/Balance ONNX and 516-input contract as our `Gr00tPolicy`. Its sim needs the Holosoma checkpoint (`HIM_HOLOSOMA_ONNX`; pinned URL in its `.github/workflows/test.yml`).

**What it doesn't provide** (our image fills these, [[submission-image]]): lidar (none in its sim; on the robot the container has host networking, so `rt/utlidar/...` is readable), moving obstacles, a live velocity input (its examples play fixed schedules), odometry on the robot, which scene is loaded (its sim tells the container nothing). Also missing: remote buttons in the sim's `rt/lowstate`, and any way to reach rviz2 inside its loopback-only sim network.

**Setup on a laptop:** clone it beside this repo (`HIM_REPO` to override), create its `.venv-policy-safety` as its README says, download the Holosoma checkpoint (`HIM_HOLOSOMA_ONNX`), have Docker Desktop running. Our scripts write only to its ignored `.cache/harp/` (Compose mounts only that checkout, so scenes and the joint profile must be inside it). Its docs say physical stop behavior is not yet accepted.

**State of the branch (2026-10-28):** `submission/` works in HIM's sim (walker on `/cmd_vel`, leg odometry, move to a point, and a first Nav2 course), [[submission-image]]. Nothing has run on the real robot. The old `ros2/` stack is broken on this branch: its worlds, `movers.py` and Nav2 config were moved into `submission/`, so `run_sim` fails on import. The README doesn't describe `submission/` yet.

**Next steps, in order:**
1. Run the move on the robot through HIM's launcher (preflight, dry run, real; HIM's walker example with `stand.yaml` first, then ours standing, then a short `{"to": ...}`), and check leg odometry with a tape.
2. Add the README section for `submission/` (setup, sim with the viewer, move to a point, robot steps) and a milestone under "Run safely on the real robot".
3. A parity checklist, kept as README milestones: each old feature, where it lives now (HIM or our image) or why it was dropped, and the test that shows it matches the old numbers.
4. Run the other courses with `submission/course.py` (`open_floor`, `slalom`, `obstacle_course`, `crossing`, `oncoming`) against their criteria.
5. Port the rest of `move_distance` (straight walks, turns in place).
6. `lidar_relay` in the image for the robot (CycloneDDS and Unitree's SDK built in), then Nav2 on the robot.
7. Only then remove the old `ros2/` stack, and rewrite the memories that describe it ([[safety]], [[controls]], [[mujoco-sim]], [[nav2-sim]], [[run-logging]], [[gr00t-wbc]], [[ros2-mac]], [[move-distance]]). Tell the teammate behind the remote-button rule in [[controls]] first that HIM's controls replace ours.
