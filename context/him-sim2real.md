---
name: him-sim2real
description: Read before anything that touches HARP's safety harness and simulator (him-sim2real-management): the decision, its contract, what it owns and doesn't, setup, and the first robot session
metadata:
  type: project
  updated: 2026-11-02T22:04:05Z
  covers: []
---

**Decision (the maintainer's call, 2026-10-27; ask before changing it):** our code runs inside HIMRobotics' `him-sim2real-management` (github.com/HIMRobotics/him-sim2real-management, private), which is both HARP's safety harness and its MuJoCo simulator. It is an **external dependency**: never edit it, never copy its files into this repo; keep only our code on top of it. Where it and our code overlap, its version wins (walker, stand-up, stops, sim). Where it provides nothing, our code fills the gap.

**Odometry (the maintainer's call):** `/odom` is always our leg odometry, in the sim too, so the sim drifts like the robot (which usually has no HIM base position). HIM's sim `position` is the exact MuJoCo pose: ground truth only, for drift checks and course scoring, never for Nav2 or the planned moves.

**Contract** (its `containers/INSTRUCTIONS.md`): a submission is one `linux/arm64` Docker image (`docker save` → `submission.tar`) on `ros:foxy-ros-base` (ROS 2 Foxy, Python 3.8, Fast DDS), with its `ros2/him_safety_interfaces` built in at build time (from a checkout, not our repo) and no network at run time. It reads `/safety/policy_state` (JSON, 50 Hz: 29 `q`/`dq`, IMU `quat`/`gyro`/`accel`, `observation_id`, `control_ready`; `position`/`base_velocity` always in sim, on the robot only if a `SportModeState_` source is configured), arms through `/safety/checker/arm` (plus `controller_stop/ready`, `motor_gate/ready`, `motor_gate/arm` on the robot), answers each state with a full 29-joint `JointCommand` on `/safety/precheck` within 0.2 s, and exits on `/safety/abort`. Never publish `rt/lowcmd`. The host replaces our gains with its joint profile. Test with `./desktop_sim.sh` (Mac, Docker Desktop) and `--scene` for our worlds; pass is `report.json` `status: within_limits`.

**What it owns:** stand-up (GR00T Balance for 5 s in sim; on the robot until D-pad Left + B hands over), stops (double Start = damping, double F3 = Holosoma recovery, 42° tilt, pelvis height, torque, speed, thrash, deadlines), the G1 model (`g1_29dof_rev_1_0`, Menagerie-based), recording. Its walker example (`NvidiaWalker` in `packages/policy_safety/.../adapter.py`) is the same GR00T Walk/Balance ONNX and 516-input contract as our old ROS 2 stack's `Gr00tPolicy` (git history, before `5a5c272`). Its sim needs the Holosoma checkpoint (`HIM_HOLOSOMA_ONNX`; pinned URL in its `.github/workflows/test.yml`).

**What it doesn't provide** (our image fills these, [[submission-image]]): lidar (none in its sim; on the robot the container has host networking, so the lidar's UDP stream to PC2 is reachable, [[g1-lidar-measured]]), moving obstacles, a live velocity input (its examples play fixed schedules), odometry on the robot, which scene is loaded (its sim tells the container nothing). Also missing: remote buttons in the sim's `rt/lowstate`, and any way to reach rviz2 inside its loopback-only sim network.

**Setup on a laptop:** clone it beside the main checkout of this repo (`HIM_REPO` to override; worktrees find it there too, [[worktrees]]), create its `.venv-policy-safety` as its README says, download the Holosoma checkpoint (`HIM_HOLOSOMA_ONNX`), have Docker Desktop running. Our scripts write only to its ignored `.cache/harp/` (Compose mounts only that checkout, so scenes and the joint profile must be inside it). Its docs say physical stop behavior is not yet accepted.

**Dropped when our code moved onto it** (the old ROS 2 Jazzy stack, `ros2/`, is in git history, last at `9ee9255`):
- Our remote and keyboard controls, `/walker/start`/`stop`, the elastic band: HIM owns the remote, start-up and stops.
- rviz2: HIM's sim network is loopback-only; the Rerun view replaces it ([[sim-run-logs]]).
- The run recorder, bags and runs CLI: only the container's output leaves it; run folders describe themselves ([[course-runs]]).
- Parallel course runs: HIM runs one sim at a time ([[worktrees]]).
- Lam's offline tests (about 127, PR #2): not ported. Add a test only where it is load-bearing: code that runs on the robot or tooling our results depend on, catching a failure a sim run wouldn't; standard-library `unittest` in `submission/tests/`, run in the image (README).

**First robot session** (steps: README "On the robot", [[safety]]): HIM's walker example standing, then ours standing (`G1_TASK_COMMAND='[0,0,0]'`), then `{"moves": [{"walk": 0.5}]}` and `{"to": [1, 1, 45]}` through HIM's launcher (preflight, dry run, real), checking leg odometry with a tape. Rehearsed in the sim (2026-10-28, all `within_limits`): the walk ended 0.51 m; the curve about 7 cm and 5° off ([[planned-moves]]). Then real lidar into ROS 2 ([[lidar-nav2-research]]), then Nav2 on the robot.
