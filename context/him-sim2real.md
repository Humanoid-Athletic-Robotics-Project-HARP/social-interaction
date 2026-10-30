---
name: him-sim2real
description: Read before anything that touches HARP's safety harness/simulator (him-sim2real-management), or to see what moving our code onto it dropped and what is left to do
metadata:
  type: project
  updated: 2026-10-30T19:06:32Z
  covers: []
---

**Decision (the maintainer's call, 2026-10-27; ask before changing it):** our code runs inside HIMRobotics' `him-sim2real-management` (github.com/HIMRobotics/him-sim2real-management, private), which is both HARP's safety harness and its MuJoCo simulator. It is an **external dependency**: never edit it, never copy its files into this repo; keep only our code on top of it. Where it and our code overlap, its version wins (walker, stand-up, stops, sim). Where it provides nothing, our code fills the gap.

**Odometry (the maintainer's call):** `/odom` is always our leg odometry, in the sim too, so the sim drifts like the robot (which usually has no HIM base position). HIM's sim `position` is the exact MuJoCo pose: ground truth only, for drift checks and course scoring, never for Nav2 or the planned moves.

**Contract** (its `containers/INSTRUCTIONS.md`): a submission is one `linux/arm64` Docker image (`docker save` → `submission.tar`) on `ros:foxy-ros-base` (ROS 2 Foxy, Python 3.8, Fast DDS), with its `ros2/him_safety_interfaces` built in at build time (from a checkout, not our repo) and no network at run time. It reads `/safety/policy_state` (JSON, 50 Hz: 29 `q`/`dq`, IMU `quat`/`gyro`/`accel`, `observation_id`, `control_ready`; `position`/`base_velocity` always in sim, on the robot only if a `SportModeState_` source is configured), arms through `/safety/checker/arm` (plus `controller_stop/ready`, `motor_gate/ready`, `motor_gate/arm` on the robot), answers each state with a full 29-joint `JointCommand` on `/safety/precheck` within 0.2 s, and exits on `/safety/abort`. Never publish `rt/lowcmd`. The host replaces our gains with its joint profile. Test with `./desktop_sim.sh` (Mac, Docker Desktop) and `--scene` for our worlds; pass is `report.json` `status: within_limits`.

**What it owns:** stand-up (GR00T Balance for 5 s in sim; on the robot until D-pad Left + B hands over), stops (double Start = damping, double F3 = Holosoma recovery, 42° tilt, pelvis height, torque, speed, thrash, deadlines), the G1 model (`g1_29dof_rev_1_0`, Menagerie-based), recording. Its walker example (`NvidiaWalker` in `packages/policy_safety/.../adapter.py`) is the same GR00T Walk/Balance ONNX and 516-input contract as our `Gr00tPolicy`. Its sim needs the Holosoma checkpoint (`HIM_HOLOSOMA_ONNX`; pinned URL in its `.github/workflows/test.yml`).

**What it doesn't provide** (our image fills these, [[submission-image]]): lidar (none in its sim; on the robot the container has host networking, so `rt/utlidar/...` is readable), moving obstacles, a live velocity input (its examples play fixed schedules), odometry on the robot, which scene is loaded (its sim tells the container nothing). Also missing: remote buttons in the sim's `rt/lowstate`, and any way to reach rviz2 inside its loopback-only sim network.

**Setup on a laptop:** clone it beside this repo (`HIM_REPO` to override), create its `.venv-policy-safety` as its README says, download the Holosoma checkpoint (`HIM_HOLOSOMA_ONNX`), have Docker Desktop running. Our scripts write only to its ignored `.cache/harp/` (Compose mounts only that checkout, so scenes and the joint profile must be inside it). Its docs say physical stop behavior is not yet accepted.

**State (2026-10-28):** merged into `main` (PR #3, `186a653`): everything lives in `submission/` ([[submission-image]]). The old ROS 2 Jazzy stack (`ros2/`) is only in git history (last at `9ee9255`, with Lam's PR #2). Nothing has run on the real robot.

**Dropped in the migration, and why** (the checklist of what moved is the README's "Run safely on the real robot"; the old code is in git history):
- Our keyboard and remote controls (B stop, L1 hand driving, R1 combos, height keys), `/walker/start`, `/walker/stop` and `auto_start`: HIM owns the remote (D-pad Left + B hands over, double Start damps, double F3 recovers) and the start-up, and a submission can't take over by hand.
- The elastic band, its keys and scripted remote presses: HIM's sim starts standing under GR00T balance and has no remote.
- `/walker/policy` (GR00T's observation and action): the walker runs inside HIM's client.
- rviz2 and its configs: HIM's sim network is loopback-only, so nothing outside the container sees our topics; HIM's browser viewer and replays show the robot, not costmaps.
- `run_recorder`, bags (extras, Nav2 replay) and the speed-tracking and lidar-health metrics: only the container's output leaves it, so a run keeps HIM's `report.json` and `trace.npz` plus `policy.log`; `sim.sh --output DIR` does that for any run.
- `runs list/show/pin/set` and retention: run folders describe themselves (`run.json`, `summary.json`); `course.py --compare` and `--prune` cover the rest.
- Parallel course runs (DDS domain pairs): HIM's Compose project runs one sim at a time.
- The pixi environment, our STVL source copy and the MuJoCo window: the Docker image (Foxy's STVL binary) and HIM's viewer.
- Lam's offline tests (PR #2, about 127 under `ros2/src/*/test/`, last at `9ee9255` on main): not ported. Most test code we deleted (walker, run_recorder, replay, runs CLI) or that HIM now owns (stops, damping, manual takeover). `submission/tests/` holds only `harp/approach_stop.py`'s rule (run in the image: README). Re-add a test only if it is load-bearing: it covers code that runs on the robot (`harp_node.py`, `harp/state.py`, `harp/leg_odometry.py`, `harp/moves.py`, `harp/lidar_relay.py`, the Nav2 settings) or tooling our results depend on (`course.py` scoring, `harp/route.py`), and it catches a failure a sim run wouldn't. Use the standard library's `unittest`, in `submission/tests/`.

**To do, without the robot** (the maintainer's call):
- [ ] Find what makes states late 3–8 s into course runs, around HIM's balance handing over to our node (one stop in 12 runs, [[sim-deadline-stops]]).

**To do, with the robot:** the first session (below), then real lidar through `lidar_relay`, then Nav2 on the robot.

The first robot session's moves are rehearsed in the sim (2026-10-28, all `within_limits`): standing; `{"moves": [{"walk": 0.5}]}` ended 0.51 m (a `"to"` straight ahead stops inside its 8 cm tolerance, 0.43 m for 0.5 m, so walk straight lines); `{"to": [1, 1, 45]}` about 7 cm and 5° off. The session's steps are the README's "On the robot".

The session: HIM's walker example with `stand.yaml` first, then ours standing (`G1_TASK_COMMAND='[0,0,0]'`: with no task our image walks its default move, [[submission-image]]), then the moves above through HIM's launcher (preflight, dry run, real), checking leg odometry with a tape ([[safety]]).
