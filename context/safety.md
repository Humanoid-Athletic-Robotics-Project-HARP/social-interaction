---
name: safety
description: Read before changing how the walker stops or what makes it stop, adding a safety check, planning a real-robot session, or when HARP's safety layer comes up
metadata:
  type: reference
  updated: 2026-10-28T03:10:37Z
  covers: [ros2/src/harp_control/harp_control/gr00t/walk.py, ros2/src/harp_control/harp_control/remote.py]
---

On the real robot our code alone keeps the G1 up (Unitree's controller is off), so every protection lives in `Walker` (`gr00t/walk.py`), which `walk`, `move_distance` and `walker_node` all share. Add checks there, not in one program.

**What stops it** (the constants are in `walk.py`): B on the remote, alone or with anything else held (so `g1_ctrl`'s LT+B works too; read from `rt/lowstate`'s `wireless_remote` every 5 ms; the whole button map and its reasons: [[controls]]), Space or `o`, Ctrl-C, `rt/lowstate` late by over 0.1 s on the robot (0.5 s in the sim), pelvis tilt past 60° once the policy runs, or any exception. Before starting: Unitree's controller must be off (motion_switcher) and `mode_machine` must be 5. Every command is clipped to the joint limits, and the stand-up ramps over 3 s.

**How it stops:** everything goes through `Walker.stop(reason)` (`quit`, `ctrl_c`, `remote_estop`, `state_lost`, `fall`, `error`; `run()` returns it, the first three exit 0, the mains only map it to an exit code). It logs a `walker` `stop` event (failures other than a fall also log `error`, for [[run-logging]]'s outcome), damps (kp 0) for 1 s at 200 Hz even through Ctrl-C, and with `real` keeps damping until `o` or Ctrl-C (not Space, so a stop pressed twice doesn't end it), so the joints are never left uncommanded. Sim-tested mid-course with the old L2+B chord (same code path): press to `estop` in about 5 ms, outcome `stopped` (`2026-10-28_00-02-49Z_sim_one_box_estop-test`, pinned); the hold only as `Walker(real=True)` against the sim. The sim's remote: [[mujoco-sim]].

**Not yet built** (proposed 2026-10-27, in this order):
- Checks that call `stop()`: NaN or inf in state, IMU or action; a joint over a speed limit (set from recorded sim runs plus margin; GR00T uses 6 rad/s for arms); another program sending `rt/lowcmd`, both before starting and while running; motor temperature and `motorstate` error flags logged, not acted on, until their meaning is known.
- First-session tools: `walk --check` (reads and records state, never sends `rt/lowcmd`), an optional reduced-kp stand-up (GR00T can't balance at low gains), and a printed checklist to confirm with `--real`.
- Nav2's collision monitor, a separate stop at the `/cmd_vel` level ([[nav2-sim]]).

**HARP's safety layer (open question, blocks every robot run):** the "Lecture 2" PDF (not in the repo) publishes `JointCommand` to `/safety/precheck` and reads `RobotFeedback` on `/safety/policy_observation`. HARP's onboarding repo (github.com/Humanoid-Athletic-Robotics-Project-HARP/onboarding, read 2026-10-27) puts that in HIMRobotics' private `him-sim2real-management`: `./sim.sh` checks a linux/arm64 "submission container" publishing 29-joint `JointCommand` there, and `policy-safety prepare-g1` checks walker ONNX files. The same onboarding also runs Unitree's `g1_ctrl` straight on `rt/lowcmd` with an operator in charge, him only recording. Still to get from HARP: which path we must take, and him's `START_HERE.md`, `POLICY_GUIDE.md`, `CONNECTION_GUIDE.md`, `ONNX_BUNDLE.md`. If commands must go through him, `Walker.send()` is the one place that publishes them.

**Sessions** (HARP onboarding plus usual practice): an operator owns power, the gantry, the remote and stopping; someone else runs the laptop and the recording. Hang the robot (rope through the shoulder buckles, feet clear), L1+A (damping; it sinks, so lower it by hand), then L2+R2 (debug mode). Record a stationary run before any controller starts, prove it in the sim first, never run two motor controllers at once, small reversible motions first. Control computer on 192.168.123.222 by Ethernet. Stopping a recording doesn't stop the robot.

**What others do:** Unitree `g1_ctrl` (unitree_rl_mjlab): an FSM starting in Passive, LT+B to Passive from any state, Passive on lost state, refuses to start while another program sends `rt/lowcmd`; its fall check (`bad_orientation`) is written but always returns false. NVIDIA GR00T WBC: `JointSafetyMonitor` (arm speed over 6 rad/s shuts down, arm position limits) and a deploy checklist (sim first, check state with no commands, first test at 2–5× lower kp, an e-stop ready). humanoid_bridge: latches damping, and keeps sending it, on lost state, NaN, overspeed or a second `rt/lowcmd` sender. Go2 projects report motor power cut after about 20 ms without `rt/lowcmd`; unconfirmed for the G1, which is why we keep damping instead of going quiet.
