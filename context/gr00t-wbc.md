---
name: gr00t-wbc
description: Read when changing the walking controller (harp_control's gr00t/walk.py), its policy config or ONNX files, or preparing it for the real robot
metadata:
  type: reference
  updated: 2026-10-28T00:45:04Z
  covers: [ros2/src/harp_control/harp_control/gr00t/**, ros2/src/harp_control/harp_control/leg_odometry.py]
---

**What it is:** `harp_control/gr00t/walk.py` runs the lower-body policy from NVlabs/GR00T-WholeBodyControl ("Decoupled WBC"): two small ONNX MLPs, Balance (used when |cmd| < 0.05) and Walk, controlling 15 joints (12 leg + 3 waist) at 50 Hz. Input: 6 stacked frames of 86 values (cmd vx/vy/yaw rate, base height 0.74, torso rpy 0, gyro, gravity, all 29 joint positions and velocities, last action). Arms are held at NVIDIA's `loco_upper_body_dof_pos`.
- `Gr00tPolicy` mirrors NVIDIA's `G1GearWbcPolicy` (observation layout, history, Balance/Walk switch, action × 0.25 + defaults). DDS I/O, the 200 Hz loop, stand-up ramp, safety checks and damping are ours.
- ONNX files are vendored in `gr00t/policy/`; `NOTICE.txt` has the source commit and sha256.
- Config matches NVIDIA's real-robot deploy, `sim2mujoco/resources/robots/g1/g1_gear_wbc.yaml` (ang_vel scale 0.5, cmd scale [2, 2, 0.5], 516 inputs) plus MOTOR_KP/KD from `g1_29dof_gear_wbc.yaml`. The other `teleop/configs/g1_gear_wbc.yaml` (0.25 scales, 570 inputs) doesn't fit these ONNX files, and LeRobot uses those wrong scales.

**Behavior in the sim:** walks at about 85% of the commanded speed. The policy must be running with the feet on the floor before the band releases: started in the air it flails, and without it the robot falls. On release it often steps back and turns 5–15° while catching itself.

**Licenses:** code Apache-2.0 (keep `LICENSE-GR00T-WholeBodyControl.txt`). Weights under the NVIDIA Open Model License: keep its text and the Notice line, follow NVIDIA's Trustworthy AI terms, don't bypass safety guardrails.

**Real robot:** Unitree's controller must be off (gantry, L1+A, L2+R2), so our code alone keeps the robot up. The scripts check motion_switcher and `mode_machine` 5 (29-joint rev 1.0, NVIDIA's target). Low-level command flags (`mode_pr` 0, motor `mode` 1, `mode_machine` copied from lowstate, CRC) match Unitree's G1 low-level example. State timeout is 0.1 s on the robot, 0.5 s in the sim (macOS throttles the sim window).
- How it stops, and every check that stops it: [[safety]].

**Not used:** GEAR-SONIC (whole-body motion tracking, maybe for gestures later) needs Linux, an NVIDIA GPU and exact TensorRT, and on the Orin a JetPack 6 reflash ([[unitree-resources]]). The full Decoupled WBC stack (teleop, IK) needs Ubuntu 22.04, a GPU and Docker.
