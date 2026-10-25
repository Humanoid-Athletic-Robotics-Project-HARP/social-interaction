---
name: gr00t-wbc
description: NVIDIA GR00T Decoupled WBC as our G1 walking controller (gr00t/walk.py): where it comes from, config choices, licenses, real-robot consequences
metadata:
  type: reference
---

**What we run:** `gr00t/walk.py`, implemented 2026-10-25. It uses the lower-body policy from NVIDIA's NVlabs/GR00T-WholeBodyControl ("Decoupled WBC"), which is two small ONNX MLPs: Balance, used when |cmd| < 0.05, and Walk. They control 15 joints (12 leg plus 3 waist) at 50 Hz. Inputs are 6 stacked frames of 86 values: the command (vx, vy, yaw rate), base height (0.74), torso rpy (0), gyro, gravity, all 29 joint positions and velocities, and the last action. Arms are held at NVIDIA's `loco_upper_body_dof_pos`. This matches our 29-joint G1 exactly.
- The ONNX files are vendored in `gr00t/policy/`, copied from `decoupled_wbc/sim2mujoco/resources/robots/g1/policy/`. NOTICE.txt has the pinned commit and sha256.
- Config follows NVIDIA's real-robot deploy, not its sim2mujoco yaml: scales from `g1_gear_wbc.yaml` (ang_vel 0.5, cmd [2, 2, 0.5]) and gains from `g1_29dof_gear_wbc.yaml` (MOTOR_KP/KD). LeRobot's `gr00t_locomotion.py` uses 0.25 scales, which differ.
- In `sim/run_sim.py` ([[mujoco-sim]]) it stands, walks at about 85% of the commanded speed, and turns. Start-up order matters: policy on with the feet on the floor, then release. Started in the air it flails; released without it, the robot falls. `move_distance.py` adds leg odometry, which reads about 7% short on distance, and holds each walk's starting heading (about 2° in the sim). On band release the robot often steps back and turns 5–15° while catching itself. Not run on the robot yet.

**Licenses:** code is Apache-2.0 (keep `gr00t/LICENSE-GR00T-WholeBodyControl.txt`). The weights are under the NVIDIA Open Model License: keep its text and the Notice line "Licensed by NVIDIA Corporation under the NVIDIA Open Model License", follow NVIDIA's Trustworthy AI terms, and don't bypass safety guardrails, or the license ends.

**Real robot:** Unitree's controller must be off (hang, L1+A damping, then L2+R2 debug mode), so our code is all that keeps the robot up. The scripts check motion_switcher and `mode_machine` (5 means 29-joint rev 1.0, NVIDIA's target). Services that need Unitree's controller (LocoClient, arm gestures over `rt/arm_sdk`, probably `lf/odommodestate`) are likely unavailable. Motor firmware and PC1 stay closed regardless.

**Other options in the same repo, not used:** GEAR-SONIC, a whole-body motion-tracking policy (possibly useful later for gestures). Deploying it needs Linux, an NVIDIA GPU and an exact TensorRT version, and running it on the G1's Orin means reflashing it to JetPack 6, which we avoid ([[unitree-resources]]). The full Decoupled WBC stack (teleop, IK) needs Ubuntu 22.04, an NVIDIA GPU and Docker. LeRobot also has Python G1 controllers (GR00T, Holosoma, SONIC), but it fetches the GR00T ONNX from a third-party Hugging Face re-host.
