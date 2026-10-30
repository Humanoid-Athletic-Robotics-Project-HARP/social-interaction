---
name: gr00t-wbc
description: Read when the walking policy comes up: what HIM's GR00T walker is, its contract, how it behaves, its license
metadata:
  type: reference
  updated: 2026-10-28T21:49:38Z
  covers: [submission/harp_node.py]
---

**What it is:** the lower-body policy from NVlabs/GR00T-WholeBodyControl ("Decoupled WBC"): two small ONNX MLPs, Balance (when |cmd| < 0.05) and Walk, for 15 joints (12 leg + 3 waist) at 50 Hz; arms hold neutral targets. In our image it is HIM's walker example (`NvidiaWalker`, contract `nvidia-decoupled-wbc-516-v1`, its `walker.yaml`, `LICENSE` and provenance), run by HIM's client, which `harp_node.py` steers with `/cmd_vel` ([[submission-image]]). ONNX hashes: Walk `7c82255b…`, Balance `f645da59…`.

**Contract:** 6 stacked frames of 86 values (cmd vx/vy/yaw rate scaled [2, 2, 0.5], height 0.74, torso rpy 0, gyro × 0.5, gravity, 29 joint positions and velocities × 0.05, last action), 516 inputs, action × 0.25 + defaults. NVIDIA's other `teleop/configs/g1_gear_wbc.yaml` (0.25 scales, 570 inputs) doesn't fit these files, and LeRobot uses those wrong scales. Gains come from HIM's joint profile for the walker (`submission/joint_profile.json`).

**Behavior in HIM's sim:** walks at roughly 85% of the command; drifts a few cm after it stops; standing, its heading creeps about 0.1°/s. HIM's GR00T balance stands it up and holds it before handover.

**License:** weights under the NVIDIA Open Model License: keep its text and the Notice, follow NVIDIA's Trustworthy AI terms, don't bypass safety guardrails.

**Not used:** GEAR-SONIC (whole-body motion tracking, maybe for gestures later) needs Linux, an NVIDIA GPU and exact TensorRT, and on the Orin a JetPack 6 reflash ([[unitree-resources]]). The full Decoupled WBC stack (teleop, IK) needs Ubuntu 22.04, a GPU and Docker.
