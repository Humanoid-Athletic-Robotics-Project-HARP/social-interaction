---
name: gr00t-wbc
description: NVIDIA GR00T whole-body control for the G1: what exists, licenses, Mac/sim/real-robot paths (research 2026-10-25)
metadata:
  type: reference
---

Researched 2026-10-25 after the mentors suggested GR00T (see [[project-overview]]). Not tried yet.

**NVlabs/GR00T-WholeBodyControl** (about 3.8k stars, active as of 2026-09). Code is Apache-2.0; weights are under the NVIDIA Open Model License. It targets the Unitree G1 and holds two things:
- **Decoupled WBC**, the one that fits "walk and balance". An RL lower-body policy controls 15 joints (12 legs plus 3 waist) at 50 Hz. Its inputs are a velocity command (vx, vy, yaw rate), a base height (default 0.74) and a torso rpy. It is two small ONNX MLPs, Balance (used when the command is near zero) and Walk. Inputs stack 6 frames of 86 values. The upper body is left to IK or teleop. The official files are `decoupled_wbc/sim2mujoco/resources/robots/g1/policy/GR00T-WholeBodyControl-{Balance,Walk}.onnx` (git LFS), and `decoupled_wbc/sim2mujoco/scripts/run_mujoco_gear_wbc.py` is a pure-Python MuJoCo runner (mujoco, onnxruntime, torch, pynput). The full Decoupled WBC stack needs Ubuntu 22.04, an NVIDIA GPU and Docker.
- **GEAR-SONIC**, a motion-tracking foundation policy (whole-body motion, teleop, VLA). Deploying it takes a C++ program with an exact TensorRT version, so an NVIDIA GPU on Linux. Running it on the G1's own Orin needs JetPack 6, which means reflashing (see [[unitree-resources]]: don't reflash lightly). Its sim side runs under mjpython on a Mac, but the deploy program has to run on Linux.

**LeRobot already implements GR00T for the G1 29-DoF** (Apache-2.0), in `src/lerobot/robots/unitree_g1/controllers/gr00t_locomotion.py` (about 200 lines). It uses CPU onnxruntime and talks to the robot or sim through unitree_sdk2py on `rt/lowcmd` and `rt/lowstate`, the same code for sim and real, run from a laptop or on the robot. It downloads the ONNX files from a third-party Hugging Face re-host (`nepyope/GR00T-WholeBodyControl_g1`), so prefer NVIDIA's copy. It also has Holosoma and SONIC controllers.

**Vendoring into our repo is allowed.** Apache-2.0 code needs its license and notices kept. The weights need a copy of the NVIDIA Open Model License and a Notice file saying "Licensed by NVIDIA Corporation under the NVIDIA Open Model License". Use must follow NVIDIA's Trustworthy AI terms, and the license ends if safety guardrails are bypassed.

**Real-robot consequences of running our own policy.** Unitree's controller has to be off (debug mode: the robot hanging and damped, then L2+R2, or a motion_switcher release), so our code is what keeps the robot up. Services that sit on top of the sport controller (LocoClient, the arm gestures over `rt/arm_sdk`, probably `lf/odommodestate` odometry) likely won't be available, so walking a set distance needs our own state estimate. Motor firmware and PC1's low-level relay stay closed no matter what. Check `lowstate.mode_machine` on our robot; 5 means 29-DoF rev 1.0.
