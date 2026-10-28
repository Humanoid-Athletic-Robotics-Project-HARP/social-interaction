---
name: unitree-resources
description: Read when looking for G1 code to build on, or deciding the ROS 2 / JetPack version for the robot's Jetson (PC2)
metadata:
  type: reference
  updated: 2026-10-28T09:01:40Z
  covers: [submission/Dockerfile]
---

**How we depend on Unitree:** never forked or vendored. Our image installs `unitree_sdk2_python` at a pinned commit (`submission/Dockerfile`) for the lidar relay; the G1 model and the robot's low-level I/O are HIM's ([[him-sim2real]]).

**Other official repos (github.com/unitreerobotics):** `unitree_sdk2` (C++), `unitree_rl_gym` / `unitree_rl_lab` (RL training with sim2sim and sim2real), `xr_teleoperate`, `unitree_lerobot`. The robot's DDS is CycloneDDS 0.10.2.

**Community:**
- Walking: NVlabs/GR00T-WholeBodyControl (ours, [[gr00t-wbc]]), BeyondMimic.
- Voice: SaxionMechatronics/unitree_converse (on-robot STT → LLM → TTS), experientialtech/g1-audio-driver. The mic streams over UDP multicast with no official SDK API.
- Lidar and Nav2: see [[lidar-nav2-research]].
- Index: shaoxiang/awesome-unitree-robots.

**ROS 2 on PC2 (Jetson Orin NX):** depends on its JetPack image. JetPack 5 → Ubuntu 20.04 → Foxy only; Unitree's G1 JetPack 6.2 image (`g1-nx-j6.2.img.bz2`) → 22.04 → Humble; Jazzy needs 24.04, so a container. HIM's harness runs on PC2 with ROS 2 Foxy (Python 3.8), so our robot is presumably on JetPack 5; our image is Foxy too, so the robot's own ROS 2 doesn't matter to it. Check with `ls /opt/ros`, `lsb_release -a`, `cat /etc/nv_tegra_release` on PC2.

**Reflashing PC2** erases it and may void the warranty: ask the team first, and use only Unitree's G1 image.
