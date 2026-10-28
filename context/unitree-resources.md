---
name: unitree-resources
description: Read when looking for G1 code to build on, or deciding the ROS 2 / JetPack version for the robot's Jetson (PC2)
metadata:
  type: reference
  updated: 2026-10-28T00:25:23Z
  covers: [README.md, ros2/pixi.toml]
---

**How we depend on Unitree:** their repos are cloned next to ours at pinned commits (README "Setup"), never forked or vendored. We use `unitree_sdk2_python` for DDS, `unitree_mujoco` only for the G1 model, and `unitree_ros2` only for its message packages (`unitree_hg` is the G1's), built by `pixi run build` ([[ros2-mac]]).

**Other official repos (github.com/unitreerobotics):** `unitree_sdk2` (C++), `unitree_rl_gym` / `unitree_rl_lab` (RL training with sim2sim and sim2real), `xr_teleoperate`, `unitree_lerobot`. The robot's DDS is CycloneDDS 0.10.2.

**Community:**
- Walking: NVlabs/GR00T-WholeBodyControl (ours, [[gr00t-wbc]]), BeyondMimic.
- Voice: SaxionMechatronics/unitree_converse (on-robot STT → LLM → TTS), experientialtech/g1-audio-driver. The mic streams over UDP multicast with no official SDK API.
- Lidar and Nav2: see [[lidar-nav2-research]].
- Index: shaoxiang/awesome-unitree-robots.

**ROS 2 on PC2 (Jetson Orin NX):** depends on its JetPack image. JetPack 5 → Ubuntu 20.04 → Foxy only; Unitree's G1 JetPack 6.2 image (`g1-nx-j6.2.img.bz2`) → 22.04 → Humble; Jazzy needs 24.04, so a container. Community G1 work uses Humble or Jazzy, mostly on a separate PC. Our robot is unchecked: run `ls /opt/ros`, `lsb_release -a`, `cat /etc/nv_tegra_release` on PC2. Its system Python (3.8 or 3.10) is older than our 3.12.

**Reflashing PC2** erases it and may void the warranty: ask the team first, and use only Unitree's G1 image.
