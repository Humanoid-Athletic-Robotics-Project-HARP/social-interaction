---
name: unitree-resources
description: Key official and community repos for the G1, and how we depend on them (cloned next to the repo, not vendored)
metadata:
  type: reference
---

Research done 2026-10-24 for [[project-overview]].

**Official (github.com/unitreerobotics)**
- `unitree_ros2`: ROS 2 message packages (`unitree_go`, `unitree_hg` for humanoids/G1, `unitree_api`) plus examples. The robot speaks CycloneDDS, so ROS 2 talks to it directly. Tested on Ubuntu 20.04/Foxy and 22.04/Humble. The GitHub README calls Humble "recommended", but Unitree's G1 "Ros2 Communication Routine" doc says the examples were developed under Foxy. The robot's DDS is CycloneDDS 0.10.2. On Foxy you must build CycloneDDS 0.10.x and `rmw_cyclonedds` (foxy branch) in `cyclonedds_ws` with ROS *not* sourced; Humble skips that step. PC2 (Jetson Orin NX) most likely runs Ubuntu 20.04 + Foxy, based on community G1 guides and the Go2 EDU's JetPack 5 image, but this is not confirmed on our robot yet (check `ls /opt/ros` and `lsb_release -a`). Don't upgrade or reflash the Jetson to get Humble: it needs Unitree's custom BSP. Our current scripts don't use ROS 2. If they run on the Jetson, its Python is probably 3.8, and mujoco/onnxruntime wheels for it are untested.
- `unitree_sdk2` (C++, ships a prebuilt library) and `unitree_sdk2_python` (open, BSD-3): the main SDK. Our scripts use the Python one for DDS. Also has `LocoClient` (walking, closed sport service) and `AudioClient` (TTS/speaker).
- `unitree_mujoco`: simulator; we use only its G1 model ([[mujoco-sim]]). G1 uses `unitree_hg` messages.
- `unitree_rl_gym` and `unitree_rl_lab` (IsaacLab): RL locomotion training with sim2sim and sim2real for the G1.
- `xr_teleoperate`, `unitree_lerobot`: teleoperation and imitation learning.

**Community**
- Walking: NVlabs/GR00T-WholeBodyControl (we use its Decoupled WBC policy, see [[gr00t-wbc]]; also SONIC) and BeyondMimic.
- Voice: SaxionMechatronics/unitree_converse (on-robot STT, LLM, and TTS), experientialtech/g1-audio-driver. The mic streams over UDP multicast and has no official SDK API.
- Lidar: automaticdai/unitree-g1-navigation (FAST-LIO2 + Nav2 prototype, not validated on hardware), deepglint/FAST_LIO_LOCALIZATION_HUMANOID, norkraigh/Unitree-G1-SLAM.
- Index: shaoxiang/awesome-unitree-robots.

**Decision:** clone Unitree's repos next to ours as external dependencies; don't fork or vendor them. Write our own code in this repo. (Recommended to the team 2026-10-24.) `unitree_ros2` is currently unused, since our scripts talk DDS through unitree_sdk2_python.
