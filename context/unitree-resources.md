---
name: unitree-resources
description: Key official and community repos for the G1, and the decision on how to use unitree_ros2
metadata:
  type: reference
---

Research done 2026-10-24 for [[project-overview]].

**Official (github.com/unitreerobotics)**
- `unitree_ros2`: ROS 2 message packages (`unitree_go`, `unitree_hg` for humanoids/G1, `unitree_api`) plus examples. The robot speaks CycloneDDS, so ROS 2 talks to it directly. Tested on Ubuntu 20.04/Foxy and 22.04/Humble. The GitHub README calls Humble "recommended", but Unitree's G1 "Ros2 Communication Routine" doc says the examples were developed under Foxy. The robot's DDS is CycloneDDS 0.10.2. On Foxy you must build CycloneDDS 0.10.x and `rmw_cyclonedds` (foxy branch) in `cyclonedds_ws` with ROS *not* sourced; Humble skips that step. PC2 (Jetson Orin NX) most likely runs Ubuntu 20.04 + Foxy, based on community G1 guides and the Go2 EDU's JetPack 5 image, but this is not confirmed on our robot yet (check `ls /opt/ros` and `lsb_release -a`). Don't upgrade or reflash the Jetson to get Humble: it needs Unitree's custom BSP. `move_distance.py` runs on Foxy and Python 3.8 as written.
- `unitree_sdk2` (C++) and `unitree_sdk2_python`: the main SDK. Includes the high-level `LocoClient` (walking) and `AudioClient` (TTS/speaker).
- `unitree_mujoco`: simulator. G1 uses `unitree_hg` messages.
- `unitree_rl_gym` and `unitree_rl_lab` (IsaacLab): RL locomotion training with sim2sim and sim2real for the G1.
- `xr_teleoperate`, `unitree_lerobot`: teleoperation and imitation learning.

**Community**
- Walking: NVlabs/GR00T-WholeBodyControl (SONIC; G1 checkpoints and a C++/TensorRT deploy) and BeyondMimic.
- Voice: SaxionMechatronics/unitree_converse (on-robot STT, LLM, and TTS), experientialtech/g1-audio-driver. The mic streams over UDP multicast and has no official SDK API.
- Lidar: automaticdai/unitree-g1-navigation (FAST-LIO2 + Nav2 prototype, not validated on hardware), deepglint/FAST_LIO_LOCALIZATION_HUMANOID, norkraigh/Unitree-G1-SLAM.
- Index: shaoxiang/awesome-unitree-robots.

**Decision:** Do not fork or vendor `unitree_ros2`. Clone it as an external dependency and build its message packages. Write our own packages in this repo. (Recommended to the user 2026-10-24; not yet confirmed.)
