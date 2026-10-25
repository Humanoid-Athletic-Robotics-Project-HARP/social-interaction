---
name: unitree-resources
description: Key official and community repos for the G1, and the decision on how to use unitree_ros2
metadata:
  type: reference
---

Research done 2026-10-24 for [[project-overview]].

**Official (github.com/unitreerobotics)**
- `unitree_ros2`: ROS 2 message packages (`unitree_go`, `unitree_hg` for humanoids/G1, `unitree_api`) plus examples. The robot speaks CycloneDDS, so ROS 2 talks to it directly. Targets Ubuntu 22.04/Humble (recommended) and 20.04/Foxy.
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
