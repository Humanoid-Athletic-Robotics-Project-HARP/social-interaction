---
name: unitree-resources
description: Key official and community repos for the G1, and how we depend on them (cloned next to the repo, not vendored)
metadata:
  type: reference
  updated: 2026-10-26T21:32:46Z
  covers: [README.md, ros2/pixi.toml]
---

Research done 2026-10-24 for [[project-overview]].

**Official (github.com/unitreerobotics)**
- `unitree_ros2`: ROS 2 message packages (`unitree_go`, `unitree_hg` for humanoids/G1, `unitree_api`, in `cyclonedds_ws/src/unitree/`) plus examples. The robot speaks CycloneDDS, so ROS 2 talks to it directly. Tested on Ubuntu 20.04/Foxy and 22.04/Humble. The GitHub README calls Humble "recommended", but Unitree's G1 "Ros2 Communication Routine" doc says the examples were developed under Foxy. The robot's DDS is CycloneDDS 0.10.2. On Foxy you must build CycloneDDS 0.10.x and `rmw_cyclonedds` (foxy branch) in `cyclonedds_ws` with ROS *not* sourced; Humble skips that step. Our walking code talks DDS through unitree_sdk2_python; `walker_node` (`harp_ros`) is our only ROS 2 code ([[ros2-mac]]).

**ROS 2 version for the G1** (updated 2026-10-25): plan for **Humble or Jazzy**. A teammate found that most online sources use one of those with the G1, and community G1 work agrees (for example jsk-ros-pkg/g1_ros builds on Jazzy). Those sources run ROS 2 on the development computer or an added PC. What PC2 (Jetson Orin NX) itself runs depends on its JetPack image:
- JetPack 5 means Ubuntu 20.04, where only Foxy has binaries. One forum user reports their stock G1 as Foxy + 20.04 + JetPack 5.
- Unitree's G1 JetPack 6.2 image (`g1-nx-j6.2.img.bz2`, linked from NVIDIA's GR00T-WholeBodyControl JetPack 6 guide) gives Ubuntu 22.04, where Humble fits.
- Jazzy needs Ubuntu 24.04, which no PC2 image offers, so on PC2 it would run in a container.

Our robot hasn't been checked: run `ls /opt/ros`, `lsb_release -a` and `cat /etc/nv_tegra_release` on PC2. Reflashing erases PC2 and may void the warranty, so ask the team first, and use only Unitree's G1 image (a generic Orin NX image lacks the carrier-board config). PC2's system Python is 3.8 on 20.04 and 3.10 on 22.04, older than the 3.12 our pins need, so running our scripts there is untested.
- `unitree_sdk2` (C++, ships a prebuilt library) and `unitree_sdk2_python` (open, BSD-3): the main SDK. Our scripts use the Python one for DDS. Also has `LocoClient` (walking, closed sport service) and `AudioClient` (TTS/speaker).
- `unitree_mujoco`: simulator; we use only its G1 model ([[mujoco-sim]]). G1 uses `unitree_hg` messages.
- `unitree_rl_gym` and `unitree_rl_lab` (IsaacLab): RL locomotion training with sim2sim and sim2real for the G1.
- `xr_teleoperate`, `unitree_lerobot`: teleoperation and imitation learning.

**Community**
- Walking: NVlabs/GR00T-WholeBodyControl (we use its Decoupled WBC policy, see [[gr00t-wbc]]; also SONIC) and BeyondMimic.
- Voice: SaxionMechatronics/unitree_converse (on-robot STT, LLM, and TTS), experientialtech/g1-audio-driver. The mic streams over UDP multicast and has no official SDK API.
- Lidar: automaticdai/unitree-g1-navigation (FAST-LIO2 + Nav2 prototype, not validated on hardware), deepglint/FAST_LIO_LOCALIZATION_HUMANOID, norkraigh/Unitree-G1-SLAM.
- Index: shaoxiang/awesome-unitree-robots.

**Decision:** clone Unitree's repos next to ours as external dependencies; don't fork or vendor them. Write our own code in this repo. (Recommended to the team 2026-10-24.) The repo doesn't use `unitree_ros2`: our scripts talk DDS through unitree_sdk2_python. Its message packages are built by `ros2/` (`pixi run build`, [[ros2-mac]]).
