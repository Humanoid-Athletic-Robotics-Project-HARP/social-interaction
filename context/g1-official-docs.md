---
name: g1-official-docs
description: Links to Unitree's official G1 developer docs, plus key hardware facts confirmed there (PC2 IP and login, sensors)
metadata:
  type: reference
---

Index: https://support.unitree.com/home/en/G1_developer (collected 2026-10-24). The site renders with JavaScript, so WebFetch returns nothing. Use the in-app browser.

**Key facts from "About G1"**
- G1 EDU has two computers. PC1 runs Unitree's motion control and is closed to developers. PC2 (Jetson Orin NX 16GB, 2TB) is ours to develop on, at `192.168.123.164`, initial login `unitree` / `123`.
- Unitree deploys no services on the Jetson.
- Head sensors: Livox MID-360 lidar and Intel RealSense D435i depth camera.
- From "Architecture Description": modules talk over DDS, whose IDL is ROS 2 compatible. Developer interfaces are DDS (C++/Python), ROS 2, and GStreamer (video only).

**Pages** (all under https://support.unitree.com/home/en/G1_developer/)
- Operational guidance: `quick_start`, `remote_control`, `voice_assistant_instructions`, `audio_playback`
- Application development: `sdk_overview`, `architecture_description`, `get_sdk`, `quick_development`
- Basic motion: `basic_motion_routine`, `joint_motor_sequence`, `remote_control_data`
- Services: `dds_services_interface`, `robot_state_client_interface`, `basic_services_interface`, `sport_services_interface`, `odometer_service_interface`, `VuiClient_Service`, `lidar_services_interface`, `slam_navigation_services_interface`, `motion_witcher_service_interface` (sic)
- High-level motion: `rpc_routine`, `arm_control_routine`, `arm_action_interface`
- More cases: `rl_control_routine`, `ros2_communication_routine`, `dds_communication_routine`, `lidar_Instructions`, `depth_camera_instruction`
- Other: `debugging_specification`, `FAQ`, `common_istakes_and_definitions` (sic)

Related: [[unitree-resources]], [[project-overview]]
