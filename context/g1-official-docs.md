---
name: g1-official-docs
description: Read when you need a Unitree G1 developer doc page, or the robot's network address, login and sensors
metadata:
  type: reference
  updated: 2026-10-26T22:40:10Z
  covers: []
---

Index: https://support.unitree.com/home/en/G1_developer. The site renders with JavaScript, so WebFetch returns nothing; use the in-app browser.

**Hardware facts**
- PC1 runs Unitree's motion control and is closed. PC2 (Jetson Orin NX 16GB) is ours, at `192.168.123.164`, initial login `unitree` / `123`. Unitree deploys no services on it.
- Head sensors: Livox MID-360 lidar, Intel RealSense D435i depth camera.
- Interfaces: DDS (ROS 2-compatible IDL, C++/Python), ROS 2, GStreamer (video).

**Pages** (append to `https://support.unitree.com/home/en/G1_developer/`)
- Operation: `quick_start`, `remote_control`, `voice_assistant_instructions`, `audio_playback`
- Development: `sdk_overview`, `architecture_description`, `get_sdk`, `quick_development`
- Motion: `basic_motion_routine`, `joint_motor_sequence`, `remote_control_data`, `rpc_routine`, `arm_control_routine`, `arm_action_interface`, `rl_control_routine`
- Services: `dds_services_interface`, `robot_state_client_interface`, `basic_services_interface`, `sport_services_interface`, `odometer_service_interface`, `VuiClient_Service`, `lidar_services_interface`, `slam_navigation_services_interface`, `motion_witcher_service_interface` (sic)
- Sensors and comms: `lidar_Instructions`, `depth_camera_instruction`, `ros2_communication_routine`, `dds_communication_routine`
- Other: `debugging_specification`, `FAQ`, `common_istakes_and_definitions` (sic)
