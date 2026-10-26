---
name: g1-builtin-apis
description: The G1's built-in high-level functions (walking, posture, arm gestures, speech, LED), with API ids, read from Unitree's official headers
metadata:
  type: reference
---

Read from the official headers on 2026-10-25. We no longer walk with these (see [[gr00t-wbc]]). The ones that sit on Unitree's controller (sport, arm) are probably unavailable while our controller runs in debug mode; voice and LED likely still work, but that's untested. Every call is a `unitree_api/Request` with an `api_id` and a JSON `parameter` string, sent on `/api/<service>/request`.

**Walking: `sport` service** (`unitree_ros2/example/src/include/g1/g1_loco_client.hpp`; the only client the ROS 2 examples wrap)
- 7105 SetVelocity `{"velocity":[vx,vy,omega],"duration":s}`. `StopMove` = all zeros. `Move(..., continuous)` sets duration to 864000.
- 7101 SetFsmId `{"data":id}`: 0 ZeroTorque, 1 Damp, 2 Squat, 3 Sit, 4 StandUp, 500 Start (the ROS 2 header uses 500; older SDK versions may use 200).
- 7102 SetBalanceMode (0 BalanceStand, 1 ContinuousGait, i.e. keep stepping while standing still). 7103 SetSwingHeight. 7104 SetStandHeight (HighStand/LowStand use the max/min value). 7107 SetSpeedMode.
- 7106 SetTaskId: 0 wave, 1 wave and turn, 2 and 3 handshake stages.
- 7001–7005: reads for FSM id/mode, balance mode, swing height, stand height.

**Arm gestures: `arm` service** (`unitree_sdk2/include/unitree/robot/g1/arm/`). 7106 ExecuteAction `{"action_id":n}`. Presets: 99 release, 11 two-hand kiss, 12 kiss, 15 hands up, 17 clap, 18 high five, 19 hug, 20 heart, 21 right heart, 22 reject, 23 right hand up, 24 x-ray, 25 face wave, 26 high wave, 27 shake hand. 7108 runs a taught custom action by name, 7113 stops it, 7107 lists actions. Built on `rt/arm_sdk`. The header warns that some actions may make the robot fall.

**Voice: `voice` service** (`unitree_sdk2/include/unitree/robot/g1/audio/`). 1001 TTS `{"index","speaker_id","text"}`, 1003/1004 play/stop a PCM stream, 1005/1006 get/set volume, 1010 RGB LED `{"R","G","B"}`. 1002 ASR exists as an id but has no client method.

**Other services** (each has a client class in unitree_sdk2; the G1 docs list pages for all but `agv`)
- `motion_switcher` (MotionSwitcherClient, in the b2 folder): 1001 check, 1002 select, 1003 release the built-in controller. Releasing it hands the joints to low-level control, so the robot goes limp unless something else is commanding the motors.
- `robot_state` (RobotStateClient, go2 folder): list services and turn them on or off.
- `vui` (VuiClient, go2 folder): volume and brightness switches.
- `agv` (G1AgvClient): move and height adjust, apparently for the wheeled G1-D (see `example/g1/g1d`). Not ours.
- The docs also list lidar, SLAM navigation and odometer service pages. These have no headers in unitree_sdk2.

**Odometry: `odometer_service_interface` page** (read 2026-10-25). The "State Estimator" service publishes `SportModeState_` (unitree_go) on `rt/odommodestate` (500 Hz) and `rt/lf/odommodestate` (20 Hz): position, velocity, rpy, yaw rate, quaternion. The frame's origin is the ground point under the base at start-up, with x forward. The docs say it "relies on the motion control system" and can't be used in development (low-level) mode, so it is gone whenever our controller runs. Its algorithm isn't documented. In unitree_sdk2_python issue #135 (unanswered), a user reports that its velocity stays flat when the hung robot is pushed. That suggests leg kinematics with contact from the controller, not IMU integration.

**SLAM and navigation: `slam_operate` service** (read 2026-10-25). Lidar SLAM on the MID-360 and its own IMU (`rt/utlidar/cloud_livox_mid360`, `rt/utlidar/imu_livox_mid360`), enabled from the App (`unitree_slam`, `lidar_driver`). The APIs are 1801/1802 to start and end mapping (saves a .pcd), 1804 to initialize the pose in a map, 1102 to navigate to a pose (at most 10 m away, in a straight line), 1201/1202 to pause and resume, and 1901 to close. The pose comes on `rt/unitree/slam_relocation/odom` (nav_msgs Odometry) and in `rt/slam_info` JSON. Its status shows a `"ctrName":"pid"` follower that outputs vx, vy and vyaw. The docs scope it to static, feature-rich indoor floors under 45 m. Navigation needs Unitree's walking controller. Whether localization alone runs in debug mode is untested.

"Clients" are helper classes that run on our side. Each one knows its service name and api ids, builds the JSON, and waits for the matching response. Streams such as `rt/lowcmd`, `rt/lowstate`, `rt/arm_sdk` and odometry are topics, not services.

Calling the arm and voice services over raw ROS 2 (for example `/api/arm/request`) follows the same pattern but has not been tested. Related: [[g1-official-docs]], [[unitree-resources]].
