---
name: g1-builtin-apis
description: Read when you want a built-in G1 function (speech, LED, arm gestures, Unitree's odometry or SLAM) — service names, api ids, and whether it can work alongside our controller
metadata:
  type: reference
  updated: 2026-10-26T22:40:10Z
  covers: []
---

From Unitree's official headers and docs. Each call is a `unitree_api/Request` (`api_id` + JSON `parameter`) on `/api/<service>/request`; client classes in unitree_sdk2 wrap this. None tested by us.

**Availability with our controller (debug mode):** anything that sits on Unitree's motion control (sport, arm, odometry, SLAM navigation) is probably unavailable. Voice and LED likely still work.

- **voice** (`unitree_sdk2/include/unitree/robot/g1/audio/`): 1001 TTS `{"index","speaker_id","text"}`, 1003/1004 play/stop a PCM stream, 1005/1006 get/set volume, 1010 RGB LED `{"R","G","B"}`. 1002 ASR has an id but no client method.
- **arm** (`.../g1/arm/`, on `rt/arm_sdk`): 7106 ExecuteAction `{"action_id":n}` — 99 release, 11 two-hand kiss, 12 kiss, 15 hands up, 17 clap, 18 high five, 19 hug, 20 heart, 21 right heart, 22 reject, 23 right hand up, 24 x-ray, 25 face wave, 26 high wave, 27 shake hand. 7107 lists, 7108 runs a taught action by name, 7113 stops. Some actions may make the robot fall.
- **sport** (`unitree_ros2/example/src/include/g1/g1_loco_client.hpp`): 7105 SetVelocity `{"velocity":[vx,vy,omega],"duration":s}`; 7101 SetFsmId (0 ZeroTorque, 1 Damp, 2 Squat, 3 Sit, 4 StandUp, 500 Start); 7102 balance mode; 7103 swing height; 7104 stand height; 7106 SetTaskId (0 wave, 1 wave and turn, 2/3 handshake); 7001–7005 reads.
- **motion_switcher**: 1001 check, 1002 select, 1003 release the built-in controller (the robot goes limp unless something else commands the motors). Also `robot_state` (list/toggle services) and `vui` (volume, brightness).
- **Odometry** (`odometer_service_interface`): `SportModeState_` on `rt/odommodestate` (500 Hz) and `rt/lf/odommodestate` (20 Hz). Needs Unitree's motion control, so it's gone while ours runs; hence our leg odometry ([[move-distance]]).
- **SLAM** (`slam_operate`): lidar SLAM on the MID-360; 1801/1802 start/end mapping, 1804 init pose, 1102 navigate (≤10 m, straight line), 1201/1202 pause/resume, 1901 close. Pose on `rt/unitree/slam_relocation/odom`. Navigation needs Unitree's walking controller; whether localization alone runs in debug mode is unknown.

Related: [[g1-official-docs]].
