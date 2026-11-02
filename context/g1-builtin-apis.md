---
name: g1-builtin-apis
description: Read when you want a built-in G1 function (speech, LED, arm gestures, Unitree's odometry or SLAM) — service names, api ids, and whether it can work alongside HIM's walker
metadata:
  type: reference
  updated: 2026-11-02T22:23:54Z
  covers: []
---

From Unitree's official headers and docs. Each call is a `unitree_api/Request` (`api_id` + JSON `parameter`) on `/api/<service>/request`; client classes in unitree_sdk2 wrap this. None tested by us.

**Availability while HIM's walker runs** (HIM's harness releases Unitree's motion control through motion_switcher): anything that sits on Unitree's motion control (sport, arm, odometry, SLAM navigation) is probably unavailable. Voice and LED likely still work.

- **voice** (`unitree_sdk2/include/unitree/robot/g1/audio/`; Python `G1AudioClient` in `unitree_sdk2py/g1/audio/`, in our image's pinned commit): 1001 TTS `{"index","speaker_id","text"}`, 1003/1004 play/stop a PCM stream, 1005/1006 get/set volume, 1010 RGB LED `{"R","G","B"}`. 1002 ASR has an id but no client method. From Unitree's `VuiClient_Service` page and community code ([[listening]], [[talking]] for how we would use them):
  - TTS is offline; `speaker_id` 0 Chinese, 1 English (English may need a firmware update from Unitree support, sdk2_python issue #60); quality is poor. The Python client's `tts_index` stays 0 (a bug).
  - PlayStream takes 16 kHz mono 16-bit PCM; the same `stream_id` queues, a new one interrupts; PlayStop takes `app_name`. Unitree's examples send 3 s chunks, community code 50–200 ms.
  - Mic: one processed channel, 16 kHz mono s16le in 5120-byte packets, multicast `239.168.123.161:5555` from PC1 (join on PC2's 192.168.123.x interface). It flows only in the voice assistant's wake-up mode (switch: L1+L2 or the app; g1-audio-driver uses undocumented 1008 `{"mode":1}`, 1007 reads it). Some firmware (1.4.5–1.4.9) sends zeros (issue #143).
  - Unitree's ASR (offline, not streaming) publishes JSON on `rt/audio_msg`: text, angle 0–180, speaker_id, confidence, language, is_final. Wake-up mode answers "Hello Robot" with its own assistant.
- **arm** (`.../g1/arm/`, on `rt/arm_sdk`): 7106 ExecuteAction `{"action_id":n}` — 99 release, 11 two-hand kiss, 12 kiss, 15 hands up, 17 clap, 18 high five, 19 hug, 20 heart, 21 right heart, 22 reject, 23 right hand up, 24 x-ray, 25 face wave, 26 high wave, 27 shake hand. 7107 lists, 7108 runs a taught action by name, 7113 stops. Some actions may make the robot fall.
- **sport** (`unitree_ros2/example/src/include/g1/g1_loco_client.hpp`): 7105 SetVelocity `{"velocity":[vx,vy,omega],"duration":s}`; 7101 SetFsmId (0 ZeroTorque, 1 Damp, 2 Squat, 3 Sit, 4 StandUp, 500 Start); 7102 balance mode; 7103 swing height; 7104 stand height; 7106 SetTaskId (0 wave, 1 wave and turn, 2/3 handshake); 7001–7005 reads.
- **motion_switcher**: 1001 check, 1002 select, 1003 release the built-in controller (the robot goes limp unless something else commands the motors). Also `robot_state` (list/toggle services) and `vui` (volume, brightness).
- **Odometry** (`odometer_service_interface`): `SportModeState_` on `rt/odommodestate` (500 Hz) and `rt/lf/odommodestate` (20 Hz). Needs Unitree's motion control, so it's probably gone while HIM's walker runs (HIM calls it unverified); hence our leg odometry ([[planned-moves]]).
- **SLAM** (`slam_operate`): lidar SLAM on the MID-360; 1801/1802 start/end mapping, 1804 init pose, 1102 navigate (≤10 m, straight line), 1201/1202 pause/resume, 1901 close. Pose on `rt/unitree/slam_relocation/odom`. Navigation needs Unitree's walking controller; whether localization alone runs once Unitree's controller is released is unknown.

Related: [[g1-official-docs]].
