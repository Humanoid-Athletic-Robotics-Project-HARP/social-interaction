---
name: robot-status
description: Read before running or changing testground/robot/status_server.py or the dashboard's Robot status view, or when reading the G1's battery, motor temperatures or HARP's harness state
metadata:
  type: reference
  updated: 2026-11-11T18:23:16Z
  covers: [testground/robot/**, viewer/robot.js, viewer/feeds.py]
---

`testground/robot/status_server.py` on PC2 serves `/status` on port 28735; the dashboard's `pc2-robot` source (`G1_HOST`) polls it each second and `viewer/robot.js` draws it ([[unified-viewer]]). It only subscribes, at nice 10.

Where each reading comes from (as HARP's harness reads them, its `robot_safety/unitree_transport.py` and `feedback.py`):
- Battery: `rt/lf/bmsstate`, `unitree_hg` `BmsState_`, field `soc` (0 to 100). Other BMS fields are unread and unverified.
- Motor temperatures: `rt/lowstate` `motor_state[i].temperature`, two values per motor: casing, then winding. The first 29 of 35 slots are the G1's joints, in `submission/joint_profile.json`'s order.
- Harness: `/safety/hardware_status`, a JSON `std_msgs/String`, reliable, depth 1, on HARP's ROS 2 network (domain 97, localhost, Fast DDS), sent on change and at least every 0.5 s. The script keeps a few fields (`HARNESS`); HIM's `read_hardware_status.py` reads the same topic.
- Limits shown are where the harness stops the robot (its `robot_safety/joint_checks.py`): casing 85 °C, winding 120 °C, battery 10%; it also stops on a battery reading older than 5 s. The page marks 80% of a limit amber.

The hub calls motors stale past 1 s, the battery past 5 s, the harness past 2 s; the harness is optional (only during a session), so its absence leaves the source Live.

Unverified (as of 2026-11-09, only tested with fake messages): that our robot publishes `rt/lf/bmsstate`, the CPU cost of Python callbacks on `rt/lowstate` at about 1 kHz beside the harness, and `rclpy` with Unitree's CycloneDDS in one process (the hybrid viewer's `--source ros` does a similar mix, [[robot-pc2]]). Check `top` on the first run.
