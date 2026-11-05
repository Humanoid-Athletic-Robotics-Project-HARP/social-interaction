---
name: waypoint-recorder
description: Read when changing waypoint capture, its terminal launcher, or using recorded routes across sessions
metadata:
  type: reference
  updated: 2026-11-04T23:51:18Z
  covers: [submission/harp/record_waypoints.py, submission/record_waypoints.sh, submission/tests/test_record_waypoints*.py]
---

`harp/record_waypoints.py` subscribes to `/odom` only. Its terminal commands and host launcher are in the README. The input thread queues whole lines while the main thread spins ROS and owns the captured points. Export uses `harp/route.py`'s existing schema; save uses a same-directory temporary file and an exclusive hard link, so existing files and symlinks stay intact.

Capture needs finite `odom` -> `base_link` data, a usable quaternion, and fresh receipt and ROS timestamps. A backward timestamp disables the recording. A publisher restarting with an unchanged clock can reset the origin without being detected: restart the recorder whenever odometry or the submission restarts. Routes belong to the recorded odom origin and orientation, not a persistent map. The launcher runs inside the active container to share its loopback DDS network and generated Fast DDS discovery profile, then copies the saved YAML to the host before removing its temporary files.

**Tests:** the interactive ROS tests run in the image's test command (README "Tests") with `-e ROS_LOCALHOST_ONLY=1 -e ROS_DOMAIN_ID=179` added, a local domain apart from the robot's. The data and file checks need no ROS: `PYTHONPATH=submission python3 -m unittest discover -s submission/tests -p test_record_waypoints.py`.

Validated 2026-10-31 by data, file and ROS terminal tests (synthetic odometry, ROS 2 Jazzy, not Foxy); the Docker launcher and the Foxy image still need a check in the image and HIM's sim.
