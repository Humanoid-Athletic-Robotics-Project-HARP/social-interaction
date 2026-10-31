---
name: waypoint-recorder
description: Read when changing waypoint capture, its terminal launcher, or using recorded routes across sessions
metadata:
  type: reference
  updated: 2026-10-31T09:34:41Z
  covers: [submission/harp/record_waypoints.py, submission/record_waypoints.sh, submission/tests/test_record_waypoints*.py]
---

`harp/record_waypoints.py` subscribes to `/odom` only. Its terminal commands and host launcher are in the README. The input thread queues whole lines while the main thread spins ROS and owns the captured points. Export uses `harp/route.py`'s existing schema; save uses a same-directory temporary file and an exclusive hard link, so existing files and symlinks stay intact.

Capture needs finite `odom` -> `base_link` data, a usable quaternion, and fresh receipt and ROS timestamps. A backward timestamp disables the recording. A publisher restarting with an unchanged clock can reset the origin without being detected: restart the recorder whenever odometry or the submission restarts. Routes belong to the recorded odom origin and orientation, not a persistent map. The launcher runs inside the active container to share its loopback DDS network and generated Fast DDS discovery profile, then copies the saved YAML to the host before removing its temporary files.

Validation on macOS, 2026-10-31: six data/file tests and two real ROS terminal tests passed in the retained Jazzy/Python 3.12 environment. The latter used synthetic odometry on a dedicated localhost domain and exercised capture, undo, save/load, and discard. Foxy's single-message subscription callback API was checked against upstream source. Docker, the Foxy image, and HIM's simulator were unavailable on this machine; the Docker launcher and target runtime still need an image/simulator check.
