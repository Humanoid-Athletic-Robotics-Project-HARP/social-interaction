---
name: g1-sensor-fusion
description: Read before combining the head lidar and the head camera's depth (a fused cloud or costmap, seeing ahead of the robot) — the frame, the offsets, and how well they fit
metadata:
  type: project
  updated: 2026-11-02T22:04:05Z
  covers: [testground/hybrid/*]
---

**Why:** the lidar is blind on the floor within about 1 m, and ahead too with the face
visor on (a cone of about 120°, −60° to +66°; off, it sees ahead but not the wedges either side:
[[g1-face-visor]]) ([[g1-lidar-measured]]); the camera looks down at exactly that
([[g1-head-camera]]). Together they see all around, near the feet included, except
far ahead (past about 2.7 m) and anything above about 0.8 m at 1 m ahead.

**How `testground/hybrid/hybrid_viewer.py` puts them together:** one frame with x
forward, y left, z up, the origin on the floor below the lidar. Each sensor is
levelled on its own floor fit (removes the torso's lean and any tilt or height error
of either mount); the camera is then moved by its offset from the lidar in Unitree's
`g1_29dof_rev_1_0.urdf` (5.7 cm ahead, 1.8 cm left; same heading). No time sync:
each shows its newest data (lidar the last 0.3 s, camera the last frame).
Its tabs also serve the lidar's and the camera's own viewers from the same streams,
so it is the one program to run when someone wants to watch either sensor.
With `--source ros` it opens no hardware, so it can run beside whoever owns the sensors:
the lidar from Unitree's `lidar_driver` on its DDS (`rt/utlidar/cloud_livox_mid360`; a
`--lidar-topic` not starting `rt/` is read from ROS 2) and depth from a ROS 2 `Image`
(`depth_node.py`).

**What it showed** (2026-11-01, robot standing indoors):
- The camera's points fill the lidar's +x gap and meet the lidar's at the cone's
  edges without a visible step, so **the lidar's +x is the robot's front** (the
  camera faces forward) and the URDF offset lines them up at this scale.
- The camera sees 64–69% of the lidar's blind area out to 2.5 m (10 cm cells).
- Floor fits: lidar 1.21 m up, camera 1.26 m and 52–56° down; the URDF puts them
  at the same height, so about 5 cm is unexplained, likely the camera's nominal
  intrinsics (Intel's 87° × 58°, not its own calibration).

**Not done:** a measured alignment (point-to-point distance where both see a wall),
the camera's real intrinsics (librealsense or its calibration), a fused costmap for
Nav2.
