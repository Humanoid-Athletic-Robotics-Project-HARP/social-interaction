---
name: g1-head-camera
description: Read before using the G1's head camera (RealSense D435i) — to see ahead where the lidar is blind, detect people, or record depth or colour on PC2
metadata:
  type: project
  updated: 2026-11-01T23:18:25Z
  covers: [testground/pc2/v4l2_survey.py, testground/camera/*]
---

**What's there** (checked read-only on PC2, 2026-11-01): an Intel RealSense D435i
on USB 3, as `/dev/video0` (depth, Z16: up to 1280×720 at 30 fps or 848×480 at 90),
`/dev/video2` (infrared: Y8, Y8I both imagers, Y12I calibration), `/dev/video4`
(colour) and `/dev/video1`, `3`, `5` (metadata). Unitree's `videohub_pc4` reads
`/dev/video4` and serves colour over DDS (`rt/api/videohub`); its error 3102 meant the
camera was unplugged. `videohub_pc4_chest` expects `/dev/video10`, which doesn't
exist: no chest camera. No pyrealsense2 or `rs-*` tools; librealsense2 2.50 comes
with ROS Noetic's `realsense2_camera`, which would open every stream, colour too.
Depth streams fine straight through V4L2 while `videohub` serves colour (tested
2026-11-01; `testground/pc2/v4l2_survey.py` lists the modes). Infrared on `/dev/video2`
won't stream alongside depth that way (EIO): it would need librealsense. Live view:
`testground/camera/camera_viewer.py`; shared on ROS 2 by `testground/camera/depth_node.py`
(15 fps depth image, camera info, point cloud; tested 2026-11-01 with the hybrid viewer
reading it at 14–16 fps). Colour from `videohub`: [[robot-pc2]]. [[robot-pc2]]

**Where it looks** (official: Unitree's `unitree_ros` `g1_29dof_rev_1_0.urdf`, our
revision): `d435_joint` on `torso_link` at (0.0576, 0.0175, 0.4299), pitched down
0.831 rad (47.6°): 5.7 cm ahead of and 1.8 cm left of the lidar, at its height
([[g1-lidar-measured]]). Unitree's `depth_camera_instruction` drawing agrees (axis 42°
from vertical, 47.6 mm ahead, 462.7 mm above the torso block's bottom). The
downward angle is by design (aimed at the hands and the ground ahead), not a fault;
changing it would mean reworking the head mount. The older
`g1_29dof.urdf` (in `~/holosoma` on PC2) has z 0.4199. Measured from its own depth (floor fit, robot standing,
2026-11-01): 1.25 m up, optical axis 56° below horizontal (the torso leans forward).
About 81% of depth pixels have a value indoors. With Intel's fields of view (depth 87° × 58°,
colour 69° × 42°) depth covers 25° to 83° below the horizon: the floor from 0.16 m to
2.7 m ahead, about ±56° wide at 1 m and ±46° at 2 m. Heights it sees ahead: up to
0.76 m at 1 m, 0.30 m at 2 m (colour: 0.58 m at 1 m, floor only past 1.9 m).

**What that means:** it fills the lidar's blind cone ahead and its blind ring at the
feet, but only near and low: a good obstacle sensor out to about 2.5 m, and a person
in front shows only their legs (hips at 1 m, shins at 2 m), never a face. People
farther than about 2 m ahead are seen by neither sensor; the lidar sees them anywhere
beyond about 60° from straight ahead, out to about 11 m.

**Next:** people at 1–3 m ahead in the viewer, and recordings for offline work.
