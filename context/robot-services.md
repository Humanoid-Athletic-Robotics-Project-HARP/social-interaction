---
name: robot-services
description: Read before reading the real G1's sensors or Unitree's services over DDS or ROS 2 — lidar_driver, videohub, the robot's DDS topics and which ROS 2 domain to use
metadata:
  type: reference
  updated: 2026-11-09T20:18:47Z
  covers: [testground/pc2/topics.py]
---

Checked on our robot's PC2, 2026-11-01/02 ([[robot-pc2]] for logging in and where we may work). Unitree's API calls (speech, gestures, odometry, SLAM): [[g1-builtin-apis]]. The battery, motor temperatures and harness status the dashboard reads: [[robot-status]].

**Unitree's services** (`robot_state` client in `unitree_sdk2py.go2.robot_state`:
ServiceList reads, ServiceSwitch turns one on or off; status 0 is running, 1 off).
Switching changes the robot for everyone: ask first.
- `lidar_driver`: Unitree's lidar service, off after a reboot (seen 2026-11-02; switched
  on for each session so far). It
  re-points the lidar to itself, so PC2 gets no raw UDP while it runs (`record.py`,
  `lidar_viewer.py` and any Livox driver see nothing), and publishes on Unitree's DDS
  (domain 0), readable by any number of programs: `rt/utlidar/cloud_livox_mid360`
  (`PointCloud2_`, about 10 Hz nominal, 20,064 points, in the lidar's own upside-down frame; x y z
  intensity ring time; stamped about 0.1 s behind PC2's clock) and
  `rt/utlidar/imu_livox_mid360`. Off again: ServiceSwitch("lidar_driver", False).
  Over DDS it loses clouds the raw stream didn't: 19 of 158 intervals over 1.5 periods,
  up to 0.5 s (`dds-trial-20s`, read by Python beside the hybrid viewer); 6.2 clouds a
  second over 60 s with the depth node, the hybrid viewer and the camera recorder also
  running (2026-11-02). It drops the raw stream's stray returns ahead (visor on: 0–2 a
  cloud, raw 36) and passes real ones (visor off: [[g1-face-visor]]).
  Long runs beside HARP's harness are untested.
- `unitree_slam`, `auto_test_*`, `ota_box` off; `state_estimator`, the videohubs on.
- Colour: `videohub` answers any number of programs (GetImageSample: 1920×1080 JPEG,
  about 233 KB, 27 ms a request, up to 36 a second). Depth: no Unitree service; our
  `testground/camera/depth_node.py` owns the camera and publishes it on ROS 2. ROS 1's
  `realsense2_camera` (Noetic) is installed; no ROS 2 RealSense or Livox driver is.
- numpy on the Jetson: set `OPENBLAS_NUM_THREADS=1` before importing it (our scripts do);
  its BLAS pool otherwise keeps 7 threads spinning (the hybrid viewer used 6.4 cores, then
  0.7).
- Unitree's SDK beside a sourced ROS 2 Foxy: put `~/cyclonedds_ws/install/cyclonedds/lib`
  first on `LD_LIBRARY_PATH`, or its CycloneDDS fails ("undefined symbol
  ddsi_sertype_v0") against Foxy's.

**ROS 2 on the robot:** HARP's harness uses domain 97, localhost only, Fast DDS (only
`/safety/*` topics). Our depth node ran there (2026-11-01, `ROS_DOMAIN_ID=97 ROS_LOCALHOST_ONLY=1
RMW_IMPLEMENTATION=rmw_fastrtps_cpp`): depth at 15 fps, and the harness's status still
2 a second, its CPU 55–61% as before, while it waited with no policy. On domain 0,
Unitree's DDS domain, `ros2 topic list` fails with
`bad_alloc` (discovery of Unitree's hundreds of entities), so our ROS 2 nodes use
another domain (42 in testing).

**The robot's DDS** (`pc2/topics.py`, 89 topics on domain 0, eth0): `rt/lowstate` (2
publishers, about 1 kHz together), `rt/lowcmd`, `rt/sportmodestate`,
`rt/state_estimator/*` odometry, `rt/secondary_imu`, Unitree's `rt/api/*` services. No
lidar or point-cloud topic while `lidar_driver` is off (as in that survey); with it on,
`rt/utlidar/*` appear.
