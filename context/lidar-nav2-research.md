---
name: lidar-nav2-research
description: Read when bringing up the real G1 lidar, changing lidar_relay, adding mapping/localization, or looking for other G1 Nav2 setups to borrow from
metadata:
  type: reference
  updated: 2026-11-11T18:26:38Z
  covers: [submission/harp/lidar_relay.py]
---

Research, not yet tried on hardware, and our lidar relay. What we built from it: [[nav2-sim]], [[submission-image]].

**MID-360:** 360° × −7° to +52°, 0.1 m blind zone, about 20k points per frame at 10 Hz, ≤ 2 cm noise, non-repetitive scan, built-in IMU. Ours is a MID-360, not the newer Mid360s; what it really does on our robot is in [[g1-lidar-measured]].

**On the G1** (Unitree `lidar_Instructions`, `lidar_services_interface`):
- Lidar at 192.168.123.120. Unitree's built-in service (PointCloud2 on `rt/utlidar/cloud_livox_mid360`) is `lidar_driver`, off after a reboot (switch it on: [[robot-services]]); with it off the lidar has streamed raw Livox SDK2 UDP to PC2, but not after the 2026-11-02 reboot ([[g1-lidar-measured]]).
- **`harp/lidar_relay.py`** (in our image, robot only): reads `rt/utlidar/cloud_livox_mid360` through unitree_sdk2py on `ROBOT_DOMAIN`/`ROBOT_INTERFACE` (HIM's launcher sets them), republishes it restamped and without the robot's body ([[self-filter]]) for Nav2, and the same cloud turned 180° on `/utlidar/cloud_rear` (`livox_rear`) for Nav2's clearing, as the sim does ([[nav2-sim]]). Both at 10 Hz on PC2 against `lidar_driver` (2026-11-01, outside the image; `tests/test_lidar_relay.py` checks the turning and the self-filter's plumbing). The SDK needs CycloneDDS 0.10.2's C library, built from source in the Dockerfile (no wheel for Python 3.8 on ARM64); it sits beside ROS's Fast DDS in one process without trouble. Not yet run in a session through HARP's harness, and its self-filter not yet on the robot. With `lidar_driver` off it gets nothing ([[robot-services]]).
- **Reading the UDP stream** (with `lidar_driver` off): `livox_ros_driver2` (Unitree's documented route; `MID360_config.json`: lidar 192.168.123.120, host = the computer running it), which we would have to build into our image, or our own decoder (`testground/lidar/capture.py` decodes the packets with numpy). The driver's stamps are the lidar's own clock (`lddc.cpp`, no host-time option), so they need restamping too. Only one program can receive the lidar's ports.
- **Mounted upside down,** per Unitree's rev 1.0 URDF, as `harp/lidar_mount.py` and HIM's G1 model use, and as measured ([[g1-lidar-measured]]); the older `g1_29dof.urdf` has no flip and a lower mount. deepglint's driver fork also rotates the lidar's IMU (stock rotates only points).
- **Blind ring:** the floor and low obstacles within about 0.8–0.9 m are never seen (measured, [[g1-lidar-measured]]), so the costmap must remember them. The head also blinds it ahead: see there.

**Nav2 and mapping in our image (Foxy, Nav2 0.4.7):** no Graceful Controller (ours is a port, [[nav2-sim]]), and no collision monitor, velocity smoother or MPPI (all Humble and later); slam_toolbox and AMCL exist.

**Other G1 stacks** (none validated on hardware):
- Adyansh04/grove-g1 (BSD-3, Jazzy, MuJoCo, own policy; closest to ours): RotationShim + RPP with regulation off, VoxelLayer, `obstacle_min_range` 0.6, `obstacle_max_range` 3.0, separate server nodes. Uses a map (slam_toolbox/AMCL), so borrow its local side.
- automaticdai/unitree-g1-navigation (Apache-2.0, Humble, FAST-LIO2 + Nav2): crop-box self-filter (`sensor_adapter.py`) and a `collision_monitor.yaml` worth borrowing.
- deepglint/FAST_LIO_LOCALIZATION_HUMANOID (inverted-mount driver fix), norkraigh/Unitree-G1-SLAM (offline SLAM comparison).
- thdhyan/g1_perception_ws (Foxy on the robot, MID-360 + D435i + Nav2, depth into the costmap): its pitfalls are worth knowing: the D435i's IMU deadlocks on the Jetson (`unite_imu_method:=2`), five camera streams exhaust its USB buffers (it turns infrared off), and DDS discovery with Unitree's 280+ entities spiked memory to 14 GB (staggered starts, `ROS_DOMAIN_ID=1`).
- Skvayzer/humanoid_navigation (Foxy, containerized Fast-LIO + NDT + Nav2 on a G1, 0.2 m/s and 0.2 rad/s caps), hucebot/g1pilot (Humble, Livox and RealSense launches). None we found deals with the head's blind cone.
- No-map recipe: Nav2's GPS tutorial (rolling global costmap, `global_frame: odom`).

**Sim lidar alternatives not used:** `rangefinder` sensors on cameras (regular grid, 4–6 cameras for 360°); `mujoco-lidar` (MIT) with a real MID-360 pattern, whose CPU backend is the same `mj_multiRay`.
