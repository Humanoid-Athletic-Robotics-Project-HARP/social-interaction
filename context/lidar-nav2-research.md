---
name: lidar-nav2-research
description: Read when bringing up the real G1 lidar, changing lidar_relay, adding mapping/localization, or looking for other G1 Nav2 setups to borrow from
metadata:
  type: reference
  updated: 2026-10-28T08:59:18Z
  covers: [submission/harp/lidar_relay.py]
---

Research, not yet tried on hardware. What we built from it: [[nav2-sim]], [[submission-image]].

**MID-360:** 360° × −7° to +52°, 0.1 m blind zone, about 20k points per frame at 10 Hz, ≤ 2 cm noise, non-repetitive scan, built-in IMU. G1s made after April 2026 ship a "Mid360s" needing newer SDK and driver: check ours.

**On the G1** (Unitree `lidar_Instructions`, `lidar_services_interface`):
- Lidar at 192.168.123.120. Either Unitree's built-in service (PointCloud2 on `rt/utlidar/cloud_livox_mid360`, 10 Hz, `livox_frame`; IMU on `rt/utlidar/imu_livox_mid360`) or our own `livox_ros_driver2` (re-points the lidar's host IP, so pick one). Whether the built-in service runs in debug mode is unknown: `ros2 topic list` on the robot.
- **Into our ROS 2: `harp/lidar_relay.py`** in our image (tested against a robot-style cloud, not on the robot): reads `rt/utlidar/cloud_livox_mid360` through unitree_sdk2py on the robot's domain and republishes it, restamped with arrival time, for Nav2 ([[submission-image]]).
- **Fallback if the service doesn't run in debug mode:** `livox_ros_driver2` on the laptop, Unitree's documented route (`MID360_config.json`: lidar 192.168.123.120, host = the laptop). Its stamps are the lidar's own clock (`lddc.cpp`, no host-time option) and macOS has no easy PTP, so they need restamping too. G1s made after April 2026 have a Mid360s, which may need a newer driver than RoboStack's 1.2.4.
- **First check on the robot:** in a shell with CycloneDDS on `en0` and `ROS_DOMAIN_ID=0`, `ros2 topic hz /utlidar/cloud_livox_mid360`; then `lidar_relay real:=true interface:=en0` and look in rviz2.
- **Mounted upside down.** `g1_29dof_rev_1_0.urdf`: `mid360_joint` at (0.0003, 0, 0.428434) on `torso_link`, rpy (π, 0.0511, 0). The older `g1_29dof.urdf` (which our sim matches) has (0.0003, 0, 0.40618), pitch 0.0401, no flip. Unitree's `lidar_services_interface` page gives the lidar at (0, 0, −0.47618) from "the robot coordinate system" (frame not named), inverted, pitch −2.3° (our 0.0401 rad matches), and the lidar's IMU at (0.011, 0.02329, −0.04412) from the lidar, unrotated. Check our robot's revision and real mount. deepglint's driver fork also rotates the IMU (stock rotates only points).
- **Blind ring:** the floor and low obstacles within about 0.9–1.1 m are never seen, so the costmap must remember them.

**Nav2 and mapping in our image (Foxy, Nav2 0.4.7):** no Graceful Controller (ours is a port, [[nav2-sim]]), and no collision monitor, velocity smoother or MPPI (all Humble and later); slam_toolbox and AMCL exist.

**Other G1 stacks** (none validated on hardware):
- Adyansh04/grove-g1 (BSD-3, Jazzy, MuJoCo, own policy; closest to ours): RotationShim + RPP with regulation off, VoxelLayer, `obstacle_min_range` 0.6, `obstacle_max_range` 3.0, separate server nodes. Uses a map (slam_toolbox/AMCL), so borrow its local side.
- automaticdai/unitree-g1-navigation (Apache-2.0, Humble, FAST-LIO2 + Nav2): crop-box self-filter (`sensor_adapter.py`) and a `collision_monitor.yaml` worth borrowing.
- deepglint/FAST_LIO_LOCALIZATION_HUMANOID (inverted-mount driver fix), norkraigh/Unitree-G1-SLAM (offline SLAM comparison).
- No-map recipe: Nav2's GPS tutorial (rolling global costmap, `global_frame: odom`).

**Sim lidar alternatives not used:** `rangefinder` sensors on cameras (regular grid, 4–6 cameras for 360°); `mujoco-lidar` (MIT) with a real MID-360 pattern, whose CPU backend is the same `mj_multiRay`.
