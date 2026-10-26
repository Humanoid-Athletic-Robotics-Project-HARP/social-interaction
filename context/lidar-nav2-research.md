---
name: lidar-nav2-research
description: Research for "Get around obstacles" (2026-10-26): simulating the MID-360 in MuJoCo, adding boxes, the real G1 lidar (IP, topics, inverted mount, extrinsics), Nav2 setup on RoboStack, testing lidar apart from walking
metadata:
  type: reference
---

Research done 2026-10-26 for the "Get around obstacles" goal ([[project-overview]]). The sim lidar and boxes are built (`run_sim --lidar --box`, details in [[mujoco-sim]]); Nav2 isn't yet. Status lives in the README.

**MID-360 facts** (livoxtech.com/mid-360/specs): 360° × −7° to +52° FOV, 0.1 m blind zone, 200k points/s at 10 Hz (about 20k per frame), ≤2 cm range noise, non-repetitive scan, built-in IMU.

**On the G1** (Unitree `lidar_Instructions` and `lidar_services_interface` pages, read in the in-app browser):
- Lidar IP 192.168.123.120. Use Livox-SDK2 / `livox_ros_driver2` directly (Humble and Jazzy builds; set host IPs in `MID360_config.json`), or Unitree's built-in service, which publishes PointCloud2 on DDS `rt/utlidar/cloud_livox_mid360` (10 Hz, frame `livox_frame`) and `rt/utlidar/imu_livox_mid360` (200 Hz). Whether the built-in service keeps running in debug mode is unknown; check with `ros2 topic list` on the robot. Running our own driver re-points the lidar's host IP, so pick one.
- **Mounted upside down.** Unitree's `g1_29dof_rev_1_0.urdf` (unitree_ros): `mid360_joint` from `torso_link` at xyz (0.0003, 0, 0.428434), rpy (π, 0.0511, 0). The older `g1_29dof.urdf` says (0.0003, 0, 0.40618), rpy (0, 0.0401, 0), without the flip. Our MuJoCo model matches the older URDF, so the sim uses its numbers plus the flip. Which revision our robot is, and its real mount, are still to check. deepglint's G1 driver fork rotates the IMU too, since the stock driver only rotates points.
- G1s made after April 2026 ship a "Mid360s", which needs newer SDK and driver. Check which ours has.
- **Floor blind ring:** inverted, the lowest ray points about 52° down from about 1.25–1.28 m up (computed from our MuJoCo model's standing pose), so the floor and low obstacles closer than about 0.9–1.1 m from the robot are never seen. The costmap has to remember obstacles after they leave view.
- It will see its own shoulders and arms: filter with a body-frame crop box or by robot geom.

**Simulating it in MuJoCo 3.17** (measured on this Mac):
- `mujoco.mj_multiRay` casts all rays from one point: 20k rays in about 5 ms, 10k in about 2.7 ms. It releases the GIL, so a thread with its own `MjData` (copy qpos, `mj_kinematics`) casts at 10 Hz without slowing the 5 ms physics loop. A Python loop over `mj_ray` is about 60 ms per 20k, too slow.
- Our G1's geoms share groups 0/1 with the floor, so `geomgroup` can't skip the robot. `bodyexclude` takes one body only. The sim keeps robot hits on purpose ([[mujoco-sim]]); the Nav2 step has to filter them, e.g. a crop cylinder of about 0.4 m radius around `base_link` (`pcl_ros` CropBox or `obstacle_min_range`).
- The `rangefinder` sensor with `camera=` gives a regular per-pixel ray grid (perspective only, so 4–6 cameras for 360°). Workable, but not the Livox pattern.
- `mujoco-lidar` (github.com/discoverse-dev/MuJoCo-LiDAR, MIT, PyPI 0.3.5) has a MID-360 pattern (`mid360.npy`, 800k angle pairs, 24k-ray sliding window); its CPU backend is just `mj_multiRay`. Livox's Gazebo plugin (`livox_laser_simulation`) replays `mid360.csv` with σ = 1 cm noise.

**Boxes:** load Unitree's scene with `mujoco.MjSpec.from_file` and add static box geoms to `spec.worldbody` before compiling, so layouts can come from a flag or file. MuJoCo contacts between robot geoms and box geoms give a ground-truth collision check for tests.

**Nav2 on RoboStack Jazzy (osx-arm64, checked with `pixi search`):** present: navigation2 and nav2-bringup 1.3.13, nav2-graceful-controller, nav2-mppi-controller, nav2-regulated-pure-pursuit-controller, nav2-collision-monitor, nav2-velocity-smoother, nav2-costmap-2d, nav2-voxel-grid, nav2-rviz-plugins, nav2-simple-commander, pointcloud-to-laserscan, nonpersistent-voxel-layer, pcl-ros, robot-localization, livox-ros-driver2. Missing: spatio-temporal-voxel-layer and nav2-docking. Stock `navigation_launch.py` starts `docking_server`, so we need our own launch file.
- No map: rolling-window global costmap (no static layer, about 15 × 15 m; goals outside it fail), and either `global_frame: odom` everywhere or a static identity map → odom. Goals are then relative to where the walker started.
- Cloud into costmap: obstacle_layer or voxel_layer with `data_type: PointCloud2` and a height band (about 0.1–1.6 m). The height filter applies in the costmap's frame, so planar odom means height above the floor.
- Graceful Controller (Jazzy) outputs only `linear.x` and `angular.z` and collision-checks its own trajectory. Defaults: `v_linear_max` 0.5, `v_angular_max` 1.0, `v_linear_min` 0.1, `v_angular_min_in_place` 0.25. Keep minimums well above GR00T's Balance switch (|cmd| < 0.05, `walk.py`). MPPI with the Omni model is the option if we want sideways steps.
- `walker_node`'s `base_link` is planar (x, y, yaw, z = 0), really a `base_footprint`. A static base_link → lidar transform ignores torso sway, which tilts rays a few degrees and can mark the floor far away as obstacles; limit marking range or put torso orientation into TF.

**Community G1 stacks** (none says it has been validated on hardware): automaticdai/unitree-g1-navigation (Humble, FAST-LIO2 + Nav2 + collision monitor, Unitree SDK walking), cristidragomir97/grove-g1 (Jazzy, own policy over `rt/lowcmd`, FAST-LIO2 + Nav2, sim-first; closest to ours), deepglint/FAST_LIO_LOCALIZATION_HUMANOID (`humble` branch, inverted-mount driver fix), norkraigh/Unitree-G1-SLAM (offline SLAM comparison, static TF roll 180°).

Related: [[mujoco-sim]], [[ros2-mac]], [[g1-builtin-apis]], [[unitree-resources]]
