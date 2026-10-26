---
name: ros2-mac
description: Read when building or changing the ros2/ environment (pixi.toml, packages, executables), hitting a pixi or macOS library error, or changing walker_node
metadata:
  type: reference
  updated: 2026-10-26T22:40:10Z
  covers: [ros2/pixi.toml, ros2/src/*/setup.py, ros2/src/*/package.xml, ros2/src/harp_ros/harp_ros/walker_node.py]
---

`ros2/` is the project's only environment (pixi, RoboStack Jazzy, Python 3.12) and its colcon workspace. Tested only on Apple Silicon Macs ([[team-context]]). Setup steps: README.

**Always `pixi run --as-is ...` or `pixi shell --as-is`.** Plain `pixi run`/`pixi shell` reinstalls the editable `unitree_sdk2py` each time, and for about 0.25 s its import files are missing; a sim or node starting in that window dies with "No module named 'unitree_sdk2py'". Use plain `pixi install` / `pixi run build` only to change the environment, never while something else is starting.

**Packages** (ament_python, `ros2/src/`): `harp_control` (GR00T walker, leg odometry, curves, `move_distance`, `runlog.py`; no ROS 2 dependencies), `harp_sim` (the sim, courses and criteria in `worlds/`), `harp_ros` (`walker_node`, Nav2 config and launch, run logging). `package.xml` uses the placeholder `todo@todo.todo`: no personal emails in the repo.

**walker_node:** wraps the GR00T walker. `/cmd_vel` in (clipped to walk.py's limits, zeroed after `cmd_timeout` 0.5 s of silence); `/odom` and `/tf` (`odom` → `base_link`, planar, really a base_footprint) at 50 Hz, plus `base_link` → `livox_frame` from `LegOdometry.lidar_pose()` ([[nav2-sim]]); `/joint_states`, `/imu`, `/walker/policy`, `/harp/events`. Services `/walker/start` (refused before the stand pose), `/walker/stop` (damp and exit). `auto_start` (sim only) starts the policy on the sim's `feet_down` event. rclpy spins in a thread with `SignalHandlerOptions.NO` so Ctrl-C reaches the walker loop and damps.

**DDS domains:** CycloneDDS can't host ROS 2 and unitree_sdk2py on the same domain in one process ("create domain error" / "Precondition Not Met"). So ROS 2 is on domain 2; Unitree topics on 0 (robot) and 1 (sim). Raw sim topics: `ROS_DOMAIN_ID=1 ros2 topic echo /lowstate`. Run only one sim: every sim publishes `rt/lowstate` on domain 1.

**Environment choices (`ros2/pixi.toml`):**
- Python pinned to 3.12: unpinned it re-solved to 3.14, where cyclonedds-python 0.10.2 doesn't compile.
- One DDS library: `no-binary = ["cyclonedds"]` with `CYCLONEDDS_HOME=$CONDA_PREFIX` builds the SDK's bindings against RoboStack's CycloneDDS 0.10.5, so ROS 2 and the SDK share a process.
- Only the Nav2 packages we use; no nav2_bringup or metapackage, since RoboStack lacks nav2-docking.
- Activation sets `ROS_DOMAIN_ID=2`, `CYCLONEDDS_URI` (pins `lo0`), `HARP_G1_SCENE`, `HARP_RUNS_DIR`.

**Build (`pixi run build`):** colcon with `--merge-install --symlink-install` over our packages and `unitree_ros2`'s messages. Symlink install means Python edits need no rebuild. It also symlinks `libunitree_*.dylib` into the env's `lib/`, because ROS finds type-support libraries through `DYLD_LIBRARY_PATH`, which pixi and SIP strip on macOS ("Could not load library libunitree_hg__rosidl_typesupport_introspection_c.dylib"). Rerun the build after reinstalling the environment.

**Harmless:** "Failed to parse type hash" warnings (unitree_sdk2py isn't a ROS 2 publisher); rviz2's GLSL "Sampler error" for Map displays on the Mac.
