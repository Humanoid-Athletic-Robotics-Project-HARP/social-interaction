---
name: ros2-mac
description: Read when building or changing the ros2/ environment (pixi.toml, packages, executables), hitting a pixi or macOS library error, or changing walker_node
metadata:
  type: reference
  updated: 2026-10-27T21:31:55Z
  covers: [ros2/pixi.toml, ros2/src/*/setup.py, ros2/src/*/package.xml, ros2/src/harp_ros/harp_ros/walker_node.py, ros2/src/spatio_temporal_voxel_layer/HARP_PATCHES.md, ros2/src/spatio_temporal_voxel_layer/spatio_temporal_voxel_layer/CMakeLists.txt]
---

`ros2/` is the project's only environment (pixi, RoboStack Jazzy, Python 3.12) and its colcon workspace. Tested only on Apple Silicon Macs ([[team-context]]). Setup steps: README.

**Always `pixi run --as-is ...` or `pixi shell --as-is`.** Plain `pixi run`/`pixi shell` reinstalls the editable `unitree_sdk2py` each time, and for about 0.25 s its import files are missing; a sim or node starting in that window dies with "No module named 'unitree_sdk2py'". Use plain `pixi install` / `pixi run build` only to change the environment, never while something else is starting.

**Packages** (ament_python, `ros2/src/`): `harp_control` (GR00T walker, leg odometry, curves, `move_distance`, `runlog.py`; no ROS 2 dependencies), `harp_sim` (the sim, courses and criteria in `worlds/`), `harp_ros` (`walker_node`, Nav2 config and launch), `harp_eval` (course tests, run logging, replay; depends on the other three). `package.xml` uses the placeholder `todo@todo.todo`: no personal emails in the repo.
- **STVL** (`ros2/src/spatio_temporal_voxel_layer`, C++, the Nav2 costmap layer whose marks fade, [[nav2-sim]]): a plain copy of upstream (no `.git`), not on RoboStack. Its `HARP_PATCHES.md` has the upstream commit, our two macOS patches to its CMakeLists.txt (Boost without `system`, no `--no-undefined` on Apple) and how to update it. Its deps are in `pixi.toml` (openvdb, laser-geometry, pcl-conversions, tf2-sensor-msgs, ament-cmake-vendor-package; about 26 MB download).

**walker_node:** wraps the GR00T walker. `/cmd_vel` in (clipped to walk.py's limits, zeroed after `cmd_timeout` 0.5 s of silence); `/odom` and `/tf` (`odom` → `base_link`, planar, really a base_footprint) at 50 Hz, plus `base_link` → `livox_frame` from `LegOdometry.lidar_pose()` ([[nav2-sim]]); `/joint_states`, `/imu`, `/walker/policy`, `/harp/events`. Services `/walker/start` (refused before the stand pose), `/walker/stop` (damp and exit). `auto_start` (sim only) starts the policy on the sim's `feet_down` event. rclpy spins in a thread with `SignalHandlerOptions.NO` so Ctrl-C reaches the walker loop and damps.

**DDS domains:** CycloneDDS can't host ROS 2 and unitree_sdk2py on the same domain in one process ("create domain error" / "Precondition Not Met"). So by hand ROS 2 is on domain 2 and Unitree topics on 0 (robot) and 1 (sim): only one hand-started sim at a time. `course` claims its own pair per run (`run_sim --domain`, `walker_node` `domain`, `ROS_DOMAIN_ID`), 10/11 up to 98/99, via lock files in `$TMPDIR/harp-dds-domains/` (freed when the course process is gone), so several `course` commands can run at once. Domain ids map to UDP ports (7400 + 250 each): the valid range is 0–232, and above about 101 they hit Linux's ephemeral ports. Raw sim topics: `ROS_DOMAIN_ID=1 ros2 topic echo /lowstate` (the run's sim domain for a course run).

**Environment choices (`ros2/pixi.toml`):**
- Python pinned to 3.12: unpinned it re-solved to 3.14, where cyclonedds-python 0.10.2 doesn't compile.
- One DDS library: `no-binary = ["cyclonedds"]` with `CYCLONEDDS_HOME=$CONDA_PREFIX` builds the SDK's bindings against RoboStack's CycloneDDS 0.10.5, so ROS 2 and the SDK share a process.
- Only the Nav2 packages we use; no nav2_bringup or metapackage, since RoboStack lacks nav2-docking.
- Activation sets `ROS_DOMAIN_ID=2`, `CYCLONEDDS_URI` (pins `lo0`), `HARP_G1_SCENE`, `HARP_RUNS_DIR`.

**Build (`pixi run build`):** colcon with `--merge-install --symlink-install` over our packages and `unitree_ros2`'s messages. Symlink install means Python edits need no rebuild. It also symlinks `libunitree_*.dylib` and `libspatio_temporal_voxel_layer__*.dylib` into the env's `lib/`, because ROS finds type-support libraries through `DYLD_LIBRARY_PATH`, which pixi and SIP strip on macOS ("Could not load library libunitree_hg__rosidl_typesupport_introspection_c.dylib"). **Any package we build that defines messages or services needs the same link.** Without it STVL fails as "Failed to initialize costmap plugin stvl_layer! could not create service: type_support is null", Nav2's bringup aborts, and the robot stands still. Rerun the build after reinstalling the environment.
- Deleting or renaming a file a package installs (a world, a config) leaves dangling links, and the next build fails with "can't copy …: doesn't exist or not a regular file". Remove `build/<package>` and the dead link under `install/share/<package>/`, then build again.
- CMake args: `CMAKE_MODULE_PATH=$CONDA_PREFIX/lib/cmake/OpenVDB` so STVL's `openvdb_vendor` finds the env's OpenVDB (it only looks in Linux paths, and would otherwise download and build OpenVDB 10); `CMAKE_IGNORE_PREFIX_PATH=/opt/homebrew` so CMake doesn't pick up Homebrew's Boost.

**Harmless:** "Failed to parse type hash" warnings (unitree_sdk2py isn't a ROS 2 publisher); rviz2's GLSL "Sampler error" for Map displays on the Mac.
