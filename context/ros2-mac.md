---
name: ros2-mac
description: The ROS 2 workspace in ros2/ — the single pixi environment (RoboStack Jazzy, Python 3.12, MuJoCo, onnxruntime, unitree_sdk2py) and colcon workspace with our packages; how it's built and the macOS gotchas
metadata:
  type: reference
---

Since 2026-10-25 `ros2/` is the project's only environment and its colcon workspace. Tested on one teammate's Apple Silicon Mac (macOS 15.6); other OSes untested ([[team-context]]). The README has the steps: `pixi install --locked` and `pixi run build` in `ros2/`, then `pixi shell` in every terminal and `ros2 run ...`.

**Layout:** three ament_python packages, a teammate's choice on 2026-10-25, in a workspace inside the repo (like unitree_ros2's `cyclonedds_ws/`): `ros2/src/harp_control` (GR00T walker, `leg_odometry.py`, `smooth_control.py`, `move_distance`; no ROS 2 dependencies), `ros2/src/harp_sim` (the MuJoCo sim) and `ros2/src/harp_ros` (ROS 2 nodes). Executables: `ros2 run harp_control walk`, `ros2 run harp_control move_distance`, `ros2 run harp_sim run_sim`, `ros2 run harp_ros walker_node`. package.xml needs a valid maintainer email, so they use the placeholder `todo@todo.todo` (license `TODO`); no personal emails in the repo.

**walker_node** (`harp_ros`, 2026-10-25): wraps `Walker`; GR00T and the 200 Hz loop are unchanged. Subscribes `/cmd_vel` (Twist → `walker.cmd`, clipped to walk.py's limits, zeroed after `cmd_timeout` 0.5 s of silence); publishes `/odom` (frame `odom` → `base_link`, yaw-only orientation, smoothed body velocity) and `/tf` at 50 Hz while the policy runs, and once at start-up `/tf_static` `base_link` → `livox_frame` from `harp_control/lidar_mount.py` (the head lidar, standing height; torso lean isn't in it; 2026-10-26); services `/walker/start` (refused before the stand pose) and `/walker/stop` (damp and exit); `]`/`o` also work from a terminal. Params `real`, `interface`, `scene`, `cmd_timeout`. rclpy spins in a background thread; `SignalHandlerOptions.NO` so Ctrl-C reaches the walker loop and damps. Sim test: 0.3 m/s × 4 s → 1.02 m true, 1.02 m `/odom`; 0.5 rad/s × 3 s → 85.4° true, 85.5° `/odom`; tf2 lookup equals `/odom`; 0.01 m drift in 2 s with no `/cmd_vel`.

**DDS domains:** CycloneDDS can't have ROS 2 (rmw) and unitree_sdk2py on the same domain in one process: whichever comes second fails ("create domain error" / "rmw_create_node: failed to create domain, error Precondition Not Met"). So ROS 2 runs on `ROS_DOMAIN_ID=2`; Unitree topics stay on 0 (robot) and 1 (sim). Raw sim topics with ros2 tools: `ROS_DOMAIN_ID=1 ros2 topic echo /lowstate`. See [[mujoco-sim]], [[gr00t-wbc]].

**Environment (`ros2/pixi.toml`)**
- Channels `["robostack-jazzy", "conda-forge"]`; platform `{ name = "mac", platform = "osx-arm64", macos = "14.0" }` (the mujoco 3.17 and onnxruntime 1.31 wheels need macOS 14+; pixi rejects a virtual-package entry named like its subdir).
- `python = "3.12.*"` is pinned: unpinned, adding pip packages re-solved to Python 3.14, where cyclonedds-python 0.10.2 doesn't compile (`_Py_IsFinalizing`).
- Conda: `ros-jazzy-desktop`, `ros-jazzy-rmw-cyclonedds-cpp`, `ros-jazzy-rosidl-generator-dds-idl` (Unitree's messages need it), colcon, compilers, cmake. Pip: `mujoco==3.17.0` (its wheel brings `mjpython`), `onnxruntime==1.31.0`, `unitree_sdk2py` (editable, from `../../unitree_sdk2_python`). numpy 2.5.4 comes from conda.
- One DDS library: RoboStack Jazzy ships CycloneDDS 0.10.5 (same 0.10 series as the robot's 0.10.2). `CYCLONEDDS_HOME=$CONDA_PREFIX` plus `[pypi-options] no-binary = ["cyclonedds"]` builds unitree_sdk2py's cyclonedds 0.10.2 bindings against it; `otool -L` on `cyclonedds/_clayer*.so` shows `libddsc.0.dylib` 0.10.5. So ROS 2 and the SDK can share one process. The source-built CycloneDDS and `sim-venv` are no longer used.
- Activation: `scripts = ["install/setup.sh"]`; env `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`, `ROS_DOMAIN_ID=2` (see DDS domains), `CYCLONEDDS_URI` pinning `lo0` (the sim's interface; the robot is on its 192.168.123.x interface), `HARP_G1_SCENE` (Unitree's G1 model, used by the sim and leg odometry).
- unitree_sdk2py's CRC uses its pure-Python path on macOS (its compiled helpers are Linux-only), which works.

**Build (`pixi run build`):** `colcon build --merge-install --symlink-install --base-paths src ../../unitree_ros2/cyclonedds_ws/src/unitree` with `-DCMAKE_INSTALL_RPATH=<ros2>/install/lib`, then symlinks `install/lib/libunitree_*.dylib` into the env's `lib/`. `--symlink-install` runs our Python from `src/`, so edits need no rebuild and `walk.py` finds `policy/` next to itself. Unitree's messages: `unitree_ros2` at commit 668d1ec5a05d1c38d3306bdca7d59f2ba3581a88.

**The DYLD gotcha:** ROS loads message type-support libraries by bare name. colcon tells it where they are through `DYLD_LIBRARY_PATH`, but pixi doesn't pass any `DYLD_*` variable on macOS (even set in `[activation.env]`), and SIP strips it in `/bin/bash`, `/usr/bin/env` and `perl`. Symptom: `ros2 topic echo` fails with "Could not load library libunitree_hg__rosidl_typesupport_introspection_c.dylib". The build task's symlinks fix it; rerun `pixi run build` after reinstalling the environment.

**mjpython:** `ros2 run` starts plain python, but MuJoCo's viewer on macOS needs `mjpython`, so `run_sim.py` re-execs itself as `mjpython -m harp_sim.run_sim` (guarded by `HARP_SIM_UNDER_MJPYTHON`) unless `--headless`.

**Scripted tests:** run exactly one sim, since every sim publishes `rt/lowstate` on domain 1 and the walker would read both. `ros2 service call` waits until the service exists, so after a failed run stop leftover calls (`pkill -f "ros2 service call"`) before starting the next walker. With `--auto-release`, start the policy within 3 s of "Feet down".

**Verified 2026-10-25:** from the packages, `ros2 run harp_sim run_sim` opened the viewer and `ros2 run harp_control move_distance --to 2 1 --speed 0.3` estimated (1.96, 0.94) m facing 30°, the same as before the move. ROS 2 saw `/lowstate` and `/lowcmd` with the `unitree_hg` types, `/lowcmd` at 200 Hz. "Failed to parse type hash" warnings are expected, because unitree_sdk2py isn't a ROS 2 publisher. pixi skips gdk-pixbuf/graphviz/gtk3 post-link scripts by default; that didn't affect anything. rviz2 hasn't been tried.
