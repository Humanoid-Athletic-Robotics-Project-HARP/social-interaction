---
name: ros2-mac
description: ROS 2 Jazzy on macOS via RoboStack/pixi, reading our MuJoCo sim's topics; working setup and the DYLD gotcha
metadata:
  type: reference
---

Working on one teammate's Apple Silicon Mac (macOS 15.6, 2026-10-25): ROS 2 **Jazzy** from RoboStack, installed with pixi (`brew install pixi`). The workspace is `ros2/` in this repo (`pixi.toml`, `pixi.lock`; the environment goes in the git-ignored `ros2/.pixi/`), set up with `cd ros2 && pixi install --locked && pixi run build`. The README's "ROS 2 (macOS)" section has the steps. Jazzy uses Python 3.12, like `sim-venv`. The two environments stay separate and only talk over DDS. No ROS 2 code uses it yet. See [[mujoco-sim]] and [[unitree-resources]].

**Setup that works**
- `pixi.toml`: channels `["robostack-jazzy", "conda-forge"]`. Packages: `ros-jazzy-desktop`, `ros-jazzy-rmw-cyclonedds-cpp`, `ros-jazzy-rosidl-generator-dds-idl` (Unitree's messages need it), `colcon-common-extensions`, `compilers`, `cmake`, `pkg-config`, `make`, `ninja`.
- Unitree's messages come from `unitree_ros2` (cloned next to the repo, commit 668d1ec5a05d1c38d3306bdca7d59f2ba3581a88), `cyclonedds_ws/src/unitree/`, built with `colcon build --merge-install ... -DBUILD_TESTING=OFF -DCMAKE_INSTALL_RPATH=<ros2>/install/lib` (the `build` task).
- Activation: `scripts = ["install/setup.sh"]`, plus env `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`, `ROS_DOMAIN_ID=1`, `CYCLONEDDS_URI` pinning `NetworkInterface name="lo0"` (the sim's domain and interface; the robot is domain 0 on its 192.168.123.x interface).
- Result: `ros2 topic list` shows `/lowstate` and `/lowcmd` with the `unitree_hg` types, `/lowstate` runs at 200 Hz, and echo shows IMU data and `mode_machine` 5. "Failed to parse type hash" warnings are expected, because unitree_sdk2py isn't a ROS 2 publisher.

**The DYLD gotcha:** ROS loads message type-support libraries by bare name. colcon tells it where they are through `DYLD_LIBRARY_PATH`, but pixi doesn't pass any `DYLD_*` variable on macOS (even set in `[activation.env]`), and SIP strips it in `/bin/bash`, `/usr/bin/env` and `perl`. Symptom: `ros2 topic echo` fails with "Could not load library libunitree_hg__rosidl_typesupport_introspection_c.dylib". Fix: symlink `install/lib/libunitree_*.dylib` into the pixi env's `lib/`, where the loader looks. The `build` task in `pixi.toml` builds and links in one step (`pixi run build`); rerun it after reinstalling the pixi environment.

pixi skips gdk-pixbuf/graphviz/gtk3 post-link scripts by default; that didn't affect the ros2 CLI. rviz2 hasn't been tried. Linux and Windows don't need any of this: use the normal ROS 2 packages there ([[team-context]]).
