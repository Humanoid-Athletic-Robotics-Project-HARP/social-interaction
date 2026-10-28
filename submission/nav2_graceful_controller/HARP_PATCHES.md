# HARP's copy of nav2_graceful_controller

Nav2's Graceful Controller, ported to ROS 2 Foxy for our image (submission/Dockerfile builds it).
Upstream: https://github.com/ros-navigation/navigation2, `nav2_graceful_controller/` at commit
`d5d768b7b38a05e7cfd2b6c693b786b402161bed` (jazzy branch, 2026-10-27), Apache-2.0 (`LICENSE`).
Left out: `test/` and `doc/`. Every change is marked `HARP (Foxy port)` in the code, except the
renamed exceptions and includes:

- `SmoothControlLaw` (`smooth_control_law.*`) is unchanged; `ego_polar_coords.hpp` includes
  Foxy's `tf2_geometry_msgs/tf2_geometry_msgs.h` instead of `.hpp`.
- `GracefulController` implements Foxy's `nav2_core::Controller`: `configure` takes the node's
  `SharedPtr`, `computeVelocityCommands` gets no goal checker (so the distance where it stops and
  only turns is its own parameter, `goal_dist_tolerance`, default 0.25 m, in `ParameterHandler`),
  and there is no `setSpeedLimit` (or its costmap-filter include).
- `failure_tolerance` (s, default 0): when no collision-free trajectory is found, the controller
  commands zero and tries again for this long before raising its error. Jazzy's controller
  server does this for every controller (its `failure_tolerance`); Foxy's aborts the path on the
  first error, and each abort was a recovery (6 of them on `crossing` while the person passed).
- Foxy has one controller exception, `nav2_core::PlannerException` (`nav2_core/exceptions.hpp`); it
  replaces `ControllerException`, `ControllerTFError`, `InvalidPath` and `NoValidControl`.
- `min_by`, `first_after_integrated_distance` and `calculate_path_length`, which Foxy's
  `nav2_util::geometry_utils` lacks, are copied from Nav2 jazzy's into `utils.hpp`.
- `CMakeLists.txt` and `package.xml`: C++17 (Foxy's `nav2_package()` defaults to C++14, and the
  control law uses `std::clamp`), `visualization_msgs` and `nav_msgs` added, tests removed.

To update: copy the package from a newer upstream commit, reapply these changes, build the image
and rerun the courses (`submission/course.py`).
