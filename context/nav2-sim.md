---
name: nav2-sim
description: Read before tuning Nav2 (nav2_params.yaml, nav2.launch.py, the behavior tree), changing a course, or running Nav2 by hand — why each setting is what it is, current results and what's next
metadata:
  type: reference
  updated: 2026-10-28T04:48:12Z
  covers: [ros2/src/harp_ros/config/nav2_params.yaml, ros2/src/harp_ros/launch/nav2.launch.py, ros2/src/harp_ros/behavior_trees/*, ros2/src/harp_sim/worlds/*.xml, ros2/src/harp_sim/harp_sim/movers.py, ros2/src/harp_control/harp_control/lidar_mount.py]
---

Values are in `harp_ros/config/nav2_params.yaml`; this note gives the reasons. Research behind it: [[lidar-nav2-research]].

**Setup and why**
- `nav2.launch.py` starts each server as its own node (controller, planner, behavior, bt_navigator, one lifecycle manager, rviz2) rather than nav2_bringup, which starts docking_server (not on RoboStack). Args: `rviz:=false`, `params_file:=`, `use_sim_time:=true` (for replay, [[run-logging]]).
- No map: everything in `odom`, goals relative to where the policy started; global costmap is a 20 × 20 m rolling window.
- NavFn (`allow_unknown`) + Graceful Controller (same law as `move_distance --to`). Speed minimums stay well above GR00T's 0.05 Balance switch.
- `behavior_trees/navigate.xml`: stock tree minus costmap clearing (would forget obstacles in the blind ring near the feet) and BackUp (risky for a walker). Recovery waits, then retries planning. Spin and BackUp are disabled: the installed behavior server can publish motion from cached costmaps without checking STVL freshness. Normal turning stays in the freshness-checked path controller.
- `default_server_timeout` 200 ms: at Nav2's 20 ms, bt_navigator gave up waiting for the planner's ack on a busy Mac, and each replan became a recovery.
- **Footprint:** a circle, `robot_radius` 0.4. The body's farthest point (forearms, held bent) is 0.355 m from `base_link`. A polygon would give NavFn an inscribed radius of only 0.09 m.
- **Inflation** 0.8 m, factor 3 keeps the robot's centre about 0.8 m (body edge about 0.45 m, personal space) from obstacles. It's only a preference; only `robot_radius` blocks a gap. Graceful follows NavFn's path and doesn't read the gradient. Inflation 1.0 / factor 2 was rejected: on `slalom` NavFn then preferred a long detour round the side walls (198 s, 42.5 m vs 52 s, 9.9 m).
- **Live lidar TF:** `walker_node` publishes the lidar's real pose (`LegOdometry.lidar_pose()`, IMU + joints). A fixed transform made the floor 2.5–3 m out look ±0.12 m high (the torso tilts about 2°) and filled the costmap with phantom obstacles.

**Lidar layer: STVL** (`spatio_temporal_voxel_layer`, built from `ros2/src`, [[ros2-mac]]). Marks inside the lidar's view that aren't seen again fade in about 2 s; marks out of view after 15 s. The YAML header explains each setting.
- Nav2's VoxelLayer was replaced because a moving person left a permanent wall: its clearing rays start at the lidar (1.21 m) and slope down, so marks above that height never clear. On `crossing` it detoured round the person's whole track. Its settings, for comparison runs: `nav2_params.yaml` at commit 001e68f, passed with `course --params FILE`.
- STVL has no minimum range, so the arms get marked, but they're inside the footprint, which STVL clears.
- Marking buffers in both costmaps require a cloud within 1 s, so a lost sensor makes the costmap stale. Clearing buffers use the same cloud but keep their deadline disabled: this STVL version returns before refreshing its clearing-only timestamp. The offline watchdog test links the built STVL library and checks expiry and recovery on empty/height-filtered clouds. In pinned `2026-10-28_04-46-09Z_sim_one_box_review-final-lidar-B` (`ab33d11`), stopping the relay led to a zero velocity request in 1.443 s with no subsequent nonzero requests; B then ended the run. This is simulator evidence, not a physical stopping-distance bound.
- **`track_unknown_space: false` (both costmaps):** STVL never marks cells free, so with it on the map stayed unknown around obstacles, inflation left no gradient there (it only spreads inscribed/lethal costs into unknown cells), and NavFn hugged every obstacle (`obstacle_course`: closest 0.10–0.12 m, 5–11 recoveries, one failure; `2026-10-27_08-13-24Z_sim_obstacle_course_stvl`, nav bag).

**Courses** (`harp_sim/worlds/`): `open_floor`, `one_box`, `slalom`, `obstacle_course` (table, pillars, thin pole, arch); with people ([[mujoco-sim]]): `crossing` (crosses the path once, 3 s after the goal), `oncoming` (walks toward the robot, 0.5 m to its left). `crossing_shuttle` (endless back and forth) is kept only for old runs. Each static course has a ≥ 1.2 m path everywhere and a closed far end, but `slalom` and `obstacle_course` are **open at the start end**: close them before comparing costmap settings.

**Current results** (STVL, 1 run each so far, all pinned; 0 contacts):
- `obstacle_course` (`2026-10-27_08-19-50Z_..._stvl`, nav bag): 51.8 s, 9.6 m, closest 0.17 m, 0 recoveries — the same as VoxelLayer (`2026-10-27_08-12-03Z_..._v2`, nav bag).
- `crossing` (`2026-10-27_08-21-27Z_..._stvl`): 24.3 s, 4.9 m straight, closest 0.84 m. VoxelLayer: 55 s, 9.6 m round the track (`2026-10-27_08-00-34Z_..._v2`).
- `oncoming` (`2026-10-27_08-23-14Z_..._stvl`): 30.2 s, closest **0.09 m** (left wrist).
- `slalom`, `one_box`, `open_floor` not yet run with STVL. No pass criteria yet for `crossing`/`oncoming`.

**Open problem — oncoming people:** both layers let the person brush past (STVL 0.09 m, VoxelLayer 0.18 m): the robot mostly stops and moves only 8–13 cm aside. Nav2 treats the person as standing still, and they're only marked within 3 m, about 2 s before they reach the robot.

**To do in the sim** (README milestone "more on obstacles and path following"): repeat runs per course with STVL (GR00T's gait varies) and set criteria for `crossing`/`oncoming`; yielding to oncoming people; slalom with the 200 ms timeout (one earlier recovery was a NavFn path-extraction failure, which the timeout won't fix); collision_monitor; close the courses' start ends. Then the real lidar.

**By hand** (README "Walk around obstacles with Nav2"; `course` automates all this):
1. Sim with `--lidar --world W`; wait for its banner before walker_node.
2. Start the policy after "Feet down" and within 3 s (the band releases 3 s later). With `--auto-release`, use 15: walker_node needs about 8 s to reach its stand pose.
3. Launch Nav2 once the policy runs; it aborts after about 3.5 s without `odom` → `base_link`.
4. Send goals in `odom` with `BasicNavigator.goToPose`; don't call `waitUntilNav2Active()` (it waits for AMCL).
5. Restart all three between courses. A fall right on band release means the policy started too early or late.
