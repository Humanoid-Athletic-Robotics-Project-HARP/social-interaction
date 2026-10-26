---
name: nav2-sim
description: Read before tuning Nav2 (nav2_params.yaml, nav2.launch.py, the behavior tree), changing a course, or running Nav2 by hand — why each setting is what it is, current results and what's next
metadata:
  type: reference
  updated: 2026-10-26T22:40:10Z
  covers: [ros2/src/harp_ros/config/nav2_params.yaml, ros2/src/harp_ros/launch/nav2.launch.py, ros2/src/harp_ros/behavior_trees/*, ros2/src/harp_sim/worlds/*.xml, ros2/src/harp_control/harp_control/lidar_mount.py]
---

Values are in `harp_ros/config/nav2_params.yaml`; this note gives the reasons. Research behind it: [[lidar-nav2-research]].

**Setup and why**
- `nav2.launch.py` starts each server as its own node (controller, planner, behavior, bt_navigator, one lifecycle manager, rviz2) rather than nav2_bringup, which starts docking_server (not on RoboStack). Args: `rviz:=false`, `params_file:=`, `use_sim_time:=true` (for replay, [[run-logging]]).
- No map: everything in `odom`, goals relative to where the policy started; global costmap is a 20 × 20 m rolling window.
- NavFn (`allow_unknown`) + Graceful Controller (same law as `move_distance --to`). Speed minimums stay well above GR00T's 0.05 Balance switch.
- `behavior_trees/navigate.xml`: stock tree minus costmap clearing (would forget obstacles in the blind ring near the feet) and BackUp (risky for a walker). Recoveries are Wait and Spin.
- **Footprint:** a circle, `robot_radius` 0.4. The body's farthest point (forearms, held bent) is 0.355 m from `base_link`. A polygon would give NavFn an inscribed radius of only 0.09 m.
- **Inflation** 0.8 m, factor 3 keeps the robot's centre about 0.8 m (body edge about 0.45 m, personal space) from obstacles. It's only a preference; only `robot_radius` blocks a gap. Graceful follows NavFn's path and doesn't read the gradient.
- **Self-filter:** marking ignores points within 0.6 m of the sensor (`obstacle_min_range`); body points are 0.11–0.56 m away. Things standing on the floor nearby are still marked from their lower part; only something at sensor height 0.4–0.6 m from the centre could be missed. A box filter node was tried and set aside ([[self-filter-trial]]).
- **Live lidar TF:** `walker_node` publishes the lidar's real pose (`LegOdometry.lidar_pose()`, IMU + joints). A fixed transform made the floor 2.5–3 m out look ±0.12 m high (the torso tilts about 2°) and filled the costmap with phantom obstacles.
- **Clearing only from floor points** (`max_obstacle_height` 0.05 on the clearing source). Clearing with all points let rays skim over a thin tabletop and erase it.

**Inflation 1.0 / factor 2 was tried and rejected:** a bit more room at thin obstacles, but on `slalom` the gaps cost more than unknown space outside the course, so the robot walked around the side walls (198 s, 42.5 m vs 52 s, 9.9 m). Wider, gentler inflation makes NavFn prefer long detours through unknown space over visible gaps.

**Courses** (`harp_sim/worlds/`): `open_floor`, `one_box`, `slalom`, `obstacle_course` (table, pillars, thin pole, arch). Each has a ≥ 1.2 m path everywhere and a closed far end, but `slalom` and `obstacle_course` are **open at the start end**: close them before comparing costmap settings.

**Current baseline** (`runs check --tag v2`, `runs compare`): all courses 0 contacts. `slalom` 52–63 s, closest 0.27–0.29 m, 0–2 recoveries; `obstacle_course` about 50 s, closest 0.16–0.18 m; `one_box` 22–29 s, but one v2 run failed its 40 s limit after a planner failure. Pinned: `2026-10-26_20-48-51Z_sim_slalom_rangecut`, `2026-10-26_20-50-13Z_sim_slalom_rangecut` (both with nav bags).

**Open problem — slalom first-gap recoveries:** `nav2.log` shows bt_navigator timing out waiting for the planner to acknowledge a replan (`default_server_timeout: 20` ms) on a busy Mac, then running Wait. A timing problem, not an obstacle one. Next: raise it to 100–200 ms and compare with `course slalom --repeat 5`.

**To do in the sim** (README milestone "more on obstacles and path following"): tighter margins; collision_monitor before the robot; moving obstacles and people; the timeout fix; close the courses' start ends; repeat runs per setting (GR00T's gait varies). Then the real lidar.

**By hand** (README "Walk around obstacles with Nav2"; `course` automates all this):
1. Sim with `--lidar --world W`; wait for its banner before walker_node.
2. Start the policy after "Feet down" and within 3 s (the band releases 3 s later). With `--auto-release`, use 15: walker_node needs about 8 s to reach its stand pose.
3. Launch Nav2 once the policy runs; it aborts after about 3.5 s without `odom` → `base_link`.
4. Send goals in `odom` with `BasicNavigator.goToPose`; don't call `waitUntilNav2Active()` (it waits for AMCL).
5. Restart all three between courses. A fall right on band release means the policy started too early or late.
