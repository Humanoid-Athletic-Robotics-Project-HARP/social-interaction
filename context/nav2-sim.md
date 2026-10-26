---
name: nav2-sim
description: Our Nav2 setup (harp_ros nav2.launch.py, config/nav2_params.yaml, behavior_trees/navigate.xml) and the obstacle courses: why each setting is what it is, footprint and inflation, the bugs found (tilted floor, vanishing tabletop), the slalom first-gap recoveries (planner acknowledge timeout), measured course results, the inflation trial, what's still to do
metadata:
  type: reference
  updated: 2026-10-26T21:36:04Z
  covers: [ros2/src/harp_ros/config/nav2_params.yaml, ros2/src/harp_ros/launch/nav2.launch.py, ros2/src/harp_ros/behavior_trees/*, ros2/src/harp_sim/worlds/*.xml, ros2/src/harp_control/harp_control/lidar_mount.py]
---

Built and sim-tested 2026-10-26 for "Get around obstacles" ([[project-overview]]). Background research is in [[lidar-nav2-research]]; the sim side (lidar, `--world`, contact report) is in [[mujoco-sim]].

**Pieces**
- `harp_ros/launch/nav2.launch.py` starts each server as its own node, like grove-g1: controller_server, planner_server, behavior_server and bt_navigator, plus one lifecycle manager and rviz2 (`rviz:=false` to skip it). It also takes `params_file:=` (another Nav2 parameter file) and `use_sim_time:=true` (for replay.launch.py, [[run-logging]]). We don't use nav2_bringup, because its launch files start docking_server, which RoboStack lacks. The pixi env has only the Nav2 packages we use (`ros2/pixi.toml`).
- Frames: everything is in `odom`, so goals are relative to where the policy started, and there's no map. The global costmap is a 20 × 20 m rolling window.
- Planner: NavFn, `allow_unknown`.
- Controller: Graceful (`v_linear` 0.1–0.3, `v_angular_max` 0.6, minimums well above GR00T's 0.05 standing switch). Goal tolerance 0.25 m / 0.35 rad; progress checker 0.3 m in 20 s.
- `behavior_trees/navigate.xml` is the stock replanning-and-recovery tree minus costmap clearing and BackUp. Clearing would forget obstacles in the blind ring near the feet; backing up is risky for a walker. Recoveries are only Wait and Spin.
- Costmaps: VoxelLayer, `robot_radius` 0.4, inflation 0.8 m with `cost_scaling_factor` 3. Two observation sources on the lidar:
  - marking: 0.08–1.8 m high, 0.6–3.0 m range. The minimum range is the self-filter (body points sit 0.11–0.56 m from the sensor). A box self-filter node was tried on 2026-10-26 and set aside: [[self-filter-trial]].
  - clearing: only points below 0.05 m, i.e. rays that end on the floor.

**Footprint and margins** (measured 2026-10-26 from the MuJoCo meshes in GR00T's stand pose, relative to `base_link`, which is the pelvis on the floor):
- The body spans x −0.09 to +0.23 m and y ±0.29 m. The hands and forearms stick out forward and to the sides, because the arms are held fixed with the elbows bent. The farthest point is 0.355 m from `base_link`, so `robot_radius` 0.4 covers it with about one 5 cm cell to spare. Keep the footprint a circle: a polygon's inscribed radius would be only 0.09 m, and NavFn plans with the inscribed radius.
- Inflation is only a preference. Nothing blocks a gap except `robot_radius`. With 0.8 m and factor 3, the cost is 187 at 0.5 m from an obstacle, 138 at 0.6, 102 at 0.7 and 76 at 0.8, then drops straight to 0. 0.8 m centre to obstacle means about 0.45 m from the body's edge, which is roughly the edge of personal space. Graceful doesn't read the gradient: it follows NavFn's path and checks the footprint for collisions.
- The marking source's `obstacle_min_range` 0.6 m (the self-filter) hides only points within 0.6 m of the *sensor* (1.21 m up). Something standing on the floor close by is still marked from its lower part: people-sized boxes 0.5 m from the centre were marked as well as with a box filter ([[self-filter-trial]]). What it can miss is something only at about sensor height, 0.4–0.6 m from the centre (a shelf edge, an outstretched arm). Inside 0.4 m, footprint clearing erases marks anyway.

**Bugs found and fixed (2026-10-26)**
- *Floor marked as obstacles.* With a fixed `base_link` → `livox_frame` transform, the floor 2.5–3 m away appeared ±0.12 m high, because GR00T stands with the torso tilted about 2°. About 4,000 phantom lethal cells. Fix: `LegOdometry.lidar_pose()` computes the lidar's real pose from the IMU and joints (MuJoCo FK, pelvis height from the planted ankle at `ANKLE_HEIGHT` 0.035 m), and `walker_node` publishes it on `/tf` with each odometry step. It matches MuJoCo exactly. Before the policy runs, a 10 Hz timer publishes the upright guess (`lidar_mount.from_base_link()`). After the fix the floor reads 0.00 m out to 3 m, and only the box (plus its 0.4 m inscribed ring) is lethal.
- *Thin tabletop erased.* Clearing with every point let rays to walls and boxes skim just above the 4 cm table top, through its 12.5 cm voxel, so the table (top and legs) read as free space. Fix: clearing only from floor points (`max_obstacle_height` 0.05 on `lidar_clearing`). Afterwards the table, legs and a 5 cm pole all stayed lethal, and the gap under the 2.05 m arch beam stayed passable (cost 72, from inflation off the posts).

**Course results** (sim; goals sent with a `nav2_simple_commander` script; truth from `run_sim --log`, and from `--record` with true body-to-obstacle clearance (`mj_geomDistance`) on 2026-10-26 afternoon):

| Course | Goal (x, y) | Time | Contacts | Truly ended |
|---|---|---|---|---|
| `one_box` | 4, 0 | 29 s | 0 | 3.91, −0.23 |
| `slalom` | 7.5, 0 | 55 s | 0 | 7.36, −0.05 |
| `obstacle_course` (table not yet mapped) | 7.8, 1.65 | 51 s | 0 | 7.57, 1.57 |
| `obstacle_course`, via (2.6, −2.4) by the table, after the clearing fix | 7.8, 1.65 | 23 s + 45 s | 0 | 7.61, 1.56 |
| `open_floor` (afternoon, self-filter branch, see [[self-filter-trial]]) | 4, 0 | 18 s | 0 | 3.89, −0.10 |
| `one_box`, after the clearing fix | 4, 0 | 22 s, path 4.55 m | 0, closest 0.50 m | 3.89, 0.06 |
| `slalom`, after the clearing fix | 7.5, 0 | 52 s, path 9.9 m | 0, closest 0.28 m (gate_right) | 7.30, −0.08 |
| `obstacle_course`, direct, after the clearing fix | 7.8, 1.65 | 48 s, path 9.4 m | 0, closest 0.16 m (l_forward; thin pole 0.23 m) | 7.64, 1.56 |

Recorded with the `course` command ([[run-logging]]; pinned, so `runs show NAME` still has them):

| Run | Time | Path | Closest | Contacts | Recoveries |
|---|---|---|---|---|---|
| `2026-10-26_20-46-52Z_sim_one_box_first-test` | 25 s | 4.66 m | 0.51 m | 0 | 0 |
| `2026-10-26_20-48-51Z_sim_slalom_rangecut` (nav bag) | 54.5 s | 9.97 m | 0.28 m | 0 | 0 |
| `2026-10-26_20-50-13Z_sim_slalom_rangecut` (nav bag) | 63 s | 10.02 m | 0.29 m | 0 | 2 |

The first three rows of the first table ran before the clearing fix. Each course was checked to have a path at least 1.2 m wide everywhere (bottleneck robot radius 0.6 m, counting only obstacles in the 0.08–1.8 m band) and to be closed at the far end. But `slalom` (and `obstacle_course`) are open at the *start* end: the robot can walk around the outside of the side walls, and NavFn plans through unknown space as free. With inflation 1.0 / factor 2 it did exactly that on `slalom` (below). Close the start end before using slalom to compare costmap settings.

**Running course tests:** use the `course` command ([[run-logging]]): `pixi run --as-is ros2 run harp_ros course slalom --tag T --repeat 3`, then `runs compare`. It starts everything in the right order, starts the policy at the right moment, records the run and summarizes it. Starting the pieces by hand (README "Walk around obstacles with Nav2") is for looking around. If you do, the rules the course command automates are:
1. Start the sim (`--lidar --world W`), and wait for its banner before starting walker_node.
2. Start the policy after the sim's "Feet down" and within 3 s of it (the band releases 3 s later). A policy started while the robot hangs from the band flails, and the robot falls as soon as the band releases. With `--auto-release`, use 15, not 9: walker_node needs about 8 s to reach its stand pose.
3. Launch Nav2 once the policy runs. Once `odom` → `base_link` exists, activation takes about 2.5 s (2.4–2.7 s measured in 5 course runs); without it, bringup aborts after about 3.5 s of waiting for the transform.
4. Send goals in `odom` with `BasicNavigator.goToPose`. Don't call `waitUntilNav2Active()`: it waits for AMCL.

Notes:
- Run tests in a Terminal-panel tab so the teammate can watch; the MuJoCo and rviz2 windows open either way.
- Restart all three between courses: costmaps are in `odom`, and `walker_node` can't restart without the band.
- A fall on band release (pelvis 0.78 → 0.35 within a second, before Nav2 starts) means the policy was started too early or too late (step 2).
- Stopping `walker_node` damps the robot, so the sim then shows it falling; that's expected.
- rviz2 logs a GLSL "Sampler error" for the Map displays on the Mac. The costmaps still showed for the teammate.

**Inflation 1.0 m / factor 2 tried, not kept (2026-10-26)** (against 0.8 / 3; true clearance from the body):
- `obstacle_course`: 45 s against 48 s, nearest 0.18 against 0.16 m. More room at the thin pole (0.23 → 0.43 m) and the L-wall (0.53 → 0.74 m).
- `slalom`: 198 s and 42.5 m against 52 s and 9.9 m. The gaps cost more than the unknown space outside, so the robot went around the outside of the side walls, found the far end closed, and came back. Inside the course the path didn't change (nearest 0.25 against 0.28 m).
- Takeaway for crowded rooms: wider, gentler inflation makes NavFn prefer long detours through open or unknown space over gaps it can see.

**Measuring runs:** the course results above (afternoon of 2026-10-26) were measured with a temporary `run_sim --record` and scratch scripts. Since then, `course` and `run_recorder` record every run, and `summary.json` gives the same clearance metric ([[run-logging]]).

**Slalom first-gap recoveries: diagnosed 2026-10-26, not fixed yet.** In run `2026-10-26_20-50-13Z_sim_slalom_rangecut` (range-cut setup, nav extras recorded in its bag), Nav2 ran 2 recoveries at the first gap, around (2.6, 1.1), and still finished in 63 s with no contacts. Run `..._20-48-51Z_...` had none (54.5 s). `nav2.log` gives the cause: bt_navigator logged "Timed out while waiting for action server to acknowledge goal request for compute_path_to_pose", then "Failed to cancel/get result for follow_path", and the tree ran its Wait recovery. So the planner didn't acknowledge the 1 Hz replanning request within `default_server_timeout: 20` ms (our nav2_params.yaml, the Nav2 default) on a Mac that is also running the sim, the lidar and rviz2. That's a timing problem, not an obstacle problem, and it likely explains the self-filter runs' trouble at the same spot ([[self-filter-trial]]). Next: raise `default_server_timeout` (e.g. 100–200 ms), then compare with `course slalom --repeat 5` before and after.

**Runs with the logging additions (2026-10-26, tag v2; `runs check --tag v2`):**
- `one_box` FAILED its 40 s limit: 42 s, with 1 Wait recovery after a planner failure ("Failed to create plan with tolerance of 0.300" from (1.77, 1.04)). That's a real planning hiccup, not the acknowledge timeout.
- `slalom` ×3: 2 passed (60 s and 57 s, closest 0.27–0.29 m, 2 and 0 recoveries). One died at Nav2 start from sim lag ([[run-logging]] gotchas).
- `obstacle_course` passed (50 s, closest 0.18 m).
- Achieved over commanded forward speed is 0.82–0.85 (odom and truth).

**Still to do in the sim** (the README's open "Sim: more on obstacles and path following" milestone; a teammate wants the goal kept in progress until it's done, 2026-10-26):
- tighter margins (`robot_radius` 0.4 and inflation 0.8 m keep the robot's centre about 1 m from obstacle faces);
- collision_monitor (a last-layer stop, wanted before the robot) and maybe velocity_smoother;
- moving obstacles and people;
- raise bt_navigator's `default_server_timeout` and check the slalom first-gap recoveries go away (above);
- close `slalom` and `obstacle_course` at the start end;
- repeat runs (`course --repeat`): the comparisons above are one or two runs per setting, and GR00T's gait varies run to run.

After that comes the real lidar on the robot.
