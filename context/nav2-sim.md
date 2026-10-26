---
name: nav2-sim
description: Our Nav2 setup (harp_ros nav2.launch.py, config/nav2_params.yaml, behavior_trees/navigate.xml) and the obstacle courses: why each setting is what it is, the bugs found (tilted floor, vanishing tabletop), measured course results, how to run scripted course tests
metadata:
  type: reference
---

Built and sim-tested 2026-10-26 for "Get around obstacles" ([[project-overview]]). Background research is in [[lidar-nav2-research]]; the sim side (lidar, `--world`, contact report) is in [[mujoco-sim]].

**Pieces**
- `harp_ros/launch/nav2.launch.py` starts each server as its own node, like grove-g1: controller_server, planner_server, behavior_server and bt_navigator, plus one lifecycle manager and rviz2 (`rviz:=false` to skip it). We don't use nav2_bringup, because its launch files start docking_server, which RoboStack lacks. The pixi env has only the Nav2 packages we use (`ros2/pixi.toml`).
- Frames: everything is in `odom`, so goals are relative to where the policy started, and there's no map. The global costmap is a 20 × 20 m rolling window.
- Planner: NavFn, `allow_unknown`.
- Controller: Graceful (`v_linear` 0.1–0.3, `v_angular_max` 0.6, minimums well above GR00T's 0.05 standing switch). Goal tolerance 0.25 m / 0.35 rad; progress checker 0.3 m in 20 s.
- `behavior_trees/navigate.xml` is the stock replanning-and-recovery tree minus costmap clearing and BackUp. Clearing would forget obstacles in the blind ring near the feet; backing up is risky for a walker. Recoveries are only Wait and Spin.
- Costmaps: VoxelLayer, `robot_radius` 0.4, inflation 0.8 m with `cost_scaling_factor` 3. Two observation sources on the lidar:
  - marking: 0.08–1.8 m high, 0.6–3.0 m range. The minimum range is the self-filter (body points sit 0.11–0.56 m from the sensor).
  - clearing: only points below 0.05 m, i.e. rays that end on the floor.

**Bugs found and fixed (2026-10-26)**
- *Floor marked as obstacles.* With a fixed `base_link` → `livox_frame` transform, the floor 2.5–3 m away appeared ±0.12 m high, because GR00T stands with the torso tilted about 2°. About 4,000 phantom lethal cells. Fix: `LegOdometry.lidar_pose()` computes the lidar's real pose from the IMU and joints (MuJoCo FK, pelvis height from the planted ankle at `ANKLE_HEIGHT` 0.035 m), and `walker_node` publishes it on `/tf` with each odometry step. It matches MuJoCo exactly. Before the policy runs, a 10 Hz timer publishes the upright guess (`lidar_mount.from_base_link()`). After the fix the floor reads 0.00 m out to 3 m, and only the box (plus its 0.4 m inscribed ring) is lethal.
- *Thin tabletop erased.* Clearing with every point let rays to walls and boxes skim just above the 4 cm table top, through its 12.5 cm voxel, so the table (top and legs) read as free space. Fix: clearing only from floor points (`max_obstacle_height` 0.05 on `lidar_clearing`). Afterwards the table, legs and a 5 cm pole all stayed lethal, and the gap under the 2.05 m arch beam stayed passable (cost 72, from inflation off the posts).

**Course results** (sim, `--auto-release 9`; goals sent with a `nav2_simple_commander` script; truth from `run_sim --log`):

| Course | Goal (x, y) | Time | Contacts | Truly ended |
|---|---|---|---|---|
| `one_box` | 4, 0 | 29 s | 0 | 3.91, −0.23 |
| `slalom` | 7.5, 0 | 55 s | 0 | 7.36, −0.05 |
| `obstacle_course` (table not yet mapped) | 7.8, 1.65 | 51 s | 0 | 7.57, 1.57 |
| `obstacle_course`, via (2.6, −2.4) by the table, after the clearing fix | 7.8, 1.65 | 23 s + 45 s | 0 | 7.61, 1.56 |

The first three ran before the clearing fix, which only narrows clearing, so they weren't rerun. `open_floor` hasn't been run on its own. Each course was checked to have a path at least 1.2 m wide everywhere (bottleneck robot radius 0.6 m, counting only obstacles in the 0.08–1.8 m band) and to be closed at the far end, so it can't be bypassed outside the side walls.

**Running scripted course tests** (how the results above were made):
1. `run_sim --lidar --world NAME --auto-release 9 --log` with `PYTHONUNBUFFERED=1 ... | tee log`, so the log can be watched.
2. Start `walker_node`. When the log says "Feet down", call `/walker/start` (within 3 s).
3. Launch Nav2 once the policy runs. It takes about 30 s to reach "Managed nodes are active", and it aborts bringup if `odom` never appears.
4. Send the goal with `BasicNavigator.goToPose` in `odom`. Don't call `waitUntilNav2Active()`: it waits for AMCL.

Notes:
- Run the sim, `walker_node` and Nav2 in Terminal-panel tabs so the teammate can watch.
- Restart all three between courses: costmaps are in `odom`, and `walker_node` can't restart without the band.
- Once, GR00T fell forward on band release (pelvis 0.78 → 0.35 within a second, before Nav2 started); the rerun was fine.
- Stopping `walker_node` damps the robot, so the sim then shows it falling; that's expected.
- rviz2 logs a GLSL "Sampler error" for the Map displays on the Mac. The costmaps still showed for the teammate.

**Still to do in the sim** (the README's open "Sim: more on obstacles and path following" milestone; a teammate wants the goal kept in progress until it's done, 2026-10-26):
- tighter margins (`robot_radius` 0.4 and inflation 0.8 m keep the robot's centre about 1 m from obstacle faces);
- collision_monitor (a last-layer stop, wanted before the robot) and maybe velocity_smoother;
- moving obstacles and people;
- an `open_floor` run, and rerunning `one_box` and `slalom` after the clearing fix.

After that comes the real lidar on the robot.
