---
name: nav2-sim
description: Read before tuning Nav2 (submission/nav2/, the Graceful port), changing a course, or comparing course results — why each setting is what it is, current results and what's next
metadata:
  type: reference
  updated: 2026-10-28T08:59:18Z
  covers: [submission/nav2/**, submission/nav2_graceful_controller/**, submission/worlds/*, submission/harp/movers.py, submission/harp/lidar.py, submission/harp/lidar_mount.py]
---

Nav2 runs in our image on Foxy ([[submission-image]]); values are in `submission/nav2/nav2_params.yaml`, this note gives the reasons. Research behind it: [[lidar-nav2-research]].

**Setup and why**
- `nav2.launch.py` starts each server as its own node (controller, planner, recoveries, bt_navigator, one lifecycle manager). No map: everything in `odom` (leg odometry), goals in that frame; the global costmap is a 20 × 20 m rolling window.
- NavFn (`allow_unknown`) + the **Graceful Controller** (Park & Kuipers, the same law as our `{"to": ...}` move), so a goal is reached in one curve already facing its heading: the way to walk up to a person. Foxy lacks it, so it is our port, `submission/nav2_graceful_controller/` (Nav2 commit `d5d768b`, `HARP_PATCHES.md`). Speed minimums stay well above GR00T's 0.05 Balance switch.
- `failure_tolerance` 0.3 (ours, in the port): without it each moment with no collision-free trajectory (a person crossing) aborted the path, 6 recoveries and a failed `crossing`. `goal_dist_tolerance` 0.25 matches the goal checker, which Foxy doesn't hand to the controller.
- Regulated Pure Pursuit was the first Foxy controller (settings in commit `ca382d5`): faster (slalom 40 s), but no final-heading curve, and its 1 s collision look-ahead tripped on the walker's sway (0.5 s needed).
- `navigate.xml`: the stock tree minus costmap clearing (would forget obstacles in the blind ring near the feet) and BackUp (risky for a walker). Recoveries are Wait and Spin.
- **Footprint:** a circle, `robot_radius` 0.4: the forearms reach 0.355 m from `base_link`, and a polygon gives NavFn an inscribed radius of only 0.09 m.
- **Inflation** 0.8 m, factor 3 keeps the body edge about 0.45 m from obstacles (personal space). Only `robot_radius` blocks a gap. 1.0 / factor 2 made NavFn detour round slalom's side walls.
- **Live lidar TF** from leg kinematics (`harp/state.py`): a fixed transform made the floor 2.5–3 m out look ±0.12 m high (torso tilt).
- **STVL** (Foxy binary): marks in view that aren't seen again fade in about 2 s, out of view after 15 s. Nav2's VoxelLayer was rejected: its clearing rays slope down from the 1.2 m lidar, so a passing person left a permanent wall. STVL has no minimum range, but the arms are inside the footprint, which it clears. `track_unknown_space: false` on both costmaps: STVL never marks free, so with it the map stayed unknown and NavFn hugged obstacles.

**Courses** (`submission/worlds/`): `open_floor`, `one_box`, `slalom`, `obstacle_course` (table, pillars, thin pole, arch); people: `crossing` (crosses once, 3 s after the goal), `oncoming` (walks toward the robot, 0.5 m to its left). `crossing_shuttle` is unused. `slalom` and `obstacle_course` are open at the start end. Movers exist only in our lidar's world, so contacts with them come from `course.py`'s replay.

**Results with Graceful** (2026-10-28, one run each, `runs/*_graceful*`, all `within_limits`, 0 contacts, drift under 4 cm): open_floor 17.4 s; one_box 24.8 s, closest 0.60 m; slalom 52.5 s, 0.37 m, 0 recoveries; obstacle_course 48.0 s, 0.25 m, 0 recoveries; crossing 32.1 s, 0.62 m, 0 recoveries; oncoming 31.2 s, 0.17 m, 1 recovery. RPP's (`runs/*_rpp-ttc05`): 16.8, 20.5, 40.4, 43.2, 29.2, 25.1 s. The live viewer often makes HIM stop Nav2 runs on a MacBook Air (`host_stall`, `inconclusive`): numbers come from `--no-view` reruns when that happens.

**Open problem — oncoming people:** the person passes within 0.16–0.17 m (wrist): Nav2 treats them as standing still, and they're marked only within 3 m, about 2 s before they arrive.

**Next in the sim:** criteria for `crossing`/`oncoming`, repeat runs for spread, yielding to oncoming people, a collision monitor, closing the courses' start ends. Then the real lidar ([[him-sim2real]]).
