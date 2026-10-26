---
name: self-filter-trial
description: A box self-filter node for the head lidar (instead of Nav2's 0.6 m range cut), tried in the sim on 2026-10-26 and set aside; what it does, test results, the open slalom question, and how to bring it back from the self-filter branch
metadata:
  type: project
---

**Status:** set aside on 2026-10-26 (a teammate's call). main uses the 0.6 m `obstacle_min_range` cut ([[nav2-sim]]). The work is in commit `93058bd` on the branch `self-filter`. As of 2026-10-26 that branch is local to one teammate's Mac and not pushed. Check `git branch -a`; if it's missing, ask the team.

**What it is:** `harp_ros/self_filter.py` subscribes to `/utlidar/cloud_livox_mid360`. It looks up `base_link` → `livox_frame` (the newest transform from walker_node), drops every point inside a box around the body in `base_link`, and publishes the rest unchanged (all fields, byte for byte) on `/utlidar/cloud_self_filtered`.
- The box is x −0.15 to 0.29, y ±0.35, z 0 to 1.45: the stand-pose body (x −0.09 to +0.23, y ±0.29, from the MuJoCo meshes) plus 6 cm for waist sway. Parameters `box_min`, `box_max`, `input`, `output`, `base_frame`.
- `nav2.launch.py` starts it, and on that branch also takes `params_file:=`. Both costmaps' lidar sources read the filtered topic with `obstacle_min_range` 0.0.
- If the arms ever move (gestures), a fixed box won't hold; the filter would have to follow the arms, e.g. from forward kinematics.

**Results (sim, one or two runs each; truth from `run_sim --record`):**
- No body points get through: 750 scans standing and walking 4 m on `open_floor`, and 1,098 scans walking two goals with sharp turns. No point to be marked within 0.8 m, no lethal cells.
- Four 0.3 × 0.3 × 1.7 m boxes about 0.5 m from the centre: marked the same as with the range cut (20/19/11/29 against 19/18/11/28 lethal cells by bearing). The range cut keeps the lower part of anything standing that close, so it wasn't hiding people. The filter's only expected gain is for things at about sensor height 0.4–0.6 m from the centre (shelf edges, outstretched arms). That's untested; it needs a world with a raised bar near the robot.
- `obstacle_course`: 47 s, 0 contacts, nearest 0.16 m (range cut: 48 s, 0.16 m).
- `slalom`, two runs: 0 contacts, nearest 0.26–0.27 m (range cut: 0.28 m). **Open problem:** both runs had trouble at the first gap, around (1.6–2.1, 1.1) where the robot turns hard. Run 1 had a ~15 s hesitation loop (69 s total) and came within 0.29 m of wall_1 (range cut: 0.43 m). Run 2 had one Nav2 recovery, but its path and time (52 s) matched the range cut. The range-cut run had no recovery. Body points are ruled out by the turns test. Untested guess: near points of wall_1 that the range cut used to drop are now marked close to the robot and fail Graceful's collision check. Which recovery ran and why wasn't logged.

**To bring it back:**
1. `git cherry-pick 93058bd` onto the current branch (or `git merge self-filter`), then `pixi run --as-is build` in `ros2/`.
2. Before keeping it, settle the slalom question:
   - Run `slalom` 3–5 times with each filter.
   - Record the controller_server log, `/plan`, `/cmd_vel` and the local costmap at the first gap.
   - If near wall points cause it, try keeping a small `obstacle_min_range` (say 0.3) alongside the box.
3. Test the case it's meant for: a bar or shelf at 0.9–1.4 m height, 0.45–0.6 m in front of the robot.
