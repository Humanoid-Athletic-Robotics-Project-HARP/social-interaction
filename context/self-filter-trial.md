---
name: self-filter-trial
description: Read when revisiting how the lidar filters out the robot's own body, or if footprint clearing starts hiding real obstacles
metadata:
  type: project
  updated: 2026-10-28T08:59:18Z
  covers: []
---

A box self-filter node was tried in place of the VoxelLayer's 0.6 m `obstacle_min_range` cut and set aside by a teammate. Now the costmaps use STVL, which has no range cut: the body is marked but lies inside the footprint, which STVL clears ([[nav2-sim]]). It lives in commit `93058bd` on the local branch `self-filter` (not pushed; check `git branch -a`, ask the team if missing).

**What it does:** `harp_ros/self_filter.py` (on the old ROS 2 stack, commit `93058bd`) drops cloud points inside a box around the body in `base_link` (x −0.15 to 0.29, y ±0.35, z 0 to 1.45: the stand pose plus 6 cm) and republishes on `/utlidar/cloud_self_filtered`. A fixed box won't hold once the arms move.

**Result:** no body points got through, and obstacles near the robot were marked the same as with the range cut, so it gained nothing on the current courses. Its only expected gain was for things at sensor height 0.4–0.6 m from the centre (a shelf edge, an outstretched arm), untested; with STVL only things within the 0.4 m footprint are hidden. Its slalom trouble at the first gap is likely the planner acknowledge timeout ([[nav2-sim]]).

**To bring it back:** port it from commit `93058bd` into `submission/harp/` as its own process, fix the timeout first, compare with `course.py slalom --repeat 5 --tag ...` and `--compare` ([[submission-image]]), and add a world with a bar at 0.9–1.4 m height.
