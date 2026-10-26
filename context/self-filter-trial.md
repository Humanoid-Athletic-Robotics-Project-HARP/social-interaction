---
name: self-filter-trial
description: Read when revisiting how the lidar filters out the robot's own body, or if the 0.6 m range cut starts hiding real obstacles
metadata:
  type: project
  updated: 2026-10-26T22:40:11Z
  covers: []
---

A box self-filter node was tried in place of Nav2's 0.6 m `obstacle_min_range` cut and set aside by a teammate. It lives in commit `93058bd` on the local branch `self-filter` (not pushed; check `git branch -a`, ask the team if missing).

**What it does:** `harp_ros/self_filter.py` drops cloud points inside a box around the body in `base_link` (x −0.15 to 0.29, y ±0.35, z 0 to 1.45: the stand pose plus 6 cm) and republishes on `/utlidar/cloud_self_filtered`. A fixed box won't hold once the arms move.

**Result:** no body points got through, and obstacles near the robot were marked the same as with the range cut, so it gained nothing on the current courses. Its only expected gain is for things at sensor height 0.4–0.6 m from the centre (a shelf edge, an outstretched arm), untested. Its slalom trouble at the first gap is likely the planner acknowledge timeout ([[nav2-sim]]).

**To bring it back:** cherry-pick `93058bd` (keep main's `nav2.launch.py`, add only the `self_filter` node), fix the timeout first, compare with `course slalom --repeat 5 --tag ...` ([[run-logging]]), and add a world with a bar at 0.9–1.4 m height, 0.45–0.6 m in front of the robot.
