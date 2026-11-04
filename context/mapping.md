---
name: mapping
description: Read before building or using a map in the sim (course.py --mapping / --map), changing harp/mapper.py, occupancy.py or localizer.py, or judging map results
metadata:
  type: project
  updated: 2026-11-04T10:23:19Z
  covers: [submission/harp/mapper.py, submission/harp/occupancy.py, submission/harp/localizer.py, submission/harp/map_floor.py, submission/tests/test_mapping.py]
---

**What it is:** a saved building map plus localization for Nav2, so the planner keeps walls it has seen before, also out of view and long ago (without one Nav2 plans through unseen space and forgets walls after STVL's 15 s: the office's route flip-flops and the held-out partition doorways, [[predictive-avoidance]]). Lam chose to have it built (Nov 3), in place of designing it himself.

**How a run gets one:**
- `course.py WORLD --mapping` walks the course in an empty building (`--crowd none` unless given: the movers parked far away, `harp/movers.py` `parked_path`) building the map; Nav2 already plans on the growing map. The map leaves the container as "map" events every 5 s; scoring keeps the last as `map.pgm`/`map.yaml` in the run folder.
- `course.py WORLD --map RUN` puts that map in the task (`G1_TASK_COMMAND` "map", base64 zlib, about 15 KB); the robot is taken to start where the mapping walk did (`"map_pose"` otherwise).
- `entrypoint.sh` starts `harp/mapper.py` only for such a task; `nav2.launch.py` then puts a static layer (on `/map`) first in both costmaps, still in odom; `course_goal.py` sends each goal through map -> odom.

**Map** (`harp/occupancy.py`): 5 cm log-odds cells from `/harp/static_cloud` (moving objects left out). A return at 0.08–1.8 m marks; a ray clears only cells it passes below 0.4 m (a ray high over a low box proves nothing about it), so floor returns clear.

**Localization** (`harp/localizer.py`): a correlative search (Olson 2009) on a smooth likelihood field round the estimate, coarse to fine, turned about the robot. Turned about odom's origin, 8 m behind, each 1° of the torso's heading sway moved the robot 14 cm and the window couldn't reach the fix: HIM showed corrections jumping 28 cm. The estimate takes 30% of each match's position and 5% of its heading (heading jitters about 1° scan to scan). Matches need 80 inlier returns and 40% fit, and may move the robot at most 0.3 m.

**Results (HIM, aware crowd, 0.5 m/s; 7 held-out worlds, office, atrium, building; code `c28b5be`):** mapping walks reach their goals with clean maps. At the goal: game core with the map 7 of 11, without 6 of 10 (passed every check 3 and 3); NMPC 6 of 10 and 5 of 10 (passed 0 and 0); median time at the goal 46 / 70 s (game), 61 / 80 s (NMPC). A small gain from the map, a larger one from the game core. Map-only misses: walkers waiting in the only doorway, so Nav2 found no path and gave up (6 recoveries); `course_goal.py` now sends the goal again. Drift test (`--odom-drift 0.03 6`): localization found the drift (−9.5°) but the goal stayed where it was sent, 0.5 m off; `course_goal.py` now resends it. Localization offline: within about 5 cm of the truth at the robot. Benchmark: [[predictive-avoidance]].

**Fresh held-out worlds 20261127–32** (code `3c7ced2`, with both goal fixes): with the map 4 of 6 at the goal (2 passed every check), without 3 of 6 (1). The resend after drift cut the drift test's end error from about 0.53 to 0.34–0.39 m; the resend after a failure didn't help where it was needed (a person stood by the door all run). **The map's own failure:** a still person by the only doorway is a wall to Nav2 and, with every wall known, leaves it no route (held-out 20261129: 37 planning failures). `HARP_PREDICTIVE_MAP_FLOOR=1` leaves still person-sized tracks on the map's open floor out of Nav2's map (`harp/map_floor.py`); Nav2 then kept its route (1 failure), but the robot stood off at the doorway with the avoidance core yielding, and none of 5 such runs got through, so it is off by default. What's left is the doorway standoff, map or not.

**Not done:** nothing on the robot (its odometry drifts more and its map would need a mapping walk there); no map editing (a standing person on the mapping walk stays a wall); doors closing after mapping.
