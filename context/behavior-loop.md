---
name: behavior-loop
description: Read before changing the autonomous loop (harp/behavior.py, the stand-in detector harp/sim_people.py, the people world) or replacing its placeholders with a real detector or speech
metadata:
  type: project
  updated: 2026-10-30T23:49:04Z
  covers: [submission/harp/behavior.py, submission/harp/sim_people.py, submission/worlds/people.*]
---

The robot on its own: wander, notice a person, walk up in one curve to about 1 m facing them, "talk", wander again. One node, `harp/behavior.py`, started by `entrypoint.sh` for `G1_TASK_COMMAND` `{"behavior": {...}}` in place of `course_goal.py`; it drives only through Nav2, so `approach_stop.py` still stands between Nav2 and our node ([[nav2-sim]]). It reuses `course_goal.py`'s Nav2 client (`go_to` with an `interrupt`, which cancels the goal). Its states, settings and events are in its docstring.

**Placeholders** (the maintainer asked for the full loop before the detector and speech exist, 2026-10-30)
- **Person detection:** `harp/sim_people.py`, sim only: every mover is a person; it publishes what a D435i detector should, the people in an 87° view out to 4 m, not hidden (one level ray from the head), ±5 cm, in `base_link`, as JSON on `/harp/people` with a stable id per person (a tracker's). A real detector should publish the same message; `behavior.py` then needs no change. On the robot nothing publishes it yet, so the robot only wanders.
- **Talk:** a `talk` event with the line it would say, then standing until the person leaves (2 s out of sight or 2 m away) or 30 s pass, the stand-in for them saying goodbye (the maintainer's numbers, 2026-10-30). Speech goes there ([[g1-builtin-apis]] for Unitree's TTS); a goodbye heard would end it too.

**Design reasons**
- Only people standing still are walked up to (under 0.3 m/s over 1 s): someone walking past is busy, and `approach_stop.py` stands the robot while they come at it. A person who moves 0.5 m from where it aims gets a new goal, up to 3 times.
- Wander points are in `odom` within a radius of the start: there is no map, so a point can land in an obstacle; Nav2 then fails or times out (60 s) and the next point is taken. A seed (`people.yaml`) makes the points repeat between runs.
- The approach goal is 1 m from the person's centre: the robot's centre stays 0.75 m from their surface, outside Nav2's lethal 0.4 m (`robot_radius`), so NavFn plans to it; the goal checker allows 0.25 m and 20°.
- Not walked up to again for 120 s (by id). A real tracker's ids may not last that long.

**Robot:** the mode starts Nav2, `approach_stop.py` and the loop on the robot too (with `lidar_relay.py`), after D-pad Left + B. Never run there yet: Nav2 has not run on real lidar ([[nav2-sim]], [[safety]]).

**Scoring:** `course.py people` (the task is in `people.yaml`): summary `loop` with each talk's true distance to that person and facing error (medians over its first 3 s, before anyone leaves) and how it ended, wander goals, people noticed, farthest from the start; `min_talks`, `talk_distance_m`, `max_talk_facing_deg` criteria. Events carry no sim time, so each is timed by the last `odom`/`timing` line before it (within 1 s).

**Results** (2026-10-30, `runs/2026-10-30_23-38-50Z_sim_people_loop-talk-end`, 150 s): all three people walked up to, first talk 39 s after the loop started, facing within 5°, at 1.16–1.32 m (the goal checker's 0.25 m lets it stop short of the 1 m goal; 1.32 failed `talk_distance_m`); person_a's talk ended as they walked off (13 s), person_b's at 30 s, person_c's when the run ended; 0 contacts, closest 0.53 m, farthest 3.1 m from the start, drift 1 cm. In `people`, person_a walks off at a fixed time after the robot sets off (movers can't react), timed from these runs' seed.
