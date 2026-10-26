---
name: project-overview
description: What this project is (Unitree G1 EDU 29-DoF humanoid, social interaction) and its current draft goals and status
metadata:
  type: project
---

Stanford Humanoids project on a **Unitree G1 EDU with 29 joints**: 6 per leg, 3 in the waist (yaw, roll, pitch) and 7 per arm (including wrist pitch and yaw). Use 29-DoF models, joint indices and examples (e.g. `scene_29dof.xml`, see [[mujoco-sim]]).

Draft goals, all still in the research phase as of 2026-10-24:

1. Social interaction with people.
2. Smooth, human-like walking.
3. Spoken conversation through the robot's microphone and speaker, driven by an LLM.
4. Obstacle avoidance with the built-in lidar (Livox MID-360).

**HARP** stands for Humanoid Athletics Robotics Project ("Intro to Humanoid Robotics"). It runs as a lecture series with teams by topic, and this repo is the **Human Interaction** team ("realistic social engagement"). Other teams cover dance, paddle sports, combat, ball sports, and precision sports. Lecture 2 (a course PDF; it is not in the repo) covers G1 specs, ROS 2 `rt/lowcmd`/`rt/lowstate`, and a method for reviewing prior research. Its ROS 2 code example publishes `JointCommand` to `/safety/precheck` and subscribes to `RobotFeedback` on `/safety/policy_observation`, with `session_id` and `observation_id` fields. That points to a HARP-provided safety layer that team code goes through. Its source and rules are not known yet (as of 2026-10-24).

**Current concrete goal:** make the robot walk a specific distance. Since 2026-10-25 we use an open policy, GR00T ([[gr00t-wbc]]), instead of Unitree's closed sport service. `move_distance.py` walks with `gr00t/walk.py` and its own leg odometry. It works in our sim (`sim/run_sim.py`, [[mujoco-sim]]) and hasn't been tried on the real robot. See [[feedback-minimal-setup]].

**End goal and next movement goal** (a teammate, 2026-10-25): the robot should walk around, see people, walk up to them, talk to them and interact; mapping the surroundings may come later. Global odometry or positioning isn't required. The next movement goal is natural, human-like motion: reach a point relative to the robot (e.g. 2 m forward, 1 m left) in one smooth curved walk, turning while moving, so it can approach a person who is ahead and to one side. Keep `move_distance.py`'s discrete straight walks and in-place turns working alongside it. Done in the sim on branch `smooth-movement` (2026-10-25): `move_distance.py --to X Y [HEADING]` uses Park & Kuipers' control law (`smooth_control.py`, same law as Nav2's Graceful Controller) and ends within about 8 cm and 5°. **Next goal** (a teammate, 2026-10-25): lidar obstacle avoidance with Nav2. ROS 2 is adopted for it ([[feedback-minimal-setup]]), with Jazzy on development machines ([[ros2-mac]]). The repo has the ROS 2 environment (`ros2/`) but no ROS 2 code yet. How to detect and talk to people is undecided.

**Open question for the real robot:** the course's HARP safety layer (above) may have to sit between our controller and the motors. Find out before the first hardware run.

**Why:** Everything is a draft. Expect the goals to change as research progresses.

**How to apply:** Treat plans as exploratory. Before committing to a design, check the official and open-source resources ([[unitree-resources]]).
