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

**Open question for the real robot:** the course's HARP safety layer (above) may have to sit between our controller and the motors. Find out before the first hardware run.

**Why:** Everything is a draft. Expect the goals to change as research progresses.

**How to apply:** Treat plans as exploratory. Before committing to a design, check the official and open-source resources ([[unitree-resources]]).
