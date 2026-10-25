---
name: project-overview
description: What this project is (Unitree G1 EDU humanoid, social interaction) and its current draft goals and status
metadata:
  type: project
---

Stanford Humanoids project on a **Unitree G1 EDU** humanoid. Draft goals, all still in the research phase (as of 2026-10-24 no code has been written):

1. Social interaction with people.
2. Smooth, human-like walking.
3. Spoken conversation through the robot's microphone and speaker, driven by an LLM.
4. Obstacle avoidance with the built-in lidar (Livox MID-360).

**HARP** stands for Humanoid Athletics Robotics Project ("Intro to Humanoid Robotics"). It runs as a lecture series with teams by topic, and this repo is the **Human Interaction** team ("realistic social engagement"). Other teams cover dance, paddle sports, combat, ball sports, and precision sports. Lecture 2 (a PDF the user has; it is not in the repo) covers G1 specs, ROS 2 `rt/lowcmd`/`rt/lowstate`, and a method for reviewing prior research. Its ROS 2 code example publishes `JointCommand` to `/safety/precheck` and subscribes to `RobotFeedback` on `/safety/policy_observation`, with `session_id` and `observation_id` fields. That points to a HARP-provided safety layer that team code goes through. Its source and rules are not known yet (as of 2026-10-24).

**Why:** Everything is a draft. Expect the goals to change as research progresses.

**How to apply:** Treat plans as exploratory. Before committing to a design, check the official Unitree resources ([[unitree-resources]]).
