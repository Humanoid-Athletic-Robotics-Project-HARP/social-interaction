---
name: project-overview
description: Read when you need the big picture — the robot, the team, the decisions that shape the code, and the open safety-layer question
metadata:
  type: project
  updated: 2026-10-27T21:32:08Z
  covers: [README.md]
---

Stanford HARP project on a **Unitree G1 EDU with 29 joints** (6 per leg, 3 waist, 7 per arm). Use 29-DoF models, indices and examples (`scene_29dof.xml`).

**Goals and status live only in the README's "Goals and progress"** ([[readme-and-memory-guidelines]]). End goal: walk around, notice people, walk up to them naturally, talk with them, avoid obstacles. Mapping may come later; global positioning isn't required.

**HARP** (Humanoid Athletics Robotics Project) is a robotics team with sub-teams by topic; this repo is the **Human Interaction** team. Never call it a course.

**Decisions that shape the code**
- Walking: NVIDIA's open GR00T policy, not Unitree's closed sport service (the mentors' request) ([[gr00t-wbc]], [[feedback-minimal-setup]]).
- ROS 2 Jazzy, four packages under `ros2/src/` (walking, sim, ROS 2 and Nav2, testing) plus a copy of the STVL costmap layer ([[ros2-mac]]). `walker_node` drives GR00T from `/cmd_vel`; Nav2 drives it around obstacles with the head lidar ([[nav2-sim]]).
- Everything is tested in our MuJoCo sim first ([[mujoco-sim]]) and recorded under `runs/` ([[run-logging]]). Nothing has run on the real robot yet (as of 2026-10-26).
- How to detect and talk to people is undecided.

**Open question before the first hardware run:** HARP's "Lecture 2" PDF (not in the repo) has a ROS 2 example that publishes `JointCommand` to `/safety/precheck` and subscribes to `RobotFeedback` on `/safety/policy_observation` (`session_id`, `observation_id`). That suggests a HARP safety layer between team code and the motors. Its source and rules are unknown.

Plans are exploratory; check official and open-source resources before committing to a design ([[unitree-resources]]).
