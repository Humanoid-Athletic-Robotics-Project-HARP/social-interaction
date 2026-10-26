---
name: project-overview
description: What this project is (Unitree G1 EDU 29-DoF humanoid, HARP Human Interaction team), where goals live (README), the decisions that shape the code, and the open safety-layer question
metadata:
  type: project
  updated: 2026-10-26T21:32:46Z
  covers: [README.md]
---

Stanford Humanoids project on a **Unitree G1 EDU with 29 joints**: 6 per leg, 3 in the waist (yaw, roll, pitch) and 7 per arm (including wrist pitch and yaw). Use 29-DoF models, joint indices and examples (e.g. `scene_29dof.xml`, see [[mujoco-sim]]).

**Goals and their status live in the README's "Goals and progress" section** (set 2026-10-26): move like a person, get around obstacles, find people and walk up to them, talk with people, and run safely on the real robot. How to edit it: [[readme-and-memory-guidelines]].

The team's original draft goals (2026-10-24): social interaction with people; smooth, human-like walking; spoken conversation through the robot's microphone and speaker, driven by an LLM; obstacle avoidance with the built-in lidar (Livox MID-360).

**HARP** (Humanoid Athletics Robotics Project) is a robotics team, not a course: don't call it a course or refer to course staff. It has sub-teams by topic, and this repo is the **Human Interaction** team ("realistic social engagement"). Other teams cover dance, paddle sports, combat, ball sports, and precision sports. HARP's "Lecture 2" document (a PDF, not in the repo) covers G1 specs, ROS 2 `rt/lowcmd`/`rt/lowstate`, and a method for reviewing prior research. Its ROS 2 code example publishes `JointCommand` to `/safety/precheck` and subscribes to `RobotFeedback` on `/safety/policy_observation`, with `session_id` and `observation_id` fields. That points to a HARP-provided safety layer that team code goes through. Its source and rules are not known yet (as of 2026-10-24).

**Decisions that shape the code** (facts; the status of each goal is only in the README):
- Walking: an open policy, NVIDIA's GR00T ([[gr00t-wbc]]), instead of Unitree's closed sport service (the mentors' request, 2026-10-25). `move_distance` walks set distances, turns and smooth curves to a point with it (`move_distance --to X Y [HEADING]`, Park & Kuipers' law in `smooth_control.py`, the same law as Nav2's Graceful Controller; merged through PR #1 on 2026-10-25), with our own leg odometry ([[move-distance]]).
- The end goal (a teammate, 2026-10-25): walk around, notice people, walk up to them naturally, talk with them, and avoid obstacles. Mapping may come later; global positioning isn't required.
- ROS 2 Jazzy is adopted (2026-10-25), with all code in three packages under `ros2/src/` ([[ros2-mac]]). `walker_node` drives GR00T from `/cmd_vel` and publishes `/odom` and `/tf`. Nav2 drives it around obstacles using the head lidar ([[nav2-sim]]).
- Everything is tested in our MuJoCo sim first ([[mujoco-sim]]). Nothing has run on the real robot yet (as of 2026-10-26).
- How to detect and talk to people is undecided.

**Testing:** every sim test whose numbers matter is recorded under `runs/` with the `course` command or run_recorder ([[run-logging]]).

**Open question for the real robot:** HARP's safety layer (above) may have to sit between our controller and the motors. Find out before the first hardware run.

**Why:** Everything is a draft. Expect the goals to change as research progresses.

**How to apply:** Treat plans as exploratory. Before committing to a design, check the official and open-source resources ([[unitree-resources]]).
