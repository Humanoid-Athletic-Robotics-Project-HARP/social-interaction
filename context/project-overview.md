---
name: project-overview
description: Read when you need the big picture — the robot, the team and the decisions that shape the code
metadata:
  type: project
  updated: 2026-10-30T21:15:06Z
  covers: [README.md]
---

Stanford HARP project on a **Unitree G1 EDU with 29 joints** (6 per leg, 3 waist, 7 per arm). Use 29-DoF models, indices and examples (HIM's `g1_29dof_rev_1_0` model).

**Goals and status live only in the README's "Goals and progress"** ([[readme-and-memory-guidelines]]). End goal: walk around, notice people, walk up to them naturally, talk with them, avoid obstacles. Mapping may come later; global positioning isn't required.

**HARP** (Humanoid Athletics Robotics Project) is a robotics team with sub-teams by topic; this repo is the **Human Interaction** team. Never call it a course.

**Decisions that shape the code**
- Walking: NVIDIA's open GR00T policy, not Unitree's closed sport service (the mentors' request) ([[gr00t-wbc]], [[feedback-minimal-setup]]).
- Our code runs inside HARP's safety harness and simulator, HIMRobotics' `him-sim2real-management`, kept as an unmodified external dependency ([[him-sim2real]], [[safety]]): one Docker image (ROS 2 Foxy) in `submission/` ([[submission-image]]). `harp_node.py` drives HIM's GR00T walker from `/cmd_vel` and publishes our leg odometry; planned moves ([[planned-moves]]) and Nav2 with the head lidar ([[nav2-sim]]) drive `/cmd_vel`.
- Everything is tested in HIM's sim first and scored under `runs/` (`submission/course.py`). Nothing has run on the real robot yet (as of 2026-10-28).
- How to detect and talk to people is undecided.

Plans are exploratory; check official and open-source resources before committing to a design ([[unitree-resources]]).
