---
name: project-overview
description: What this project is (Unitree G1 EDU 29-DoF humanoid, social interaction) and its current draft goals and status
metadata:
  type: project
---

Stanford Humanoids project on a **Unitree G1 EDU with 29 joints**: 6 per leg, 3 in the waist (yaw, roll, pitch) and 7 per arm (including wrist pitch and yaw). Use 29-DoF models, joint indices and examples (e.g. `scene_29dof.xml`, see [[mujoco-sim]]).

**Goals and their status live in the README's "Goals and progress" section** (set 2026-10-26): move like a person, get around obstacles, find people and walk up to them, talk with people, and run safely on the real robot. How to edit it: [[goals-workflow]].

The team's original draft goals (2026-10-24): social interaction with people; smooth, human-like walking; spoken conversation through the robot's microphone and speaker, driven by an LLM; obstacle avoidance with the built-in lidar (Livox MID-360).

**HARP** (Humanoid Athletics Robotics Project) is a robotics team, not a course: don't call it a course or refer to course staff. It has sub-teams by topic, and this repo is the **Human Interaction** team ("realistic social engagement"). Other teams cover dance, paddle sports, combat, ball sports, and precision sports. HARP's "Lecture 2" document (a PDF, not in the repo) covers G1 specs, ROS 2 `rt/lowcmd`/`rt/lowstate`, and a method for reviewing prior research. Its ROS 2 code example publishes `JointCommand` to `/safety/precheck` and subscribes to `RobotFeedback` on `/safety/policy_observation`, with `session_id` and `observation_id` fields. That points to a HARP-provided safety layer that team code goes through. Its source and rules are not known yet (as of 2026-10-24).

**Current concrete goal:** make the robot walk a specific distance. Since 2026-10-25 we use an open policy, GR00T ([[gr00t-wbc]]), instead of Unitree's closed sport service. `move_distance` walks with `gr00t/walk.py` and our leg odometry (all in `harp_control`). It works in our sim (`harp_sim`, [[mujoco-sim]]) and hasn't been tried on the real robot. See [[feedback-minimal-setup]].

**End goal and next movement goal** (a teammate, 2026-10-25): the robot should walk around, see people, walk up to them, talk to them and interact; mapping the surroundings may come later. Global odometry or positioning isn't required. The next movement goal is natural, human-like motion: reach a point relative to the robot (e.g. 2 m forward, 1 m left) in one smooth curved walk, turning while moving, so it can approach a person who is ahead and to one side. Keep `move_distance`'s discrete straight walks and in-place turns working alongside it. Done in the sim and merged into `main` through PR #1 (2026-10-25; the `smooth-movement` branch has since been deleted): `move_distance --to X Y [HEADING]` uses Park & Kuipers' control law (`smooth_control.py`, same law as Nav2's Graceful Controller) and ends within about 8 cm and 5°. **Next goal** (a teammate, 2026-10-25): lidar obstacle avoidance with Nav2. ROS 2 is adopted for it ([[feedback-minimal-setup]]), with Jazzy on development machines ([[ros2-mac]]). Since 2026-10-25 all code is in ROS 2 packages under `ros2/src/` (`harp_control`, `harp_sim`, `harp_ros`). `harp_ros`'s walker node drives GR00T from `/cmd_vel` and publishes `/odom` and `/tf` ([[ros2-mac]]), sim-tested. Nav2 drives it through the sim courses (`one_box`, `slalom`, `obstacle_course`) with the simulated head lidar, with no contacts (2026-10-26, [[nav2-sim]]); more sim work on obstacles and path following is still to do, and nothing is on the robot yet. How to detect and talk to people is undecided.

**Open question for the real robot:** HARP's safety layer (above) may have to sit between our controller and the motors. Find out before the first hardware run.

**Why:** Everything is a draft. Expect the goals to change as research progresses.

**How to apply:** Treat plans as exploratory. Before committing to a design, check the official and open-source resources ([[unitree-resources]]).
