---
name: feedback-minimal-setup
description: Keep the project minimal and goal-driven, and prefer open-source code we can read over Unitree's closed services
metadata:
  type: feedback
  updated: 2026-10-26T21:32:46Z
  covers: [ros2/src/harp_ros/setup.py, ros2/src/harp_ros/launch/*.py]
---

Build only what the current concrete goal needs. Don't add architecture ahead of decisions: no multi-node skeletons, custom messages or deploy tooling until a goal requires them. Prefer open-source code we can read, and keep it in the project: NVIDIA's GR00T policy is our walking controller ([[gr00t-wbc]]). Avoid Unitree's closed services (sport mode, LocoClient and the like) where possible. Unitree's open pieces are fine to use: the message definitions, unitree_sdk2_python, and the MuJoCo model. Prefer official sources over random community libraries.

**Why:** On 2026-10-24 a teammate pushed back on a full ROS 2 skeleton an agent had built ("you've put a bunch of code") and narrowed the scope to one goal: walk a set distance. On 2026-10-25 the mentors asked the team to move off Unitree's closed libraries and onto open code such as GR00T.

**How to apply:** Start from an official example. Add the smallest amount of our own code, and explain what each piece is for. When a goal is reached, ask before expanding the structure. **ROS 2 is adopted** (a teammate's call, 2026-10-25): lidar obstacle avoidance with Nav2 is the next goal after curved moves, and ROS 2 is the standard in robotics. The rule above still applies to it: add ROS 2 pieces (nodes, launch files, custom messages) only when a goal needs them. As of 2026-10-26 the ROS 2 pieces are `walker_node` (`/cmd_vel` in, `/odom` and `/tf` out), Nav2's launch and config ([[nav2-sim]]), and the run logging and course tests (`run_recorder`, `course`, `course_goal`, `runs`, [[run-logging]]), each added for a goal or a teammate's request. Events go as JSON on `std_msgs/String`, so there are still no custom messages. The walking code itself still talks DDS through unitree_sdk2_python. Still open: whether HARP's safety layer requires ROS 2 for joint commands. See [[project-overview]] and [[ros2-mac]].
