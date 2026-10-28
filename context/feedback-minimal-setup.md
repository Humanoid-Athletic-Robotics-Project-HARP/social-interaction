---
name: feedback-minimal-setup
description: Read before adding new nodes, packages, messages or tooling — build only what the current goal needs, prefer open code
metadata:
  type: feedback
  updated: 2026-10-28T00:18:02Z
  covers: [ros2/src/harp_ros/setup.py, ros2/src/harp_ros/launch/*.py, ros2/src/harp_eval/setup.py, ros2/src/harp_eval/launch/*.py]
---

Build only what the current goal needs. No architecture ahead of decisions: no node skeletons, custom messages or deploy tooling until a goal requires them. Start from an official example, add the smallest amount of our own code, and explain what each piece is for. When a goal is reached, ask before expanding the structure.

Prefer open code we can read and keep in the project (GR00T, [[gr00t-wbc]]). Avoid Unitree's closed services (sport mode, LocoClient). Unitree's open pieces are fine: message definitions, unitree_sdk2_python, the MuJoCo model. Prefer official sources over random community libraries.

**Why:** a teammate rejected a full ROS 2 skeleton an agent built unasked, and the mentors asked the team to move off Unitree's closed libraries.

ROS 2 is adopted, but the same rule applies to it: add ROS 2 pieces only when a goal needs them. Events go as JSON on `std_msgs/String`, so there are still no custom messages.
