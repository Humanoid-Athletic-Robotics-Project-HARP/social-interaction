---
name: feedback-minimal-setup
description: Keep project setup minimal and goal-driven, using only official Unitree code and ROS 2
metadata:
  type: feedback
---

Build only what the current concrete goal needs. Don't add architecture ahead of decisions: no multi-node skeletons, custom messages or deploy tooling until a goal requires them. Prefer Unitree's official code (unitree_ros2, unitree_sdk2) over community libraries. Use ROS 2.

**Why:** On 2026-10-24 a teammate pushed back on a full ROS 2 skeleton an agent had built ("you've put a bunch of code"). They narrowed scope to one simple goal (walk a set distance), asked whether we were using official examples or random community libraries, and asked for ROS 2.

**How to apply:** Start from the official example. Add the smallest amount of our own code, and explain what each piece is for. When a goal is reached, ask before expanding the structure. See [[project-overview]].
