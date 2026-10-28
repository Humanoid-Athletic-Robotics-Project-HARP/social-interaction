---
name: rl-ideas
description: Read when someone asks about using reinforcement learning on the G1 — options, tools and a suggested order (brainstorm, not a team goal)
metadata:
  type: project
  updated: 2026-10-28T08:59:18Z
  covers: []
---

Brainstorm, nothing tried. Making any of it a goal is the team's call ([[readme-and-memory-guidelines]]).

**Two levels**
- **High level** (policy outputs `/cmd_vel` into `harp_node.py`): cheap to train on a laptop (2D env with the walker's measured lag, PPO via Stable-Baselines3), low sim-to-real risk since GR00T hides the legs. Tasks: social navigation (goal progress, personal-space and smoothness penalties; compare against Nav2), approaching moving people or groups, deciding whom to approach once a person detector exists. Test as a ROS 2 node in `harp_sim` before the robot.
- **Low level** (joint targets at 50 Hz, replacing or extending GR00T, [[gr00t-wbc]]): needs an NVIDIA GPU and thousands of parallel robots; high risk. Tasks: human-like gait from mocap (AMASS/LAFAN1 + AMP), gestures via a motion tracker (BeyondMimic, ASAP, GEAR-SONIC), fall recovery. Tools: IsaacLab + `unitree_rl_lab`, or MuJoCo Playground. GPU time would come from Stanford, HARP or a cloud. Deploy as ONNX with GR00T's interface, check sim-to-sim in `harp_sim`, then the gantry; train with domain randomization.
- Before training gestures, try moving the arms under HIM's GR00T walker in the sim: the decoupled design is meant to let arms move while the legs balance.

**Bigger ideas:** co-speech gestures; learning approach style from people's ratings; handshakes (need force-aware arms, maybe hands); one policy that walks while gesturing; perceptive stairs from the lidar.

**Suggested order:** GR00T on the real robot first (settles the sim-to-real gap and the safety-layer question), social navigation meanwhile, gait or gestures once there's GPU access.
