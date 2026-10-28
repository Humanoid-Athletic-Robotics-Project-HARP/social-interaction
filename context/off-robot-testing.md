---
name: off-robot-testing
description: Read when considering a simulator other than ours
metadata:
  type: reference
  updated: 2026-10-28T00:11:43Z
  covers: []
---

Web research, not tried by us.

**The key split:** code that calls Unitree's `sport` service can't be simulated (that controller is closed, on PC1). Code that runs its own controller over `rt/lowcmd`, as ours does, carries over between sim and robot almost unchanged.

**Other sims**
- unitree_rl_lab: train in IsaacLab, C++ `g1_ctrl` for sim2sim against unitree_mujoco and sim2real. Linux + NVIDIA.
- LeRobot: G1 MuJoCo sim with its own walking controllers (Holosoma, GR00T), on unitree_sdk2_python. Mac support unknown.
- unitree_sim_isaaclab: same DDS as the robot, manipulation tasks; Linux + RTX GPU.
- unitree_rl_gym `deploy/deploy_mujoco`: its own 12-joint leg policy in MuJoCo, no DDS.
- No official G1 Gazebo walking setup.

Real-robot sessions: [[safety]].
