---
name: off-robot-testing
description: Read when considering a simulator other than ours, or planning real-robot practice (gantry, damping, first motions)
metadata:
  type: reference
  updated: 2026-10-26T22:40:10Z
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

**Real-robot practice:** start hanging from a gantry (rope through the shoulder buckles, feet clear). L1+A is damping, the safe state; the robot sinks, so lower it by hand. Small, reversible motions first; check the mode before sending commands. Code can run from a laptop on 192.168.123.x instead of the Jetson.
