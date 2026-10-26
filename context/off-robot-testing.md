---
name: off-robot-testing
description: How people test G1 code without the robot (web research 2026-10-25): own-controller vs sport-service split, sim options, gantry practice
metadata:
  type: reference
  updated: 2026-10-26T21:32:46Z
  covers: []
---

From web research on 2026-10-25. Not tried by us unless [[mujoco-sim]] or [[gr00t-wbc]] says so.

**The key split.** Code that calls Unitree's built-in `sport` service (LocoClient) can't be simulated, because that controller is closed and runs on PC1. Code that runs its *own* walking controller over `rt/lowcmd` carries over almost unchanged between sim and robot. Research groups mostly work the second way, and we now do too ([[gr00t-wbc]]).

**Sim options found**
- unitree_mujoco: low-level only ("no /api/sport service in simulation"). We use its model with our own runner ([[mujoco-sim]]).
- unitree_rl_lab: train in IsaacLab, then the C++ `g1_ctrl` runs sim2sim against unitree_mujoco and sim2real with `--network eth0` after the onboard controller is shut off. Remote: L2+Up stands, R1+X runs the policy. Linux/apt build.
- LeRobot (Hugging Face): G1 29/23-DoF MuJoCo sim with its own walking controllers (Holosoma, GR00T) and gamepad driving, built on unitree_sdk2_python. Python 3.12/conda. Mac support isn't stated or tested.
- unitree_sim_isaaclab (official): same DDS as the real robot, but the tasks are manipulation-focused, and it needs Linux plus an NVIDIA RTX GPU. No sport service.
- No official G1 Gazebo walking setup. unitree_ros says Gazebo can't do high-level walking.

**Real-robot practice.** Start hanging on a gantry or harness (rope through the shoulder buckles, feet clear of the floor). L1+A is damping, the safe state; the robot sinks, so lower it by hand. Start with small, reversible motions. Check the robot's mode before sending commands. Code can run from a laptop on 192.168.123.x over Ethernet instead of on the Jetson. A G1 EDU+ on firmware 1.5.3 reportedly accepted high-level loco calls and then didn't move (unitree_sdk2_python#170).

Related: [[g1-builtin-apis]], [[project-overview]]
