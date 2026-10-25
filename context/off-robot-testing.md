---
name: off-robot-testing
description: How people test G1 code without the robot (web research 2026-10-25): sim options, sport-service stand-ins, gantry practice
metadata:
  type: reference
---

From web research on 2026-10-25. Not tried by us unless [[mujoco-sim]] says so.

**The key split.** Code that calls Unitree's built-in `sport` service (LocoClient, `move_distance.py`) can't be simulated directly, because that controller is closed and runs on PC1. Code that runs its *own* walking controller over `rt/lowcmd` (an RL policy) carries over almost unchanged between unitree_mujoco and the robot. That's why research groups mostly work the second way. On the real robot that path means turning off Unitree's controller (debug mode, L2+R2 with the robot hanging and damped), after which our code is all that keeps the robot up.

**Sim options found**
- unitree_mujoco: low-level only, confirmed again by others ("no /api/sport service in simulation"). See [[mujoco-sim]].
- unitree_rl_lab: train in IsaacLab, then the C++ `g1_ctrl` runs sim2sim against unitree_mujoco and sim2real with `--network eth0` after the onboard controller is shut off. Remote: L2+Up stands, R1+X runs the policy. Linux/apt build.
- LeRobot (Hugging Face): G1 29/23-DoF MuJoCo sim with its own walking controllers (Holosoma, GR00T) and gamepad driving, built on unitree_sdk2_python. Python 3.12/conda. Mac support isn't stated or tested.
- unitree_sim_isaaclab (official): same DDS as the real robot, but the tasks are manipulation-focused, and it needs Linux plus an NVIDIA RTX GPU. No sport service.
- Living-Proof-Labs/unitree-foxglove-sdk: a Docker "loco" container stands in for LocoClient using the rl_gym `motion.pt` in MuJoCo. Very new (0 stars), useful only as a reference design. g1pilot skips LocoClient in sim and sends velocity commands to an Isaac Lab gait instead.
- No official G1 Gazebo walking setup. unitree_ros says Gazebo can't do high-level walking.

**Real-robot practice.** Start hanging on a gantry or harness (rope through the shoulder buckles, feet clear of the floor). L1+A is damping, the safe state; the robot sinks, so lower it by hand. Start with small, reversible motions. Check the robot's mode before sending commands. Code can run from a laptop on 192.168.123.x over Ethernet instead of on the Jetson. A G1 EDU+ on firmware 1.5.3 reportedly accepted high-level loco calls and then didn't move (unitree_sdk2_python#170).

Related: [[g1-builtin-apis]], [[project-overview]]
