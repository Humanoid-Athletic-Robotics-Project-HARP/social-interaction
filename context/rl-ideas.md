---
name: rl-ideas
description: Ideas for RL on the G1 (2026-10-26, not a team goal yet): high-level /cmd_vel policies vs low-level joint policies, tools, deployment path, bigger-scope projects, suggested order
metadata:
  type: project
---

Brainstorm from a teammate's question on 2026-10-26. None of this is a goal in the README yet; adding one needs the team ([[goals-workflow]]). Nothing here has been tried.

**Two levels.** RL can sit above `walker_node` (outputs `/cmd_vel`) or replace or extend GR00T (outputs joint targets at 50 Hz, like `gr00t/walk.py`, [[gr00t-wbc]]).
- High level: cheap to train (simple 2D sim, CPU), low sim-to-real risk because GR00T hides the leg dynamics. The gap is mostly perception.
- Low level: needs thousands of parallel robots on an NVIDIA GPU, high sim-to-real risk, and it's the robot's balance on the line.

**High-level tasks** (start here):
- Social navigation: reach a goal while keeping personal space around people and boxes. Rewards: progress, a penalty inside about 1 m of a person, a collision penalty, a smoothness penalty. Compare it against Nav2.
- Approaching people: `move_distance --to` already handles a person who stands still. RL is worth it for people who move, groups (join a circle without cutting through it), and people turning away.
- Deciding whether to approach someone, from their position and movement, once a person detector exists.
- Method: a small 2D env (point robot with a speed limit and the walker's measured lag, scripted pedestrians) or an existing CrowdNav-style sim, PPO (Stable-Baselines3) on a laptop. Then run the policy as a ROS 2 node publishing `/cmd_vel` into `harp_sim` with `--lidar --box` ([[mujoco-sim]]) before the robot.

**Low-level tasks:**
- Human-like gait: a style reward learned from human motion capture (AMASS or LAFAN1 retargeted to the G1) with Adversarial Motion Priors (AMP) or motion imitation. Natural arm swing, a human stride, and better speed tracking (GR00T walks at about 85% of the commanded speed).
- Gestures: a whole-body motion tracker that plays retargeted clips (waving, nodding with the waist, pointing, a small bow) while balancing. Examples: BeyondMimic, ASAP, NVIDIA's GEAR-SONIC.
- Fall recovery and getting back up: useful, but the riskiest to test on hardware.
- Check first: GR00T's lower-body policy observes all 29 joints, and NVIDIA's decoupled design is meant to let the arms move while the legs balance. Try moving the arms under `walk.py` in the sim before training anything for gestures.
- Tools: IsaacLab with `unitree_rl_lab` (G1 tasks ready, Linux and NVIDIA only, [[off-robot-testing]]), or MuJoCo Playground (JAX, has G1 joystick walking tasks, needs a GPU to train at a useful speed). The team's Macs can't train these; GPU time would come from Stanford (e.g. Sherlock), HARP or a cloud provider.
- Deployment path: export to ONNX with the same 50 Hz interface as GR00T, check it sim-to-sim in `harp_sim` (a different simulator from training), then the gantry. Train with randomized friction, mass, motor strength, latency and sensor noise.

**Bigger-scope ideas:**
- Co-speech gestures: the LLM's speech goes through a gesture model, and the RL tracker performs the motion while balancing. Combines moving like a person, body language and talking.
- Learning approach style from people's ratings (natural vs creepy): fit a reward model to the ratings and train the approach policy on it (RLHF for robot behavior).
- Handshakes and high-fives: need compliant, force-aware arm control and probably hands (whether ours has Dex3 hands is unconfirmed, [[gr00t-wbc]]).
- One whole-body policy that walks with style while gesturing, e.g. walking up to someone while waving.
- Perceptive locomotion over stairs and curbs from the MID-360 height map: off-mission, but the classic humanoid RL demo.

**Suggested order:**
1. Run GR00T on the real robot first. Low-level RL depends on knowing the real sim-to-real gap, and the HARP safety-layer question has to be settled anyway ([[project-overview]]).
2. Meanwhile, social navigation as the first RL project: it runs on the hardware we have, it's safe, and it serves "get around obstacles" and "walk up to people".
3. Gait style or gestures once there's GPU access.
