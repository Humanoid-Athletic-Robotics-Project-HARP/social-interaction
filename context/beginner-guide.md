---
name: beginner-guide
description: Read only when someone is new to the repo (just cloned it, "how do I get started") — the order to walk them through the README
metadata:
  type: reference
  updated: 2026-10-28T07:49:42Z
  covers: [README.md]
---

Walk the newcomer through one stage at a time, and ask which they need: the sim (laptop) or the robot. The README has the commands; point to its section for each stage instead of retyping it.

**0. Orientation:** the robot and team ([[project-overview]]), the house rule ([[feedback-minimal-setup]]), and that `context/` is shared agent memory indexed in `context/MEMORY.md`.

**1. Sim (start here).** Ask their OS first ([[team-context]]). On the `simulation-migration` branch the sim is HARP's harness: README "HARP's safety harness: `submission/`" (setup, build, a move, a course; [[submission-image]]); the `ros2/` steps below are the stack it replaces, broken on that branch ([[him-sim2real]]).
1. README "Setup", steps 1–4.
2. README "Run it in the sim": `run_sim` and `walk`. The start-up order is the usual stumbling block: feet on the floor, Shift+P, then release the band. Band keys go to the MuJoCo window, the rest to the controller's terminal.
3. README "Planned moves: `move_distance`" (detail in [[move-distance]]).
4. README "Drive it through ROS 2: `walker_node`".
5. README "Test runs and logs": `course one_box`, then `runs compare` ([[run-logging]]).

**2. Robot (not tested on hardware yet).** README "On the real robot", plus [[g1-official-docs]] for the network and login. Plan the first session with someone on the gantry rope, and don't reflash the Jetson without asking the team ([[unitree-resources]]).

**3. Next:** [[off-robot-testing]], [[mujoco-sim]], [[g1-builtin-apis]].
