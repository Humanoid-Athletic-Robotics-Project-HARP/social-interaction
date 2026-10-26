---
name: beginner-guide
description: BEGINNER GUIDE ONLY. Step-by-step setup and first use of this repo for someone who just cloned it (GR00T + our sim). Skip unless the user says they are new to the repo or asks how to get started.
metadata:
  type: reference
  updated: 2026-10-26T21:32:46Z
  covers: [README.md]
---

**Only read this when the user is new to the repo** (just cloned it, asks "how do I get started", "how do I set this up", or similar). Regular contributors don't need it.

Walk the newcomer through it one stage at a time, and ask which stage they need: they may only want the simulator (on their laptop) or only the robot (Jetson). The README has the exact commands; use this file for the order, the context, and the pitfalls, and point them at the README section for each stage instead of retyping it.

**0. Orientation (2 minutes)**
- This is the HARP **Human Interaction** team's repo for a **Unitree G1 EDU with 29 joints** ([[project-overview]]). Goals are drafts; the only concrete one so far is walking a set distance.
- All code is in ROS 2 packages under `ros2/src/`. We walk with NVIDIA's GR00T policy (`harp_control`'s `gr00t/walk.py`), not Unitree's closed sport mode ([[gr00t-wbc]]). The same code talks to our MuJoCo sim (`harp_sim`) and the robot over `rt/lowstate`/`rt/lowcmd`. `move_distance` walks a set distance, turns a set angle, or curves to a point with it. `README.md` has all the setup, and `context/` is shared memory for Claude and other agents, indexed in `context/MEMORY.md`. Unitree's code is cloned *next to* the repo, never inside it ([[unitree-resources]]).
- House rule: add only what the current goal needs, and prefer open code we can read ([[feedback-minimal-setup]]).

**1. Simulator path (start here; runs on a laptop; only tested on a Mac so far)**
Ask which OS they're on first ([[team-context]]). The pixi environment is only set up for Apple Silicon Macs (macOS 14+). Linux would need its platform added to `ros2/pixi.toml` and `lo` instead of `lo0`; untested. Windows doesn't work: `TerminalKeys` uses termios.
1. README "Setup", steps 1 to 4: install git and pixi, make a dedicated parent folder, clone this repo into it, clone `unitree_sdk2_python`, `unitree_mujoco` and `unitree_ros2` at the pinned commits as siblings of this repo, then `pixi install --locked` and `pixi run build` in `ros2/`. Every terminal then needs `pixi shell --as-is` in `ros2/`. unitree_mujoco is only needed for the G1 model files; none of its local fixes are needed.
2. README "Run it in the sim": `ros2 run harp_sim run_sim` in one terminal and `ros2 run harp_control walk` in another. The order matters: wait for the stand pose, lower the feet to the floor (`8` in the viewer), press `]`, then release the band (`9`). Started in the air, the policy flails. Released before `]`, the robot falls. Controller keys (`]`, `g`, `o`) go to the controller's terminal; only `7`/`8`/`9` go to the MuJoCo window.
3. Then `ros2 run harp_control move_distance --distance 1.0`, a curved walk to a point such as `--to 2 1` (2 m forward, 1 m left), or a sequence such as `--distance 2 --angle 90 --to 1 1 90` (the flags repeat and run in order): the same start-up, then `g`. The README's "Planned moves: `move_distance`" lists the demos (walk-turn-walk, a curve that ends facing left, the weave); measured results and edge cases are in [[move-distance]].
4. Optional, through ROS 2: README "Drive it through ROS 2: `walker_node`". `ros2 run harp_ros walker_node` instead of `walk`, the same start-up (`]` or `ros2 service call /walker/start std_srvs/srv/Trigger`), then drive with `teleop_twist_keyboard` or `ros2 topic pub ... /cmd_vel`, and watch `/odom`. ROS 2 is on DDS domain 2, so `ros2 topic list` shows `/cmd_vel`, `/odom` and `/tf`, not `/lowstate`.
5. Obstacles with Nav2, recorded: README "Test runs and logs". `ros2 run harp_ros course one_box` runs a whole course test in one command (the windows open, the robot walks to the goal, it prints a summary), and `ros2 run harp_ros runs compare --world one_box` compares runs. Details: [[run-logging]].

**2. Robot path (not tested on hardware yet)**
1. Join the robot's `192.168.123.x` network; the Jetson is `ssh unitree@192.168.123.164` (initial password `123`, see [[g1-official-docs]]). Don't reflash the Jetson without asking the team ([[unitree-resources]]).
2. Our controller replaces Unitree's, so the robot must be on a gantry, damped (L1+A), then in debug mode (L2+R2). The scripts refuse otherwise, and refuse if `mode_machine` isn't 5. Read README "On the real robot" first, and plan the first session with someone on the gantry rope.
3. `--real --interface <iface>` on `walk` or `move_distance`, run from a Mac laptop on the robot's network. Pressing `o` or any error damps the robot, so only do that while it's supported.

**3. Where to go next**
- How others test without the robot: [[off-robot-testing]]. Sim internals: [[mujoco-sim]].
- Unitree's built-in robot functions (gestures, speech, LED) and their API ids: [[g1-builtin-apis]]. Many need Unitree's controller, so they're probably unavailable in debug mode.
- Official Unitree docs (JavaScript site, open in a browser): [[g1-official-docs]].
