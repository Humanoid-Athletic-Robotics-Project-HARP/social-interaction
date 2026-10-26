---
name: beginner-guide
description: BEGINNER GUIDE ONLY. Step-by-step setup and first use of this repo for someone who just cloned it (GR00T + our sim). Skip unless the user says they are new to the repo or asks how to get started.
metadata:
  type: reference
---

**Only read this when the user is new to the repo** (just cloned it, asks "how do I get started", "how do I set this up", or similar). Regular contributors don't need it.

Walk the newcomer through it one stage at a time, and ask which stage they need: they may only want the simulator (on their laptop) or only the robot (Jetson). The README has the exact commands; use this file for the order, the context, and the pitfalls, and point them at the README section for each stage instead of retyping it.

**0. Orientation (2 minutes)**
- This is the HARP **Human Interaction** team's repo for a **Unitree G1 EDU with 29 joints** ([[project-overview]]). Goals are drafts; the only concrete one so far is walking a set distance.
- We walk with NVIDIA's GR00T policy (`gr00t/walk.py`), not Unitree's closed sport mode ([[gr00t-wbc]]). The same code talks to our MuJoCo sim (`sim/run_sim.py`) and the robot over `rt/lowstate`/`rt/lowcmd`. `move_distance.py` walks a set distance or turns a set angle with it. `README.md` has all the setup, and `context/` is shared memory for Claude and other agents, indexed in `context/MEMORY.md`. Unitree's code is cloned *next to* the repo, never inside it ([[unitree-resources]]).
- House rule: add only what the current goal needs, and prefer open code we can read ([[feedback-minimal-setup]]).

**1. Simulator path (start here; runs on a laptop; only tested on a Mac so far)**
Ask which OS they're on first ([[team-context]]). Linux should only need `lo` instead of `lo0` (the scripts choose automatically); that's untested. Windows is untested, and `TerminalKeys` uses termios, which is Mac/Linux only.
1. README "Setup": build CycloneDDS 0.10.x, then clone `unitree_sdk2_python` and `unitree_mujoco` and create `sim-venv` (mujoco, onnxruntime), all as siblings of this repo. unitree_mujoco is only needed for the G1 model files; none of its local fixes are needed.
2. README "Run it in the sim": `mjpython sim/run_sim.py` in one terminal and `python gr00t/walk.py` in another. The order matters: wait for the stand pose, lower the feet to the floor (`8` in the viewer), press `]`, then release the band (`9`). Started in the air, the policy flails. Released before `]`, the robot falls. Controller keys (`]`, `g`, `o`) go to the controller's terminal; only `7`/`8`/`9` go to the MuJoCo window.
3. Then `move_distance.py --distance 1.0`, a curved walk to a point such as `--to 2 1` (2 m forward, 1 m left), or a sequence such as `--distance 2 --angle 90 --to 1 1 90` (the flags repeat and run in order): the same start-up, then `g`.

**2. Robot path (not tested on hardware yet)**
1. Join the robot's `192.168.123.x` network; the Jetson is `ssh unitree@192.168.123.164` (initial password `123`, see [[g1-official-docs]]). Do not upgrade or reflash the Jetson.
2. Our controller replaces Unitree's, so the robot must be on a gantry, damped (L1+A), then in debug mode (L2+R2). The scripts refuse otherwise, and refuse if `mode_machine` isn't 5. Read README "On the real robot" first, and plan the first session with someone on the gantry rope.
3. `--real --interface <iface>` on either script. Pressing `o` or any error damps the robot, so only do that while it's supported.

**3. Where to go next**
- How others test without the robot: [[off-robot-testing]]. Sim internals: [[mujoco-sim]].
- Unitree's built-in robot functions (gestures, speech, LED) and their API ids: [[g1-builtin-apis]]. Many need Unitree's controller, so they're probably unavailable in debug mode.
- Official Unitree docs (JavaScript site, open in a browser): [[g1-official-docs]].
