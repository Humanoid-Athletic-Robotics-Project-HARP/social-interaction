---
name: beginner-guide
description: BEGINNER GUIDE ONLY. Step-by-step setup and first use of this repo for someone who just cloned it. Skip unless the user says they are new to the repo or asks how to get started.
metadata:
  type: reference
---

**Only read this when the user is new to the repo** (just cloned it, asks "how do I get started", "how do I set this up", or similar). Regular contributors don't need it.

Walk the newcomer through it one stage at a time, and ask which stage they need: they may only want the simulator (on their laptop) or only the robot (Jetson). The README has the exact commands; use this file for the order, the context, and the pitfalls, and point them at the README section for each stage instead of retyping it.

**0. Orientation (2 minutes)**
- This is the HARP **Human Interaction** team's repo for a **Unitree G1 EDU with 29 joints** ([[project-overview]]). Goals are drafts; the only concrete one so far is walking a set distance.
- The repo is deliberately tiny: `move_distance.py` (our one script), `README.md` (all setup), and `context/` (shared memory for Claude/agents, indexed in `context/MEMORY.md`). Unitree's code is cloned *next to* the repo, never inside it ([[unitree-resources]]).
- House rule: use Unitree's official code and ROS 2, and add only what the current goal needs ([[feedback-minimal-setup]]).

**1. Robot path (runs on the G1's Jetson, not the laptop)**
1. Join the robot's `192.168.123.x` network and `ssh unitree@192.168.123.164` (initial password `123`, see [[g1-official-docs]]).
2. Check the OS and ROS 2 version: `ls /opt/ros` and `lsb_release -a`. Likely Foxy on Ubuntu 20.04 (unconfirmed). Do not upgrade or reflash the Jetson.
3. README §1: clone and build `unitree_ros2` in `~/unitree_ros2`, fix the network interface in `setup.sh` (usually `eth0`, from `ip a`), then `source ~/unitree_ros2/setup.sh` in every new terminal. On Foxy, build `cyclonedds_ws` with ROS *not* sourced.
4. Sanity check: `ros2 topic echo /lf/odommodestate` should stream odometry. If nothing appears, the interface name in `setup.sh` is the usual culprit.
5. Copy `move_distance.py` to the Jetson (e.g. `scp move_distance.py unitree@192.168.123.164:~`) or clone the repo there.
6. Read README "Safety" before moving the robot: stand it up in walking mode with the remote, clear 2 m beyond the target, someone holds the remote the whole time.
7. First run: README §3, starting with `distance:=0.5`. Before the first turn, do the heading-direction check described in §3. Parameters must be decimals (`90.0`, not `90`).
8. Optional: README §2 runs Unitree's own example with no code from us.

**2. Simulator path (runs on a laptop; only tested on a Mac so far)**
Ask which OS they're on first ([[team-context]]). The steps below are what worked on an Apple Silicon Mac. On Linux, the two Mac-only fixes shouldn't be needed (use `lo`, skip PR #139) and plain `python3` should work, but that is untested. Windows is untested.
1. README "Simulation" section: builds CycloneDDS 0.10.x, `unitree_sdk2_python`, `unitree_mujoco` and a `sim-venv`, all as siblings of this repo (~830 MB). Tested on Apple Silicon with Python 3.12; needs cmake and git.
2. Three fixes are required (README lists them): apply unitree_sdk2_python PR #139 (timerfd, Mac-only), use `lo0` instead of `lo` (Mac-only), and in `unitree_sdk2py_bridge.py` replace `glfw = mujoco.glfw.glfw` with `import glfw`. Without that last one, newer MuJoCo crashes on any key press in the viewer. Re-apply all the patches after any re-clone.
3. Edit `unitree_mujoco/simulate_python/config.py` as the README shows (G1, `scene_29dof.xml`, `lo0`, elastic band on).
4. Run with `mjpython`, not `python3`. Keys: `9` band on/off, `7` raises the robot, `8` lowers it (Unitree's README has these backwards).
5. Set expectations: the sim is low-level only, with no walking controller or `sport` service, so `move_distance.py` does **not** work in it. The robot just hangs on the band. Details in [[mujoco-sim]].

**3. Where to go next**
- Built-in robot functions (walk, gestures, speech, LED) and their API ids: [[g1-builtin-apis]].
- Official Unitree docs (JavaScript site, open in a browser): [[g1-official-docs]].
- Official and community repos: [[unitree-resources]].
