---
name: mujoco-sim
description: Our MuJoCo sim (harp_sim's run_sim.py), why it replaces unitree_mujoco's runner, and Mac-specific facts (the only OS tested so far)
metadata:
  type: reference
---

**What we use:** `ros2/src/harp_sim/harp_sim/run_sim.py`, run with `ros2 run harp_sim run_sim` after `pixi shell` in `ros2/` ([[ros2-mac]]). It loads Unitree's G1 model (`unitree_mujoco/unitree_robots/g1/scene_29dof.xml`, cloned next to the repo; the path comes from `$HARP_G1_SCENE`, set by the pixi environment, or `--scene`) and speaks the robot's low-level topics. It reads `rt/lowcmd` and publishes `rt/lowstate` (unitree_hg) every 5 ms step, with `rpy` and `tick` filled in. It uses DDS domain 1 on loopback; the real robot uses 0. It has an elastic band with keys 7/8/9, and `--auto-release` for scripted tests. There is no `sport` service in any MuJoCo option, so only our own controller ([[gr00t-wbc]]) walks in it.

**Always run it with the viewer** (`ros2 run harp_sim run_sim`, plus `--auto-release` when scripting), never `--headless`, so the teammate can watch the robot move. A teammate asked for this on 2026-10-25.

**Scripted tests** (how the `--to` results were measured, 2026-10-25): start `ros2 run harp_sim run_sim --auto-release 9 --log`, and the controller with `ros2 run harp_control move_distance ...`. Run the controller in a pseudo-terminal (Python `pty.fork`), because `TerminalKeys` reads keys from a real terminal. Pass its flags as separate arguments: one quoted string fails argparse. Press `]` once the logged pelvis z drops below 0.81 (feet down), wait about 6 s, then press `g`, and press `o` after `All moves done`. With `--auto-release` under about 9 s, the band can drop before the stand pose and `]`. If the controller exits, stop the sim, or the band releases a limp robot. For ground truth, take the last `--log` line before each move's `Target` report: the line nearest it can fall inside the next move.

**Why not unitree_mujoco's `simulate_python`** (found 2026-10-25):
- Its bridge computes the PD torque once per received `rt/lowcmd`, on the DDS thread, and holds it. With GR00T this blew up the physics on landing. `run_sim.py` recomputes PD every step from the latest command, like NVIDIA's GR00T sim and the robot's motor drivers.
- Its sleep-based pacing drifts to about 0.8x real time on the Mac (lowstate at 140–170 Hz instead of 200), so a wall-clock 50 Hz policy runs too fast relative to sim time. `run_sim.py` paces against an absolute schedule.
- Running it needs three local fixes. Our scripts need none of them, and we checked they don't import `unitree_sdk2py.utils.thread`. The fixes are: unitree_sdk2_python PR #139 for the Linux-only `timerfd`; `import glfw` in `MujuocoKeyCallback`, because MuJoCo 3.17 dropped `mujoco.glfw`; and editing `config.py`.

**Mac facts** (one teammate's Apple Silicon Mac, Python 3.12, 2026-10-25; other OSes untested, see [[team-context]]):
- Loopback is `lo0`, not `lo`. Our scripts pick it automatically.
- The viewer (`launch_passive`) needs `mjpython`, not `python3`. `ros2 run` starts plain python, so `run_sim.py` restarts itself under `mjpython` (from the pip mujoco wheel in the pixi env) unless `--headless`.
- The window throttling shows up as an occasional stall: "the sim fell ~115 ms behind". Once (2026-10-25) a stall reached 0.5 s and the controller stopped with "Robot state stopped for 502 ms"; rerunning worked.
- The setup (the pixi environment in `ros2/` plus the Unitree clones) is in the README. `sim-venv` and the source-built CycloneDDS are no longer used (2026-10-25).
- Band keys, per the code: 7 shortens (raises), 8 lengthens (lowers), 9 toggles. Unitree's README has 7 and 8 reversed.

**Alternative, not used:** unitree_rl_gym's `deploy/deploy_mujoco` runs its own 12-leg-joint policy directly in MuJoCo, without DDS.

Related: [[off-robot-testing]], [[project-overview]]
