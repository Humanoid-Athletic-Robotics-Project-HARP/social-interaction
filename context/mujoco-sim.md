---
name: mujoco-sim
description: Our MuJoCo sim (sim/run_sim.py), why it replaces unitree_mujoco's runner, and Mac-specific facts (the only OS tested so far)
metadata:
  type: reference
---

**What we use:** `sim/run_sim.py`. It loads Unitree's G1 model (`unitree_mujoco/unitree_robots/g1/scene_29dof.xml`, cloned next to the repo) and speaks the robot's low-level topics. It reads `rt/lowcmd` and publishes `rt/lowstate` (unitree_hg) every 5 ms step, with `rpy` and `tick` filled in. It uses DDS domain 1 on loopback; the real robot uses 0. It has an elastic band with keys 7/8/9, and `--headless --auto-release` for scripted tests. There is no `sport` service in any MuJoCo option, so only our own controller ([[gr00t-wbc]]) walks in it.

**Why not unitree_mujoco's `simulate_python`** (found 2026-10-25):
- Its bridge computes the PD torque once per received `rt/lowcmd`, on the DDS thread, and holds it. With GR00T this blew up the physics on landing. `run_sim.py` recomputes PD every step from the latest command, like NVIDIA's GR00T sim and the robot's motor drivers.
- Its sleep-based pacing drifts to about 0.8x real time on the Mac (lowstate at 140–170 Hz instead of 200), so a wall-clock 50 Hz policy runs too fast relative to sim time. `run_sim.py` paces against an absolute schedule.
- Running it needs three local fixes. Our scripts need none of them, and we checked they don't import `unitree_sdk2py.utils.thread`. The fixes are: unitree_sdk2_python PR #139 for the Linux-only `timerfd`; `import glfw` in `MujuocoKeyCallback`, because MuJoCo 3.17 dropped `mujoco.glfw`; and editing `config.py`.

**Mac facts** (one teammate's Apple Silicon Mac, Python 3.12, 2026-10-25; other OSes untested, see [[team-context]]):
- Loopback is `lo0`, not `lo`. Our scripts pick it automatically.
- The viewer (`launch_passive`) needs `mjpython`, not `python3`; `--headless` works with plain python.
- The setup (CycloneDDS 0.10.x C library, unitree_sdk2_python, unitree_mujoco, `sim-venv` with mujoco and onnxruntime) is in the README.
- Band keys, per the code: 7 shortens (raises), 8 lengthens (lowers), 9 toggles. Unitree's README has 7 and 8 reversed.

**Alternative, not used:** unitree_rl_gym's `deploy/deploy_mujoco` runs its own 12-leg-joint policy directly in MuJoCo, without DDS.

Related: [[off-robot-testing]], [[project-overview]]
