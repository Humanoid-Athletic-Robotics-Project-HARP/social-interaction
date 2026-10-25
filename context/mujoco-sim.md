---
name: mujoco-sim
description: What Unitree's MuJoCo options can and can't simulate for the G1, and how they were set up on one teammate's Mac (the only OS tested so far)
metadata:
  type: reference
---

Read from the official READMEs on 2026-10-25.

**unitree_mujoco** (official): only simulates low-level DDS topics: `rt/lowcmd`, `rt/lowstate`, `SportModeState`, and `rt/secondary_imu` for the G1. There is no built-in walking controller and no `sport` service, so `/api/sport/request` gets no answer. `move_distance.py` can't run against it as written. Our robot's model is `unitree_robots/g1/scene_29dof.xml` (29 joints, see [[project-overview]]), using `unitree_hg` messages. Set `robot: g1` / `ROBOT = "g1"`. Sim uses domain id 1 on `lo` (the real robot uses 0). Its test programs send `unitree_go` messages, so they need changing for the G1. The C++ build is apt/Linux-only. The Python version (`simulate_python`) needs `unitree_sdk2_python` (which needs CycloneDDS) and `mujoco`. Turn on the elastic band for humanoids. The keys are below, with the glfw fix.

**unitree_rl_gym `deploy/deploy_mujoco`** (official): runs a pretrained G1 walking policy (`deploy/pre_train/g1/motion.pt`, 12 leg joints) directly in MuJoCo, with no DDS. It only controls the legs, so it doesn't exercise our robot's waist or arm joints. Run it with `deploy_mujoco.py g1.yaml`. The walk command is fixed by `cmd_init` in the yaml (default 0.5 m/s forward). Don't `pip install -e` it: setup.py pins isaacgym and numpy 1.20. Running from the repo root with `PYTHONPATH=.` and installing mujoco, torch and pyyaml should be enough, but this is untested.

**Tested machine:** one teammate's Apple Silicon Mac with Python 3.12 and no ROS 2. Other teammates use other OSes, which haven't been tried yet (see [[team-context]]). MuJoCo's `launch_passive` viewer needs `mjpython` on macOS, not `python3`.

**Set up and working on the Mac (2026-10-25):** cyclonedds (0.10.x C library), unitree_sdk2_python, unitree_mujoco and `sim-venv` are siblings of this repo's clone, about 830 MB in total. The README's Simulation section has the steps. Mac-specific findings:
- Loopback is `lo0`, not `lo`. This applies to config.py and to `ChannelFactoryInitialize(1, "lo0")`.
- The SDK's `RecurrentThread` uses Linux-only `timerfd`. A teammate approved applying the open community PR unitree_sdk2_python#139 (only `utils/thread.py`, with a sleep fallback off Linux) to the local clone. It is not merged upstream, so re-apply it after re-cloning, and other Mac teammates need it too.
- Verified: G1 29-DoF lowstate arrives at about 140–170 Hz (200 Hz target). The sim leaves `imu_state.rpy` and `tick` at 0, so use `imu_state.quaternion`. The base starts at about 1.28 m, hanging on the elastic band.

**Not Mac-specific (applies on any platform with MuJoCo 3.17+):** with the elastic band on, any key press in the viewer crashed the sim. `MujuocoKeyCallback` in `simulate_python/unitree_sdk2py_bridge.py` used `mujoco.glfw.glfw`, which MuJoCo 3.17 dropped. A teammate approved replacing it locally with `import glfw` (the standalone package is a MuJoCo dependency). There is no upstream fix as of 2026-10-25, so re-apply it after re-cloning. They confirmed the keys work in the live viewer after the fix. Band keys per the code: 9 toggles the band, 7 shortens it (raises the robot), 8 lengthens it (lowers the robot). Unitree's README has 7 and 8 reversed.

**Fake sport service, built 2026-10-25:** `sim/fake_sport.py` in this repo. It's a standalone MuJoCo script that loads the G1 model from the sibling unitree_mujoco clone and doesn't use the bridge. It subclasses unitree_sdk2py's RPC `Server` and answers 7105/7101/7001 (anything else gets 3203). It slides the robot kinematically and publishes `rt/lf/odommodestate` at 20 Hz with yaw in `rpy[2]`. Verified on the Mac with LocoClient (walk, turn, duration expiry, error codes) in headless and viewer modes. Not yet run with `move_distance.py`, which needs ROS 2. The next step would be a real walking policy instead of the slide.

Related: [[unitree-resources]], [[project-overview]]
