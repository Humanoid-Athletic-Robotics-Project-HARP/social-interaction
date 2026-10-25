# HARP Human Interaction

Robot: Unitree G1 EDU with **29 joints**. Use 29-DoF models and joint indices
for low-level control and simulation.

Current goal: make the G1 walk a specific distance with open-source code we
can read, instead of Unitree's closed walking controller ("sport mode"). We
walk and balance with NVIDIA's GR00T policy and test everything in MuJoCo
first. The same code talks to the sim and the robot over the robot's
low-level DDS topics (`rt/lowstate` in, `rt/lowcmd` out).

| File | What it does |
|---|---|
| `sim/run_sim.py` | MuJoCo sim of the G1 that speaks the robot's low-level topics |
| `gr00t/walk.py` | GR00T walking controller, driven from the keyboard |
| `move_distance.py` | Walks a set distance or turns a set angle with GR00T, using our own leg odometry |
| `gr00t/policy/` | NVIDIA's GR00T walk and balance policies (ONNX) and their license |

## Setup (on your laptop)

Tested on an Apple Silicon Mac with Python 3.12. Clone everything next to this
repo:

```bash
cd ..
git clone https://github.com/eclipse-cyclonedds/cyclonedds -b releases/0.10.x
cd cyclonedds && mkdir build install && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=../install -DCMAKE_POLICY_VERSION_MINIMUM=3.5 -DBUILD_EXAMPLES=OFF -DBUILD_TESTING=OFF
cmake --build . --target install
cd ../..
git clone --depth 1 https://github.com/unitreerobotics/unitree_sdk2_python.git
git clone --depth 1 https://github.com/unitreerobotics/unitree_mujoco.git
python3 -m venv sim-venv && source sim-venv/bin/activate
CYCLONEDDS_HOME=$PWD/cyclonedds/install pip install -e ./unitree_sdk2_python mujoco onnxruntime
```

`unitree_mujoco` is only needed for its G1 model files
(`unitree_robots/g1/scene_29dof.xml`). We don't run its simulator, so its
local fixes (the Linux-only timer patch, the `glfw` key fix, `config.py`) aren't
needed for our scripts. On Linux, use the same steps; the loopback interface is
`lo` instead of `lo0`, and the scripts pick the right one.

## Run it in the sim

Terminal 1, the sim (macOS needs `mjpython` for the viewer window):

```bash
../sim-venv/bin/mjpython sim/run_sim.py
```

The robot starts hanging from a virtual elastic band. In the viewer, `8`
lowers it, `7` raises it, and `9` releases or re-attaches the band.

Terminal 2, the controller:

```bash
../sim-venv/bin/python gr00t/walk.py
```

1. Wait for "Stand pose reached" (3 s).
2. In the viewer, press `8` a few times until the feet touch the floor.
3. In terminal 2, press `]` to start the policy.
4. In the viewer, press `9` to release the band.
5. In terminal 2, drive it: `w`/`s` forward/back, `a`/`d` left/right,
   `q`/`e` turn, `z` stop moving, `1`/`2` raise/lower. Each press changes
   the speed by 0.2.
6. Press `o` to stop. All joints go limp (damping) and the robot sinks.

Start the policy only with the feet on the floor and before releasing the
band. Started in the air, it flails; released without it, the robot falls.

For tests without a window, `python3 sim/run_sim.py --headless --auto-release 3 --log`
lowers the robot onto its feet after 3 s, waits 3 s (press `]` then), releases
the band, and prints the true pelvis position every second.

## Walk a distance or turn an angle: `move_distance.py`

With the sim running (terminal 1 above):

```bash
../sim-venv/bin/python move_distance.py --distance 1.0 --speed 0.3
../sim-venv/bin/python move_distance.py --angle 90 --turn-speed 20
```

Same start-up as above (feet on the floor, `]`, release the band), then press
`g` to start the move. It stops when the estimate reaches the target, prints
the estimated distance or angle, and keeps balancing until you press `o`.

- `--distance`: meters, negative walks backward.
- `--speed`: m/s, capped at 0.4. Default 0.2.
- `--angle`: degrees, positive turns left (counterclockwise).
- `--turn-speed`: deg/s, capped at 30. Default 20.

Set exactly one of `--distance` or `--angle`. It aborts (and damps) if the
move takes much longer than expected, a turn goes more than 10° the wrong
way, the robot tilts past 60°, or robot state stops arriving.

Without Unitree's controller there is no built-in odometry, so the script
estimates its own:
- **Heading:** from the pelvis IMU.
- **Position:** from leg kinematics. The lower foot is taken as planted, and
  the pelvis moves by minus that foot's motion.

In the sim (2026-10-25), a 90° turn was estimated at 90.1° and measured at
89.9°. Walks of 1.5 m and 1 m backward ended at 1.63 m and 1.11 m: the
odometry reads about 7% short, and the robot drifts about 0.1 m after the
stop. GR00T walked at about 85% of the commanded speed.

## On the real robot (not tested yet)

Running our own controller means turning Unitree's off, and then our code is
the only thing keeping the robot up. Before the first run:

1. Hang the robot from a gantry with its feet off the floor.
2. Press L1+A on the remote (damping), then L2+R2 (debug mode). The scripts
   check this and refuse to run while Unitree's controller is active.
3. Run from the Jetson or a laptop on the robot's network (`192.168.123.x`).
   Install the same Python packages there; the scripts need the
   `unitree_mujoco` clone next to this repo for leg odometry.

```bash
python3 gr00t/walk.py --real --interface eth0
python3 move_distance.py --real --interface eth0 --distance 0.5
```

4. Wait for the stand pose, lower the robot until its feet are on the floor,
   press `]`, then slacken the gantry. Keep the gantry rope just loose enough
   to catch a fall.

The scripts refuse to run if the robot's `mode_machine` isn't 5 (29-joint G1
rev 1.0, what NVIDIA's config is for). Pressing `o`, an error, or Ctrl-C damps
every joint, so only do that with the robot supported. Unitree's other
built-in services that need its controller (LocoClient, arm gestures,
`lf/odommodestate`) are probably unavailable in debug mode.

## Licenses

- `gr00t/policy/*.onnx` are NVIDIA's, under the NVIDIA Open Model License
  (`gr00t/policy/NVIDIA-Open-Model-License.txt`, `gr00t/policy/NOTICE.txt`).
- `gr00t/walk.py` follows NVIDIA's GR00T-WholeBodyControl deployment code
  (Apache-2.0, `gr00t/LICENSE-GR00T-WholeBodyControl.txt`).
- The G1 model comes from Unitree's unitree_mujoco (BSD-3) and isn't copied
  into this repo.
