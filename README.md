# HARP Human Interaction

Robot: Unitree G1 EDU with **29 joints**. Use 29-DoF models and joint indices
for low-level control and simulation.

Current goal: make the Unitree G1 walk a specific distance, using ROS 2 and
only Unitree's official code.

Everything below runs **on the robot's Jetson** (`ssh unitree@192.168.123.164`).

## 1. One-time setup: Unitree's official ROS 2 package

Check that ROS 2 is installed (`ls /opt/ros`), then follow the
[unitree_ros2 README](https://github.com/unitreerobotics/unitree_ros2) to clone
it to `~/unitree_ros2` and build `cyclonedds_ws`. This gives you the robot's
message types (`unitree_api`, `unitree_go`, `unitree_hg`).

In `~/unitree_ros2/setup.sh`, change the network interface (`enp3s0`) to the
Jetson's interface on the `192.168.123.x` network (find it with `ip a`, usually
`eth0`). Then, in every new terminal:

```bash
source ~/unitree_ros2/setup.sh
```

Check you can see the robot's odometry:

```bash
ros2 topic echo /lf/odommodestate
```

## 2. Optional: move with Unitree's own example (no code from us)

Build the official examples (`cd ~/unitree_ros2/example && colcon build`,
then `source install/setup.bash`) and run:

```bash
ros2 run unitree_ros2_example g1_loco_client_example --set_velocity="0.2 0 0 5"
```

That walks at 0.2 m/s for 5 s, roughly 1 m. It doesn't measure anything, so the
real distance will be off.

## 3. Walk a distance or turn an angle: `move_distance.py`

Our one script. It sends the same velocity command as Unitree's example, but
watches the robot's odometry and stops when the target is reached, then prints
how far the robot actually went or turned. Each run does one move: set exactly
one of `distance` or `angle`.

```bash
python3 move_distance.py --ros-args -p distance:=0.5 -p speed:=0.2
python3 move_distance.py --ros-args -p angle:=90.0 -p turn_speed:=20.0
```

- `distance`: meters, negative walks backward. Default 0.
- `speed`: positive m/s, capped at 0.4. Default 0.2.
- `angle`: degrees, positive turns left (counterclockwise). Default 0.
- `turn_speed`: positive deg/s, capped at 30. Default 20.

All values must be finite and written as decimals (`90.0`, not `90`), or ROS
rejects the parameter type.

The script aborts if valid odometry stops for 0.5 s, its velocity command is
rejected, the move takes much longer than expected, or a turn goes more than
10° the wrong way. Errors attempt a stop and return a nonzero exit code.
Distance and heading are estimated from odometry (heading is
`imu_state.rpy[2]`); accuracy and timeout settings need robot tests.

Before the first turn, check that heading is reported in radians and which way
it changes: turn the robot left with the remote while running
`ros2 topic echo /lf/odommodestate --field imu_state.rpy`. The third value
should increase.

## Safety

- Stand the robot up in its normal walking mode with the remote first. The
  script never changes the robot's mode.
- Clear at least 2 m beyond the target distance, and keep someone holding the
  remote the whole time.
- Start with 0.5 m, and 45° for turns.
- Commands request 0.5 s validity; verify actual expiry behavior on the robot.
- Reaching the target, an error, or Ctrl-C sends the stop command 5 times,
  0.1 s apart, in case some are lost. If the network is down, none get through.

## Simulation (MuJoCo, on a Mac)

This runs on your laptop, not the Jetson. It uses Unitree's official
[unitree_mujoco](https://github.com/unitreerobotics/unitree_mujoco) Python
simulator, which only simulates the low-level topics (`rt/lowcmd`,
`rt/lowstate`, `rt/sportmodestate`). It has no built-in walking controller, so
`move_distance.py` doesn't work in it directly (see "Fake sport service" below).
The G1 hangs from a virtual elastic band until your own code commands its joints.

Setup was done on an Apple Silicon Mac with Python 3.12, with everything cloned
next to this repo (about 830 MB in total):

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
CYCLONEDDS_HOME=$PWD/cyclonedds/install pip install -e ./unitree_sdk2_python mujoco pygame
```

Three fixes (the first two are Mac-only):

- **The SDK's timer is Linux-only** (`timerfd_create: symbol not found`). Apply
  the open upstream fix
  [unitree_sdk2_python#139](https://github.com/unitreerobotics/unitree_sdk2_python/pull/139).
  It changes only `unitree_sdk2py/utils/thread.py` and leaves Linux behavior
  unchanged:
  `curl -sL https://github.com/unitreerobotics/unitree_sdk2_python/pull/139.diff | git -C unitree_sdk2_python apply`
- **The loopback interface is `lo0`, not `lo`.**
- **Any key press in the viewer crashes the sim** (`module 'mujoco' has no
  attribute 'glfw'`). Newer MuJoCo (3.17 here) dropped `mujoco.glfw`. In
  `unitree_mujoco/simulate_python/unitree_sdk2py_bridge.py`, in
  `MujuocoKeyCallback`, replace `glfw = mujoco.glfw.glfw` with `import glfw`.
  The standalone `glfw` package is already installed as a MuJoCo dependency.

In `unitree_mujoco/simulate_python/config.py`, set:

```python
ROBOT = "g1"
ROBOT_SCENE = "../unitree_robots/" + ROBOT + "/scene_29dof.xml"
INTERFACE = "lo0"
USE_JOYSTICK = 0           # 1 if you have an Xbox/Switch gamepad
ENABLE_ELASTIC_BAND = True
```

Run it with `mjpython` (macOS needs it for the viewer window):

```bash
cd unitree_mujoco/simulate_python && ../../sim-venv/bin/mjpython unitree_mujoco.py
```

In the viewer, `9` attaches or releases the band, `7` shortens it (raises the
robot) and `8` lengthens it (lowers the robot). Unitree's README has `7` and `8`
the other way around, but the code works as described here. Code talks to the sim with `ChannelFactoryInitialize(1, "lo0")`
(domain 1). The real robot uses domain 0 on its own network interface, so the
two don't cross-talk.

Sim quirks: state is published at about 140–170 Hz on a Mac (200 Hz target), and
`imu_state.rpy` and `tick` stay 0. Use `imu_state.quaternion` instead.

### Fake sport service: `sim/fake_sport.py`

Stands in for the robot's built-in walking service so code that uses it
(`LocoClient`, `move_distance.py`) can run without the robot. It answers
`SetVelocity` (7105), `SetFsmId` (7101) and `GetFsmId` (7001) on the same
DDS topics as the robot and publishes odometry on `rt/lf/odommodestate`.
It doesn't walk: it slides the whole robot, legs fixed, at the commanded
velocity. Use it to check a script's logic (commands, odometry, stopping),
not how the real robot moves. It needs the sim venv and the `unitree_mujoco`
clone (for the G1 model), not `unitree_mujoco.py`:

```bash
../sim-venv/bin/mjpython sim/fake_sport.py
```

Add `--headless` to run without a window (plain `python3` works then). It
uses domain 1 on `lo0` (Mac) or `lo` (Linux); `--domain` and `--interface`
change that. Then point a client at domain 1, for example `LocoClient` after
`ChannelFactoryInitialize(1, "lo0")`.

Tested on a Mac with `LocoClient`: walking, turning, the command expiring after
its `duration`, and error codes. `move_distance.py` itself hasn't been run
against it, because it needs ROS 2, which the Mac setup doesn't have. On a
Linux machine with unitree_ros2 it should work with `ROS_DOMAIN_ID=1` and
CycloneDDS set to the loopback interface (untested).
