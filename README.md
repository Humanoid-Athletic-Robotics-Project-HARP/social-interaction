# HARP Human Interaction

Robot: Unitree G1 EDU with **29 joints**. Use 29-DoF models and joint indices
for low-level control and simulation.

Current goal: make the G1 walk a specific distance with open-source code we
can read, instead of Unitree's closed walking controller ("sport mode"). We
walk and balance with NVIDIA's GR00T policy and test everything in MuJoCo
first. The same code talks to the sim and the robot over the robot's
low-level DDS topics (`rt/lowstate` in, `rt/lowcmd` out).

All the code lives in a ROS 2 workspace, `ros2/`, as three ROS 2 packages:
`harp_control` (walking, no ROS 2 needed), `harp_sim` (the sim) and `harp_ros`
(ROS 2 nodes). The walking code talks to the robot directly over DDS; the
walker node puts a ROS 2 interface on it (`/cmd_vel` in, `/odom` and `/tf`
out) for Nav2 later.

| Path | What it does |
|---|---|
| `ros2/src/harp_sim/harp_sim/run_sim.py` | MuJoCo sim of the G1 that speaks the robot's low-level topics |
| `ros2/src/harp_control/harp_control/gr00t/walk.py` | GR00T walking controller, driven from the keyboard |
| `ros2/src/harp_control/harp_control/gr00t/policy/` | NVIDIA's GR00T walk and balance policies (ONNX) and their license |
| `ros2/src/harp_control/harp_control/move_distance.py` | Walks a set distance, turns a set angle, or walks to a point in one smooth curve with GR00T |
| `ros2/src/harp_control/harp_control/leg_odometry.py` | Our leg odometry: position and heading from the joint angles and the IMU |
| `ros2/src/harp_control/harp_control/smooth_control.py` | The control law for those curves (Park & Kuipers, the same as Nav2's Graceful Controller) |
| `ros2/src/harp_ros/harp_ros/walker_node.py` | ROS 2 node around the GR00T walker: `/cmd_vel` in, `/odom` and `/tf` out, start/stop services |
| `ros2/pixi.toml`, `ros2/pixi.lock` | The environment: ROS 2 Jazzy, Python 3.12, MuJoCo, onnxruntime, Unitree's SDK, pinned |

## Setup (on your laptop)

Tested on an Apple Silicon Mac with macOS 15; macOS 14 or newer is needed. The
environment is only set up for Apple Silicon Macs so far: Linux would need
its platform added to `ros2/pixi.toml` (and `lo` instead of `lo0`), which
nobody has tried yet. Windows doesn't work: `walk.py` reads keys with
`termios`, which only macOS and Linux have.

### 1. Install the tools

You need git and [pixi](https://pixi.sh). Everything else (Python 3.12, ROS 2
Jazzy, compilers, MuJoCo, onnxruntime, CycloneDDS) comes from pixi.

```bash
xcode-select --install
brew install pixi
```

### 2. Make a folder for everything and clone this repo into it

The build looks for Unitree's code right next to this repo, so everything
goes in one folder of its own:

```
harp/
├── stanford-harp-social-interaction/   this repo
├── unitree_sdk2_python/                Unitree's Python SDK
├── unitree_mujoco/                     only for its G1 model files
└── unitree_ros2/                       only for its ROS 2 message definitions
```

```bash
mkdir harp && cd harp
# clone this repo here
```

Run step 3 from `harp/`.

### 3. Clone Unitree's SDK, G1 model and messages

```bash
git clone --depth 1 https://github.com/unitreerobotics/unitree_sdk2_python.git
git -C unitree_sdk2_python fetch --depth 1 origin 814556d15970dd2ecf1c9984e845ca02ab07e206
git -C unitree_sdk2_python checkout 814556d15970dd2ecf1c9984e845ca02ab07e206
git clone --depth 1 https://github.com/unitreerobotics/unitree_mujoco.git
git -C unitree_mujoco fetch --depth 1 origin 1eb6642e3f3fdfb7fb13a9794fd6a2dd93ea0e7d
git -C unitree_mujoco checkout 1eb6642e3f3fdfb7fb13a9794fd6a2dd93ea0e7d
git clone --depth 1 https://github.com/unitreerobotics/unitree_ros2.git
git -C unitree_ros2 fetch --depth 1 origin 668d1ec5a05d1c38d3306bdca7d59f2ba3581a88
git -C unitree_ros2 checkout 668d1ec5a05d1c38d3306bdca7d59f2ba3581a88
```

The checkouts print a "detached HEAD" note; that's expected. On a Mac, cloning
`unitree_mujoco` also warns that `terrain.STL` and `terrain.stl` collided.
That's a Go2-W file we don't use, so ignore it.

`unitree_mujoco` is only needed for its G1 model files
(`unitree_robots/g1/scene_29dof.xml`). We don't run its simulator, so its
local fixes (the Linux-only timer patch, the `glfw` key fix, `config.py`) aren't
needed. `unitree_ros2` is only needed for its message packages.

### 4. Install the environment and build the packages

```bash
cd stanford-harp-social-interaction/ros2
pixi install --locked
pixi run build
```

`pixi install --locked` installs exactly the versions in `pixi.lock` (a few
GB, into `ros2/.pixi/`, which git ignores). It builds Unitree's SDK against
the same CycloneDDS that ROS 2 uses. `pixi run build` builds our packages and
Unitree's messages with colcon. Our Python files are linked rather than
copied, so edits take effect without rebuilding; run `pixi run build` again
after adding a file or a package, or after reinstalling the environment.

The commits in step 3 and the versions in `pixi.lock` are the ones we tested.
Newer ones can break things (MuJoCo 3.17 already broke a script in
`unitree_mujoco`), so update them together and re-test in the sim.

Check it:

```bash
pixi run ros2 pkg executables harp_control
```

That lists `harp_control move_distance` and `harp_control walk`.

## Run it in the sim

In every terminal, start with `pixi shell` from `ros2/`. It sets up ROS 2, the
DDS settings for the sim and the path to the G1 model.

Terminal 1, the sim (on macOS it restarts itself under `mjpython`, which the
viewer window needs):

```bash
ros2 run harp_sim run_sim
```

The robot starts hanging from a virtual elastic band. In the viewer, `8`
lowers it, `7` raises it, and `9` releases or re-attaches the band.

Terminal 2, the controller:

```bash
ros2 run harp_control walk
```

Keys for the controller (`]`, `g`, `o`, `w`...) go to terminal 2. Only the
band keys (`7`, `8`, `9`) go to the MuJoCo window. Click a window before
typing in it.

1. Wait for "Stand pose reached" (3 s).
2. In the viewer, press `8` a few times until the feet touch the floor.
3. In terminal 2, press `]` to start the policy ("Policy on").
4. In the viewer, press `9` once to release the band. Terminal 1 prints
   "Band released"; `9` toggles, so a second press re-attaches it.
5. In terminal 2, drive it: `w`/`s` forward/back, `a`/`d` left/right,
   `q`/`e` turn, `z` stop moving, `1`/`2` raise/lower. Each press changes
   the speed by 0.2.
6. Press `o` to stop. All joints go limp (damping) and the robot sinks.

Start the policy only with the feet on the floor and before releasing the
band. Started in the air, it flails; released without it, the robot falls.

For tests without a window, `ros2 run harp_sim run_sim --headless --auto-release 3 --log`
lowers the robot onto its feet after 3 s, waits 3 s (press `]` then), releases
the band, and prints the true pelvis position every second.

## Drive it through ROS 2: `walker_node`

`walker_node` runs the same GR00T walker as `walk`, but takes its velocity from
the `/cmd_vel` topic and publishes our leg odometry, so other ROS 2 nodes (and
later Nav2) can drive it.

| | Name | Type | |
|---|---|---|---|
| Subscribes | `/cmd_vel` | `geometry_msgs/Twist` | `linear.x`, `linear.y` (m/s), `angular.z` (rad/s) |
| Publishes | `/odom` | `nav_msgs/Odometry` | `odom` → `base_link`, 50 Hz while the policy runs |
| Publishes | `/tf` | `odom` → `base_link` | the same pose, for tf2 |
| Service | `/walker/start` | `std_srvs/Trigger` | starts the policy, like `]` |
| Service | `/walker/stop` | `std_srvs/Trigger` | damps every joint and quits, like `o` |

If `/cmd_vel` goes quiet for 0.5 s (the `cmd_timeout` parameter), the command
drops to zero and GR00T balances in place. The speed limits of `walk` still
apply. The `odom` frame starts where the policy started.

Terminal 1, the sim: `ros2 run harp_sim run_sim`. Terminal 2:

```bash
ros2 run harp_ros walker_node
```

Start-up is the same as for `walk`: wait for "Stand pose reached", lower the
feet (`8` in the viewer), then start the policy with `]` in terminal 2 or

```bash
ros2 service call /walker/start std_srvs/srv/Trigger
```

and release the band (`9`). Then, in terminal 3, drive it with the keyboard
(hold a key: it stops 0.5 s after the last message) or send a command:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

```bash
ros2 topic pub -r 10 -t 40 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}}"
```

That second one walks forward at 0.3 m/s for 4 s. Watch where the robot thinks
it is with `ros2 topic echo /odom --field pose.pose`, and stop with `o` in
terminal 2 or `ros2 service call /walker/stop std_srvs/srv/Trigger`.

In the sim (2026-10-25), 0.3 m/s for 4 s moved the robot 1.02 m and `/odom`
said 1.02 m; a 0.5 rad/s turn for 3 s turned it 85.4° and `/odom` said 85.5°.
With no `/cmd_vel` it drifted 0.01 m in 2 s.

**DDS domains:** ROS 2 runs on domain 2. The robot's own topics are on
domain 0 on the robot and 1 in the sim, and CycloneDDS can't have ROS 2 and
Unitree's SDK on the same domain in one program (`walker_node` has both). So
`ros2 topic list` shows only ROS 2 topics. To look at the sim's raw topics:

```bash
ROS_DOMAIN_ID=1 ros2 topic echo /lowstate --once --field imu_state
```

Ignore the "Failed to parse type hash" warnings there: the sim publishes
through unitree_sdk2py, not ROS 2.

## Walk distances and turn angles: `move_distance.py`

With the sim running (terminal 1 above), after `pixi shell`:

```bash
ros2 run harp_control move_distance --distance 1.0 --speed 0.3
ros2 run harp_control move_distance --angle 90 --turn-speed 20
ros2 run harp_control move_distance --distance 2 --angle 90 --distance 2 --speed 0.3
ros2 run harp_control move_distance --to 2 1 --speed 0.3
```

`--distance`, `--angle` and `--to` can be repeated and mixed, and the moves
run in the order given, with the robot balancing in place for 1.5 s between
them. Start-up is the same as above (feet on the floor, `]`, release the
band), then press `g` to start the moves. Give the robot a few seconds to
settle after the release: it often takes a step and turns a little while
catching itself, and "forward" is wherever it faces when you press `g`. After
each move it prints the estimated distance, angle or position. After the last
one the robot keeps balancing until you press `o`.

- `--distance`: meters, negative walks backward.
- `--angle`: degrees, positive turns left (counterclockwise).
- `--to X Y [HEADING]`: walk to X m forward and Y m left of where the robot
  stands when the move starts, in one smooth curve (see below). It ends facing
  HEADING degrees (positive left), or, without it, facing away from where it
  started.
- `--speed`: m/s for every walk (the top speed for `--to`), capped at 0.4.
  Default 0.2.
- `--turn-speed`: deg/s for every turn, capped at 30. Default 20. For `--to`
  it is the fastest the robot turns while walking.

It aborts (and damps) if a move takes much longer than expected, a turn goes
more than 10° the wrong way, the robot tilts past 60°, or robot state stops
arriving. That's 0.1 s on the robot and 0.5 s in the sim, which pauses when
macOS throttles its window. If a walk times out with the robot stepping in
place, the band was still on.

### Example: walk 2 m, turn left 90°, walk 2 m

Terminal 1 (after `pixi shell` in `ros2/`, as in every terminal):

```bash
ros2 run harp_sim run_sim
```

Terminal 2:

```bash
ros2 run harp_control move_distance --distance 2 --angle 90 --distance 2 --speed 0.3
```

| Step | Where | Do | You should see |
|---|---|---|---|
| 1 | terminal 2 | wait about 3 s | `Stand pose reached. Lower the robot onto its feet ...` |
| 2 | MuJoCo window | press `8` about five times | the feet touch the floor |
| 3 | terminal 2 | press `]` | `Policy on: GR00T is balancing. Release the band ...` |
| 4 | MuJoCo window | press `9` once | terminal 1: `Band released: the robot is on its own` |
| 5 | | wait a few seconds | the robot steadies itself |
| 6 | terminal 2 | press `g` | the three moves, one after another |
| 7 | terminal 2 | press `o` when it says `All moves done` | `Stopped: all joints damped, so the robot sinks.` |

Terminal 2 after step 6 (from a sim run on 2026-10-25):

```
[1/3] Walking +2.00 m at 0.30 m/s
  Target 2.00 m, estimated displacement 2.00 m
[2/3] Turning +90.0 deg at 20.0 deg/s
  Target +90.0 deg, estimated turn +89.4 deg
[3/3] Walking +2.00 m at 0.30 m/s
  Target 2.00 m, estimated displacement 2.02 m
All moves done. GR00T keeps balancing; support the robot, then press o to stop (it will sink).
```

In that run the robot really walked 2.01 m, turned 88.8° and walked 2.01 m,
ending 2.24 m forward and 1.88 m to the left of where it started (the sim's
true position, from `--log`). Each move only knows about itself, so the 1°
short turn and the 0.1 m drift after the first walk carry into the end
position. `--to` moves have the same limit. To run it again,
restart both terminals: after `o` the robot lies on the floor, and the band is
anchored above where it started.

Without Unitree's controller there is no built-in odometry, so we estimate
our own (`harp_control/leg_odometry.py`):
- **Heading:** from the pelvis IMU.
- **Position:** from leg kinematics. The lower foot is taken as planted, and
  the pelvis moves by minus that foot's motion.

In the sim (2026-10-25), a 90° turn was estimated at 90.1° and measured at
89.9°. A 2 m walk was estimated at 2.03 m and really went 2.03 m, and 1 m
backward was estimated at 1.11 m and really went 1.12 m. The robot drifts
about 0.1 m after the stop command, so walks end a little long. GR00T walked
at about 85% of the commanded speed. Each walk steers to hold the heading it
started with. Without that, a 2 m walk drifted 5–10° off straight; with it,
the heading stayed within about 2°.

### Curved moves: `--to`

`--to X Y [HEADING]` walks to a point in one movement, turning while it walks,
the way a person walks up to someone ahead and to one side. The robot keeps
walking forward and steers along a smooth curve that ends at the point,
already facing the final heading. It slows down on sharp curves and near the
point. If it still faces more than 5° off when it gets there, it finishes with
a short turn in place. The point is relative to where the robot stands when
the move starts, not to where the run started.

It never steps sideways, so a point close by and off to one side takes a
small loop to reach. A point behind the robot is reached in a wider curve
rather than by turning around first. Points closer than 0.16 m are refused:
use `--angle` to turn in place.

```bash
ros2 run harp_control move_distance --to 2 1 --speed 0.3
ros2 run harp_control move_distance --to 1.5 -1 -90 --speed 0.3
ros2 run harp_control move_distance --distance 1 --angle 90 --to 1 1 90 --speed 0.3
```

The curve comes from Park and Kuipers' control law ("A Smooth Control Law for
Graceful Motion of Differential Wheeled Mobile Robots in 2D Environments",
ICRA 2011), in `harp_control/smooth_control.py`. It's the same law, with the same default gains, as
Nav2's Graceful Controller, so moving to Nav2 later keeps the same behavior.

Results in the sim (2026-10-25, `--speed 0.3`, true positions from `--log`):

| Move | Target (x, y, heading) | Really ended | Estimated |
|---|---|---|---|
| `--to 2 1` | 2.00, 1.00, 27° | 1.97, 0.92, 32° | 1.96, 0.94, 32° |
| `--to 1.5 -1 -90` | 1.50, −1.00, −90° | 1.45, −0.94, −85° | 1.45, −0.93, −85° |
| `--to 1 0 180` | 1.00, 0.00, 180° | 1.07, 0.00, 175° | 1.08, 0.01, 175° |
| `--to -1 0.5` (behind) | −1.00, 0.50, 153° | −0.93, 0.54, 156° | −0.93, 0.54, 156° |
| `--to 1 1 90` after a walk and a turn | 1.00, 1.00, 90° | 1.03, 0.93, 94° | 1.04, 0.94, 94° |

It stops within 8 cm and 5° of the target by its own estimate. Moves took
13–21 s, including the 1.5 s settle. Only the robot's own estimate guides it,
so errors from earlier moves carry over, as in the example above.

### Example: weave left, then right, with two curves

The robot curves 1 m to the left over 2 m, straightens out facing forward
again, then curves 1 m back to the right. It ends 4 m straight ahead of where
it started, facing the way it started, without stopping to turn in place.

Terminal 1 (after `pixi shell` in `ros2/`, as in every terminal):

```bash
ros2 run harp_sim run_sim
```

Terminal 2:

```bash
ros2 run harp_control move_distance --to 2 1 0 --to 2 -1 0 --speed 0.3
```

| Step | Where | Do | You should see |
|---|---|---|---|
| 1 | terminal 2 | wait about 3 s | `Stand pose reached. Lower the robot onto its feet ...` |
| 2 | MuJoCo window | press `8` about five times | the feet touch the floor |
| 3 | terminal 2 | press `]` | `Policy on: GR00T is balancing. Release the band ...` |
| 4 | MuJoCo window | press `9` once | terminal 1: `Band released: the robot is on its own` |
| 5 | | wait a few seconds | the robot steadies itself |
| 6 | terminal 2 | press `g` | a curve to the left that straightens out, a pause, then a curve to the right |
| 7 | terminal 2 | press `o` when it says `All moves done` | `Stopped: all joints damped, so the robot sinks.` |

Terminal 2 after step 6 (from a sim run on 2026-10-25):

```
[1/2] Walking to +2.00 m forward, +1.00 m left, ending facing +0 deg, at up to 0.30 m/s
  Target (+2.00, +1.00) m facing +0 deg, estimated (+1.94, +0.96) m facing +3.2 deg
[2/2] Walking to +2.00 m forward, -1.00 m left, ending facing +0 deg, at up to 0.30 m/s
  Target (+2.00, -1.00) m facing +0 deg, estimated (+1.93, -1.03) m facing +4.8 deg
All moves done. GR00T keeps balancing; support the robot, then press o to stop (it will sink).
```

In that run the robot really ended 3.93 m forward and 0.01 m to the left of
where it started, facing 7° left (the sim's true position, from `--log`). The
two moves took about 15 s each. To approach someone instead, give the final
heading toward them: `--to 2 1 90` ends 2 m ahead and 1 m to the left, facing
left.

## On the real robot (not tested yet)

Running our own controller means turning Unitree's off, and then our code is
the only thing keeping the robot up. Before the first run:

1. Hang the robot from a gantry with its feet off the floor.
2. Press L1+A on the remote (damping), then L2+R2 (debug mode). The scripts
   check this and refuse to run while Unitree's controller is active.
3. Run from a laptop on the robot's network (`192.168.123.x`), set up as
   above, after `pixi shell`. The environment is only set up for Macs, so
   running on the robot's Jetson would need its own setup (untested).

```bash
ros2 run harp_control walk --real --interface en0
ros2 run harp_control move_distance --real --interface en0 --distance 0.5
ros2 run harp_ros walker_node --ros-args -p real:=true -p interface:=en0
```

   `--interface` is the laptop's network interface on `192.168.123.x`
   (`en0` is only an example; `ifconfig` shows which one).

4. Wait for the stand pose, lower the robot until its feet are on the floor,
   press `]`, then slacken the gantry. Keep the gantry rope just loose enough
   to catch a fall.

The scripts refuse to run if the robot's `mode_machine` isn't 5 (29-joint G1
rev 1.0, what NVIDIA's config is for). Pressing `o`, an error, or Ctrl-C damps
every joint, so only do that with the robot supported. Unitree's other
built-in services that need its controller (LocoClient, arm gestures,
`lf/odommodestate`) are probably unavailable in debug mode.

## Licenses

In `ros2/src/harp_control/harp_control/gr00t/`:
- `policy/*.onnx` are NVIDIA's, under the NVIDIA Open Model License
  (`policy/NVIDIA-Open-Model-License.txt`, `policy/NOTICE.txt`).
- `walk.py` follows NVIDIA's GR00T-WholeBodyControl deployment code
  (Apache-2.0, `LICENSE-GR00T-WholeBodyControl.txt`).
- The G1 model comes from Unitree's unitree_mujoco (BSD-3) and isn't copied
  into this repo.
