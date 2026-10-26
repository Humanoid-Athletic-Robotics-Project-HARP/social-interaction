# HARP Human Interaction

The Stanford HARP Human Interaction team's **Unitree G1 EDU (29 joints)**.
The aim: it walks around, notices people, walks up to them naturally, talks
with them and avoids obstacles, on open code we can read.

- It walks and balances with NVIDIA's GR00T policy (legs and waist). Our
  controller commands all 29 motors over the robot's low-level topics
  (`rt/lowstate`, `rt/lowcmd`).
- Everything runs in a MuJoCo sim of the G1 first; the same code drives the
  sim and the robot.
- The code is a ROS 2 workspace in `ros2/`:

| Path | What's in it |
|---|---|
| `ros2/src/harp_control` | Walking: the GR00T controller (`walk`), moves and curves to a point (`move_distance`), our leg odometry |
| `ros2/src/harp_sim` | The MuJoCo sim of the G1 (`run_sim`), with the head lidar and obstacle courses (`worlds/`) |
| `ros2/src/harp_ros` | ROS 2: `walker_node` (`/cmd_vel` in, `/odom` and `/tf` out) and Nav2 (`nav2.launch.py`, `config/`) |
| `ros2/pixi.toml`, `pixi.lock` | The environment: ROS 2 Jazzy, Python 3.12, MuJoCo, Unitree's SDK, pinned |
| `context/` | Shared notes for Claude and Codex: the detail behind this README |

## Goals and progress

✅ done · 🟡 in progress · ⬜ not started · ⛔ blocked

| Goal | Sim | Robot | Next step |
|---|---|---|---|
| Move like a person | ✅ | ⬜ | Run it on the robot |
| Get around obstacles | 🟡 | ⬜ | **Next:** more obstacle and path-following work in the sim |
| Find people and walk up to them | ⬜ | ⬜ | Pick a person detector for the camera |
| Talk with people | ⬜ | ⬜ | Make the speaker talk |
| Run safely on the real robot | – | ⛔ | Settle how HARP's safety layer fits in |

### Move like a person

- Done when: it stands, walks, turns and stops, reaches a point in one smooth
  curve (within about 10 cm and 5°), and can be driven over ROS 2.
- ✅ Sim: GR00T stands, walks, turns and stops safely (`walk`).
- ✅ Sim: set distances, turns and curves to a point and heading
  (`move_distance`), with our own leg odometry.
- ✅ Sim: driven over ROS 2, `/cmd_vel` in and `/odom` + `/tf` out (`walker_node`).
- ⬜ Robot: all of the above, and leg odometry checked against a tape measure.
- ⬜ Body language: facing the person, gestures.

### Get around obstacles

- Done when: it walks to a spot a few meters away and around boxes in its
  way, using the MID-360 lidar and Nav2.
- ✅ Sim: a simulated head lidar and boxes (`run_sim --lidar --box`).
- ✅ Sim: Nav2 (Graceful Controller, the same law as our curves) driving
  `walker_node` through the courses (`run_sim --world`), up to the
  obstacle course with a table, pillars, a thin pole and an archway, without
  touching anything; the robot's own body is filtered out of the cloud.
- ⬜ Sim: more on obstacles and path following: closer margins around
  obstacles, a collision-monitor stop, moving obstacles and people.
- ⬜ Robot: real lidar data in ROS 2, then the same walk.

### Find people and walk up to them

- Done when: it notices a person, walks up in one curve and stops about 1 m
  away, facing them.
- ⬜ A person detector on the D435i camera: where people are, to within
  about 0.3 m out to 4 m.
- ⬜ Sim: a camera, or recorded camera data, to test with.
- ✅ Building block: the curved move to a point and heading.
- ⬜ Sim, then robot: walk up to a person.

### Talk with people

- Done when: someone speaks to it and it answers out loud within a few
  seconds, with an LLM choosing what to say.
- ⬜ Speaker: Unitree's text-to-speech, also while our controller runs.
- ⬜ Microphone: read its audio stream.
- ⬜ Speech-to-text → LLM → text-to-speech loop, on a laptop first.

### Run safely on the real robot

Needed before any "Robot" box above.

- ⛔ Find out whether HARP's safety layer sits between our code and the
  motors, and route our commands through it if so.
- ⬜ An e-stop besides the keyboard, e.g. on the remote.
- ⬜ Check our robot: `mode_machine` 5, its hands, its ROS 2 version.
- ⬜ A gantry procedure and a first session with someone on the rope.

## Setup (on your laptop)

Runs on Apple Silicon Macs with macOS 14 or newer (tested on macOS 15). For
Linux, add its platform to `ros2/pixi.toml`.

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
├── unitree_mujoco/                     Unitree's G1 model files
└── unitree_ros2/                       Unitree's ROS 2 message definitions
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

The "detached HEAD" notes are expected, and so is a macOS warning that
`terrain.STL` and `terrain.stl` collided.

### 4. Install the environment and build the packages

```bash
cd stanford-harp-social-interaction/ros2
pixi install --locked
pixi run build
```

- `pixi install --locked` installs exactly the tested versions in `pixi.lock`
  (a few GB, into `ros2/.pixi/`, ignored by git).
- `pixi run build` builds our packages and Unitree's messages. Python edits
  take effect right away; rebuild after adding a file or package.
- Update versions (step 3's commits, `pixi.lock`) together, and re-test in the sim.

## Run it in the sim

Start every terminal with `pixi shell --as-is` in `ros2/`.

Terminal 1, the sim (a MuJoCo window opens):

```bash
ros2 run harp_sim run_sim
```

Terminal 2, the controller:

```bash
ros2 run harp_control walk
```

The robot starts hanging from an elastic band. Start-up is the same for every
controller below:

1. Wait for "Stand pose reached" (3 s).
2. MuJoCo window: press `8` a few times until the feet touch the floor.
3. Terminal 2: press `]` to start the policy. It has to be running before step 4.
4. MuJoCo window: press `9` to release the band, and give it a few seconds.
5. Terminal 2: drive with `w`/`s`, `a`/`d` (sideways), `q`/`e` (turn), `z` (stop
   moving); `o` stops, and the robot goes limp and sinks.

Keys go to the window you clicked last: band keys `7`/`8`/`9` to MuJoCo, the
rest to terminal 2. After `o`, restart both terminals to go again.

## Planned moves: `move_distance`

In terminal 2, in place of `walk`. Same start-up, then press `g` to start
the moves, and `o` once it says "All moves done".

```bash
ros2 run harp_control move_distance --distance 1.0 --speed 0.3
```

```bash
ros2 run harp_control move_distance --distance 2 --angle 90 --distance 2 --speed 0.3
```

```bash
ros2 run harp_control move_distance --to 2 1 90 --speed 0.3
```

```bash
ros2 run harp_control move_distance --to 2 1 0 --to 2 -1 0 --speed 0.3
```

- The first walks 1 m. The second walks 2 m, turns left 90° and walks 2 m.
- `--to X Y [HEADING]` walks to X m forward and Y m left in one smooth curve,
  ending facing HEADING degrees (positive left): the third ends facing left,
  as if walking up to someone. The fourth weaves left, then right, and ends
  4 m straight ahead.
- Moves repeat and mix, and run in order. Positive angles turn left.
- In the sim they end within about 8 cm and 5° of the target.

## Drive it through ROS 2: `walker_node`

The same GR00T walker, driven by any ROS 2 program (later Nav2):

| | Topic or service | Type |
|---|---|---|
| In | `/cmd_vel` | `geometry_msgs/Twist` (m/s, rad/s); stops 0.5 s after the last message |
| Out | `/odom`, `/tf` | `nav_msgs/Odometry`, `odom` → `base_link` |
| Out | `/tf` | `base_link` → `livox_frame`: where the head lidar is, from the joints and IMU |
| Service | `/walker/start`, `/walker/stop` | `std_srvs/Trigger`, like `]` and `o` |

Terminal 2: `ros2 run harp_ros walker_node`, then the usual start-up (`]` or
`ros2 service call /walker/start std_srvs/srv/Trigger`). Terminal 3:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

```bash
ros2 topic pub -r 10 -t 40 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}}"
```

```bash
ros2 topic echo /odom --field pose.pose
```

- Keyboard driving: hold the key down. `ros2 topic pub` walks 0.3 m/s for 4 s.
- ROS 2 runs on DDS domain 2, separate from the robot's own topics (0 on the
  robot, 1 in the sim). To see the sim's raw topics, prefix a command with
  `ROS_DOMAIN_ID=1`.

## Boxes and the lidar in the sim

```bash
ros2 run harp_sim run_sim --lidar --box 2 0 --box 0 -3 0.3 1.0 2.0
```

```bash
rviz2 -d $(ros2 pkg prefix harp_sim)/share/harp_sim/rviz/lidar.rviz
```

- `--box X Y [W D H]` puts a box X m ahead and Y m left of the start
  (default 0.5 m cube); repeatable.
- `--lidar` publishes the head lidar (Livox MID-360) as a point cloud on
  `/utlidar/cloud_livox_mid360` in `livox_frame`, the names Unitree uses on
  the robot, at 10 Hz.
- rviz2 needs `walker_node` running (above) for where the lidar sits on
  the robot (upside down, as on the G1). Stand the robot up so the floor
  lines up with the grid; drive it with teleop to walk toward the boxes.
- The mount's numbers are in `harp_control/lidar_mount.py`, shared by the
  sim's lidar and `walker_node`.
- The lidar can't see the floor within about 1 m of the robot, and it sees
  the robot's own shoulders and arms.

## Walk around obstacles with Nav2

Terminal 1, a course (a MuJoCo window opens; it prints the course's goal):

```bash
ros2 run harp_sim run_sim --lidar --world obstacle_course
```

Terminal 2: `ros2 run harp_ros walker_node`, then the usual start-up. Once
GR00T is balancing on its own, terminal 3 (rviz2 opens):

```bash
ros2 launch harp_ros nav2.launch.py
```

- In rviz2, wait until "Navigation" in the Navigation 2 panel is active
  (about 30 s), then click "2D Goal Pose" and click-drag the goal and the
  direction to face. The robot plans a path around what the lidar has seen
  and walks it.
- Courses, in order: `open_floor`, `one_box`, `slalom`, `obstacle_course`
  (in `harp_sim/worlds/`, MuJoCo files you can copy and edit). Goals are in
  meters from where the robot started (x ahead, y left).
- The sim prints any contact between the robot and an obstacle; a run with
  none never touched anything.
- Start Nav2 after the policy runs: it needs `/odom`. Restart all three to go
  again.

## On the real robot

Our code drives the motors directly and keeps the robot up on its own. Read
"Run safely on the real robot" in the goals before the first session.

1. Hang the robot from a gantry, feet off the floor.
2. Remote: L1+A (damping), then L2+R2 (debug mode). The scripts check for
   debug mode and `mode_machine` 5 before they start.
3. From a Mac on the robot's network (`192.168.123.x`), after `pixi shell`:

```bash
ros2 run harp_control walk --real --interface en0
```

4. Stand pose, lower until the feet are on the floor, `]`, then slacken the
   gantry, leaving the rope just loose enough to catch a fall.

- `--interface` is the laptop's interface on `192.168.123.x` (`ifconfig`
  shows it). `move_distance` takes the same flags; `walker_node` takes
  `--ros-args -p real:=true -p interface:=en0`.
- `o`, Ctrl-C or any error damps every joint, so keep the robot supported.

## More detail

Measured results, limits and design notes live in `context/` (start at
`context/MEMORY.md`): ask Claude or Codex, which read it first, to explain
or summarize anything here.

## Licenses

In `ros2/src/harp_control/harp_control/gr00t/`:
- `policy/*.onnx` are NVIDIA's, under the NVIDIA Open Model License
  (`policy/NVIDIA-Open-Model-License.txt`, `policy/NOTICE.txt`).
- `walk.py` follows NVIDIA's GR00T-WholeBodyControl deployment code
  (Apache-2.0, `LICENSE-GR00T-WholeBodyControl.txt`).
- The G1 model comes from Unitree's unitree_mujoco (BSD-3), cloned next to
  this repo.
