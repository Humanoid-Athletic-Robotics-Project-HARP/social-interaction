# HARP Human Interaction

This project is the workspace for the Human Interaction subteam within
Stanford HARP (Human Athletics Robotics Project). In this subteam, we are
focused on creating a natural, human-like robot. Some preliminary goals
include: walking around, avoiding obstacles, detecting people, walking up to
people, remembering people, talking and listening to people. All of our code
is open source, and we would love for people to build on top of it!

This project uses ROS 2 and a MuJoCo simulator. Here's a brief overview of
the project structure:

| Path | Description |
|---|---|
| `ros2/src/harp_control` | Walking. The GR00T controller (`walk`), planned moves and curves (`move_distance`), and our leg odometry. |
| `ros2/src/harp_sim` | Our MuJoCo sim of the G1 (`run_sim`), including the head lidar and the obstacle courses in `worlds/`. |
| `ros2/src/harp_ros` | The ROS 2 side. `walker_node` takes `/cmd_vel` and publishes `/odom` and `/tf`; `lidar_relay` brings the robot's lidar into ROS 2; Nav2's launch file and config. |
| `ros2/src/harp_eval` | Testing. Course tests in the sim (`course`), recording any run (`run_recorder`), comparing and checking runs (`runs`), and replaying a run into Nav2. |
| `ros2/src/spatio_temporal_voxel_layer` | STVL, the Nav2 costmap layer that lets lidar marks fade once an obstacle has moved on. A copy of the upstream package, built from here; our changes and how to update it are in `HARP_PATCHES.md`. |
| `submission/` | Our code inside HARP's safety harness and simulator (HIMRobotics' `him-sim2real-management`): the Docker image the harness runs on the robot and in its sim, the move to a point, leg odometry, the lidar sim, Nav2 and the course tests in `worlds/`. |
| `ros2/pixi.toml`, `pixi.lock` | The pinned environment: ROS 2 Jazzy, Python 3.12, MuJoCo and Unitree's SDK. |
| `context/` | Notes that Claude and Codex read before working here. This is where the details behind this README live. |

The robot is a Unitree G1 EDU with 29 joints. It walks and balances with
NVIDIA's GR00T policy, and our controller talks to all 29 motors over the
robot's low-level topics (`rt/lowstate`, `rt/lowcmd`). The same code drives
both the sim and the real robot, and we try everything in the sim first.

## Goals and progress

✅ done · 🟡 in progress · ⬜ not started · ⛔ blocked

| Goal | Sim | Robot | Next step |
|---|---|---|---|
| Move like a person | ✅ | ⬜ | Run it on the robot |
| Get around obstacles | 🟡 | ⬜ | **Next:** more obstacle and path-following work in the sim |
| Find people and walk up to them | ⬜ | ⬜ | Pick a person detector for the camera |
| Talk with people | ⬜ | ⬜ | Make the speaker talk |
| Run safely on the real robot | 🟡 | ⬜ | **Next:** first robot run of the move to a point through HARP's harness |

### Move like a person

- Done when: it stands, walks, turns and stops, reaches a point in one smooth
  curve (within about 10 cm and 5°), and can be driven over ROS 2.
- ✅ Sim: GR00T stands, walks, turns and stops safely (`walk`).
- ✅ Sim: set distances, turns and curves to a point and heading
  (`move_distance`), with our own leg odometry.
- ✅ Sim: driven over ROS 2, `/cmd_vel` in and `/odom` + `/tf` out (`walker_node`).
- ✅ Sim: inside HARP's harness, `/cmd_vel` drives HARP's GR00T walker, our leg
  odometry gives `/odom`, and walks, turns and moves to a point work
  (`submission/`, `G1_TASK_COMMAND`).
- ⬜ Robot: all of the above, and leg odometry checked against a tape measure.
- ⬜ Body language: facing the person, gestures.

### Get around obstacles

- Done when: it walks to a spot a few meters away and around boxes in its
  way, using the MID-360 lidar and Nav2.
- ✅ Sim: a simulated head lidar and boxes (`run_sim --lidar --box`).
- ✅ Sim: Nav2 (Graceful Controller, the same law as our curves) drives
  `walker_node` through every course (`run_sim --world`), including the
  obstacle course with a table, pillars, a thin pole and an archway, without
  touching anything. The robot's own body is filtered out of the lidar.
- 🟡 Sim: more on obstacles and path following: closer margins, a
  collision-monitor stop, moving obstacles and people. Started: the
  costmaps forget where a person was once they've moved on (STVL), tested
  with a person crossing the robot's path (`crossing`) and one walking
  toward it (`oncoming`).
- 🟡 Robot: real lidar data in ROS 2, then the same walk. Started:
  `lidar_relay`, so far only in the sim (course runs go through it).

### Find people and walk up to them

- Done when: it notices a person, walks up in one curve and stops about 1 m
  away, facing them.
- ⬜ A person detector on the D435i camera that places people to within
  about 0.3 m, out to 4 m.
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

This has to happen before any of the "Robot" boxes above.

- ✅ Our commands go through HARP's safety harness (`him-sim2real-management`),
  which owns stand-up, stops and the motors ("HARP's safety harness" below).
- 🟡 Move everything onto the harness (`submission/`). Sim: the walker, leg
  odometry, walks, turns and moves to a point, the lidar and Nav2 on
  `one_box`. Robot: not yet run.
- 🟡 Everything from the remote, the same as the keyboard, in `walk`,
  `move_distance` and `walker_node` ("Controls" below): B stops, L1 + sticks
  take over by hand from any program. Sim: tested through scripted remote
  presses. Robot: not yet tried.
- ⬜ Check our robot: `mode_machine` 5, which hands it has, its ROS 2 version.
- ⬜ A gantry procedure, and a first session with someone on the rope.

## Setup

So far we've run this on Apple Silicon Macs with macOS 14 or newer (tested
on macOS 15). Linux should work once its platform is added to
`ros2/pixi.toml`.

### 1. Install the tools

All you need is git and [pixi](https://pixi.sh). Pixi brings in everything
else: Python 3.12, ROS 2 Jazzy, compilers, MuJoCo, onnxruntime and CycloneDDS.

```bash
xcode-select --install
brew install pixi
```

### 2. Make a folder and clone this repo into it

The build expects Unitree's code to sit right next to this repo, so it's
easiest to give everything its own folder:

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

### 3. Clone Unitree's SDK, G1 model and messages

From inside `harp/`:

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

Don't worry about the "detached HEAD" messages, or a macOS warning that
`terrain.STL` and `terrain.stl` collided. Both are expected.

### 4. Install the environment and build

```bash
cd stanford-harp-social-interaction/ros2
pixi install --locked
pixi run build
```

`pixi install --locked` installs exactly the versions we tested with (a few
GB, into `ros2/.pixi/`, which git ignores). `pixi run build` builds our
packages and Unitree's messages. Python edits take effect right away, but
you'll need to rebuild after adding a new file or package. If you update
any versions (the commits in step 3 or `pixi.lock`), update them together
and re-test in the sim.

## HARP's safety harness: `submission/`

HARP's safety harness and simulator, HIMRobotics' `him-sim2real-management`,
is the only way our code reaches the motors. It stands the robot up, stops
it, checks every joint command, and runs our code as a Docker image. We use
it unchanged, cloned next to this repo, and keep only our code here.

### Setup

Clone it and download its recovery policy next to this repo. The repo is
private, so you need HARP's GitHub access. Docker Desktop must be running.

```
harp/
├── stanford-harp-social-interaction/   this repo
├── him-sim2real-management/            HARP's harness and simulator
└── models/fastsac_g1_29dof.onnx        its recovery policy (Holosoma)
```

From inside `harp/`:

```bash
git clone https://github.com/HIMRobotics/him-sim2real-management.git
```

```bash
mkdir -p models && curl -L -o models/fastsac_g1_29dof.onnx https://raw.githubusercontent.com/amazon-far/holosoma/d1e8499c10670f44c49d161b9876a4c17ac305f7/src/holosoma_inference/holosoma_inference/models/loco/g1_29dof/fastsac_g1_29dof.onnx
```

```bash
echo "8346fd90778439395922a8c7256f24125ae84b8dea949128bac9e23c02bc7717  models/fastsac_g1_29dof.onnx" | shasum -a 256 -c
```

Then, inside `him-sim2real-management/`, make its Python environment and
download its 3D viewer once:

```bash
python3 -m venv .venv-policy-safety && .venv-policy-safety/bin/pip install './packages/policy_safety[test]' certifi
```

```bash
SSL_CERT_FILE="$(.venv-policy-safety/bin/python -m certifi)" .venv-policy-safety/bin/python containers/sim/viewer_assets.py
```

### Build and try it in the sim

From this repo, build the image (`submission/submission.tar`, for the robot,
and `submission/joint_profile.json`, its joint limits and gains):

```bash
submission/build.sh
```

This walks the robot in one curve to 1 m ahead and 1 m left, ending facing
45° left:

```bash
G1_TASK_COMMAND='{"to": [1, 1, 45]}' submission/sim.sh --seconds 35
```

A page opens in your browser; press Start. For the first 5 s HARP's GR00T
balance holds the robot (on the robot, until the operator hands over), then
our image takes over and the move starts 2 s later. `"to": [X, Y, HEADING]`
is meters ahead, meters left and degrees left of where the robot stands.

Moves can also be chained, like `move_distance`: this walks 1 m, turns 90°
left, then curves to a point. They run in order, 2 s apart, each from where
the last one ended:

```bash
G1_TASK_COMMAND='{"moves": [{"walk": 1}, {"turn": 90}, {"to": [1, 1, 45]}]}' submission/sim.sh --seconds 50
```

`"walk"` is meters (negative walks backward), `"turn"` degrees (positive
turns left). `"speed"` (m/s, up to 0.4) and `"turn_speed"` (deg/s, up to 30)
are optional, next to `"moves"` or `"to"`. Without moves the robot stands.
The run is passed when `report.json` in the output folder (`--output DIR`)
says `within_limits`.

If the browser view makes your computer stall, HARP's harness can stop the
run (`host_stall`, `observation_timeout`). Add `--no-view` and watch the run
afterwards from `him-sim2real-management/`:

```bash
./desktop_sim.sh replay RUN_FOLDER/trace.npz
```

Course tests run Nav2 (our Foxy port of its Graceful Controller) with the
simulated lidar on one of `submission/worlds/` and score it against that
world's criteria, in a folder in `runs/`:

```bash
python3 submission/course.py one_box
```

### On the robot

HARP's operators set the harness up on the robot (its checkout in
`~/him-sim2real-management` and its background safety service). Copy our
image and joint profile over (`<g1>` is the robot's address):

```bash
scp submission/submission.tar unitree@<g1>:~/him-sim2real-management/containers/upload/
```

```bash
scp submission/joint_profile.json unitree@<g1>:~/harp_joint_profile.json
```

Hang the robot from the gantry with its feet bearing weight, and put it in
Unitree's locked standing mode. Then, on the robot, in
`~/him-sim2real-management`, check the image (nothing moves), run it with
the motors off, and finally run it for real:

```bash
./setup.sh --profile ~/harp_joint_profile.json
```

```bash
G1_TASK_COMMAND='{"to": [1, 1, 45]}' ./setup.sh --mode dry-run --profile ~/harp_joint_profile.json
```

```bash
G1_TASK_COMMAND='{"to": [1, 1, 45]}' ./submit.sh --mode real --profile ~/harp_joint_profile.json
```

GR00T balance takes over from Unitree's controller. Once it prints
`startup_phase: ready`, press **D-pad Left + B** to hand control to our
image. **Double Start damps the robot** at any time, and double F3 switches
to the recovery policy. Before running again, double-press Start, check it
damped, and put the robot back in locked standing mode.

## Run it in the sim

Open each terminal in `ros2/` and start it with `pixi shell --as-is`.

In the first terminal, start the sim. A MuJoCo window will open:

```bash
ros2 run harp_sim run_sim
```

In a second terminal, start the controller:

```bash
ros2 run harp_control walk
```

The robot starts out hanging from an elastic band. Every controller below
starts up the same way:

1. Wait for "Stand pose reached" (about 3 s).
2. In the MuJoCo window, press `8` a few times until the feet touch the floor.
3. In terminal 2, press Shift+P to start the policy. It has to be running
   before the next step.
4. In the MuJoCo window, press `9` to release the band, and give the robot a
   few seconds to settle.
5. Now drive it from terminal 2 (keys below). Space stops the controller,
   and the robot goes limp and sinks.

Keys go to whichever window you clicked last, so the band keys (`7`/`8`/`9`)
go to MuJoCo and everything else to terminal 2. `6` in MuJoCo presses B on
the robot's remote, the e-stop. After a stop, restart both terminals to go again.

### Controls

The keyboard (in the controller's terminal) and the robot's remote do the
same things in `walk`, `move_distance` and `walker_node`. Urgent actions are
one button; deliberate ones are R1 plus a button, or Shift plus a key.

| Action | Remote | Keyboard |
|---|---|---|
| Stop: damp every joint, the robot sinks | B | Space (or `o`) |
| Drive by hand, overriding the program | hold L1: left stick moves, right stick turns | `w`/`s`, `a`/`d`, `q`/`e` |
| Stop moving and balance in place | let go of L1 | `z` |
| Start the policy (balance) | R1 + start | Shift+P |
| Hand control back to the program | R1 + A | Shift+G |
| Raise / lower | R1 + up / down | Shift+R / Shift+F |

After driving by hand, the robot balances in place: `walker_node` ignores
`/cmd_vel` and `move_distance` cancels the moves it was making, until you
hand control back. In the sim there is no remote; scripted tests press its
buttons with a `remote` event on `/harp/events` (see `run_sim.py`).

## Planned moves: `move_distance`

Run this in terminal 2 instead of `walk`. Start-up is the same; then press
Shift+G (R1+A) to start the moves, and Space once it says "All moves done".

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

The first one walks 1 m. The second walks 2 m, turns 90° left, and walks
another 2 m. `--to X Y [HEADING]` walks in one smooth curve to a point X m
ahead and Y m to the left, ending up facing HEADING degrees (positive is
left). So the third ends facing left, the way you'd walk up to someone, and
the fourth weaves left, then right, and ends 4 m straight ahead. You can
repeat and mix moves as much as you like; they run in order. In the sim they
end within about 8 cm and 5° of the target.

## Drive it through ROS 2: `walker_node`

`walker_node` is the same GR00T walker, but any ROS 2 program (like Nav2)
can drive it:

| | Topic or service | Type |
|---|---|---|
| In | `/cmd_vel` | `geometry_msgs/Twist` (m/s, rad/s); stops 0.5 s after the last message |
| Out | `/odom`, `/tf` | `nav_msgs/Odometry`, `odom` → `base_link` |
| Out | `/tf` | `base_link` → `livox_frame`: where the head lidar is, from the joints and IMU |
| Out | `/joint_states`, `/imu`, `/harp/events` | the joints, the pelvis IMU, and events (policy on, fall, ...) for run logs |
| Service | `/walker/start`, `/walker/stop` | `std_srvs/Trigger`, like Shift+P and Space |

In terminal 2, run `ros2 run harp_ros walker_node` and do the usual
start-up (press Shift+P, or call
`ros2 service call /walker/start std_srvs/srv/Trigger`). Then, from a third
terminal, drive it with the keyboard, send it a fixed speed, or watch its
odometry:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

```bash
ros2 topic pub -r 10 -t 40 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}}"
```

```bash
ros2 topic echo /odom --field pose.pose
```

With the keyboard, hold the key down. The `ros2 topic pub` line walks at
0.3 m/s for 4 s. Our ROS 2 traffic runs on DDS domain 2, separate from the
robot's own topics (domain 0 on the robot, 1 in the sim). To look at the
sim's raw topics, put `ROS_DOMAIN_ID=1` in front of the command.

## Boxes and the lidar in the sim

```bash
ros2 run harp_sim run_sim --lidar --box 2 0 --box 0 -3 0.3 1.0 2.0
```

```bash
rviz2 -d $(ros2 pkg prefix harp_sim)/share/harp_sim/rviz/lidar.rviz
```

`--box X Y [W D H]` puts a box X m ahead and Y m left of where the robot
starts (a 0.5 m cube by default), and you can add as many as you want.
`--lidar` turns on the head lidar (a Livox MID-360), which publishes a
point cloud at 10 Hz on `/utlidar/cloud_livox_mid360` in `livox_frame`,
the same names Unitree uses on the real robot. `--lidar-dds` publishes it
the way the robot does instead, and `ros2 run harp_ros lidar_relay` brings
it into ROS 2; course tests run that way.

rviz2 needs `walker_node` running to know where the lidar sits on the
robot (it's mounted upside down, like on the G1). Stand the robot up so the
floor lines up with the grid, then drive it toward the boxes with teleop.
Keep in mind that the lidar can't see the floor within about 1 m of the
robot, and that it does see the robot's own shoulders and arms.

## Walk around obstacles with Nav2

In terminal 1, start the sim with a course. The MuJoCo window opens and the
sim prints the course's goal:

```bash
ros2 run harp_sim run_sim --lidar --world obstacle_course
```

In terminal 2, run `ros2 run harp_ros walker_node` and do the usual
start-up. Once GR00T is balancing on its own, start Nav2 in terminal 3
(rviz2 opens with it). Nav2 needs `/odom`, so it has to come after the
policy is running.

```bash
ros2 launch harp_ros nav2.launch.py
```

In rviz2, wait a few seconds for "Navigation" in the Navigation 2 panel to
turn active. Then click "2D Goal Pose" and click-drag to set the goal and
the direction to face. The robot plans a path around whatever the lidar has
seen, and walks it.

The courses, from easiest to hardest, are `open_floor`, `one_box`, `slalom`
and `obstacle_course`. With people: in `crossing` someone walks across the
robot's path once, and in `oncoming` someone walks toward it. They're
MuJoCo files in `harp_sim/worlds/`, so feel free to copy and edit them (how
to make something move: `harp_sim/movers.py`). Goals are in meters from
where the robot started (x ahead, y left). The sim prints any contact
between the robot and an obstacle, so if you don't see any, it never
touched anything. To go again, restart all three terminals.

If you want to keep the results, use `course` (next section) instead. It
does all of this in one command and records the run.

## Test runs and logs

This runs a course test in the sim three times and records each run:

```bash
ros2 run harp_eval course slalom --tag baseline --repeat 3
```

And this compares the runs and plots their paths:

```bash
ros2 run harp_eval runs compare --world slalom --plot paths.png
```

`course` starts the sim, `walker_node`, Nav2 and the course's goal for you
(the windows open as usual), and stops when the robot arrives, gives up or
falls. Each repeat starts fresh, and Ctrl-C stops it.

Every run gets its own folder in `runs/` (which git ignores). It has the
settings, the git commit, what happened when, the true and estimated
paths, every program's output, and a `summary.json` with the result and
pass/fail, time, closest distance to an obstacle, contacts, Nav2
recoveries, odometry error, speed tracking and lidar health.

A few more things you can do:

- `--tag` says what a run is testing. `runs compare` groups runs by course
  and tag, and `runs check` shows pass rates against each course's criteria
  in `harp_sim/worlds/<course>.yaml`.
- `--params FILE` runs Nav2 with other settings than
  `harp_ros/config/nav2_params.yaml`, to compare them (give each its own
  `--tag`).
- To record anything that isn't a course (driving by hand, `move_distance`),
  start `ros2 run harp_eval run_recorder --ros-args -p tag:=NAME` first.
  Ctrl-C on it ends the run.
- `--extras nav,lidar --extras-reason "why"` also saves Nav2's costmaps or
  the lidar to a bag (see `harp_eval/config/logging.yaml`).
- `--replay --replay-reason "why"` saves enough to replay the run into Nav2
  later with different settings (`replay.launch.py`). These recordings are
  large, so only make one when someone asks for it.
- `runs list` and `runs show RUN` let you browse, `runs set RUN key=value`
  records conditions such as `tether=gantry`, and `runs pin RUN` keeps a
  run around for good. Older runs get trimmed automatically.

## On the real robot

On the real robot, our code drives the motors directly and is the only
thing keeping the robot up. Read "Run safely on the real robot" in the
goals above before your first session.

1. Hang the robot from a gantry with its feet off the floor.
2. On the remote, press L1+A (damping), then L2+R2 (debug mode). The scripts
   check for debug mode and `mode_machine` 5 before they start.
3. From a Mac on the robot's network (`192.168.123.x`), run `pixi shell --as-is`
   and then:

```bash
ros2 run harp_control walk --real --interface en0
```

4. Wait for the stand pose, lower the robot until its feet are on the
   floor, press R1+start on the remote (or Shift+P), then slacken the
   gantry, leaving the rope just loose enough to catch a fall.

`--interface` is your laptop's network interface on `192.168.123.x`
(`ifconfig` will show it). `move_distance` takes the same flags, and
`walker_node` takes `--ros-args -p real:=true -p interface:=en0`.

**B on the remote is the e-stop.** It, Space, Ctrl-C, a fall or any error
damps every joint. On the real robot the program then keeps damping until
you press `o` or Ctrl-C, so press that only once the robot is secured. The
operator with the remote can take over at any time by holding L1
("Controls" above).

For Nav2, also run `ros2 run harp_ros lidar_relay --ros-args -p real:=true
-p interface:=en0`, which brings the head lidar into ROS 2.

## More detail

Measured results, limits and design notes are in `context/` (start with
`context/MEMORY.md`). Claude and Codex read it before they start, so you can
ask either of them to explain or summarize anything here.

## Licenses

In `ros2/src/harp_control/harp_control/gr00t/`:
- `policy/*.onnx` are NVIDIA's, under the NVIDIA Open Model License
  (`policy/NVIDIA-Open-Model-License.txt`, `policy/NOTICE.txt`).
- `walk.py` follows NVIDIA's GR00T-WholeBodyControl deployment code
  (Apache-2.0, `LICENSE-GR00T-WholeBodyControl.txt`).
- The G1 model comes from Unitree's unitree_mujoco (BSD-3), cloned next to
  this repo.

`ros2/src/spatio_temporal_voxel_layer/` is Steve Macenski's STVL, under
LGPL-2.1 (`LICENSE`). Its `openvdb_vendor` builds OpenVDB, which is MPL-2.0.

`submission/nav2_graceful_controller/` is Nav2's Graceful Controller (Alberto
J. Tudela Roldán), under Apache-2.0 (`LICENSE`), ported to ROS 2 Foxy; the
upstream commit and our changes are in its `HARP_PATCHES.md`.
