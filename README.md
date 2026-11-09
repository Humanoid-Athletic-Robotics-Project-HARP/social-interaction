# HARP Social Interaction

We are the social interaction team of
[HARP](https://github.com/Humanoid-Athletic-Robotics-Project-HARP), Stanford's
Humanoid Athletic Robotics Project. We're building a humanoid robot that walks
around, finds people, walks up to them and talks with them.

## What it does

The robot can:

- walk, turn and curve to a point you give it
- walk to a goal a few meters away, around boxes, pillars and tables
- stand still while someone walks at it, then carry on
- wander on its own, notice people standing nearby, walk up to them and stop
  about 1 m away, facing them

On a laptop, our speech code listens to a person and answers out loud with a
local LLM and voice.

[Goals and progress](#goals-and-progress) shows what works in the sim and on
the robot.

## How it works

- The robot is a Unitree G1 EDU with 29 joints, a Livox MID-360 lidar and a
  RealSense D435i depth camera in its head.
- It walks with NVIDIA's GR00T whole-body control policy.
- Our code reaches the motors only through HARP's safety harness, HIMRobotics'
  [`him-sim2real-management`](https://github.com/HIMRobotics/him-sim2real-management).
  The harness stands the robot up, checks every joint command and stops the
  robot. It runs our code the same way on the robot and in its MuJoCo
  simulator, so we test everything in the sim first.
- Our code is one ROS 2 Foxy Docker image. Any program in it drives the robot
  by publishing `/cmd_vel`, and reads `/odom`, `/tf`, `/joint_states` and
  `/imu`. Nav2 plans the walks around obstacles.
- Speech runs beside the image as two programs: `listen.py` turns speech into
  text, and `talk.py` answers out loud. `web.py` serves a page to run and
  watch both.

## Repository layout

| Path | What's in it |
|---|---|
| `submission/` | Our Docker image. `build.sh` builds it, `sim.sh` runs it in the sim, `course.py` runs and scores test courses. |
| `submission/harp_node.py`, `harp/` | The node that drives GR00T from `/cmd_vel` and publishes odometry. Also planned moves, the people loop, standing for oncoming people, the simulated sensors and the real lidar's relay. |
| `submission/nav2/`, `nav2_graceful_controller/` | Nav2's settings, and our ROS 2 Foxy port of its Graceful Controller (`HARP_PATCHES.md`). |
| `submission/worlds/`, `routes/` | Test courses with their pass criteria, and waypoint routes. |
| `submission/tests/` | Unit tests. |
| `speech/` | Listening and talking ([speech/README.md](speech/README.md)). |
| `viewer/` | One web dashboard for simultaneous simulation, speech, PC2 sensors and saved diagnostics ([viewer/README.md](viewer/README.md)). |
| `testground/` | Read-only tests on the real robot, and tools to analyse their recordings ([testground/README.md](testground/README.md)). |
| `third_party/` | Pinned forks of Ultralytics (YOLO26) and BoT-SORT, for choosing a person detector ([third_party/README.md](third_party/README.md)). |
| `context/` | Notes for Claude and Codex: measured results, limits and the reasons behind our design. |
| `docs/architecture/` | The architecture guide: every part and how they connect, with diagrams. `python3 docs/architecture/build.py` builds the PDF. |

## Goals and progress

✅ done · 🟡 in progress · ⬜ not started · ⛔ blocked

| Goal | Sim | Robot | Next step |
|---|---|---|---|
| Move like a person | ✅ | ⬜ | Run it on the robot |
| Get around obstacles | 🟡 | ⬜ | More obstacle and path-following work in the sim |
| Find people and walk up to them | 🟡 | ⬜ | Pick a person detector for the camera |
| Talk with people | ⬜ | ⬜ | A live conversation on a laptop; record the robot's mic |
| Run safely on the real robot | 🟡 | ⬜ | First robot run of the move to a point through HARP's harness |

### Move like a person

- Done when: it stands, walks, turns and stops, reaches a point in one smooth
  curve (within about 10 cm and 5°), and can be driven over ROS 2.
- ✅ Sim: GR00T stands, walks, turns and stops through HARP's harness.
- ✅ Sim: walks, turns and curves to a point and heading (`harp/moves.py`),
  with our own leg odometry.
- 🟡 Sim: speed ramps and feedback guards for planned moves. Unit tests pass;
  next, check them in the sim.
- ✅ Sim: driven over ROS 2, `/cmd_vel` in, `/odom` and `/tf` out
  (`harp_node.py`).
- ⬜ Robot: all of the above, and leg odometry checked against a tape measure.
- ✅ Sim: a right-arm hello wave (`{"wave": {}}`, `harp/wave.py`) within HARP's
  checks as the robot runs them.
- ⬜ Body language: facing the person, more gestures, the wave on the robot.

### Get around obstacles

- Done when: it walks to a spot a few meters away and around boxes in its
  way, using the MID-360 lidar and Nav2.
- ✅ Sim: a simulated head lidar and test courses (`submission/worlds/`).
- 🟡 Sim: the lidar and the head depth camera as mounted on the robot, with
  the face visor off or on, both in Nav2's costmaps (`harp/lidar_mount.py`,
  `harp/lidar.py`, `harp/depth_camera.py`).
- ✅ Sim: Nav2 walks every course without touching anything, including a
  table, pillars, a thin pole and an archway (`course.py`). The robot's own
  body is cut out of the lidar's cloud from its joint angles, wherever the
  arms are (`harp/self_filter.py`; checked on two recordings from the robot with
  `testground/lidar/self_filter_check.py`).
- ✅ Sim: routes of several points, lap after lap (`course.py --route`,
  `--waypoint`, `--laps`).
- 🟡 Recording waypoints from `/odom` (`record_waypoints.sh`). Local ROS tests
  pass; next, check it in the image.
- 🟡 Sim: closer margins, a collision-monitor stop, moving obstacles and
  people. Done so far: the costmaps forget people who have moved on (STVL),
  and the robot stands while someone walks at it or the lidar goes quiet
  (`harp/approach_stop.py`, tested in `crossing` and `oncoming`).
- 🟡 Sim: predicting moving obstacles from lidar tracks, then slowing,
  standing or stepping aside (`--avoidance predictive`). Tested on
  `crossing`, the box route and `building` with walkers.
- 🟡 Robot: the real lidar in ROS 2, then the same walk. The lidar reaches
  ROS 2 on PC2 through Unitree's `lidar_driver` and `harp/lidar_relay.py`;
  next, the head camera's depth in our image, then Nav2 on the robot.
- ⬜ Robot: the self-filter in `harp/lidar_relay.py` through the harness, and
  recordings with the arms moving (`testground/lidar/self_filter_check.py`).

### Find people and walk up to them

- Done when: it notices a person, walks up in one curve and stops about 1 m
  away, facing them.
- ⬜ A person detector on the D435i camera that places people within about
  0.3 m, out to 4 m.
- 🟡 Sim: a depth camera to test with (`harp/depth_camera.py`). Colour needs
  recorded camera data.
- ✅ Building block: the curved move to a point and heading.
- ✅ Sim: wander, walk up to people standing still and "talk" with them, with
  stand-ins for the detector and speech (`harp/behavior.py`,
  `course.py people`).
- ✅ Sim: the lidar finds person candidates and the camera confirms them on
  the way in. A pillar gets checked and passed over (`course.py decoys`).
- ⬜ Robot: walk up to a person.

### Talk with people

- Done when: someone speaks to it and it answers out loud within a few
  seconds, with an LLM choosing what to say.
- ⬜ Speaker: Unitree's text-to-speech, also while our controller runs.
- 🟡 Microphone: `speech/listen.py --robot` reads the robot's mic stream,
  tested with a stand-in sender on a laptop. Next, on the robot.
- 🟡 Listening, an LLM and a voice working together on a laptop
  (`listen.py --mic | talk.py --heard`). Tested with recorded voices, not yet
  with a person.

### Run safely on the real robot

This has to happen before any of the "Robot" boxes above.

- ✅ Our commands go through HARP's safety harness, which owns stand-up, stops
  and the motors ("On the robot" below).
- 🟡 Our image runs in the harness (`submission/`). No application code has
  run on the robot yet. Checked in the sim:
  - ✅ walking, stand-up, stops and safety checks (`harp_node.py`)
  - ✅ leg odometry on `/odom` and `/tf`, the lidar's live transform,
    `/joint_states` and `/imu` (`harp/state.py`)
  - ✅ planned turns and curves (`harp/moves.py`)
  - ✅ the simulated head lidar, depth camera and moving people
    (`harp/lidar.py`, `harp/depth_camera.py`, `harp/movers.py`)
  - ✅ Nav2 with STVL and the Graceful Controller: all six obstacle courses
    pass (`nav2/`, `nav2_graceful_controller/`)
  - ✅ standing while someone walks at the robot or the lidar goes quiet
    (`harp/approach_stop.py`)
  - ✅ course tests and their scores (`course.py`)
  - 🟡 waypoint recording and straight-walk speed ramps: need checks in the
    image and the sim
  - 🟡 the robot's lidar in ROS 2 (`harp/lidar_relay.py`): works on PC2, not
    yet through the harness
  - 🟡 the head camera's depth on the harness's ROS 2 network
    (`testground/camera/depth_node.py` on PC2): not yet read by our image
- 🟡 Stopping from the remote with the harness's buttons ("On the robot"
  below). Not yet tried on the robot.
- ⬜ Check our robot: `mode_machine` 5, which hands it has, its ROS 2 version.
- ⬜ A gantry procedure, and a first session with someone on the rope.

## Setup

Tested on Apple Silicon Macs (macOS 15) with Docker Desktop running. The
harness repo is private, so you need HARP's GitHub access.

Put the folders side by side:

```
harp/
├── social-interaction/             this repo
├── him-sim2real-management/        HARP's harness and simulator
└── models/                         the harness's recovery policy, and speech models
```

From inside `harp/`, clone the harness:

```bash
git clone https://github.com/HIMRobotics/him-sim2real-management.git
```

Download its recovery policy (Holosoma) and check its hash:

```bash
mkdir -p models && curl -L -o models/fastsac_g1_29dof.onnx https://raw.githubusercontent.com/amazon-far/holosoma/d1e8499c10670f44c49d161b9876a4c17ac305f7/src/holosoma_inference/holosoma_inference/models/loco/g1_29dof/fastsac_g1_29dof.onnx
```

```bash
echo "8346fd90778439395922a8c7256f24125ae84b8dea949128bac9e23c02bc7717  models/fastsac_g1_29dof.onnx" | shasum -a 256 -c
```

Inside `him-sim2real-management/`, make its Python environment and download
its 3D viewer once:

```bash
python3 -m venv .venv-policy-safety && .venv-policy-safety/bin/pip install './packages/policy_safety[test]' certifi
```

```bash
SSL_CERT_FILE="$(.venv-policy-safety/bin/python -m certifi)" .venv-policy-safety/bin/python containers/sim/viewer_assets.py
```

The speech models also go in `models/` ([speech/README.md](speech/README.md)).

## Unified viewer

Start the dashboard on the laptop or PC2, then open http://127.0.0.1:28734:

```bash
python3 viewer/server.py
```

It follows our simulation container and connects to local speech. Jetson sensor
and speech sources use `G1_HOST` from the main checkout's gitignored `.env`.
Home shows a tile with short live details for each running process, then lists every service view,
including offline sources and HIM's local viewer. Click a tile or row to use the full window, then
choose Home to return. A line at the top says whether this computer has the memory
for a sim run. Saved runs lists every run in `runs/`: Replay one, tick two and Compare them side by
side, or open Timing to chart its states against HIM's deadline and the computer's memory. Setup and source configuration: [viewer/README.md](viewer/README.md).

## Run it in the sim

### Build the image

From this repo, build `submission/submission.tar` (the image) and
`submission/joint_profile.json` (its joint limits and gains). Rebuild after
every code change:

```bash
submission/build.sh
```

### Planned moves

Walk in one curve to 1 m ahead and 1 m left, ending facing 45° left:

```bash
G1_TASK_COMMAND='{"to": [1, 1, 45]}' submission/sim.sh --seconds 35
```

A viewer comes up at the address printed. Press Start there to watch from the
beginning. HARP's GR00T balance holds the robot for the first 5 s, then our
image takes over and the move starts 2 s later.

Chain moves. Each one starts 2 s after the last, from where it ended:

```bash
G1_TASK_COMMAND='{"moves": [{"walk": 1}, {"turn": 90}, {"to": [1, 1, 45]}]}' submission/sim.sh --seconds 50
```

- `"to": [X, Y, HEADING]` is meters ahead, meters left and degrees left of
  where the robot stands.
- `"walk"` is meters (negative walks backward), `"turn"` is degrees (positive
  turns left).
- `"speed"` (m/s, up to 0.4) and `"turn_speed"` (deg/s, up to 30) are
  optional, next to `"moves"` or `"to"`.
- `G1_TASK_COMMAND='[0,0,0]'` stands still.

### Wave

Raise the right arm, wave three times and lower it while GR00T keeps the
robot standing (`harp/wave.py`). It starts 2 s after our image takes over, and
the hand is up for 7.75 s:

```bash
G1_SIM_THRASH_MODE=enforce G1_TASK_COMMAND='{"wave": {}}' submission/sim.sh --seconds 16
```

`G1_SIM_THRASH_MODE=enforce` stops the run on joint thrashing, as on the
robot (the sim only logs it by default). To score a run against the wave's
rules (balance, sway, feet, thrashing, timing), add `--no-view --output
RUN_FOLDER`, then:

```bash
python3 submission/wave_check.py RUN_FOLDER
```

### Test courses

Run Nav2 on a course and score it against that course's criteria. Each run
gets a folder in `runs/`:

```bash
python3 submission/course.py one_box
```

Add `--view` to watch each run live at the address printed.

| Course | What it tests |
|---|---|
| `open_floor`, `one_box`, `slalom`, `obstacle_course` | Obstacles, from easiest to hardest |
| `crossing` | Someone walks across the robot's path |
| `oncoming` | Someone walks toward the robot, which stands until they pass |
| `building`, `office`, `atrium` | Crowds of walkers, for predictive avoidance |
| `people`, `people_ahead`, `decoys` | The people loop (below) |

Courses are MuJoCo files in `submission/worlds/`, so copy and edit them
freely. `submission/harp/movers.py` shows how to make something move.

Compare every scored run of a course, by tag (`--tag` names what a run is
testing):

```bash
python3 submission/course.py --compare slalom
```

To test on a world nobody has tuned against, generate one, commit it, then
run it once:

```bash
python3 submission/generate_world.py --seed 20261111
```

```bash
python3 submission/course.py heldout_20261111 --avoidance predictive --crowd aware
```

### Predictive avoidance

The default, `--avoidance reactive`, stands the robot while something comes at
it. Predictive avoidance tracks moving objects in the lidar and slows, stands
or steps aside. It publishes `/harp/obstacle_alert` (JSON) when its chosen
motion isn't clearly safe. It works with Nav2 courses and waypoint routes,
not planned moves:

```bash
python3 submission/course.py crossing --avoidance predictive --tag predictive
```

```bash
python3 submission/course.py one_box --route rectangle --avoidance predictive --tag predictive-route
```

```bash
python3 submission/course.py building --avoidance predictive --crowd aware --tag predictive-building
```

How it works and how well: [context/predictive-avoidance.md](context/predictive-avoidance.md).

### Routes and waypoints

Walk a route of several points instead of the course's goal:

```bash
python3 submission/course.py one_box --route rectangle --laps 3
```

The [rectangle route](submission/routes/rectangle.yaml) circles `one_box`'s
box. Copy it and pass `--route FILE.yaml`, or give each point directly:

```bash
python3 submission/course.py open_floor --waypoint 2 0 --waypoint 2 2 90 --waypoint 0 0
```

Each point is `X Y [HEADING]`, in meters and degrees from where the robot
started.

To record a route, start a sim run, then find its container in another
terminal:

```bash
docker ps --filter label=com.docker.compose.service=submission --format '{{.Names}}'
```

Start the recorder with that name in place of `CONTAINER`:

```bash
submission/record_waypoints.sh CONTAINER submission/routes/my_route.yaml
```

Wait for "Odometry ready". Move the robot to each spot and press Enter to
record it. `u` undoes the last point, `s` saves and exits, `q` quits without
saving. The points are relative to where odometry started, so start from the
same place when you walk the route.

### The people loop

The robot wanders within 2.5 m of where it started and walks up to anyone
standing still. It stops facing them and "talks" (the line it would say goes
to `policy.log`) until they walk away or 30 s pass. Then it turns around and
carries on. Until we pick a person detector, a stand-in finds people
(`harp/sim_people.py`).

```bash
python3 submission/course.py people
```

In `people_ahead`, someone 3.5 m straight ahead should get walked up to before
a nearer person in the lidar's blind spot to the side:

```bash
python3 submission/course.py people_ahead
```

In `decoys`, the lidar takes a pillar for a person. The robot should check the
pillar, pass over it and walk up to the real person:

```bash
python3 submission/course.py decoys
```

### Watch and inspect runs

Replay any finished run, people included (the live viewer doesn't show them):

```bash
python3 submission/course.py --replay RUN_FOLDER
```

List a run's events with their sim times:

```bash
python3 submission/course.py --events RUN_FOLDER
```

If the harness stops a run (`host_stall`, `observation_timeout`), your
computer is short of memory. Close other apps and any viewer still up from an
earlier run (Ctrl-C), or add `--no-view --output RUN_FOLDER` and replay the
run afterwards.

To see what the robot sees in [Rerun](https://rerun.io) (its lidar, where it
thinks it is and where it really is, the people, Nav2's costmap and plan), set
up `testground/`'s tools once:

```bash
cd testground && python3 -m venv .venv && .venv/bin/pip install -r requirements.txt
```

Then start the view before a run and open the address it prints. It shows
each sim run as it happens, until Ctrl-C. `--native` opens a window instead,
and `--run RUN_FOLDER` shows a finished run.

```bash
python3 submission/lidar_view.py
```

### Tests

The tests need ROS 2, so they run in the image (build it first):

```bash
docker run --rm -v "$PWD/submission:/src:ro" -w /src --entrypoint bash harp-submission:local -c 'source /opt/ros/foxy/setup.bash && PYTHONPATH=/src:$PYTHONPATH python3 -m unittest discover tests'
```

## On the robot

HARP's operators set up the harness on the robot. Then:

1. Copy our image and joint profile to the robot (`<g1>` is its address):

   ```bash
   scp submission/submission.tar unitree@<g1>:~/him-sim2real-management/containers/upload/
   ```

   ```bash
   scp submission/joint_profile.json unitree@<g1>:~/harp_joint_profile.json
   ```

2. Hang the robot from the gantry with its feet bearing weight, and put it in
   Unitree's locked standing mode.
3. On the robot, in `~/him-sim2real-management`, check the image (nothing
   moves):

   ```bash
   ./setup.sh --profile ~/harp_joint_profile.json
   ```

4. Run it with the motors off:

   ```bash
   G1_TASK_COMMAND='{"to": [1, 1, 45]}' ./setup.sh --mode dry-run --profile ~/harp_joint_profile.json
   ```

5. Run it for real:

   ```bash
   G1_TASK_COMMAND='{"to": [1, 1, 45]}' ./submit.sh --mode real --profile ~/harp_joint_profile.json
   ```

6. GR00T balance takes over from Unitree's controller. Once it prints
   `startup_phase: ready`, press **D-pad Left + B** to hand control to our
   image.

Stopping:

- **Double Start damps the robot** at any time.
- Double F3 switches to the recovery policy.
- Before running again, double-press Start, check the robot damped, and put
  it back in locked standing mode.

## More detail

`context/` holds measured results, limits and design notes (start with
`context/MEMORY.md`). Claude and Codex read it before working here, so you can
ask either of them to explain anything in this README.

## Licenses

In `viewer/` (installed separately for offline sensor views):

| Project | License |
|---|---|
| [Three.js 0.160.0](https://github.com/mrdoob/three.js/tree/r160) | MIT |
| [Rerun 0.41.0](https://github.com/rerun-io/rerun) (from `testground/requirements.txt`) | MIT OR Apache-2.0 |

In our image:

| Project | License |
|---|---|
| [Nav2 Graceful Controller](https://github.com/ros-navigation/navigation2) (our Foxy port, `submission/nav2_graceful_controller/`) | Apache-2.0 |
| [GR00T-WholeBodyControl](https://github.com/NVlabs/GR00T-WholeBodyControl) networks and walker client, from HARP's harness | NVIDIA Open Model License |
| [Unitree G1 model](https://github.com/google-deepmind/mujoco_menagerie), from MuJoCo Menagerie | BSD-3-Clause |
| [unitree_sdk2_python](https://github.com/unitreerobotics/unitree_sdk2_python) | BSD-3-Clause |
| [Eclipse CycloneDDS](https://github.com/eclipse-cyclonedds/cyclonedds) | EPL-2.0 |
| [MuJoCo](https://github.com/google-deepmind/mujoco) | Apache-2.0 |
| [NumPy](https://numpy.org) | BSD-3-Clause |
| [SciPy](https://scipy.org) | BSD-3-Clause |
| [ONNX Runtime](https://github.com/microsoft/onnxruntime) | MIT |
| [PyYAML](https://github.com/yaml/pyyaml) | MIT |
| [zipp](https://github.com/jaraco/zipp) | MIT |

In `speech/`:

| Project | License |
|---|---|
| [sherpa-onnx](https://github.com/k2-fsa/sherpa-onnx) | Apache-2.0 |
| [ONNX Runtime](https://github.com/microsoft/onnxruntime) | MIT |
| [python-sounddevice](https://github.com/spatialaudio/python-sounddevice) | MIT |
| [NumPy](https://numpy.org) | BSD-3-Clause |
| [llama.cpp](https://github.com/ggml-org/llama.cpp) | MIT |
| [Silero VAD](https://github.com/snakers4/silero-vad) v6.2.3 | MIT |
| [Smart Turn](https://github.com/pipecat-ai/smart-turn) v3.2 by Daily | BSD-2-Clause |
| [Parakeet TDT 0.6B v2](https://huggingface.co/nvidia/parakeet-tdt-0.6b-v2) by NVIDIA, converted to int8 ONNX by sherpa-onnx | [CC-BY-4.0](https://creativecommons.org/licenses/by/4.0/) |
| [Qwen3-4B-Instruct-2507](https://huggingface.co/Qwen/Qwen3-4B-Instruct-2507) by the Qwen team, Q4_K_M GGUF by [lmstudio-community](https://huggingface.co/lmstudio-community/Qwen3-4B-Instruct-2507-GGUF) | Apache-2.0 |
| [Kokoro-82M](https://huggingface.co/hexgrad/Kokoro-82M) v1.0 by hexgrad | Apache-2.0 |
| [misaki](https://github.com/hexgrad/misaki) word lists | Apache-2.0 |
| misaki's US English G2P: [the model](https://huggingface.co/PeterReid/graphemes_to_phonemes_en_us) by PeterReid, converted to ONNX with misaki's tables by [nekwo](https://huggingface.co/nekwo/misaki-en-us-g2p-onnx) | Apache-2.0 |
| Piper's [`en_US-ljspeech-medium`](https://huggingface.co/rhasspy/piper-voices) voice | Trained on public-domain LJ Speech |
| sherpa-onnx's `lexicon-en-us.txt` | Unverified |
