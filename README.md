# HARP Human Interaction

This project is the workspace for the Human Interaction subteam within
Stanford HARP (Human Athletics Robotics Project). In this subteam, we are
focused on creating a natural, human-like robot. Some preliminary goals
include: walking around, avoiding obstacles, detecting people, walking up to
people, remembering people, talking and listening to people. All of our code
is open source, and we would love for people to build on top of it!

The robot is a Unitree G1 EDU with 29 joints. Our code runs inside HARP's
safety harness and simulator (HIMRobotics' `him-sim2real-management`), which
is the only way it reaches the motors: it stands the robot up, stops it,
checks every joint command, and runs our code as a Docker image (ROS 2 Foxy)
on the robot and in its MuJoCo sim alike. The robot walks with NVIDIA's
GR00T policy, and we try everything in the sim first. We use the harness
unchanged, cloned next to this repo, and keep only our code here.

| Path | Description |
|---|---|
| `submission/` | The image: `build.sh` builds it, `sim.sh` runs it in HARP's sim, `course.py` runs and scores course tests. |
| `submission/harp_node.py`, `harp/` | Our node (drives HARP's GR00T walker from `/cmd_vel`, publishes leg odometry and the robot's state), planned moves, the simulated lidar and moving people, standing while someone walks at the robot, and the robot's lidar relay. |
| `submission/nav2/`, `nav2_graceful_controller/` | Nav2's settings and behavior tree, and our Foxy port of its Graceful Controller (`HARP_PATCHES.md`). |
| `submission/worlds/`, `routes/` | The obstacle courses and their pass criteria, and waypoint routes. |
| `submission/tests/` | Tests for code where a sim run wouldn't catch a mistake (how to run them: below). |
| `context/` | Notes that Claude and Codex read before working here. This is where the details behind this README live. |

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
- ✅ Sim: GR00T stands, walks, turns and stops safely (HARP's harness).
- ✅ Sim: set distances, turns and curves to a point and heading
  (`G1_TASK_COMMAND`, `harp/moves.py`), with our own leg odometry.
- ✅ Sim: driven over ROS 2, `/cmd_vel` in and `/odom` + `/tf` out (`harp_node.py`).
- ⬜ Robot: all of the above, and leg odometry checked against a tape measure.
- ⬜ Body language: facing the person, gestures.

### Get around obstacles

- Done when: it walks to a spot a few meters away and around boxes in its
  way, using the MID-360 lidar and Nav2.
- ✅ Sim: a simulated head lidar and obstacle courses (`submission/worlds/`).
- ✅ Sim: Nav2 (Graceful Controller, the same law as our curves) walks every
  course (`course.py`), including the obstacle course with a table, pillars,
  a thin pole and an archway, without touching anything. The robot's own
  body is filtered out of the lidar.
- ✅ Sim: walk a route of several points, lap after lap, with Nav2
  (`course.py --route`, `--waypoint`, `--laps`).
- 🟡 Sim: more on obstacles and path following: closer margins, a
  collision-monitor stop, moving obstacles and people. Started: the
  costmaps forget where a person was once they've moved on (STVL), and the
  robot stands while someone walks at it, going on once they've passed, and
  stands if the lidar goes quiet (`harp/approach_stop.py`), tested with a
  person crossing the robot's path (`crossing`) and one walking toward it
  (`oncoming`).
- ⬜ Sim: stand before someone crossing reaches the robot's path, not once
  they're in it: a moving-object detector (scans compared in `odom`, the
  points that moved grouped and their speed tracked).
- 🟡 Robot: real lidar data in ROS 2, then the same walk. Started:
  `harp/lidar_relay.py`, tested in the image against a recorded-style cloud.

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
  which owns stand-up, stops and the motors ("On the robot" below).
- ✅ Everything runs on the harness (`submission/`). Each line is tested in
  the sim; none has run on the robot yet.
  - ✅ Walking, stand-up, stops and safety checks: HARP's harness and its GR00T
    walker, driven by `/cmd_vel` (`harp_node.py`).
  - ✅ Leg odometry on `/odom` and `/tf`, the lidar's live transform,
    `/joint_states`, `/imu` (`harp/state.py`): within 5 cm of the truth on
    every course.
  - ✅ Walks, turns and curves to a point (`harp/moves.py`): 1 m walk ended
    1.02 m, 90° turn 90.2°, curve to (1, 1, 45°) within 7 cm and 5°.
  - ✅ The simulated head lidar and moving people (`harp/lidar.py`,
    `harp/movers.py`).
  - ✅ Nav2 with STVL and the Graceful Controller (`nav2/`, our Foxy port in
    `nav2_graceful_controller/`): all six courses succeed, no contacts.
  - ✅ Standing while someone walks at the robot, and while the lidar is quiet
    (`harp/approach_stop.py`, between Nav2 and our node): in `oncoming` it
    stands about 6 s and the person passes 0.36 m away; it never stops on the
    courses without people.
  - ✅ Course tests, their scores and criteria, comparing runs, and each run's
    git commit (`course.py`).
  - ✅ The robot's lidar into ROS 2 (`harp/lidar_relay.py`, started on the robot):
    tested in the image against a cloud published the way the robot does.
- 🟡 Stopping from the remote: HARP's harness (double Start damps, double F3
  switches to its recovery policy, D-pad Left + B hands control to us).
  Robot: not yet tried.
- ⬜ Check our robot: `mode_machine` 5, which hands it has, its ROS 2 version.
- ⬜ A gantry procedure, and a first session with someone on the rope.

## Setup

So far we've run this on Apple Silicon Macs (macOS 15) with Docker Desktop
running. On Linux, `submission/sim.sh` uses HARP's `sim.sh`, which needs
sudo; not yet tried.

Clone HARP's harness and download its recovery policy next to this repo. The
harness repo is private, so you need HARP's GitHub access.

```
harp/
├── social-interaction/                this repo
├── him-sim2real-management/            HARP's harness and simulator
└── models/fastsac_g1_29dof.onnx        its recovery policy (Holosoma)
```

To work on a branch in its own worktree, make it with git (not an app's
new-session button), inside this repo, on a branch of the same name:

```bash
git worktree add .claude/worktrees/NAME -b NAME main
```

In it, run `submission/build.sh` once before its first sim run (it builds
`harp-submission-NAME:local`). It finds HIM and `models/` beside the main
checkout, and its run folders go to the main checkout's `runs/`. Builds and sim
runs on one computer go one at a time: a second one waits for the first. Once
its branch is merged, check nothing uncommitted is left in it (`git status
--short --ignored`), then remove it with `git worktree remove
.claude/worktrees/NAME`, delete the branch, and remove its
`harp-submission-NAME` images from Docker.

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

## Run it in the sim

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

Moves can also be chained: this walks 1 m, turns 90°
left, then curves to a point. They run in order, 2 s apart, each from where
the last one ended:

```bash
G1_TASK_COMMAND='{"moves": [{"walk": 1}, {"turn": 90}, {"to": [1, 1, 45]}]}' submission/sim.sh --seconds 50
```

`"walk"` is meters (negative walks backward), `"turn"` degrees (positive
turns left). `"speed"` (m/s, up to 0.4) and `"turn_speed"` (deg/s, up to 30)
are optional, next to `"moves"` or `"to"`. Without `G1_TASK_COMMAND` the
image runs its default, `{"to": [0.5, 0.5]}` (`HARP_DEFAULT_TASK` in
`submission/Dockerfile`); `G1_TASK_COMMAND='[0,0,0]'` stands still.
The run is passed when `report.json` in the output folder (`--output DIR`)
says `within_limits`.

If your computer runs short of memory, HARP's harness can stop the run
(`host_stall`, `observation_timeout`). Close other apps and any viewer still
up from an earlier run (each holds about 700 MB until you press Ctrl-C). Or
add `--no-view --output RUN_FOLDER` and watch the run afterwards:

```bash
python3 submission/course.py --replay RUN_FOLDER
```

Course tests run Nav2 (our Foxy port of its Graceful Controller) with the
simulated lidar on one of `submission/worlds/` and score it against that
world's criteria, in a folder in `runs/` (which git ignores):

```bash
python3 submission/course.py one_box
```

Add `--view` to watch each run live at the address printed: press Start
there (after 2 minutes it starts the run anyway, but the page only shows a
run started from it). It closes the viewer 10 s after the run, before the
next one.
Before each run it warns about viewers still up and memory pressure, and the
run's `run.json` keeps the computer's memory during the run (`host_memory`).

Run folders are pruned after each run: the viewer's files go, and all but the
two newest runs of each world keep a slim `trace.npz` (about 1 MB instead of
53). `--replay RUN_FOLDER` still shows any run with its obstacles, and
`--prune` prunes every folder in `runs/` now.

The courses, from easiest to hardest, are `open_floor`, `one_box`, `slalom`
and `obstacle_course`; with people, `crossing` (someone walks across the
robot's path) and `oncoming` (someone walks toward it; the robot should stand
until they've passed). HARP's live viewer doesn't show the people (they live
only in our simulated lidar's world); `--replay` and the lidar view below do.
They're MuJoCo files,
so copy and edit them freely (how to make something move:
`submission/harp/movers.py`). This compares every scored run of a course, by
tag (`--tag` names what a run is testing):

```bash
python3 submission/course.py --compare slalom
```

Each run folder keeps our events in `events.jsonl`. This lists them in order,
with their sim time, and any second whose state from HARP's sim came late:

```bash
python3 submission/course.py --events RUN_FOLDER
```

To see what the robot sees (its lidar, where it thinks it is and where it
really is, the people, Nav2's costmap and plan, and the events), in
[Rerun](https://rerun.io), set it up once from the repo's top folder:

```bash
python3 -m venv .venv-view && .venv-view/bin/pip install rerun-sdk==0.39.0
```

Then start this before a run and open the address it prints; it shows each
sim run in a world as it happens, until Ctrl-C (`--native` for a window):

```bash
python3 submission/lidar_view.py
```

`--run RUN_FOLDER` shows a finished run instead.

To walk a route of several points instead of the world's goal:

```bash
python3 submission/course.py one_box --route rectangle --laps 3
```

The [rectangle route](submission/routes/rectangle.yaml) circles `one_box`'s
box; copy it and pass `--route FILE.yaml`, or give each point directly:

```bash
python3 submission/course.py open_floor --waypoint 2 0 --waypoint 2 2 90 --waypoint 0 0
```

Each point is `X Y [HEADING]`: meters and degrees in `odom`, from where the
robot started. The robot moves on after reaching each point, and the run
ends if a point fails or times out. The run folder keeps the route as
`route.yaml`, and the route's own `criteria` decide pass or fail.

Any other ROS 2 program in the image can drive the robot through `/cmd_vel`
and read `/odom`, `/tf`, `/joint_states` and `/imu` (`harp_node.py`). In a
course, Nav2 publishes `/cmd_vel_nav`, which `harp/approach_stop.py` passes on.

The tests in `submission/tests/` need ROS 2, so they run in the image (build
it first):

```bash
docker run --rm -v "$PWD/submission:/src:ro" -w /src --entrypoint bash harp-submission:local -c 'source /opt/ros/foxy/setup.bash && PYTHONPATH=/src:$PYTHONPATH python3 -m unittest discover tests'
```

## On the robot

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

The image also brings the head lidar into ROS 2 on
`/utlidar/cloud_livox_mid360` (`harp/lidar_relay.py`). GR00T balance takes over from Unitree's controller. Once it prints
`startup_phase: ready`, press **D-pad Left + B** to hand control to our
image. **Double Start damps the robot** at any time, and double F3 switches
to the recovery policy. Before running again, double-press Start, check it
damped, and put the robot back in locked standing mode.

## More detail

Measured results, limits and design notes are in `context/` (start with
`context/MEMORY.md`). Claude and Codex read it before they start, so you can
ask either of them to explain or summarize anything here.

## Licenses

`submission/nav2_graceful_controller/` is Nav2's Graceful Controller (Alberto
J. Tudela Roldán), under Apache-2.0 (`LICENSE`), ported to ROS 2 Foxy; the
upstream commit and our changes are in its `HARP_PATCHES.md`.

The image (built, not stored here) also contains, from HARP's harness, its
walker client and NVIDIA's GR00T-WholeBodyControl networks (NVIDIA Open Model
License, with their `LICENSE` and provenance) and its G1 model (from MuJoCo
Menagerie); Unitree's unitree_sdk2_python (BSD-3); and Eclipse CycloneDDS
(EPL-2.0).
