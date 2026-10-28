---
name: submission-image
description: Read before changing submission/ (our image for HIM's harness), running a course in HIM's sim, or debugging a run that HIM stopped
metadata:
  type: reference
  updated: 2026-10-28T07:16:01Z
  covers: [submission/**]
---

`submission/` is our code on top of him-sim2real-management ([[him-sim2real]]): one image, `harp-submission:local` (`build.sh`), plus a thin layer per world (`harp-submission:NAME`, `Dockerfile.world`).

**How it fits:** `build.sh` runs HIM's own `containers/examples/prepare.py walker` into `submission/.build/` (ignored) and builds on it, so the walker, its client and its joint profile are HIM's, unchanged. `harp_node.py` subclasses HIM's `client.PolicyNode`: the task velocity is the latest `/cmd_vel` (clipped to 0.6/0.4 m/s, 1 rad/s; zero after 0.5 s), and each state is republished by `harp/state.py` (`/odom`, `/tf`, `/joint_states`, `/imu`). Leg odometry runs on HIM's G1 model (`/harp/model`, MuJoCo 3.2.3, the newest for Python 3.8). The lidar mount is the rev 1.0 URDF's.

**Worlds** (`submission/worlds/`, criteria in `NAME.yaml`): `stage_world.py` writes HIM's scene (the G1, `safety_floor`, fixed obstacles; movers left out, or HIM would hold them still where they start) and the image's copy with movers. Worlds must be includable next to HIM's G1: defaults go in a named class (`obstacle`), never the top-level default. The comments' "--" breaks XML parsers, so `stage_world.py` strips comments.

**In a world's image** (`entrypoint.sh`): `harp/lidar.py`, Nav2 (`nav2/`) and `harp/course_goal.py`, each its own process, beside `harp_node.py`, unless a `{"to": ...}` target is given: only one thing drives `/cmd_vel`.
- The lidar must stay out of the policy's process: publishing its 190 KB cloud there made the node miss HIM's 0.2 s deadline (`action_timeout`, HIM's Holosoma caught the robot).
- HIM's sim Fast DDS profile finds only participants 0-4; with Nav2 we run about ten, and Nav2's bringup hung. `harp/fastdds_profile.py` lists peers for 32 on the same localhost transport, only when HIM's `/fastdds.xml` is set.
- Foxy Nav2 (0.4.7): no Graceful Controller (Regulated Pure Pursuit instead), `recoveries_server`, `plugin_lib_names` must be listed (the default names a library Foxy doesn't ship), `server_timeout="200"` on each BT action node. RPP's `max_allowed_time_to_collision` is 0.5 s: at 1.0 the walker's sway tripped its collision check about 6 times per `slalom`/`crossing` run.
- Course results with RPP, `--no-view`, one run each (2026-10-28, `runs/*_rpp-ttc05`): open_floor 16.8 s; one_box 20.5 s, closest 0.58 m; slalom 40.4 s, 0.37 m, 0 recoveries; obstacle_course 43.2 s, 0.29 m, 1 recovery (38.9 s, 0 at 1.0 s look-ahead); crossing 29.2 s, 0.65 m; oncoming 25.1 s, closest 0.16 m, 1 recovery. All `within_limits`, no contacts, drift under 5 cm. With the live viewer, Nav2 runs were often stopped by HIM's deadlines on a MacBook Air (`course.py` calls host-stall stops `inconclusive`).

**Course tests:** `python3 submission/course.py NAME [--tag T --repeat N --view]` runs `sim.sh --world` into `runs/` and scores from HIM's `trace.npz` (truth) and `policy.log` (our events and "odom" lines): outcome, pass, time, path, end error, closest, contacts (movers included), recoveries, drift. Container output is only in `policy.log`; nothing else leaves the container.

**Planned moves** (`harp/moves.py`, the old `move_distance`): `G1_TASK_COMMAND='{"moves": [{"walk": M}, {"turn": DEG}, {"to": [X, Y, HEADING]}]}'` or just `{"to": [...]}` (also `speed`, `turn_speed`) reaches our image both in HIM's sim and through its robot launcher, and HIM's client is told it isn't a velocity. Moves run 2 s apart, each from where the last ended. In the sim (one run, `within_limits`): walk 1 m ended 1.02 m, turn 90° ended 90.2°, curve to (1, 1, 45°) ended about 7 cm and 5° off; leg odometry within 1 cm of the truth each time.

**Viewing:** `submission/sim.sh` opens HIM's browser viewer by default (press Start; `--no-view` to skip, which `course.py` uses). On a MacBook Air the viewer's load can make HIM stop a run (`observation_timeout` with `host_stall`, or `checked_command_expired`) that passes without it: rerun with `--no-view` before blaming our code, then watch it with HIM's `./desktop_sim.sh replay RUN/trace.npz`. HIM's first viewer use downloads Three.js; on a Python.org Python that needs `SSL_CERT_FILE` from `certifi`.

**Not built yet:** `lidar_relay` in the image for the robot (needs CycloneDDS and Unitree's SDK in the image).
