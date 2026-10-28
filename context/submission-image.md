---
name: submission-image
description: Read before changing submission/ (our image for HIM's harness), running a course in HIM's sim, or debugging a run that HIM stopped
metadata:
  type: reference
  updated: 2026-10-28T05:16:30Z
  covers: [submission/**]
---

`submission/` is our code on top of him-sim2real-management ([[him-sim2real]]): one image, `harp-submission:local` (`build.sh`), plus a thin layer per world (`harp-submission:NAME`, `Dockerfile.world`).

**How it fits:** `build.sh` runs HIM's own `containers/examples/prepare.py walker` into `submission/.build/` (ignored) and builds on it, so the walker, its client and its joint profile are HIM's, unchanged. `harp_node.py` subclasses HIM's `client.PolicyNode`: the task velocity is the latest `/cmd_vel` (clipped to 0.6/0.4 m/s, 1 rad/s; zero after 0.5 s), and each state is republished by `harp/state.py` (`/odom`, `/tf`, `/joint_states`, `/imu`). Leg odometry runs on HIM's G1 model (`/harp/model`, MuJoCo 3.2.3, the newest for Python 3.8). The lidar mount is the rev 1.0 URDF's.

**Worlds** (`submission/worlds/`, criteria in `NAME.yaml`): `stage_world.py` writes HIM's scene (the G1, `safety_floor`, fixed obstacles; movers left out, or HIM would hold them still where they start) and the image's copy with movers. Worlds must be includable next to HIM's G1: defaults go in a named class (`obstacle`), never the top-level default. The comments' "--" breaks XML parsers, so `stage_world.py` strips comments.

**In a world's image** (`entrypoint.sh`): `harp/lidar.py`, Nav2 (`nav2/`) and `harp/course_goal.py`, each its own process, beside `harp_node.py`.
- The lidar must stay out of the policy's process: publishing its 190 KB cloud there made the node miss HIM's 0.2 s deadline (`action_timeout`, HIM's Holosoma caught the robot).
- HIM's sim Fast DDS profile finds only participants 0-4; with Nav2 we run about ten, and Nav2's bringup hung. `harp/fastdds_profile.py` lists peers for 32 on the same localhost transport, only when HIM's `/fastdds.xml` is set.
- Foxy Nav2 (0.4.7): no Graceful Controller (Regulated Pure Pursuit instead), `recoveries_server`, `plugin_lib_names` must be listed (the default names a library Foxy doesn't ship), `server_timeout="200"` on each BT action node.

**Course tests:** `python3 submission/course.py NAME [--tag T --repeat N --view]` runs `sim.sh --world` into `runs/` and scores from HIM's `trace.npz` (truth) and `policy.log` (our events and "odom" lines): outcome, pass, time, path, end error, closest, contacts (movers included), recoveries, drift. Container output is only in `policy.log`; nothing else leaves the container.

**Move to a point** (`harp/move_to_point.py`, the old `move_distance --to`): `G1_TASK_COMMAND='{"to": [X, Y, HEADING]}'` (also `speed`, `turn_speed`) reaches our image both in HIM's sim and through its robot launcher, and HIM's client is told it isn't a velocity. In the sim, (1, 1, 45°) ends about 7 cm and 5° off with leg odometry within 1 cm of the truth (`within_limits`).

**Viewing:** `submission/sim.sh` opens HIM's browser viewer by default (press Start; `--no-view` to skip, which `course.py` uses). On a MacBook Air the viewer's load can make HIM stop a run (`observation_timeout` with `host_stall`, or `checked_command_expired`) that passes without it: rerun with `--no-view` before blaming our code, then watch it with HIM's `./desktop_sim.sh replay RUN/trace.npz`. HIM's first viewer use downloads Three.js; on a Python.org Python that needs `SSL_CERT_FILE` from `certifi`.

**Not built yet:** `lidar_relay` in the image for the robot (needs CycloneDDS and Unitree's SDK in the image), the rest of `move_distance` (straight walks, turns).
