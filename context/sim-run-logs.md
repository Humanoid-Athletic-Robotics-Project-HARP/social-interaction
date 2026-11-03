---
name: sim-run-logs
description: Read before changing what our image prints, how course.py reads a run's events, the Rerun lidar view (harp/view_feed.py, lidar_view.py), or how replays show the people
metadata:
  type: reference
  updated: 2026-11-02T22:04:05Z
  covers: [submission/harp/events.py, submission/harp/view_feed.py, submission/lidar_view.py, submission/course.py, submission/entrypoint.sh]
---

**Only the container's output leaves it** (HIM's sim: `network_mode: none`). Live it is `docker logs -f g1-desktop-sim-submission-1`; after the run HIM's launcher saves it as `policy.log` (`compose logs`, each line prefixed `submission-1  | `). Docker and compose keep lines of 40 KB whole (tested 2026-10-30), so a frame is one line.

**Events** (`harp/events.py`): every program prints JSON lines starting `source`, `event`, `wall` (Unix s, the VM's clock), and `sim_time` only from programs that read HIM's state (the rest get it estimated from `wall` by `course.py --events`, and when scoring the behavior loop's talks, from the last `odom` line before them: [[behavior-loop]]). JSON on `std_msgs/String`, no custom messages ([[feedback-minimal-setup]]).
- **`events.jsonl`, one per run:** `course.py` copies our lines out of `policy.log` (at pruning, so after every `--output` run, or when first read); scoring reads only it. JSON Lines because a stopped run still leaves every earlier line readable, the same line goes live to the viewer and into the file, and it needs only the standard library (`jq`, `grep` work). Old runs get theirs made from `policy.log` the same way.
- `harp_node.py`'s `late_state` (each state 40 ms or more old) and the seconds `run.json`'s `host_memory.seconds` shows 50 MB or more decompressed line up in `course.py --events`: [[sim-deadline-stops]].
- `source: "view"` lines are frames for the viewer: they go to `view.jsonl` and out of `policy.log`.

**Lidar view:** `harp/view_feed.py`, its own process at `nice 19` in a world's image, prints the world's fixed obstacles once, then a frame every 0.25 s: truth pose, leg odometry, `base_link`→`livox_frame` from `/tf`, the cloud thinned to one point per 10 cm cube (at most 3,000, int16 cm, zlib, base64), the people (`harp/movers.py` from `mover_start` events, like the scoring), Nav2's `/plan`, and the local costmap each second. `submission/lidar_view.py` (Mac) reads that stream into Rerun (the maintainer chose Rerun; `rerun-sdk` from `testground/requirements.txt`, in `testground/.venv`, so the team has one Rerun), one recording per container, or a finished run's files with `--run`. It draws odom as if it were the world, so the two robots' gap is the drift.
- **Cost** (`crossing`, `runs/2026-10-30_22-29-18Z_sim_crossing_sim-view-feed`, Rerun following live, Mac 63% free): about 19 KB a frame, `view.jsonl` 5.3 MB for a 68 s run (kept in every run; pruning leaves it). HIM's states were at most 4 ms old all run (`timing` lines): the feed doesn't make them late.

**Replays with people** (`course.py --replay`): HIM's replay poses only `qpos`, so the replay swaps in `stage_world.py`'s `NAME_replay.xml` (each mover a free body that touches nothing; walls and blocks see-through, the goal a green disc and pole) and adds each mover's position to every frame: from `mover_start` events, or for an aware crowd from the lidar's `crowd` and `movers` events (`sim_time`, xy, each 100 ms scan; `logged_path` still reads older runs' `t`). `overview_camera` rewrites HIM's fixed starting camera in our temporary copy of the page. Live, `lidar_view.py` draws an aware crowd from `/harp/movers` (their true positions each scan).
