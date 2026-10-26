---
name: run-logging
description: How to test and record runs - the course command (scripted Nav2 course tests, repeats), pass/fail (runs check), run_recorder (any session), the runs/ folder layout, summary.json metrics, runs compare, extras, replay recording (user-asked only), retention, measured sizes, and the gotchas; read this before running or comparing any sim test
metadata:
  type: reference
  updated: 2026-10-26T21:36:04Z
  covers: [ros2/src/harp_ros/harp_ros/run_recorder.py, ros2/src/harp_ros/harp_ros/run_summary.py, ros2/src/harp_ros/harp_ros/runs.py, ros2/src/harp_ros/harp_ros/course.py, ros2/src/harp_ros/harp_ros/course_goal.py, ros2/src/harp_ros/launch/course.launch.py, ros2/src/harp_ros/launch/replay.launch.py, ros2/src/harp_ros/config/logging.yaml, ros2/src/harp_control/harp_control/runlog.py, ros2/src/harp_sim/worlds/*.yaml]
---

Built 2026-10-26 (a teammate's design; additions the same day from a side session: light Nav2 data always recorded, frame alignment, pass criteria, speed and lidar metrics, run conditions, replay). It replaced hand-started test runs, tee'd logs and throwaway analysis scripts. **Use it for every test whose numbers matter.** Results cited in `context/` should name their run folder, and that run should be pinned.

**Course tests (Nav2, sim):** from `ros2/`, `pixi run --as-is ros2 run harp_ros course WORLD [--tag T] [--note N] [--repeat N] [--extras nav,lidar --extras-reason R] [--goal X Y [YAW]] [--timeout S] [--no-rviz] [--tether T --floor-surface F --battery-percent B --people-nearby P --lighting L]`. Each run prints its summary ending in PASS or FAIL (and which criteria failed).
- WORLD is a course in `harp_sim/worlds/`: `open_floor`, `one_box`, `slalom`, `obstacle_course`. Its goal site is the goal, sent in `odom`.
- Each repeat restarts everything, because costmaps and leg odometry start where the policy starts.
- The MuJoCo and rviz2 windows open, so the team can watch. Run it in a Terminal-panel tab.
- One run takes about 1 min (`one_box`) to 2.5 min (`slalom`). Ctrl-C stops the run and any remaining repeats.
- `--tag` names what's being tested (`rangecut`, `inflation1.0`), and `runs compare` groups by world and tag.

**Pass/fail:** `ros2 run harp_ros runs check [--world W] [--tag T]` shows the pass rate per course and tag, and which criteria failed in which runs.
- Criteria live next to each world: `harp_sim/worlds/<world>.yaml`, with `must_succeed`, `max_time_s`, `min_clearance_m`, `max_contacts`, `max_recoveries` and `max_end_error_m`. They're set from the 2026-10-26 baselines, loosened for run-to-run spread; tighten them when the robot improves.
- They apply to course runs (a goal was sent). A missing value (e.g. no clearance) counts as a fail.

**Comparing:** `ros2 run harp_ros runs compare --world slalom [--tag a --tag b] [--plot paths.png]` gives n, successes, and mean ± sd of time, path, closest clearance and recoveries, plus total contacts. The plot shows the true paths over the obstacles.
- Other subcommands: `runs list` (with a pass column), `runs show RUN`, `runs resummarize RUN|--all` (after adding a metric; old runs gain it), `runs set RUN key=value ...` (conditions such as `tether=gantry`, `manual.end_position_measured="2.95 0.10"`, `note=...`; it re-summarizes), `runs pin RUN` / `unpin`, `runs prune [--dry-run]`.
- `runs/index.csv` has one line per run, for grep or a spreadsheet.

**Any other session** (driving by hand, `walk`, `move_distance`, later the real robot): start `ros2 run harp_ros run_recorder --ros-args -p tag:=T -p note:="..." [-p world:=W] [-p kind:=robot] [-p tether:=gantry -p floor_surface:=carpet -p battery_percent:=80 -p people_nearby:=2 -p lighting:=office] [-p end_position_measured:="2.95 0.10" -p video:=URL] [-p extras:=nav -p extras_reason:="..."]` *first*, then the programs. Ctrl-C on the recorder ends the run and prints the summary.

**How it fits together:**
- **Programs publish, run_recorder writes.**
  - Events (JSON `std_msgs/String`, transient-local, depth 1000) go on `/harp/events`, from run_sim, walker_node and course_goal.
  - run_sim publishes the true pose `/sim/qpos` (`[sim_t, qpos…]`, 20 Hz) and `/sim/info` (latched JSON: scene, world, boxes, goal, flags). It always starts ROS now; `--record` is gone.
  - walker_node publishes `/joint_states`, `/imu` and `/walker/policy` (obs 86 + action 15 + cmd 3), next to `/odom` and `/tf`.
- **Programs without ROS** (`move_distance`, `walk`) log through `harp_control/runlog.py`: `runlog.event(...)` and `runlog.sample(stream, ...)`. They write `events-<program>.csv` and `<stream>.csv` into the active run folder: `$HARP_RUN_DIR`, or the run named in `runs/ACTIVE` (name and recorder pid, written while a recorder runs). Summarizing merges those event files into `events.csv`.
  - `move_distance` logs start, `move_start`, `move_done` (target vs estimate), timeouts, wrong-way, errors, and leg odometry plus command at 10 Hz (`leg_odom.csv`; the summary uses it when there is no `/odom`).
  - The walker logs stand_pose, policy_on, fall, state_lost and damped. With ROS attached, the same calls publish on `/harp/events` instead.
- **`course`** (`harp_ros/course.py`):
  - Makes the run folder and sets `ROS_LOG_DIR` into it, so `ros_logs/*/launch.log` holds every program's printed output.
  - Runs `course.launch.py`, splits `launch.log` into `sim.log`, `walker.log`, `nav2.log`, `goal.log`, `recorder.log`, `rviz.log`, then summarizes.
  - It has to exist: `ROS_LOG_DIR` can't be changed from inside a launch file, and `ros2 launch` has no log-dir option.
- **`course.launch.py`:**
  - Starts the recorder, the sim (`--lidar --world W --auto-release 15 --log`), walker_node (`auto_start:=true`) and course_goal together.
  - Nav2 (with rviz2) starts when walker_node prints "Policy on".
  - The run ends, and everything shuts down, when course_goal exits (result or timeout) or walker_node exits (a fall).
  - `--auto-release 15` gives walker_node time (about 8 s) to reach its stand pose. walker_node's `auto_start` starts the policy on the sim's `feet_down` event, which removes the start-timing mistakes of hand-run tests ([[nav2-sim]]).
- **The recorder** also keeps, on every run: a small `standard_bag/` (MCAP, zstd) with every `/behavior_tree_log` message and `/plan` once per 5 s, and `lidar_stats.csv` (per scan, up to 10 Hz: time, points, min and median range; no points stored). If the recorder exits (e.g. refusing a replay), course.launch.py shuts the run down.
- **`course_goal`** waits for `/sim/info` and the walker's `policy_on`, then for bt_navigator to be active. It sends the goal (event `goal_sent`), reads Nav2's feedback, and on finishing publishes `goal_result` (status, recoveries, seconds, remaining). Then it waits 3 s so the robot comes to rest on record.

**Run folder** `runs/<UTC date>_<UTC time>Z_<sim|robot>_<world>_<tag>/`, e.g. `2026-10-26_21-09-00Z_sim_one_box_v2` (all run times are UTC; the runs from 2026-10-26 were renamed from Pacific time) (all of `runs/` is gitignored, kept per machine, a teammate's call on 2026-10-26):
- `run.yaml` (schema 2) holds:
  - tag, note, world, pinned, extras and their reason, replay (reason, path, options), the command
  - who (user, git user), machine, git (commit, branch, dirty, `git.diff` up to 1 MB), `/sim/info`
  - `conditions`: floor_surface, tether, battery_percent, people_nearby and lighting. The sim fills in "sim", "band until released" and "0".
  - `manual`: end_position_measured as x y in odom (robot runs, tape measure), and video
  - the result
- The rest of the folder:
  - `events.csv` (t, source, event, details JSON) and `summary.json`
  - `odom.csv` (10 Hz), `cmd_vel.csv` (10 Hz), `truth.npz` (sim, 20 Hz: t, sim_t, qpos), `lidar_stats.csv`, `standard_bag/`
  - copies of `nav2_params.yaml` (the file Nav2 used) and `logging.yaml`
  - the per-program logs and `ros_logs/`
  - only when asked for: `bag/` (extras) and `replay/`
- Measured sizes: see below.

**summary.json** (`harp_ros/run_summary.py`; each metric is a small function):
- outcome: succeeded / failed / timed_out / fell / error / stopped, with the reason
- duration from goal sent to result (or the moves); path length (1 s smoothed true pelvis); mean speed
- true end position, goal in world coordinates, end error
- odometry against the truth (end and largest), after mapping odom into the world with the true pelvis pose (position and yaw) at `policy_on`. `frame_alignment` says how. The rotation measured about 0°: leg odometry takes yaw from the world-referenced IMU. Robot runs instead compare odom's end against `manual.end_position_measured` (`odom_error_vs_measured_m`).
- contacts (count and which body touched which obstacle)
- closest true clearance, overall and per obstacle (`mj_geomDistance` from the robot's collision geoms at 10 Hz), and the share of time within 0.3 m
- from the behaviour-tree log (`nav_source: behavior_tree_log`):
  - `recoveries` and `recovery_types` (Wait or Spin starting)
  - `replans` (ComputePathToPose successes)
  - `planner_failures` and `controller_failures`
  - Nav2's own feedback count is kept as `recoveries_feedback`. Runs without the bag fall back to it.
- speed tracking: achieved over commanded speed, forward and turning, while moving, 0.3 s after the command (mean, sd, n). It's computed from `/odom` (sim and robot) and from the true pose (sim). Measured about 0.82–0.85 forward, matching GR00T walking at about 85% of the command.
- lidar quality: points per scan (mean, min), scan rate, gaps over 0.3 s
- pass/fail: `check` (per criterion: limit, value, pass) and `passed`
- the share of time `/cmd_vel` was stopped; sim lag warnings and the largest lag; per-move target and estimate for move_distance

**Extras and retention** live in `harp_ros/config/logging.yaml`. A new topic is one entry there.
- Extras: `joints`, `policy`, `tf`, `nav` (local plan, costmaps; the global plan and the tree log are in the standard bag), `lidar` (1 Hz).
- Retention, applied whenever a recorder starts:
  - runs older than the last 100 keep only `run.yaml`, `events.csv` and `summary.json`
  - `bag/` is kept for the last 10 runs, up to 2 GB in total
  - each run stops recording extras at 200 MB
  - pinned runs are never touched
- One teammate's Mac had 11–22 GB free (2026-10-26), so keep extras off unless they answer a question.

**Replay recording: only when a person explicitly asks for it.** Never by default, never in an extras set, never started by an agent on its own initiative.
- What it's for: record a run once (above all on the robot), then play its lidar, `/tf` and `/odom` back into Nav2 on a laptop, to tune costmaps and the controller without the robot.
- How: `course WORLD --replay --replay-reason "why" [--replay-dir DIR] [--lidar-fields xyz] [--lidar-rate 5]`, or run_recorder `replay:=true replay_reason:=...`. Both refuse without a reason, and the reason goes in run.yaml.
- What it records into `replay/` (MCAP, zstd): lidar at 10 Hz, `/tf`, `/tf_static`, `/odom`, `/joint_states`, `/imu` and `/cmd_vel`. It can go elsewhere with `--replay-dir` or `$HARP_REPLAY_DIR`, e.g. an external or shared drive.
- Its own limits (logging.yaml `replay:`):
  - 2 GB per run
  - 10 GB in its own retention pool (oldest unpinned removed first)
  - it refuses to start if the disk would be left under 5 GB free counting the 2 GB cap, and stops if free space falls under 5 GB
- Play it back: `ros2 launch harp_ros replay.launch.py run:=RUN [params_file:=FILE] [record:=true tag:=T] [rate:=1.0] [rviz:=true]`.
  - It runs `ros2 bag play --clock` (the recorded `/cmd_vel` becomes `/recorded/cmd_vel`), and 4 s later nav2.launch.py with `use_sim_time` and that params_file. Nav2 aborts its startup if `odom` → `base_link` is missing for about 3.5 s, so it has to start after the recording does.
  - Nav2 builds its costmaps from the recorded lidar.
  - `record:=true` records the replay as a run (kind replay, with Nav2's costmaps in `bag/`), so two params files can be compared on the same data.

**Measured sizes** (2026-10-26, sim, course runs of 25–60 s, `du`):
- A whole run folder without extras is 0.7–1.0 MB:
  - `truth.npz` 340–440 KB
  - `odom.csv` + `cmd_vel.csv` 90–170 KB
  - printed output and ROS logs about 110 KB
  - `standard_bag/` 48–132 KB (the estimate was about 1 MB; zstd compresses the plans and tree log well)
  - `lidar_stats.csv` 24–28 KB, about 30 KB/min, at 8.8 scans/s
- `nav` extras on slalom: about 0.5 MB.
- Replay on `one_box` (about 1 min of recording, full lidar at about 8.7 Hz received): 81 MB, so about 85 MB/min in the sim. The real MID-360's point format (point_step) is still unknown; check it on the robot before estimating.
- About 100 course runs fit in 100 MB.

**Replay test (2026-10-26):** `2026-10-26_21-15-37Z_sim_one_box_replay-test` (pinned, 81 MB replay) played into Nav2 twice with record:=true: runs `..._21-18-17Z_replay_noworld_replay-default` and `..._21-19-27Z_replay_noworld_replay-inflation1`. Both built costmaps from the recorded lidar: 122–140 lethal cells (the box) in each, and inflated cells 988 → 1,648 with inflation 1.0/2. Compare such pairs by reading the last `/global_costmap/costmap` in each replay run's `bag/` (`run_summary.read_bag`).

**Gotchas:**
- Run everything with `pixi run --as-is` ([[ros2-mac]]).
- Launch arguments can't be empty (`extras:=` is an error). `course` leaves out unset ones.
- `launch.log` is written in bursts, so a tail of it lags behind. Read the Terminal tab or wait for the run to end.
- In `launch.log`, process names are executable names (`run_sim-2`, `walker_node-3`), not node names.
- Don't give the sim a node `name=` in a launch file: the remap renames both of its nodes (sim and sim_lidar) and ROS warns.
- A run that only starts the recorder (no goal, no moves) summarizes as "stopped".
- A course run can die when Nav2 and rviz2 start: the sim falls behind real time (lag events of 100–600 ms), walker_node loses robot state for over 0.5 s and stops. It shows as outcome `error`, "Robot state (rt/lowstate) stopped". That happened in 1 of 6 runs on 2026-10-26. Rerun it; keep the MuJoCo window visible ([[mujoco-sim]]).
- `runs check` fails course runs that ended before their goal (missing values count as fails). `runs compare` counts them (`err`) but leaves them out of the means.
- The recorder uses about 3% CPU, so it isn't a cause of lag.
- `runs compare` groups by world and tag only. Keep one tag per setting being compared.
