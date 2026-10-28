---
name: run-logging
description: Read before running, recording or comparing any sim or robot test — the course command, run_recorder, runs check/compare, what a run folder holds, replay rules and gotchas
metadata:
  type: reference
  updated: 2026-10-28T03:10:38Z
  covers: [ros2/src/harp_eval/harp_eval/*.py, ros2/src/harp_eval/launch/*.py, ros2/src/harp_eval/config/logging.yaml, ros2/src/harp_control/harp_control/runlog.py, ros2/src/harp_sim/worlds/*.yaml]
---

**Record every test whose numbers matter.** Results cited in `context/` name their run folder, and that run is pinned (`runs pin RUN`). Usage and examples: README "Test runs and logs"; flags: `--help`.

**Which tool**
- Nav2 course test: `pixi run --as-is ros2 run harp_eval course WORLD --tag T [--repeat N]`. Prints a summary ending in PASS/FAIL. Each repeat restarts everything (costmaps and odometry start where the policy starts). About 1–2.5 min per run. Run it in a Terminal-panel tab; the windows open.
- Several `course` commands can run at once in different tabs: each claims its own DDS domains ([[ros2-mac]]), and run folders, `index.csv` and logs don't collide. They start up together and walk together: the lock's state goes waiting → starting → ready → walking; a run waits to start (and to summarize) while another robot walks, and `course_goal` asks the sim to release the band (`release_band` event, `run_sim --hold-band`) once Nav2 is active, no other run is still starting, and its sim has gone 5 s without lag. **Not usable on a MacBook Air (8 cores, 16 GB) as of 2026-10-27:** a start-up while another robot walked made it fall, and two start-ups together ran both sims at about 2/3 real time for 15 s, so the hanging v2 robot's walker stopped on a 504 ms gap in `rt/lowstate`; the STVL robot then walked alone with no lag. Two robots have never walked at once there. Run pairs one at a time; try parallel on a bigger machine. Those runs are tagged `*-parallel` and `*-together`.
- Anything else (driving by hand, `walk`, `move_distance`, the real robot): start `ros2 run harp_eval run_recorder --ros-args -p tag:=T` first, then the programs; Ctrl-C on the recorder ends the run.
- `runs check` (pass rate against `harp_sim/worlds/<world>.yaml`; missing values fail), `runs compare` (mean ± sd per world and tag; `--plot` draws true paths), `runs list/show/set/pin/resummarize/prune`. `runs/index.csv` has one line per run.
- `--tag` is what `runs compare` groups by, with the world: one tag per setting being compared.
- To compare Nav2 settings, give each run its file with `course ... --params FILE` (copied into the run as `nav2_params.yaml`) rather than editing the repo's `nav2_params.yaml` between runs. Alternate the settings run by run (one A, one B, check both, repeat), so a problem shows up after one pair and slow drift (heat, background load) hits both equally.
- Pass criteria were set from the 2026-10-26 baselines, loosened for spread; tighten them as the robot improves.

**How it fits together**
- Programs publish, run_recorder writes. Events are JSON `std_msgs/String` on `/harp/events` (transient-local) from run_sim, walker_node and course_goal; truth comes from `/sim/qpos` ([[mujoco-sim]]). An event missing `t`, `source` or `event`, or not JSON at all, is still recorded (filled in, or as `unparsed`), because a recorder that crashes shuts the whole run down.
- Programs without ROS (`move_distance`, `walk`) log through `harp_control/runlog.py` into the active run (`$HARP_RUN_DIR` or `runs/ACTIVE`); summarizing merges their event files.
- `course` exists because `ROS_LOG_DIR` can't be set from a launch file: it points the logs into the run folder, runs `course.launch.py`, splits `launch.log` per program and summarizes.
- `course.launch.py` starts recorder, sim (`--lidar-dds --world W --auto-release 15 --hold-band --log`), walker_node (`auto_start`), lidar_relay (the robot's lidar path, [[lidar-nav2-research]]) and course_goal; Nav2 starts on "Policy on". The run ends when course_goal or walker_node exits.

**Run folder** `runs/<UTC date>_<time>Z_<sim|robot>_<world>_<tag>/` (gitignored, per machine): `run.yaml` (settings, git state and diff, machine, conditions, manual measurements, result), `events.csv`, `summary.json`, odom/cmd_vel/truth, `lidar_stats.csv`, a small `standard_bag/` (behaviour-tree log and plans), copies of the Nav2 and logging config, per-program logs. About 1 MB per run without extras.

**summary.json** (`run_summary.py`, one function per metric): outcome, time, path, end error, odometry vs truth, contacts, closest true clearance (`mj_geomDistance`; moving obstacles placed by `truth.npz`'s `sim_t`, [[mujoco-sim]]), recoveries/replans/failures from the behaviour-tree log, achieved vs commanded speed, lidar health, pass/fail. Add a metric there and `runs resummarize --all`.

**Extras and retention** are configured in `harp_eval/config/logging.yaml` (a new topic is one entry). Keep extras off unless they answer a question: disks are small.

**Replay recording: only when a person explicitly asks**, never by default or on an agent's initiative. It records lidar, TF, odom and more (about 85 MB/min) so Nav2 can be re-run on a laptop with other params: `course ... --replay --replay-reason "why"`, then `ros2 launch harp_eval replay.launch.py run:=RUN params_file:=FILE [record:=true tag:=T]`. Its size and disk limits are in `logging.yaml`. Example pair: `2026-10-26_21-15-37Z_sim_one_box_replay-test` (pinned).

**Gotchas**
- Launch arguments can't be empty (`extras:=` errors); `course` omits unset ones.
- `launch.log` is written in bursts and uses executable names (`run_sim-2`); read the Terminal tab for live output.
- Don't give the sim a node `name=` in a launch file: it renames both its nodes.
- If Nav2 isn't active 60 s after the policy starts (a plugin failed to load), `course_goal` ends the run with outcome `error`, reason `NAV2_NOT_ACTIVE`; the cause is in `nav2.log` ([[ros2-mac]]).
- A course run can die when Nav2 and rviz2 start (sim lag → walker loses state → outcome `error`), about 1 in 6. Rerun; keep the MuJoCo window visible.
- A run with only the recorder summarizes as "stopped". So does a remote e-stop, reason `e-stop (remote, B)`; the walker's `stop` event ends the scored window when nothing else does.
