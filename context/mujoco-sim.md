---
name: mujoco-sim
description: Read before running the sim (always with the viewer and rviz2 in windows) or changing run_sim.py, its lidar, boxes, courses or the lidar mount
metadata:
  type: reference
  updated: 2026-10-26T22:40:10Z
  covers: [ros2/src/harp_sim/harp_sim/run_sim.py, ros2/src/harp_sim/harp_sim/lidar.py, ros2/src/harp_control/harp_control/lidar_mount.py]
---

**Always run it with the viewer**, never `--headless`, so the teammate can watch; likewise open rviz2 in a window and leave it up when testing anything it can show. Run long-lived windows in Terminal-panel tabs so they can be watched and stopped. `screencapture` from the agent's shell captures only the desktop background, so it can't check windows.

**What it is:** `harp_sim/run_sim.py` loads Unitree's `scene_29dof.xml` (`$HARP_G1_SCENE` or `--scene`) and speaks the robot's low-level topics on DDS domain 1, loopback: reads `rt/lowcmd`, publishes `rt/lowstate` every 5 ms step. Elastic band on keys 7 (raise), 8 (lower), 9 (toggle) — Unitree's README has 7 and 8 reversed. `--auto-release S` releases the band for scripted tests. Only our own controller walks in it (no `sport` service).
- **Why not unitree_mujoco's runner:** it computes PD torque once per received command on the DDS thread, which blew up the physics on landing with GR00T; ours recomputes PD every step from the latest command, like the motor drivers. Its sleep pacing also ran at about 0.8× real time on the Mac; ours paces against an absolute schedule.

**ROS 2 side** (domain 2, same process): true pose on `/sim/qpos` (20 Hz), latched `/sim/info` (JSON: scene, world, boxes, goal, flags), events on `/harp/events` (feet_down, band_released, band_attached, contact, lag, quit) for [[run-logging]].

**Boxes and courses:** `--box X Y [W D H]` adds fixed boxes through `MjSpec`. `--world NAME` attaches `harp_sim/worlds/NAME.xml` (names prefixed `course/`); its worldbody geoms are obstacles and a site `goal` is the goal. Each new robot–obstacle contact (pairs debounced over 1 s) is printed and logged. Courses and their criteria: [[nav2-sim]], [[run-logging]].

**Lidar (`--lidar`, `lidar.py`):** every 0.1 s of sim time a thread runs `mj_multiRay` on its own `MjData` (it releases the GIL, so the 5 ms physics loop is unaffected; about 9 ms per cast). 480 × 40 rays over 360° × −7° to +52°, 2 cm noise, 5% dropout, 0.1–40 m. Publishes xyz PointCloud2 on `/utlidar/cloud_livox_mid360` in `livox_frame` (upside down, as on the G1).
- Only `torso_link` (the head shell) is skipped. Hits on the rest of the body stay in the cloud on purpose, as on the robot, so the Nav2 self-filter is tested: standing, about 640 points 0.11–0.56 m from the sensor. Shoulders and wrists also block some low rays at the sides.
- The mount is in `harp_control/lidar_mount.py`, shared with `walker_node`: `g1_29dof.urdf`'s `mid360_joint` plus roll π. Our MJCF matches that older URDF, not `g1_29dof_rev_1_0.urdf` ([[lidar-nav2-research]]).
- rviz2 (`harp_sim/rviz/lidar.rviz`) needs `walker_node` for `base_link` → `livox_frame`. While the robot hangs from the band the lidar is about 0.5 m higher than the upright guess, so lower it (`8`) before judging the floor.
- In the lidar frame the floor isn't level (torso lean), so height cuts need TF, not raw lidar z.

**Quitting:** Ctrl-C, `--duration` or closing the window stops the viewer, shuts down ROS 2, then `os._exit(0)`. Normal interpreter exit segfaulted in about 1 in 4 full course runs (a DDS thread running after its library unloaded), so keep the `os._exit`.

**Mac:** loopback is `lo0` (picked automatically). The viewer needs `mjpython`, so `run_sim` re-execs itself under it unless `--headless` ([[ros2-mac]]). macOS throttles hidden windows: keep the MuJoCo window visible, since a stall over 0.5 s stops the controller ("Robot state stopped").
