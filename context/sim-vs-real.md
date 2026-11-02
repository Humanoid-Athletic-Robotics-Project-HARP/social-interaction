---
name: sim-vs-real
description: Read before trusting a sim result, especially anything ahead of the robot, or before changing the sim's lidar or camera, Nav2's sensors, approach_stop or person detection — what the sim models of the real G1's sensors, what it doesn't, and how to close each gap
metadata:
  type: project
  updated: 2026-11-02T00:51:20Z
  covers: [submission/harp/lidar.py, submission/harp/depth_camera.py, submission/harp/camera_mount.py, submission/harp/lidar_mount.py, submission/harp/lidar_relay.py, submission/harp/approach_stop.py, submission/nav2/nav2_params.yaml]
---

The sim is **not a 1:1 of the robot's sensors**: geometry, rates and the blind cone
match; posture, sensor detail and some robot-side plumbing don't. Measured on the robot
2026-11-01 ([[g1-lidar-measured]], [[g1-head-camera]], [[g1-sensor-fusion]]). Course
results with these sensors: [[nav2-sim]]. How to see ahead anyway: [[seeing-ahead-research]]. Recordings to check the sim
against: [[robot-reference-data]].

**Modelled** (`harp/lidar.py`, `harp/depth_camera.py`, both cast from one state):
- Both mounts from the rev 1.0 URDF on `torso_link`, following the torso's live tilt.
- Lidar: −7° to +52° in its frame, upside down; the blind cone as shares of rays kept
  (`BLIND`: 3% within ±30° of ahead, 15% at 30–60°; robot 0.6% / 3.1%, sim 0.9% / 3.7%);
  200,000 random rays a second over each 100 ms scan, 10 Hz, smeared by motion; 15%
  no-echo outside the cone; 2 cm range noise; the arms and shoulders in the cloud.
- Camera: 87° × 58° nominal, 47.6° down, 15 Hz, 0.2 m minimum, depth noise growing
  with distance², 15% random dropout, optical frame on `/camera/depth/points`.
- Both in Nav2's STVL costmaps; lidar clearing only over the 260° it sees (its cloud
  turned 180° on `/utlidar/cloud_rear`); `approach_stop.py` uses the nearer of both.

**Not modelled, biggest effect first** (each: effect; how to close):
1. **Posture.** The sim torso leans 1.6° forward (−1.8° to +1.8°, `runs/*_one_box_real-sensors-2`),
   the real one about 6° (not under GR00T yet): the sim camera sees the floor to about
   3.3 m, the robot's 2.7 m, so seeing ahead is optimistic. Measure the lean on the first
   GR00T run on the robot; if still about 6°, add the difference to the sim mounts.
2. **Blind cone returns.** The sim keeps 3% and 15% of its true returns ahead (`BLIND`);
   with the face visor on the robot's are all stray, at every elevation ([[g1-lidar-measured]]),
   so the sim is optimistic: `BLIND` should drop every return within about ±60° of ahead.
   With it off, `BLIND` should follow the measured visor-off map ([[g1-face-visor]]).
3. **Camera dropout and detail.** Sim: 106 × 60 rays, random holes; robot: 848 × 480,
   holes in patches (edges, dark, shiny, glass, sunlight). Thin or dark things may vanish
   on the robot but not in the sim. Denser grid; holes by surface or edge.
4. **No colour camera** in the sim: person detection (YOLO) can't be tested there; use
   recorded colour from `videohub` ([[robot-pc2]]).
5. **Lidar pattern and returns.** Uniform random rays (Livox's is uneven); every surface
   reflects (no glass, dark or far misses); no intensity, no lidar IMU, no 0.1 s delay.
6. **Unmeasured on the robot, guessed in the sim:** lidar range noise and camera depth
   noise (needs the tape-measured wall test, `testground/lidar/README.md`), the camera's
   real intrinsics (nominal 87° × 58° used on both sides).

**Robot side, done and tested:** the lidar from Unitree's `lidar_driver` through
`harp/lidar_relay.py`, which publishes `/utlidar/cloud_livox_mid360` and
`/utlidar/cloud_rear` (10 Hz each, run on PC2 outside the image, 2026-11-01); the depth
camera from `testground/camera/depth_node.py` (`/camera/depth/points`, optical frame,
15 fps). **Not done:** the depth camera can't be read inside our image (HIM's launcher passes
no devices: [[submission-image]]), so it needs `depth_node.py` running on PC2 itself
on HIM's ROS graph, which our container shares: tried 2026-11-01, 15 fps there
(`testground/camera/README.md`); our image reading it in a session isn't tried yet. Nav2 and
`approach_stop.py` don't run on the robot yet (`entrypoint.sh` starts them only in the
sim); the camera's TF on the robot is `harp/state.py`'s computed pose.
