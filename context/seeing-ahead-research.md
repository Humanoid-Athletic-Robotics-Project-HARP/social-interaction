---
name: seeing-ahead-research
description: Read when planning how the real robot finds people or avoids obstacles in front of it, given the lidar's blind areas and the downward camera
metadata:
  type: reference
  updated: 2026-11-02T22:04:05Z
  covers: []
---

Research 2026-11-01, not yet tried. The constraints: [[sim-vs-real]].

**What sees a person** (1.7 m tall, robot standing; [[g1-lidar-measured]], [[g1-head-camera]], [[g1-face-visor]]):
- Lidar, to about 11 m indoors: with the face visor off, ahead (±38–40°) and past about 56° to the sides, not in the wedges between; with it on, only past about 60°.
- Camera, straight ahead only: legs up to about 0.78 m at 1 m, 0.32 m at 2 m, nothing past 2.7 m. Never a face; leaning the torso back 20° wouldn't show one either.
- Nobody published a G1 blind-cone measurement or a forward add-on camera that we found (2026-11-01).

**People: design for the visor off** (the maintainer's call, 2026-11-02). The lidar finds person candidates by their shape and tracks them; the robot walks toward one until the camera confirms them by their legs within about 2 m, then on to about 1 m ([[behavior-loop]]). Lidar options: ground removal + clustering + a Kalman tracker (simplest, no training; e.g. louislelay/lidar_detection); a 2D slice at leg height into a leg detector (ROS 2 ports of angusleigh/leg_tracker, or DR-SPAAM via TakShimoda/2D_lidar_person_detection); PointPillars person models (m300-perception, NVIDIA ros2_tao_pointpillars), which need retraining for our lidar. Camera: a leg detector, or YOLO on the colour image, which only works for legs within 2 m; full bodies and faces (for talking) would need a level, forward camera, a hardware change to ask the team about. Pinned detector sources: [[person-perception-sources]].

**Obstacles ahead.** Add the camera's depth to Nav2 as a second STVL source (covers
about 64–69% of the lidar's blind cone out to 2.5 m, measured ([[g1-sensor-fusion]]), and the floor at the feet); STVL's
memory keeps what the lidar saw before it came in front. Still unseen: past 2.7 m
ahead and anything high ahead (a table top above 0.8 m at 1 m). Ways round: walk
slowly forward, turn to look before going straight into unseen space, and keep
marked obstacles longer.

**Also possible:** a small forward 2D lidar at waist or chest height (far ahead, no
camera tilt problem); a brief torso tilt back while standing, if GR00T or HIM allow it
(a look up, not while walking); a speed cap so the robot can stop within the
camera's 2.7 m.

**Open questions on the robot** (2026-11-02; what's already known: [[robot-pc2]],
[[g1-official-docs]]): why `lidar_driver` loses clouds over DDS (load, or itself:
record with nothing else running); the D435i's
own intrinsics and whether its IMU works on the Jetson; `unitree_slam` (off: a map and
global pose for free?) and `state_estimator`'s odometry against ours; the voice
service and microphones (`vui_service`, for talking); whether Dex3 hands are fitted; the torso's lean under GR00T.

