---
name: seeing-ahead-research
description: Read when planning how the real robot finds people or avoids obstacles in front of it, given that the lidar is blind ahead and the camera looks down
metadata:
  type: reference
  updated: 2026-11-01T23:52:53Z
  covers: []
---

Research 2026-11-01, not yet tried. The constraints: [[sim-vs-real]].

**What sees a person** (1.7 m tall, robot standing; [[g1-lidar-measured]], [[g1-head-camera]]):
- Lidar, anywhere more than about 50–60° from the nose, to about 11 m indoors: the
  floor up to 1.3 m at 1 m, 1.47 m at 3 m, the whole body from about 6 m.
- Camera, straight ahead only: legs up to 0.79 m at 1 m, 0.33 m at 2 m, nothing
  past 2.7 m. Never a face; leaning the torso back 20° wouldn't show one either.
- Nobody published a G1 blind-cone measurement or a forward add-on camera that we
  found; Unitree's `videohub_pc4_chest` hints at a chest-camera option on some G1s.

**People: a plan that fits the sensors.** Notice and track people with the lidar
(they are off to the side or behind most of the time); approach on a curve that keeps
the person more than about 60° off the nose until the last 1.5–2 m; then the camera's
depth finds their legs ahead for the final distance and facing ("stop about 1 m
away"). Lidar options: ground removal + clustering + a Kalman tracker (simplest, no
training; e.g. louislelay/lidar_detection); a 2D slice at leg height into a leg
detector (ROS 2 ports of angusleigh/leg_tracker, or DR-SPAAM with a ROS 2 wrapper:
TakShimoda/2D_lidar_person_detection); PointPillars person models (m300-perception,
NVIDIA ros2_tao_pointpillars), which need retraining for our lidar. YOLO on the D435i's
colour only works for legs at under 2 m; full bodies and faces (for talking) need a
level, forward camera: the one hardware change worth asking the team about.

**Obstacles ahead.** Add the camera's depth to Nav2 as a second STVL source (covers
64–88% of the lidar's blind cone out to 2.5 m and the floor at the feet); STVL's
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
record with nothing else running) and whether it filters returns ahead; the D435i's
own intrinsics and whether its IMU works on the Jetson; `unitree_slam` (off: a map and
global pose for free?) and `state_estimator`'s odometry against ours; the voice
service and microphones (`vui_service`, for talking); what `videohub_pc4_chest`'s
camera is; whether Dex3 hands are fitted; the torso's lean under GR00T.

