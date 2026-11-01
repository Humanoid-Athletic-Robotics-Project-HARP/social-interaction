---
name: robot-reference-data
description: Read when you need real recordings of the G1's lidar or camera (to check the sim, tune perception offline, or once the robot is out of reach) — what exists, where, and what's still missing
metadata:
  type: project
  updated: 2026-11-01T23:41:47Z
  covers: [testground/lidar/record.py, testground/camera/record_camera.py]
---

**Where:** on the maintainer's MacBook Air in `Humanoids/robot-captures/` (beside this
repo, HIM's checkout and `models/`; not in git), copied from the worktree's ignored
`testground/*/captures/`; PC2 also keeps its originals in
`~/Workspace/testground_social_interaction/*/captures/`. Not yet on any shared drive:
one disk. About 340 MB.

**What exists** (all 2026-11-01, robot standing indoors, the room as it was, nobody placed):
- `lidar/standing-60s`: 60 s of the raw lidar stream (every point, its IMU and status)
  and `rt/lowstate` at 50 Hz. `lidar/standing-5min-headers`: 5 min of packet timing.
  `lidar/trial-20s`, `trial-5s`: recorder checks. Read with `testground/lidar/capture.py`,
  `report.py`, `view.py`. Results: [[g1-lidar-measured]].
- `camera/trial-10s`, `trial-5s`: depth (16-bit PNG in mm, 848 × 480, 15 fps), colour
  (videohub JPEGs, 1920 × 1080, 7–10 fps) and `rt/lowstate`; no analysis tool yet (any
  image tool reads them; `meta.json` has the nominal intrinsics). Format:
  `testground/camera/record_camera.py`.

**Missing** (needs the robot; the scene list is in `testground/lidar/README.md`):
tape-measured wall at 1/2/4 m (range and depth noise), a person ahead at 1–3 m and
walking past (colour and depth for person detection), boxes on the floor (blind ring),
dark, shiny, glass and thin targets, the camera's own calibration, and anything while
GR00T stands or walks (the posture gap in [[sim-vs-real]]). `record.py` reads the raw
lidar stream, so it records nothing while Unitree's `lidar_driver` runs ([[robot-pc2]]);
it needs a DDS mode, or `lidar_driver` switched off for the recording.
