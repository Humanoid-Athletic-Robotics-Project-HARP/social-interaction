---
name: robot-reference-data
description: Read when you need real recordings of the G1's lidar or camera (to check the sim, tune perception offline, or once the robot is out of reach) — what exists, where, and what's still missing
metadata:
  type: project
  updated: 2026-11-01T23:45:05Z
  covers: [testground/lidar/record.py, testground/camera/record_camera.py]
---

**Where:** on the maintainer's MacBook Air in `Humanoids/robot-captures/` (beside this
repo, HIM's checkout and `models/`; not in git), copied from the worktree's ignored
`testground/*/captures/`; PC2 also keeps its originals in
`~/Workspace/testground_social_interaction/*/captures/`. Not yet on any shared drive:
one disk. About 410 MB.

**What exists** (all 2026-11-01, robot standing indoors, the room as it was, nobody placed):
- `lidar/standing-60s`: 60 s of the raw lidar stream (every point, its IMU and status)
  and `rt/lowstate` at 50 Hz. `lidar/standing-5min-headers`: 5 min of packet timing.
  `lidar/trial-20s`, `trial-5s`: recorder checks. `lidar/dds-trial-20s`: 20 s from Unitree's
  `lidar_driver` over DDS (`record.py --source dds`), its IMU and `rt/lowstate`. Read with `testground/lidar/capture.py`,
  `report.py`, `view.py`. Results: [[g1-lidar-measured]].
- `camera/trial-10s`, `trial-5s`: depth (16-bit PNG in mm, 848 × 480, 15 fps), colour
  (videohub JPEGs, 1920 × 1080, 7–10 fps) and `rt/lowstate`; no analysis tool yet (any
  image tool reads them; `meta.json` has the nominal intrinsics). Format:
  `testground/camera/record_camera.py`.

**Missing** (needs the robot; the scene list is in `testground/lidar/README.md`):
tape-measured wall at 1/2/4 m (range and depth noise), a person ahead at 1–3 m and
walking past (colour and depth for person detection), boxes on the floor (blind ring),
dark, shiny, glass and thin targets, the camera's own calibration, and anything while
GR00T stands or walks (the posture gap in [[sim-vs-real]]). With `lidar_driver` on (as now, [[robot-pc2]]), record the lidar with
`record.py --source dds`; the raw stream (`--source udp`) needs it off.
