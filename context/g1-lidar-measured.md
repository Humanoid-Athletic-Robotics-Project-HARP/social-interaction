---
name: g1-lidar-measured
description: Read before using the real G1's head lidar (reading it into ROS 2, Nav2 on the robot, a self-filter, the mount or the sim lidar), or when you need what it really sees — measured on our robot
metadata:
  type: project
  updated: 2026-11-01T22:00:56Z
  covers: [testground/lidar/*.py]
---

Measured 2026-11-01 with `testground/lidar/` (robot standing under HARP's harness).
Baseline captures, on the maintainer's Mac in that worktree's `testground/lidar/captures/`
(ignored by git, not on GitHub): `standing-60s` (full), `standing-5min-headers`
(packet headers only), `trial-20s`. Their `report.md` has every number below.
Live 3D view on PC2: `testground/lidar/lidar_viewer.py` (port 8080; it holds the
lidar's point port, so only one of it and `record.py` at a time).

**What it is:** a Livox MID-360 (not the newer Mid360s): serial 47MCNAL0039059,
firmware app 13.18.2.44 (built 2025-04-01), at 192.168.123.120. Its own config: point
cloud Cartesian high precision, non-repetitive pattern, FOV −7° to +52°, IMU on,
install attitude all zero (it isn't told it's upside down). Core about 58.6 °C.

**How it reaches us:** it pushes Livox SDK2 UDP straight to PC2 (192.168.123.164):
points to port 56301 (2,083 packets/s of 96 points, 1380 bytes: 200,000 points/s,
2.9 MB/s), IMU to 56401 (200 Hz; gyro rad/s, accelerometer g), status to 56201
(1 Hz, the parameters above). **No DDS lidar topic exists** ([[robot-pc2]]), so
`harp/lidar_relay.py` gets nothing on this robot ([[lidar-nav2-research]]). No packets
lost over 20–300 s if the reader starts reading as soon as it opens the ports (a
reader that opens them and then spends a second on setup loses about 5% to the
kernel's 416 KB buffer). Only one program can hold the ports. Clock: time_type 0, ns
since the lidar powered on, unsynced; it runs 6.8 ppm slow against PC2's clock
(5 min), so restamp with arrival time or fit the offset. Packets arrive about 1 ms
late (99%: 4–5 ms, rarely up to 50 ms). About 43% of points are empty (0, 0, 0: no echo).

**Mount:** officially (`unitree_ros` `g1_29dof_rev_1_0.urdf`, our revision:
`mode_machine` 5) `mid360_joint` on `torso_link` at (0.0003, 0, 0.4284), roll 180°,
pitch 2.9°, as `harp/lidar_mount.py` uses (its docstring wrongly says the URDF lacks
the roll; the older `g1_29dof.urdf` does). Measured: upside down, raw +z points at the floor;
(x, y, z) → (x, −y, −z) gives z up. Standing: the floor is 1.208 m below it, and its
up direction is 8.9° from the lidar's axis (pitched forward), against the mount's
2.9° with an upright torso; the pelvis leaned 5.4° forward, which accounts for most of
it. The lidar's own accelerometer agrees with the floor within 0.8°. Its gyro reads
about 2°/s on z at rest (the pelvis IMU 0.2°/s): subtract a bias before using it.

**What it sees:** returns from −7° to +52° in its frame (about 7° above the horizon to
52° below), out to about 11 m in the lab. The floor first shows up 0.8–0.9 m out.
**Nearly blind in a cone around +x**: within ±30° of +x only 0.3% of returns, 30–60°
about 1.5% each side, every other 30° sector 8–15%; 56° of azimuth gets nothing. Very
likely the head shell. +x is the front (URDF, and the forward-facing camera fills
exactly that gap: [[g1-sensor-fusion]]), so the robot can't see what's in front of it
with the lidar; the head camera has to. About 290 returns a cloud within 0.7 m (shoulders and
arms at the sides, as near as 0.09 m). After 1 s the pattern has filled about 62% of
its 1° cells, then barely more (the blocked part never fills).

**Not yet measured:** range accuracy on a tape-measured wall,
dark, shiny, glass and thin targets, people, arm poses.
