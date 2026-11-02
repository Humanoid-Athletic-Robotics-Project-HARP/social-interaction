# G1 MID-360 lidar: findings

Measured on 2026-11-02 on the G1 EDU (29 DoF) Jetson, `unitree@192.168.123.164`.

## TL;DR

- The Livox MID-360 at **192.168.123.120** streams by itself to this Jetson. That's about **200k points/s** plus **200 Hz IMU**. No SDK handshake is needed to receive it.
- It is **mounted upside down**. To convert raw points to x-forward, y-left, z-up, use **`(x, y, z) -> (x, -y, -z)`** (180° about x).
- Sensor height above the floor: **1.21 m standing**, **1.29 m hanging** on the gantry. Tilt is **~9° standing** and **~7° hanging**.
- There is a **persistent blind wedge of roughly ±45° around the sensor's +x axis**, which is very likely the robot's own head. If +x is the robot's front (likely, **not yet confirmed**), the lidar barely sees what is directly ahead.
- **Only one process can receive the stream.** `lidar_viewer.py` from this folder is currently running in a Claude session and holds UDP 56301. Stop it before starting `livox_ros_driver2` or any SDK2 program.

## Network

| Host | IP | Notes |
|---|---|---|
| Jetson (this machine) | `eth0` 192.168.123.164, `wlan0` on the lab Wi-Fi (address in the main checkout's `.env`) | lidar host IP |
| MID-360 | 192.168.123.120 (MAC 8c:58:23:b7:41:0b) | pings, streams |
| Robot | 192.168.123.161 | reachable (likely the motion-control computer) |
| Other | 192.168.123.200 | reachable; another `unitree` login came from here |

## Streams (host side, per `~/Workspace/test/mid360_config.json`)

| Stream | Host port | Lidar port | Rate seen |
|---|---|---|---|
| Point cloud | 56301 | 56300 | ~2,080 packets/s × 96 pts ≈ 200k pts/s |
| IMU | 56401 | 56400 | 200 Hz |
| Push / status | 56201 | 56200 | ~1 Hz |
| Command | 56101 | 56100 | not used |

### Point packet format (SDK2, as received)

- **Size:** 1380 bytes, made of a 36-byte header plus 96 points × 14 bytes.
- **Header fields:**
  - `dot_num`: uint16 at byte offset 5.
  - `data_type`: uint8 at offset 10. It is **1**, which means Cartesian high precision.
- **Each point:** `int32 x, y, z` in millimetres, then `uint8 reflectivity` and `uint8 tag`.
- **Zero returns:** points at (0,0,0) mean no echo; drop them.
- **Decoding:** works with plain numpy, see `PointBuffer` / `receive()` in `lidar_viewer.py`.

## Mounting and geometry (measured from live data)

**Upside down.** A RANSAC floor fit on raw points finds the floor at **+z**: the normal toward the sensor is (-0.095, -0.064, -0.993) in the raw frame. Apply `(x, -y, -z)` before anything else.

> The URDF `mid360_link` frame does *not* include this flip, so it is not the raw sensor frame. See `~/holosoma/src/holosoma_retargeting/holosoma_retargeting/models/g1/g1_29dof.urdf`: `mid360_joint` sits on `torso_link` at xyz (0.0003, 0.00003, 0.406) with pitch 0.040 rad.

**Height and tilt** (floor plane in the flipped frame):

| State | Lidar above floor | Floor normal (flipped frame) | Tilt |
|---|---|---|---|
| Hanging on gantry | 1.286 m | (-0.095, 0.064, 0.993) | 6.6° |
| Standing on floor | 1.21 m | (-0.154, 0.041, 0.987) | 9.2° (≈8.9° pitch, ≈2.4° roll) |

- **Compared with the URDF:**
  - With straight legs the URDF predicts the sensor at ~1.253 m above the feet: pelvis 0.793 + waist 0.054 + mid360 0.406.
  - Standing measures ~4 cm lower, which is consistent with bent knees.
  - Hanging measures ~3 cm higher, so the feet were off the floor.
- **Direction of the tilt:** if +x is the front, the sensor pitches nose-down. That matches the sign of the URDF's +0.04 rad pitch plus some forward torso lean.
- **For your own transform:** don't hard-code it. Fit the floor live, or take height and tilt from the robot state.

### Which way is forward: UNCONFIRMED

Evidence that sensor +x = robot front:

- The URDF `mid360_link` has x forward.
- The tilt is nose-down, as described above.
- A thin occluder (probably the gantry) showed up at −x.

To confirm it, have someone stand ~2 m directly in front of the robot and check where their points appear. If the answer is −x, the conversion is `(x, y, z) -> (-x, y, -z)` (180° about y) instead.

### Blind wedge around +x

Points per 30° azimuth sector, in the flipped frame:

- The sectors from -30° to +30° hold only **~500–700 points each**.
- The sectors from ±30° to ±60° hold **~3.5k each**.
- Every other sector holds **15k–25k**.

The wedge stayed in the same place after the robot moved from the gantry to the floor, while the surroundings changed. So it's self-occlusion, most likely the head shell. **If +x is the front, the lidar is nearly blind straight ahead, so use the head camera to see people in front of the robot.**

### Self-hits

- About 3.8k points per ~1.5 s fall within 0.6 m of the sensor, directly below it (|x| < 0.05 m). These are the head and shoulders.
- Filter them out: in the flipped frame, drop points with `x² + y² < 0.1` and `-1.35 < z < 0.12`, or simply drop points with `r < 0.6`.

### Vertical field of view

This part comes from the Livox spec, not from a measurement here. The MID-360 covers -7° to +52° vertically. Inverted, that becomes about +7° above the horizon down to -52° below it, so the sensor mostly looks down. With the sensor at ~1.2 m, floor returns should start ~1 m out, leaving a ~1 m blind disk on the floor around the robot.

## Software on the Jetson

- **Livox SDK2:** installed at `/usr/local/lib/liblivox_lidar_sdk_shared.so` and `/usr/local/include/livox_lidar_*.h`.
- **ROS:** Noetic and Foxy are both in `/opt/ros`, with `rviz` and `rviz2` available.
- **ROS driver:** `livox_ros_driver2` ran here before (logs in `~/.ros/log`, Nov 2023), but **no source or built workspace was found**.
- **Python:** 3.8.10 with numpy 1.24.4, OpenCV 5.0.0 and matplotlib 3.1.2. Open3d and Python Livox bindings are not installed.
- **`~/Workspace/test/g1_camera_mosaic.py`:** existing script that reads the head camera over DDS `videohub` and the lidar over SDK2 via ctypes. Today the `videohub` RPC returned error **3102**: the camera service did not answer.
- **Not available:** sudo (needs a password) and `tcpdump`.

## Viewer in this folder: `lidar_viewer.py`

```bash
python3 lidar_viewer.py     # then open http://ADDRESS:8080 from the Wi-Fi network (PC2's Wi-Fi address)
```

- **How it works:** reads UDP 56301 directly, applies the flip, and fits the floor every second with RANSAC.
- **What the page shows:** a three.js view with a mock G1, distance rings, front/back/left/right labels, click-to-measure, and a "Swap front/back" toggle.
- **`/points`:** float32 Nx4 array (x, y, z, reflectivity) in the flipped sensor frame, covering the last 0.3 s.
- **`/calib`:** JSON with `normal`, `height`, `ok`, `inliers` and `pps`. A sample reading while standing gave a height of 1.209.

## Open items

1. Confirm the front direction (see the test above).
2. Get the head camera answering, since `videohub` currently returns 3102.
3. Rebuild `livox_ros_driver2` if ROS topics are needed.
4. Open the viewer page in a real browser. So far only its HTTP endpoints have been checked.
