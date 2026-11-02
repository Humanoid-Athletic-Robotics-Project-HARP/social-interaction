---
name: robot-pc2
description: Read before logging in to the real G1's PC2 (the Jetson) or running anything on it — where we may work, how to connect, what is installed and running there
metadata:
  type: reference
  updated: 2026-11-02T00:51:20Z
  covers: [testground/README.md, testground/sync.sh, testground/pc2/*]
---

**Where we may work:** only `~/Workspace/testground_social_interaction` on PC2 (the
maintainer's rule; ask before creating or changing anything there, and read but never
change anything else). Our scripts there come from `testground/` via `sync.sh push`,
not a git clone. Scripts there only listen to the robot (the camera viewer also asks
Unitree's videohub for colour images, [[g1-head-camera]]).

**Connecting:** PC2 is `unitree@` its Wi-Fi address, kept in the main checkout's `.env`
(ignored by git; the login is Unitree's default, [[g1-official-docs]]). Agents don't
type passwords: the person opens one shared connection (`ssh -fN -M -S ~/.ssh/cm-g1
-o ControlPersist=8h ...`, `testground/README.md`) and commands reuse it with
`-S ~/.ssh/cm-g1` (`G1_SSH` for `sync.sh`). Key login isn't set up (it would mean
editing PC2's `~/.ssh/authorized_keys`, outside our folder). An agent can type that
command into a Terminal tab for the person to enter the password. `ssh -O check` can
say "Master running" for a dead connection (after PC2 reboots): if a command then asks
for a password, have the person open it again.

**A live view on the robot** (the order that worked, 2026-11-02, PC2 just rebooted):
1. Read the services (below). `lidar_driver` was off after the reboot, yet the lidar
   sent PC2 no raw UDP either (the viewer: "no lidar packets for 2 s"), so switch it on.
2. `camera/depth_node.py` on ROS 2 domain 42 (it owns the depth camera; not 97, HARP's).
3. `hybrid/hybrid_viewer.py --source ros` on the same domain, with Unitree's CycloneDDS
   first on `LD_LIBRARY_PATH` (both commands: the camera and hybrid READMEs), then
   `http://ADDRESS:8082`.
Run each at nice 10 in its own Terminal tab as `ssh -tt -S ~/.ssh/cm-g1 HOST '...'`, so
Ctrl-C there stops it on PC2 and nothing is left running. Seen: lidar 186,000
points/s, depth 15 fps; the viewer about 58% of a core, the depth node 14%, HARP's
harness unchanged at about 60%.

**What's there** (2026-11-01): Ubuntu 20.04, JetPack R35.3.1, Python 3.8.10 with
numpy 1.24.4, `unitree_sdk2py` 1.0.1 (from `~/Workspace/unitree_sdk2_python`) and
`cyclonedds` 0.10.2 in `~/.local`, ROS Noetic and Foxy, Livox SDK2 in `/usr/local`,
rsync, git, internet over Wi-Fi. No sudo password, no tcpdump. 1.8 TB free.
`eth0` 192.168.123.164 (robot network), `wlan0` on the lab Wi-Fi.

**What runs there:** HARP's harness (`him_safety_nodes safety_node hardware_gate`, about
60% of a core; [[him-sim2real]]), Unitree's `master_service` and `videohub_pc4` (head
camera) in `/unitree/module/`, and other people's SSH, VS Code and Claude sessions.
Keep heavy work off it (HIM's 0.2 s deadlines, [[safety]]); `record.py` runs at nice 10.

**Sharing with others on the robot:** the lidar's UDP port and the depth camera's
stream take one reader each (a second gets "Address already in use" / "Device or
resource busy"); videohub colour and DDS topics are shared. Our viewers say "Not
connected." for a taken one and take `--no-lidar` / `--no-depth` (tested 2026-11-01).
Two people reading the same sensor need one reader republishing it, e.g. over ROS 2 or
DDS, which the robot needs anyway for Nav2; `hybrid_viewer.py --source ros` is a
viewer that only listens. Nothing on the robot publishes the lidar or camera on ROS 2
unless we start it (above).

**Unitree's services** (`robot_state` client in `unitree_sdk2py.go2.robot_state`:
ServiceList reads, ServiceSwitch turns one on or off; status 0 is running, 1 off).
Switching changes the robot for everyone: ask first.
- `lidar_driver`: Unitree's lidar service, off after a reboot (seen 2026-11-02; switched
  on for each session so far). It
  re-points the lidar to itself, so PC2 gets no raw UDP while it runs (`record.py`,
  `lidar_viewer.py` and any Livox driver see nothing), and publishes on Unitree's DDS
  (domain 0), readable by any number of programs: `rt/utlidar/cloud_livox_mid360`
  (`PointCloud2_`, 9.4 Hz, 20,064 points, in the lidar's own upside-down frame; x y z
  intensity ring time; stamped about 0.1 s behind PC2's clock) and
  `rt/utlidar/imu_livox_mid360`. Off again: ServiceSwitch("lidar_driver", False).
  Over DDS it loses clouds the raw stream didn't: 19 of 158 intervals over 1.5 periods,
  up to 0.5 s (`dds-trial-20s`, read by Python beside the hybrid viewer); 6.2 clouds a
  second over 60 s with the depth node, the hybrid viewer and the camera recorder also
  running (2026-11-02). It drops the raw stream's stray returns ahead (visor on: 0–2 a
  cloud, raw 36) and passes real ones (visor off: [[g1-face-visor]]).
  Long runs beside HARP's harness are untested.
- `unitree_slam`, `auto_test_*`, `ota_box` off; `state_estimator`, the videohubs on.
- Colour: `videohub` answers any number of programs (GetImageSample: 1920×1080 JPEG,
  about 233 KB, 27 ms a request, up to 36 a second). Depth: no Unitree service; our
  `testground/camera/depth_node.py` owns the camera and publishes it on ROS 2. ROS 1's
  `realsense2_camera` (Noetic) is installed; no ROS 2 RealSense or Livox driver is.
- numpy on the Jetson: set `OPENBLAS_NUM_THREADS=1` before importing it (our scripts do);
  its BLAS pool otherwise keeps 7 threads spinning (the hybrid viewer used 6.4 cores, then
  0.7).
- Unitree's SDK beside a sourced ROS 2 Foxy: put `~/cyclonedds_ws/install/cyclonedds/lib`
  first on `LD_LIBRARY_PATH`, or its CycloneDDS fails ("undefined symbol
  ddsi_sertype_v0") against Foxy's.

**ROS 2 on the robot:** HARP's harness uses domain 97, localhost only, Fast DDS (only
`/safety/*` topics). Our depth node ran there (2026-11-01, `ROS_DOMAIN_ID=97 ROS_LOCALHOST_ONLY=1
RMW_IMPLEMENTATION=rmw_fastrtps_cpp`): depth at 15 fps, and the harness's status still
2 a second, its CPU 55–61% as before, while it waited with no policy. On domain 0,
Unitree's DDS domain, `ros2 topic list` fails with
`bad_alloc` (discovery of Unitree's hundreds of entities), so our ROS 2 nodes use
another domain (42 in testing).

**The robot's DDS** (`pc2/topics.py`, 89 topics on domain 0, eth0): `rt/lowstate` (2
publishers, about 1 kHz together), `rt/lowcmd`, `rt/sportmodestate`,
`rt/state_estimator/*` odometry, `rt/secondary_imu`, Unitree's `rt/api/*` services. No
lidar or point-cloud topic ([[g1-lidar-measured]]).
