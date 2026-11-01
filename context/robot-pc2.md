---
name: robot-pc2
description: Read before logging in to the real G1's PC2 (the Jetson) or running anything on it — where we may work, how to connect, what is installed and running there
metadata:
  type: reference
  updated: 2026-11-01T21:26:09Z
  covers: [testground/README.md, testground/sync.sh, testground/pc2/*]
---

**Where we may work:** only `~/Workspace/testground_social_interaction` on PC2 (the
maintainer's rule; ask before creating or changing anything there, and read but never
change anything else). Our scripts there come from `testground/` via `sync.sh push`,
not a git clone. Scripts there only listen to the robot.

**Connecting:** PC2 is `unitree@` its Wi-Fi address, kept in the main checkout's `.env`
(ignored by git; the login is Unitree's default, [[g1-official-docs]]). Agents don't
type passwords: the person opens one shared connection (`ssh -fN -M -S ~/.ssh/cm-g1
-o ControlPersist=8h ...`, `testground/README.md`) and commands reuse it with
`-S ~/.ssh/cm-g1` (`G1_SSH` for `sync.sh`). Key login isn't set up (it would mean
editing PC2's `~/.ssh/authorized_keys`, outside our folder).

**What's there** (2026-11-01): Ubuntu 20.04, JetPack R35.3.1, Python 3.8.10 with
numpy 1.24.4, `unitree_sdk2py` 1.0.1 (from `~/Workspace/unitree_sdk2_python`) and
`cyclonedds` 0.10.2 in `~/.local`, ROS Noetic and Foxy, Livox SDK2 in `/usr/local`,
rsync, git, internet over Wi-Fi. No sudo password, no tcpdump. 1.8 TB free.
`eth0` 192.168.123.164 (robot network), `wlan0` on the lab Wi-Fi.

**What runs there:** HARP's harness (`him_safety_nodes safety_node hardware_gate`, about
60% of a core; [[him-sim2real]]), Unitree's `master_service` and `videohub_pc4` (head
camera) in `/unitree/module/`, and other people's SSH, VS Code and Claude sessions.
Keep heavy work off it (HIM's 0.2 s deadlines, [[safety]]); `record.py` runs at nice 10.

**The robot's DDS** (`pc2/topics.py`, 89 topics on domain 0, eth0): `rt/lowstate` (2
publishers, about 1 kHz together), `rt/lowcmd`, `rt/sportmodestate`,
`rt/state_estimator/*` odometry, `rt/secondary_imu`, Unitree's `rt/api/*` services. No
lidar or point-cloud topic ([[g1-lidar-measured]]).
