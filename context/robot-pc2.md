---
name: robot-pc2
description: Read before logging in to the real G1's PC2 (the Jetson) or running anything on it — where we may work, how to connect, what is installed and running there
metadata:
  type: reference
  updated: 2026-11-02T22:04:05Z
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
1. Read the services ([[robot-services]]). `lidar_driver` was off after the reboot, yet the lidar
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

Unitree's services (`lidar_driver`, videohub), ROS 2 domains and the robot's DDS: [[robot-services]].
