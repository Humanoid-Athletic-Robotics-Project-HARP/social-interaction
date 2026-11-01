# Head lidar

What the G1's head lidar sends and how well it sees, measured on our robot. Results:
`context/g1-lidar-measured.md`.

The MID-360 pushes Livox SDK2 UDP packets straight to PC2 (points, its IMU, a status
message); there is no DDS lidar topic on this robot. `record.py` runs on PC2 and
saves every packet as it arrives, plus the robot's joints from `rt/lowstate`. On the
laptop, `capture.py` decodes a capture into 100 ms clouds, `report.py` answers the
questions below and `view.py` shows it in Rerun.

To watch it live instead, `lidar_viewer.py` on PC2 serves a 3D view on port 8080
(`python3 lidar_viewer.py`, then `http://ADDRESS:8080`). It holds the lidar's point
port, so stop it before recording. `LIDAR_REPORT.md` is the first look at the stream
that it came with.

## Questions, and what answers them

| Question | Answered by |
|---|---|
| Which lidar, which firmware, how it's configured | report: The lidar (its status messages) |
| How fast, and is anything lost | report: The stream, Timing |
| Field of view, how fast the scan fills in, where it's blind | report: Range and field of view, Returns by direction |
| Mount: upside down? height, tilt | report: Floor and mount, Lidar IMU |
| Which way is the robot's front in the lidar's frame | `person-front-2m`: where the person shows up in `view.py` |
| The blind ring near the feet | report: Floor (`floor-box-*`) |
| Does it see the robot's body | report: Returns within 0.7 m; `view.py` (`arms-*`) |
| Range accuracy and noise | report: Flat target (`wall-*` with `--box`, `--truth`) |
| What it misses: dark, shiny, glass, thin | `view.py`, `--box` on each (`targets`) |
| People, standing and walking | `view.py` (`person-*`) |
| Clock drift over minutes | report: Timing (`--headers-only` capture) |

## A session

Robot powered on, an operator in charge (`context/safety.md`). Nothing here moves it.
Only one program can receive the lidar's ports: stop any other lidar reader first
(`record.py` says which port is taken).

On PC2, in `~/Workspace/testground_social_interaction/lidar`, one capture per scene,
each with a `--note` saying what was where:

```bash
python3 record.py person-front-2m --seconds 20 --note "someone standing 2.0 m straight ahead of the robot's face"
```

| Capture | Scene | Length |
|---|---|---|
| `person-front-2m`, `person-back-2m` | someone 2 m straight ahead, then behind | 20 s each |
| `wall-1m`, `wall-2m`, `wall-4m` | a flat wall or board beside or behind the robot (where it sees); tape-measured from the lidar | 20 s each |
| `floor-box-0.5m`, `-1.0m`, `-1.5m` | a box (about 30 cm) on the floor at that distance, beside the robot | 15 s each |
| `targets` | black, white, shiny and glass boards and a thin pole (about 2 cm), 1.5 m away | 30 s |
| `person-walk` | someone walking around the robot 1–3 m away | 30 s |
| `arms-*` | other arm poses, only if the operator sets them | 15 s each |

Full captures are about 175 MB a minute; `--headers-only` keeps only packet headers
(timing and loss, about 100 KB/s).

On the laptop, from `testground/`:

```bash
./sync.sh pull lidar
```

```bash
cd lidar && ../.venv/bin/python report.py captures/standing-60s
```

```bash
../.venv/bin/python report.py captures/wall-2m --box -0.3 0.3 1.7 2.3 -0.8 0.5 --truth 2.00
```

```bash
../.venv/bin/python view.py captures/standing-60s --accumulate 10
```

`--box` is in `torso_link` (x forward, y left, z up, by the assumed mount); find the
target's box in `view.py` first. The report is also saved as `report.md` in the
capture's folder.

## A capture

`captures/NAME/` (ignored by git): `meta.json` (when, the scene's note, packet
counts, packets the kernel dropped), `udp.bin` (every packet: arrival time on PC2,
port, length, bytes) and `lowstate.csv` (pelvis IMU and joints, 50 Hz).
