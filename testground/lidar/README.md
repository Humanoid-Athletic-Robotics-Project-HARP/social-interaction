# Head lidar

What the G1's head lidar sends and how well it sees, measured on our robot. Results:
`context/g1-lidar-measured.md`.

The MID-360 pushes Livox SDK2 UDP packets straight to PC2 (points, its IMU, a status
message), unless Unitree's `lidar_driver` service runs, which takes the stream and
publishes it on DDS instead (`record.py --source dds`). `record.py` runs on PC2 and
saves every packet as it arrives, plus the robot's joints from `rt/lowstate`. On the
laptop, `capture.py` decodes a capture into 100 ms clouds, `report.py` answers the
questions below, `blind_map.py` maps its blind wedge ahead, `visor_model.py` models the
face visor that makes it and `view.py` shows it in Rerun.

To watch it live instead, `lidar_viewer.py` on PC2 serves a 3D view on port 8080
(`python3 lidar_viewer.py`, then `http://ADDRESS:8080`), with the directions the
robot's head blocks and the floor it can't see near the feet, measured live. It holds
the lidar's point port, so stop it before recording; if the port is already taken it
says "Not connected.", and `--no-lidar` leaves the port alone. `LIDAR_REPORT.md` is the first look at the stream
that it came with.

## Questions, and what answers them

| Question | Answered by |
|---|---|
| Which lidar, which firmware, how it's configured | report: The lidar (its status messages) |
| How fast, and is anything lost | report: The stream, Timing |
| Field of view, how fast the scan fills in, where it's blind | report: Range and field of view, Returns by direction |
| Where it's blind ahead | `blind_map.py` (any capture; again after a change to the head) |
| What the face visor blocks, and what taking it off would open | `visor_model.py` (in the submission image) |
| Mount: upside down? height, tilt | report: Floor and mount, Lidar IMU |
| Which way is the robot's front in the lidar's frame | `person-front-2m`: where the person shows up in `view.py` |
| The blind ring near the feet | report: Floor (`floor-box-*`) |
| Does it see the robot's body | report: Returns within 0.7 m; `view.py` (`arms-*`) |
| Does the self-filter remove the body, and is the model's body where the lidar sees it | `self_filter_check.py` (in the submission image; any capture with `lowstate.csv`, best with `arms-*`) |
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

With Unitree's `lidar_driver` service on (the lidar then sends PC2 nothing directly),
record its DDS topics instead: `--source dds` (shared, so viewers can run meanwhile).
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
../.venv/bin/python blind_map.py captures/standing-60s
```

```bash
docker run --rm -v "$PWD/../..:/repo" -w /repo/testground/lidar --entrypoint python3 harp-submission:local visor_model.py captures/standing-60s
```

```bash
docker run --rm -v "$PWD/../..:/repo" -w /repo/testground/lidar --entrypoint python3 harp-submission:local self_filter_check.py captures/standing-60s
```

```bash
../.venv/bin/python view.py captures/standing-60s --accumulate 10
```

`--box` is in `torso_link` (x forward, y left, z up, by the assumed mount); find the
target's box in `view.py` first. The report is also saved as `report.md` in the
capture's folder; `blind_map.py` and `visor_model.py` save their `.md` and `.svg` there,
and `self_filter_check.py` its `self_filter.md`. `visor_model.py` and
`self_filter_check.py` need the submission image (MuJoCo) and Unitree's G1 model, which
`submission/build.sh` stages in `submission/.build`.

## A capture

`captures/NAME/` (ignored by git): `meta.json` (when, the scene's note, packet
counts, packets the kernel dropped), `udp.bin` (every packet: arrival time on PC2,
port, length, bytes) and `lowstate.csv` (pelvis IMU and joints, 50 Hz).
