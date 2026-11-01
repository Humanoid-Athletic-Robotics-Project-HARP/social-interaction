# Head lidar

What the G1's head lidar really is and how well it sees, measured on our robot.
Nav2 (`submission/nav2/`), the simulated lidar (`submission/harp/lidar.py`) and its
mount (`submission/harp/lidar_mount.py`) are built on spec sheets and Unitree's docs
(`context/lidar-nav2-research.md`); these tests check them.

`record.py` runs on PC2 and only subscribes, to Unitree's lidar service
(`rt/utlidar/cloud_livox_mid360`), the lidar's IMU (`rt/utlidar/imu_livox_mid360`)
and the robot's low state (`rt/lowstate`, for the arms' and waist's pose). The
recording is copied to a laptop, where `report.py` answers the questions below and
`view.py` shows the cloud in Rerun.

## Questions, and what answers them

| Question | Assumed now | Answered by |
|---|---|---|
| Which lidar? MID-360 or the newer Mid360s (G1s from April 2026) | MID-360 | `probe.sh` topics, the cloud's fields and point count; the robot's build date or the lidar's label |
| Who publishes it, how fast, how late | Unitree's service, 10 Hz | report: Timing (`static-open`) |
| What each point carries: intensity, line, per-point time | x, y, z | report: The cloud |
| Field of view, how fast the non-repeating scan fills in | −7 to +52°, 360° | report: Range and field of view |
| Mount: upside down? height, tilt | flipped, pitch 2.9°, 0.43 m above `torso_link` | report: Floor and mount, Lidar IMU |
| The blind ring near the feet | floor unseen within about 1 m | report: Floor (`static-open`, `floor-box-*`) |
| Does it see the robot's body? Where, with what arm pose | shoulders and arms | report: Returns within 0.7 m; `view.py` (`static-open`, `arms-*`) |
| Range accuracy and noise | ≤ 2 cm | report: Flat target (`wall-*` with `--box`, `--truth`) |
| What it misses: dark, shiny, glass, thin | – | `view.py`, `--box` on each (`targets`) |
| People, standing and walking | – | `view.py` (`person-*`) |
| Clock drift, long-run stability | – | report: Timing (`static-5min`) |
| Does Unitree's service still run under HIM's harness? | unknown | later, during HIM's dry run (`context/safety.md`) |

## A session

Robot powered on, in the gantry, an operator in charge (`context/safety.md`). Nothing
here moves it. Standing at its normal height is best, so the floor test measures the
lidar where it will be when walking.

On PC2, in `~/workspaces/testground-social-interaction/social-interaction/testground`:

```bash
pc2/probe.sh
```

The run is good to go if the lidar (192.168.123.120) answers and `topics.py` lists
`rt/utlidar/cloud_livox_mid360`. Then, from `lidar/`, one capture per scene, each
with a `--note` saying what was where:

| Capture | Scene | Length |
|---|---|---|
| `static-open` | nothing within 2 m; arms at rest | 60 s |
| `static-5min` | the same | 300 s |
| `wall-1m`, `wall-2m`, `wall-4m` | a flat wall or board straight ahead; tape-measure from the lidar | 20 s each |
| `floor-box-0.5m`, `-1.0m`, `-1.5m` | a box (about 30 cm) on the floor ahead at that distance | 15 s each |
| `targets` | black, white, shiny and glass boards and a thin pole (about 2 cm), 1.5 m away; note where each is | 30 s |
| `person-2m`, `person-walk` | someone standing 2 m ahead; then walking across 1–3 m away | 20 s each |
| `arms-*` | other arm poses, only if the operator sets them | 15 s each |

```bash
python3 record.py static-open --seconds 60 --note "gantry, standing, clear 2 m around, arms at rest"
```

On the laptop, from `testground/`:

```bash
G1_HOST=unitree@ADDRESS ./sync.sh pull lidar
```

```bash
cd lidar && ../.venv/bin/python report.py captures/static-open
```

```bash
../.venv/bin/python report.py captures/wall-2m --box 1.7 2.3 -0.5 0.5 -0.8 0.5 --truth 2.00
```

```bash
../.venv/bin/python view.py captures/static-open --accumulate 10
```

`--box` is in `torso_link` (x forward, y left, z up, by the assumed mount); find the
target's box in `view.py` first. The report is also saved as `report.md` in the
capture's folder.

## A capture

`captures/NAME/` (ignored by git; about 5 MB/s): `meta.json` (when, where, the
scripts' commit, the cloud's layout, counts), `clouds.bin` (every cloud's points as
received) with `clouds.csv` (one row per cloud: PC2's arrival time, the robot's stamp,
its bytes), `imu.csv`, and `lowstate.csv` (50 Hz). `capture.py` reads it.

Not yet run against the robot. `report.py` and `view.py` were checked on a made-up
capture in this format.
