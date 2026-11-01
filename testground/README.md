# Testground

Tests on the real G1, one folder per thing being tested. The scripts that touch the
robot run on PC2 (the Jetson) from a copy of this folder; their data comes back over
SSH and is analysed on a laptop, so it can be studied without the robot.

| Path | Runs on | What |
|---|---|---|
| `pc2/probe.sh`, `pc2/topics.py`, `pc2/v4l2_survey.py` | PC2 | Read-only survey: system, installed SDK, network, disk, load, every DDS topic, the cameras' video modes. |
| `lidar/` | both | The head lidar (a Livox MID-360): recordings, analysis and a live viewer ([lidar/README.md](lidar/README.md)). |
| `camera/` | PC2 | The head camera (a RealSense D435i) live in a browser ([camera/README.md](camera/README.md)). |
| `hybrid/` | PC2 | The lidar and the camera's depth together live, the camera filling the lidar's blind cone ahead, with tabs for each sensor's own view ([hybrid/README.md](hybrid/README.md)). |
| `sync.sh` | laptop | Copy the scripts to PC2, pull a test's captures back, or open a shell there. |

## Rules

- On PC2, work only inside `~/Workspace/testground_social_interaction`; read
  anything else, change nothing else. Ask before creating or changing anything there.
- Scripts here only read from the robot (they listen; the camera viewer also asks
  Unitree's videohub for colour images) unless a test says otherwise and the team has
  agreed to it. An operator owns the robot during a
  session (power, gantry, remote, stopping: `context/safety.md`). HARP's harness may be
  running on PC2: keep heavy work off it.
- PC2's login lives in the main checkout's `.env` (`G1_HOST=user@address`), which git
  ignores. Never commit it.

## Connecting

PC2's Wi-Fi address is in `.env`. One shared connection, opened once with the
password, lets every later command (and `sync.sh`) reuse it for 8 hours:

```bash
ssh -fN -M -S ~/.ssh/cm-g1 -o ControlPersist=8h unitree@ADDRESS
```

```bash
export G1_SSH="-S $HOME/.ssh/cm-g1"
```

## Using it

From `testground/` on the laptop:

```bash
./sync.sh push
```

```bash
./sync.sh pull lidar
```

`push` copies the scripts into the folder on PC2 (files already the same there are left
untouched; nothing is ever deleted). `pull` brings a test's captures back into its
`captures/` here (ignored by git).

For the analysis tools, once:

```bash
python3 -m venv .venv && .venv/bin/pip install -r requirements.txt
```
