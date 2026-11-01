# Testground

Tests on the real G1, one folder per thing being tested. The scripts that touch the
robot run on PC2 from a clone of this repo; their data comes back over SSH and is
analysed on a laptop.

| Path | Runs on | What |
|---|---|---|
| `pc2/probe.sh`, `pc2/topics.py` | PC2 | Read-only survey: system, installed SDK, network, disk, load, every DDS topic. |
| `lidar/` | both | The head lidar: what it is and how well it sees ([lidar/README.md](lidar/README.md)). |
| `sync.sh` | laptop | Pull a test's captures from PC2, push a quick fix, or open a shell there. |

## Rules

- On PC2, work only inside `~/workspaces/testground-social-interaction`; read
  anything else, change nothing else. Ask before creating or changing anything there.
- Scripts here only read from the robot (they subscribe; they never publish) unless a
  test says otherwise and the team has agreed to it. An operator owns the robot
  during a session (power, gantry, remote, stopping: `context/safety.md`).
- No git credentials on the robot: PC2 only pulls. Data comes back with `sync.sh pull`.

## On PC2, once

The repo is public, so the clone needs no login. Only this folder is checked out:

```bash
mkdir -p ~/workspaces/testground-social-interaction && cd ~/workspaces/testground-social-interaction
git clone --filter=blob:none --sparse https://github.com/Humanoid-Athletic-Robotics-Project-HARP/social-interaction.git
cd social-interaction && git sparse-checkout set testground && git checkout lidar-visualization
```

Then `testground/pc2/probe.sh` shows whether Unitree's Python SDK (`unitree_sdk2py`,
`cyclonedds` 0.10.2) is there; the robot scripts need it.

Update with `git pull`. A quick fix mid-session without a commit: `./sync.sh push`
from the laptop (the clone then shows the changed files as modified; each capture
records the commit, with `-dirty` when that happened).

## On the laptop

```bash
cd testground && python3 -m venv .venv && .venv/bin/pip install -r requirements.txt
```

```bash
G1_HOST=unitree@ADDRESS ./sync.sh pull lidar
```
