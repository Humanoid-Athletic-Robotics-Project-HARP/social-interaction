# Unified viewer

One dashboard for independent HARP feeds on a laptop or PC2. Each source has its own
connection and status. Switching tabs changes the display while collection continues.

## Start

Python 3.8 or newer, with no packages needed for the server:

```bash
python3 viewer/server.py
```

Open http://127.0.0.1:28734. The default follows our local simulation container and
connects to local speech on port 28733.
Start speech separately with `speech/.venv/bin/python speech/web.py`; its setup is
in [speech/README.md](../speech/README.md). Microphones and speakers belong to the
machine running speech.

Install the pinned sensor rendering dependency once, while online:

```bash
python3 viewer/setup_assets.py
```

This downloads Three.js 0.160.0 from npm, checks its SHA-256 and installs the two
modules and MIT license in `.assets/`. Reload the dashboard after installation.
Sensor views then work without internet.

## Sources

Copy `viewer/config.example.json` to `viewer/config.local.json`, replace
`PC2-WIFI-ADDRESS` with PC2's reachable address, then run:

```bash
python3 viewer/server.py --config viewer/config.local.json
```

The JSON `sources` list has an independent entry for each running service:

| Field | Meaning |
| --- | --- |
| `id` | Unique source name used in URLs and to separate histories |
| `kind` | `simulation`, `speech`, `sensors` or `recording` |
| `label`, `machine` | Names displayed beside data and offline placeholders |
| `url` | Service address reachable from the dashboard server |
| `layout` | For sensors: `hybrid` (default), `lidar` or `camera` |
| `controls` | For speech: `true` explicitly enables its Start/Stop/type controls |
| `container` | For simulation: Docker container name, default `g1-desktop-sim-submission-1` |
| `docker` | Optional Docker CLI path; Docker Desktop's Mac path is detected automatically |
| `python` | Optional Python executable for Rerun; defaults to `testground/.venv/bin/python` |
| `path` | For recordings: saved run folder or `.rrd` file; relative paths use the launch directory |

On PC2, start the existing hybrid sensor service for all three sensor tabs:

```bash
python3 testground/hybrid/hybrid_viewer.py
```

Use its `--source ros` option and the publishers' ROS environment when another node
owns the sensors. See [hybrid setup](../testground/hybrid/README.md). The dashboard
only connects to the service; opening it does not claim sensor devices.

For separate existing lidar/camera services, use distinct source entries with
`layout` set to `lidar` or `camera` and the matching service URL.

Add a second `speech` source pointing to PC2 when speech is deployed there. Each
speech service keeps a separate conversation. To reach a service from another
machine, start that service with its appropriate network binding, such as
`speech/web.py --host 0.0.0.0`. To reach the dashboard itself over the local network:

```bash
python3 viewer/server.py --config viewer/config.local.json --host 0.0.0.0
```

All source connections originate on the dashboard server. A remote browser does not
need direct access to PC2. Use an empty `sources` list to start with everything offline.

## Simulation and recordings

Install the existing analysis environment once for Rerun diagnostics:

```bash
python3 -m venv testground/.venv
testground/.venv/bin/python -m pip install -r testground/requirements.txt
```

Restart the dashboard after installing the analysis environment.

The Navigation and diagnostics tab reuses `submission/lidar_view.py`'s Rerun
presentation. It reads our container's JSON logs, so HIM's isolated networking
stays intact. Start a world run using the usual simulation command; the dashboard
attaches independently and follows container restarts. Plain move runs do not
produce the same view frames and show offline in this tab.

Rerun starts when data arrives. It runs in a separate analysis process with a
200 MB server buffer and loopback listeners, accessed through the dashboard.
The browser uses the dashboard address for assets and data, including from another
computer. Closing the dashboard stops its renderer processes and log readers.

To add a saved run to Recordings, append an entry to the source list:

```json
{"id": "saved-course", "kind": "recording", "label": "Saved course", "machine": "MacBook", "path": "runs/RUN_FOLDER"}
```

Use a `.rrd` path for a saved lidar analysis export, such as one produced with
`testground/lidar/view.py --save FILE.rrd`. Each recording has its own timeline;
pausing playback keeps its Replay status. These are our data views. HIM's own
simulator viewer and replayer continue to run separately.

Each configured source is monitored whether its tab is open or closed. Large
images and point clouds are fetched when their tab is open, with shared short
caches for multiple browser clients. Starting the dashboard starts no sensors,
microphone, LLM or robot commands.

## Checks

Tests start temporary local HTTP producers, without hardware or speech models:

```bash
python3 -m unittest discover -s viewer/tests -v
```
