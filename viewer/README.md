# Unified viewer

One dashboard for independent HARP feeds on a laptop or PC2. Each source has its own
connection and status. Home lists every configured service view as a full-width row, including
offline services. Larger names and Open view actions make the rows easy to scan.
Available viewers move to the top. Unreachable services have a gray,
disabled Open view action; hover the muted status line for its full reason. Each
available service opens a full-window page with a Home button. Browser back/forward
and direct links such as `#view/mac-speech/speech` work without stopping collection.

## Start

Python 3.8 or newer, with no packages needed for the server:

```bash
python3 viewer/server.py
```

Open http://127.0.0.1:28734. The default checks the local HIM viewer, follows our simulation container and
connects to local speech on port 28733. It also registers Jetson sensor and speech
sources using `G1_HOST` from the main checkout's gitignored `.env`. The value can
be `user@address` or a hostname; only the host is used for HTTP connections.
The address stays on the server and is redacted from browser-visible status,
errors and speech events. It is never copied into source configuration.

Jetson sources stay visible as offline until their services are reachable and
send data. Sensors use the existing hybrid service on port 8082. Jetson speech
uses port 28733 and is observation only by default. Missing `G1_HOST` shows a setup
message on those sources. Restart the dashboard after editing `.env`.
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

To customize the default sources, copy `viewer/config.example.json` to
`viewer/config.local.json`, edit the service settings, then run:

```bash
python3 viewer/server.py --config viewer/config.local.json
```

The JSON `sources` list has an independent entry for each running service:

| Field | Meaning |
| --- | --- |
| `id` | Unique source name used in URLs and to separate histories |
| `kind` | `him`, `simulation`, `speech`, `sensors` or `recording` |
| `label`, `machine` | Names displayed beside data and offline placeholders |
| `url` | Service address reachable from the dashboard server |
| `host_env`, `port` | Use `"host_env": "G1_HOST"` and a service port to resolve the private Jetson address from `.env`, instead of `url` |
| `layout` | For sensors: `hybrid` (default), `lidar` or `camera` |
| `controls` | For speech: `true` explicitly enables its Start/Stop/type controls |
| `container` | For simulation: Docker container name, default `g1-desktop-sim-submission-1` |
| `docker` | Optional Docker CLI path; Docker Desktop's Mac path is detected automatically |
| `python` | Optional Python executable for Rerun; defaults to `testground/.venv/bin/python` |
| `path` | For recordings: saved run folder or `.rrd` file; relative paths use the launch directory |

On PC2, start the existing hybrid sensor service for all three sensor views:

```bash
python3 testground/hybrid/hybrid_viewer.py
```

Use its `--source ros` option and the publishers' ROS environment when another node
owns the sensors. See [hybrid setup](../testground/hybrid/README.md). The dashboard
only connects to the service; opening it does not claim sensor devices.

For separate existing lidar/camera services, use distinct source entries with
`layout` set to `lidar` or `camera` and the matching service URL.

The default PC2 speech source connects when speech is deployed there. Each
speech service keeps a separate conversation. To reach a service from another
machine, start that service with its appropriate network binding, such as
`speech/web.py --host 0.0.0.0`. To reach the dashboard itself over the local network:

```bash
python3 viewer/server.py --config viewer/config.local.json --host 0.0.0.0
```

All source connections originate on the dashboard server. A remote browser does not
need direct access to PC2. Use an empty `sources` list to start with everything offline.

## HIM simulator

Home includes a HIM simulator button for its local viewer at http://127.0.0.1:8765.
Start a simulation with the usual viewer enabled, or serve an existing HIM replay.
The button becomes Ready when HIM's page is reachable. Opening it displays HIM's
own page, controls and scene in the full window, with the dashboard's Home button.

The dashboard checks only the page's availability. It does not start a simulation
or press HIM's Start button. A paused viewer remains Ready. Closing the page or
returning Home leaves HIM running; stop its launcher separately when finished.

The default source has `kind: him`. Change its `url` in a custom configuration if
HIM uses another local port. Open the dashboard on the same computer as HIM: this
integration embeds its loopback URL directly. It does not proxy remote access or
modify HIM's code. Our simulation diagnostics remain a separate service view.

## Simulation and recordings

Install the existing analysis environment once for Rerun diagnostics:

```bash
python3 -m venv testground/.venv
testground/.venv/bin/python -m pip install -r testground/requirements.txt
```

Restart the dashboard after installing the analysis environment.

The Navigation and diagnostics page reuses `submission/lidar_view.py`'s Rerun
presentation. It reads our container's JSON logs, so HIM's isolated networking
stays intact. Start a world run using the usual simulation command; the dashboard
attaches independently and follows container restarts. Plain move runs do not
produce the same view frames and show offline in this view.

Rerun starts when data arrives. It runs in a separate analysis process with a
200 MB server buffer and loopback listeners, accessed through the dashboard.
The browser uses the dashboard address for assets and data, including from another
computer. Closing the dashboard stops its renderer processes and log readers.

To add a saved run as a replay button on Home, append an entry to the source list:

```json
{"id": "saved-course", "kind": "recording", "label": "Saved course", "machine": "MacBook", "path": "runs/RUN_FOLDER"}
```

Use a `.rrd` path for a saved lidar analysis export, such as one produced with
`testground/lidar/view.py --save FILE.rrd`. Each recording has its own timeline;
pausing playback keeps its Replay status. These are our data views. HIM's own
simulator and replayer keep running as separate processes; their hosted viewer can
also be opened from the HIM simulator button.

Each configured source is monitored whether its view is open or closed. Large
images and point clouds are fetched when their view is open, with shared short
caches for multiple browser clients. Starting the dashboard starts no sensors,
microphone, LLM or robot commands.

## Checks

Tests start temporary local HTTP producers, without hardware or speech models:

```bash
python3 -m unittest discover -s viewer/tests -v
```
