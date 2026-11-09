# Unified viewer

One dashboard for independent HARP feeds on a laptop or PC2. Each source has its own
connection and status. Switching tabs changes the display while collection continues.

## Start

Python 3.8 or newer, with no packages needed for the server:

```bash
python3 viewer/server.py
```

Open http://127.0.0.1:28734. The default connects to local speech on port 28733.
Start speech separately with `speech/.venv/bin/python speech/web.py`; its setup is
in [speech/README.md](../speech/README.md). Microphones and speakers belong to the
machine running speech.

Install the pinned sensor rendering dependency once, while online:

```bash
python3 viewer/setup_assets.py
```

This downloads Three.js 0.160.0 from npm, checks its SHA-256 and installs the two
modules and MIT license in `.assets/`. Sensor views then work without internet.

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
| `kind` | `speech` or `sensors` |
| `label`, `machine` | Names displayed beside data and offline placeholders |
| `url` | Service address reachable from the dashboard server |
| `layout` | For sensors: `hybrid` (default), `lidar` or `camera` |
| `controls` | For speech: `true` explicitly enables its Start/Stop/type controls |

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

## Checks

Tests start temporary local HTTP producers, without hardware or speech models:

```bash
python3 -m unittest discover -s viewer/tests -v
```
