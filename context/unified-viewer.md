---
name: unified-viewer
description: Read before planning or implementing the unified web viewer and its feed adapters
metadata:
  type: reference
  updated: 2026-11-09T04:10:08Z
  covers: [viewer/**]
---

The agreed scope, research and implementation sequence are in [[unified-viewer-plan]]. Keep plans in project memory, not `docs/`.

The requested scope combines our existing sensor, speech and diagnostic views through independent feeds from the robot or laptop. Missing feeds show offline. HIM's own simulator, viewer and replayer stay separate.

Collect all configured sources concurrently: MacBook Docker simulation, native MacBook speech and native PC2 lidar/camera. Speech may also run natively on PC2. There is no exclusive deployment mode. Source selection changes presentation, not collection. Keep streams and histories separate by source and session, including simultaneous simulated/real lidar or two speech services.

`viewer/server.py` is a standard-library HTTP hub. Setup: `viewer/README.md`. Sensor and speech collectors run independently; the browser uses one status SSE connection. Speech has a separate bounded history per source with reconnect deduplication and reset on service instance changes. Existing sensor pages are read as literals, without importing their drivers. Binary samples use short shared caches; measurements are never cached. Speech commands require an explicitly enabled source.

Three.js 0.160.0 is a pinned, hash-checked dependency installed by `viewer/setup_assets.py` into ignored `.assets/`, including its MIT license. Sensor pages use those local assets through the hub.

`viewer/simulation.py` follows our Docker stdout, identifies runs by container ID and start time, and uses a bounded nonblocking renderer queue. Fresh view frames establish live status; other log events do not. Recordings read saved JSON lines or `.rrd` files and keep Replay status at EOF. No HIM code or networking changes.

`viewer/rerun_worker.py` runs in the existing testground environment and reuses `submission/lidar_view.py`'s `Recording`. It starts only when data arrives, uses dynamic loopback ports and a 200 MB Rerun buffer, and stops with the hub. `viewer/diagnostics.py` proxies assets and gRPC-web through the hub. Rerun 0.41 only accepts the root `/proxy` URI: the embedded page scopes its fetches to the source's route. Rewritten POST bodies must be buffered into an ArrayBuffer; passing a Request body stream makes Chrome require HTTP/2 and fail on the local HTTP/1 server. Quiet streams have no read timeout; client disconnect or hub shutdown cancels them, with incomplete HTTP reads treated as normal cleanup.

Validation: standard-library integration tests cover concurrent sources, partial sensor failure, stale layers, history isolation and simulated Docker restart. Browser checks exercised local Three.js without internet, dashboard disconnect/reconnect, phone layout, and a real saved simulation alongside a separate `.rrd` capture in embedded Rerun. Real PC2 sensors and live simulation overhead have not been checked.

Read [[speech-web]], [[sim-run-logs]], [[feedback-minimal-setup]] and [[external-dependencies]] before implementation. Follow [[architecture-guide]] when changing processes or connections.
