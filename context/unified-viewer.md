---
name: unified-viewer
description: Read before planning or implementing the unified web viewer and its feed adapters
metadata:
  type: reference
  updated: 2026-11-09T05:12:17Z
  covers: [viewer/**]
---

The agreed scope, research and implementation sequence are in [[unified-viewer-plan]]. Keep plans in project memory, not `docs/`.

Home lists every configured source/view as a full-width rectangular row, with larger names and a right-aligned Open view action. Rows share spare vertical space on laptop screens and stack their details on narrow screens. Available viewers sort first as status changes; unreachable services have disabled gray Open view actions. Errors are a muted single line with the full reason on hover. DOM reordering preserves keyboard focus. Each opens one full-window view under a compact Home toolbar, with no tabs or surrounding cards. Hash routes `#view/SOURCE/VIEW` support direct links and browser history. Returning Home removes the iframe, not the producer or collector.

The requested scope combines our existing sensor, speech and diagnostic views through independent feeds from the robot or laptop. Missing feeds show offline. HIM stays an external system; its own locally hosted viewer can also be embedded without changing it.

Collect all configured sources concurrently: MacBook Docker simulation, native MacBook speech and native PC2 lidar/camera. Speech may also run natively on PC2. There is no exclusive deployment mode. Navigation changes presentation, not collection. Keep streams and histories separate by source and session, including simultaneous simulated/real lidar or two speech services.

`viewer/server.py` is a standard-library HTTP hub. Setup: `viewer/README.md`. Sensor and speech collectors run independently; the browser uses one status SSE connection. Speech has a separate bounded history per source with reconnect deduplication and reset on service instance changes. Existing sensor pages are read as literals, without importing their drivers. Binary samples use short shared caches; measurements are never cached. Speech commands require an explicitly enabled source.

`viewer/configuration.py` registers local sim/speech and Jetson sensors/speech by default. `host_env: G1_HOST` resolves a hostname or SSH login from the main checkout's gitignored `.env`, including from worktrees. Never copy the address into tracked JSON, docs or output. The hub redacts it from status, proxied JSON, errors and speech SSE. Missing `.env` keeps Jetson sources registered offline with a setup reason; restart after editing it. Jetson speech defaults to observation only.

Three.js 0.160.0 is a pinned, hash-checked dependency installed by `viewer/setup_assets.py` into ignored `.assets/`, including its MIT license. Sensor pages use those local assets through the hub.

`kind: him` registers HIM's local viewer (default port 8765). The collector checks its HTML identity, not frame activity, so paused viewers remain Ready. The page embeds the loopback URL directly and sends no controls during discovery. Local access only; changing the port uses source configuration. Neither HIM sources nor its lifecycle are modified.

`viewer/simulation.py` follows our Docker stdout, identifies runs by container ID and start time, and uses a bounded nonblocking renderer queue. Fresh view frames establish live status; other log events do not. Recordings read saved JSON lines or `.rrd` files and keep Replay status at EOF. No HIM code or networking changes.

`viewer/rerun_worker.py` runs in the existing testground environment and reuses `submission/lidar_view.py`'s `Recording`. It starts only when data arrives, uses dynamic loopback ports and a 200 MB Rerun buffer, and stops with the hub. `viewer/diagnostics.py` proxies assets and gRPC-web through the hub. Rerun 0.41 only accepts the root `/proxy` URI: the embedded page scopes its fetches to the source's route. Rewritten POST bodies must be buffered into an ArrayBuffer; passing a Request body stream makes Chrome require HTTP/2 and fail on the local HTTP/1 server. Quiet streams have no read timeout; client disconnect or hub shutdown cancels them, with incomplete HTTP reads treated as normal cleanup.

Validation: standard-library integration tests cover concurrent sources, partial sensor failure, stale layers, history isolation and simulated Docker restart. Browser checks exercised local Three.js without internet, dashboard disconnect/reconnect, phone layout, and a real saved simulation alongside a separate `.rrd` capture in embedded Rerun. Real PC2 sensors and live simulation overhead have not been checked.

Read [[speech-web]], [[sim-run-logs]], [[feedback-minimal-setup]] and [[external-dependencies]] before implementation. Follow [[architecture-guide]] when changing processes or connections.
