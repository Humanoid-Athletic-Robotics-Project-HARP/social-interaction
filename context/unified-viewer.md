---
name: unified-viewer
description: Read before planning or implementing the unified web viewer and its feed adapters
metadata:
  type: reference
  updated: 2026-11-09T03:43:46Z
  covers: [viewer/**]
---

The agreed scope, research and implementation sequence are in [[unified-viewer-plan]]. Keep plans in project memory, not `docs/`.

The requested scope combines our existing sensor, speech and diagnostic views through independent feeds from the robot or laptop. Missing feeds show offline. HIM's own simulator, viewer and replayer stay separate.

Collect all configured sources concurrently: MacBook Docker simulation, native MacBook speech and native PC2 lidar/camera. Speech may also run natively on PC2. There is no exclusive deployment mode. Source selection changes presentation, not collection. Keep streams and histories separate by source and session, including simultaneous simulated/real lidar or two speech services.

`viewer/server.py` is a standard-library HTTP hub. Setup: `viewer/README.md`. Sensor and speech collectors run independently; the browser uses one status SSE connection. Speech has a separate bounded history per source with reconnect deduplication and reset on service instance changes. Existing sensor pages are read as literals, without importing their drivers. Binary samples use short shared caches; measurements are never cached. Speech commands require an explicitly enabled source.

Three.js 0.160.0 is a pinned, hash-checked dependency installed by `viewer/setup_assets.py` into ignored `.assets/`, including its MIT license. Sensor pages use those local assets through the hub.

Read [[speech-web]], [[sim-run-logs]], [[feedback-minimal-setup]] and [[external-dependencies]] before implementation. Follow [[architecture-guide]] when changing processes or connections.
