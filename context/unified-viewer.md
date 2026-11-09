---
name: unified-viewer
description: Use whenever the user wants to see simulator activity, a replay, lidar, camera, speech, or any other data source; also read before changing the dashboard
metadata:
  type: project
  updated: 2026-11-09T07:01:23Z
  covers: [viewer/**]
---

Use this dashboard whenever someone wants to see the simulator, a replay, lidar, camera, speech, or any other data source. It is the default viewing entry point and replaces sending people to separate HIM, Rerun, lidar, camera or speech viewer pages. Those programs remain the underlying producers/renderers; keep HIM unchanged. Add future data views here rather than creating another standalone UI.

Check whether the hub is running before starting another copy. Launch: `python3 viewer/server.py`; Home: http://127.0.0.1:28734. Setup and source configuration live in `viewer/README.md`.

| Want to see | Dashboard view (default source IDs) |
| --- | --- |
| HIM simulation or its replay | `/#view/him-sim/him` |
| Our simulated lidar, odometry, Nav2 and events | `/#view/mac-sim/diagnostics` |
| Laptop speech/listening | `/#view/mac-speech/speech` |
| Robot lidar, camera or combined view | `/#view/pc2-sensors/lidar`, `/#view/pc2-sensors/camera` or `/#view/pc2-sensors/hybrid` |
| Jetson speech | `/#view/pc2-speech/speech` |
| Saved diagnostics or lidar capture | A configured `recording` source, opened from Home |

Resolve the paths above against the dashboard address. Give the user its Home or relevant view link, not the producer's direct port. Watched-run behavior: [[feedback-watching-runs]].

Home and service views share a compact toolbar with no introductory banner. Home has full-width rows, available viewers first, and disabled gray actions for unreachable services. Errors stay subdued with details on hover. A line above the rows says whether this computer is likely to keep a sim run's deadlines ([[sim-deadline-stops]]); diagnostics and recording rows add the run's late states and HIM's stop reason. Sources collect concurrently and retain separate sessions; navigation never stops a producer.

The hub starts no simulation, sensors, LLM, microphone or conversation. Start the required producer separately within the authorized task. In particular, an LLM on 28731 is not a speech feed: the speech supervisor `speech/web.py` must also run. Offline is expected when a producer is absent.

Jetson sources read `G1_HOST` from the main checkout's gitignored `.env`, also in worktrees. Keep its address there, never in tracked configuration, output or browser data. Restart the hub after configuration changes.

HIM embedding is local only. Keep `sim.sh`'s viewer enabled (or use `course.py --view`) for live HIM viewing; `course.py --replay RUN` serves its replay. Returning Home does not release HIM's viewer memory: close its launcher after use. Saved Rerun diagnostics instead use a `recording` source and do not require HIM's replay server.

Implementation constraints and verified limits: [[viewer-implementation]].
