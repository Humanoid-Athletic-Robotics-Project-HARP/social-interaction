---
name: unified-viewer
description: Use whenever the user wants to see simulator activity, a replay, lidar, camera, speech, or any other data source; also read before changing the dashboard
metadata:
  type: project
  updated: 2026-11-11T18:38:11Z
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
| The robot's battery, motor temperatures and harness state | `/#view/pc2-robot/robot` ([[robot-status]]) |
| A saved sim run, or two side by side | Saved runs, `/#runs` (Replay, or tick two and Compare) |
| Why a run went late or HIM stopped it | Saved runs, Timing on its row (`/#timing/RUN`) |
| A saved lidar capture (`.rrd`) | A configured `recording` source, opened from Home |

Resolve the paths above against the dashboard address. Give the user its Home or relevant view link, not the producer's direct port. Watched-run behavior: [[feedback-watching-runs]].

Home and service views share a compact toolbar with no introductory banner. Home's Running now section has a tile per reachable source (one per process, not per view; a sensor tile opens the hybrid view): a status pill, a one-line headline of what it is doing, chips for its numbers (amber and red mean look) and a run's pieces as quiet tags; below it, All services keeps full-width rows, available viewers first, and disabled gray actions for unreachable services. Errors stay subdued with details on hover. A line above the rows says whether this computer is likely to keep a sim run's deadlines ([[sim-deadline-stops]]); diagnostics and recording rows add the run's late states and HIM's stop reason. Sources collect concurrently and retain separate sessions; navigation never stops a producer.

The hub starts no simulation, sensors, LLM, microphone or conversation. Start the required producer separately within the authorized task. In particular, an LLM on 28731 is not a speech feed: the speech supervisor `speech/web.py` must also run. Offline is expected when a producer is absent.

Jetson sources read `G1_HOST` from the main checkout's gitignored `.env`, also in worktrees. Keep its address there, never in tracked configuration, output or browser data. Restart the hub after configuration changes.

HIM embedding is local only. Keep `sim.sh`'s viewer enabled (or use `course.py --view`) for live HIM viewing; `course.py --replay RUN` serves its replay. Returning Home does not release HIM's viewer memory: close its launcher after use. Saved Rerun diagnostics instead use a `recording` source and do not require HIM's replay server.

Implementation constraints and verified limits: [[viewer-implementation]].

Next steps, none started (as of 2026-11-09):
- Robot runs: start `harp/view_feed.py` on the robot too (`entrypoint.sh` starts it only in sim worlds), and have the hub read the robot container's output read-only over the existing SSH control socket (`ssh -S ~/.ssh/cm-g1 ... docker logs -f CONTAINER`, [[robot-pc2]]). Needs from a robot session: the container's name, and whether HIM keeps its logs there.
- Live run progress on the diagnostics row: time in, distance to the goal, closest approach so far.
- A topic inspector: which ROS topics a run has and their rates, from a reader beside `view_feed.py`.
- A detector page: detections against the sim's true people (`mover_start` events), for misses and false candidates.
- Speech beside the behavior loop: show `listen.py` and `talk.py` turns against the loop's talk handshake ([[talking]]).
- Compare with one shared timeline, so two runs scrub together.
- On Home's tiles: a mini map on the simulation tile (robot, goal, people from the view frames), speech's last heard and said line, small trend lines (late states over the last minute).
- A latest-run tile with its outcome and Timing and Replay; batch progress while `course.py` runs several ("3 of 8").
- Browser notifications when HIM stops a run, a run ends, or this computer turns "may stop".
- Runs: trends across runs (scores by world and commit), a run scorecard page, an event timeline, Compare's measures side by side, run notes (good, bad, baseline).
- A log viewer for `sim.log` and `policy.log`; a short history of this computer's memory.
