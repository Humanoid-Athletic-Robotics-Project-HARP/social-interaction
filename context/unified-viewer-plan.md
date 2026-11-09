---
name: unified-viewer-plan
description: Read when implementing the agreed unified viewer scope, adapters or validation
metadata:
  type: reference
  updated: 2026-11-09T05:16:19Z
  covers: []
---

# Unified viewer research and implementation plan

Research date: 2026-11-08. This document proposes an implementation. It does not describe an implemented unified viewer.

## Recommendation

Build one web application with a Home page and individual full-window service views outside the robot control image. Its server collects independently configured feeds from PC2, a laptop, our simulation output, or saved recordings. The browser uses that server as its entry point. Each panel shows offline when its required feed is absent, without preventing other panels from working.

Keep HIM's simulator and safety harness external and unchanged. Embed its existing hosted viewer through a local iframe as a separate Home service. Our simulation diagnostics remain a different feed and Rerun presentation.

## Existing code to reuse

| Component | Current data path | Reusable presentation | Integration work |
| --- | --- | --- | --- |
| `testground/lidar/lidar_viewer.py` | Livox UDP packets, binary point snapshots and JSON calibration over HTTP | Three.js lidar scene, blind regions, floor calibration and distance picking | Separate presentation from device ownership and provide source freshness |
| `testground/camera/camera_viewer.py` | V4L2 depth, Unitree videohub colour, JPEG and JSON HTTP endpoints | Colour, depth, top-down map, point measurements | Adapt image and measurement endpoints, retain independent colour/depth status |
| `testground/hybrid/hybrid_viewer.py` | Direct sensors or ROS 2 depth and ROS/Unitree DDS lidar | Combined 3D scene and existing Lidar/Camera/Hybrid tab shell | Reuse shared acquisition and processing; separate it from the viewer server |
| `speech/web.py`, `speech/web.html` | Child-process JSON output, server-sent events, HTTP commands | Pipeline stages, audio levels, turn progress, conversation, timings and logs | Separate observation from session ownership; adapt status/events and optional existing commands |
| `submission/harp/view_feed.py`, `submission/lidar_view.py` | ROS/state to JSON stdout, host Docker log reader, Rerun | Lidar, odometry/truth comparison, people, local costmap, plan, goals, events and timing | Reuse decoding and Rerun logging; expose transport and recording health |
| `testground/lidar/view.py` | Saved captures to native Rerun or `.rrd` output | Cloud timeline, lidar IMU and robot low-state plots | Display saved `.rrd` recordings in the embedded Rerun view |

Offline reports and plots such as `report.py`, `blind_map.py`, `self_filter_check.py` and `visor_model.py` remain analysis tools. Their exported results can be opened separately. Converting them into new live visualizations is additional scope.

## Constraints confirmed in the repository

- HIM's simulation container has no network. Our current solution reads our JSON stdout with `docker logs -f`; completed runs keep `view.jsonl` and `events.jsonl`. Retain this route rather than changing HIM networking.
- `entrypoint.sh` starts the current view feed only for simulation world runs. Plain planned moves do not automatically provide the same diagnostics.
- Simulated camera depth is `/camera/depth/points`, a sparse `PointCloud2`, not the real camera's depth `Image`. The current view feed does not export that camera cloud. It must be added to our feed to make simulated depth available outside the container. Simulated RGB is unavailable and should show offline.
- The hybrid viewer already shares one sensor acquisition across three views. Independent readers must not compete for the lidar UDP port or depth device. Prefer existing topic publishers when available.
- Our ROS adapter must use the publishers' domain, frames and compatible QoS. The existing hybrid adapter uses reliable depth delivery because fragmented images suffered losses with best effort. Unitree DDS and HARP ROS are distinct configurations.
- PC2 runs Python 3.8; laptop speech uses a Python 3.12 environment. The hub must start without importing speech models, ROS, camera drivers, MuJoCo, or Rerun. Optional adapters run in their appropriate environments.
- Sensor pages use Three.js 0.160.0 from a CDN today. Offline operation requires a pinned local dependency with its license and integrity recorded under the project's dependency rules.
- The current Rerun launcher hardcodes browser-facing loopback addresses and fixed ports. Remote viewing requires reachable advertised URLs or a verified proxy path, plus configurable ports.
- Existing sensors have some two-second freshness checks. Transport failure can still leave an old rendered frame visible. The unified viewer needs its own explicit stale-data behavior.

## Proposed viewer

Home uses the same compact toolbar as the individual views, without an introductory banner. Home lists each configured source and its views as full-width rectangular rows, including offline services. Larger names and right-aligned Open view actions make each row prominent; rows fill spare laptop screen height. Available viewers sort first dynamically. Unreachable services have a gray, disabled Open view action. Failure text is subdued and truncated, with the full reason on hover. Each available button opens one view in the full window below a Home toolbar. Available views are HIM simulator, Lidar, Camera, Hybrid, Navigation and diagnostics, Speech and configured recording replays. Home shows the machine, source label, status and failure reason. Each view preserves the existing display features. Navigation and diagnostics embeds Rerun. Direct links and browser back/forward work. Multiple browser pages can compare sources while the hub shares its collectors.

Collect all configured sources concurrently. There is no exclusive sim, robot or local mode. The required initial source combinations are:

| Source | Location | Connection |
| --- | --- | --- |
| Our simulation diagnostics | Docker container on the MacBook | Host adapter follows our JSON container logs |
| Listening and speech | Directly on the MacBook, outside Docker and the submission | Speech service status and event endpoints |
| Real lidar and camera | Directly on Jetson PC2 | Sensor service endpoints or a local topic adapter |
| Listening and speech on PC2 | Directly on Jetson PC2 when deployed there | Independently configured speech service |

All sources may be active together. Speech location is a deployment choice, not a requirement to run inside the submission. If MacBook and PC2 speech services both run, retain them as separate sessions.

Identify streams by source ID, session ID and feed kind, not just feed kind. Simulated lidar and PC2 lidar must coexist without overwriting each other. Keep collecting and monitoring sources when the user opens a view or returns Home. Each view belongs to one source; multiple browser pages can show sources simultaneously. Navigation affects presentation, not which producers are connected.

Do not silently combine different runs, robots or coordinate systems. Hybrid requires compatible frames, transforms and timestamps from the selected source group; otherwise show the available individual feeds with a reason the overlay cannot be formed. Show source and machine labels beside live data and offline placeholders.

Starting the viewer starts no sensors, microphone, LLM or robot motion. Existing speech Start/Stop/type controls can remain an explicit optional connection to the speech service that owns those processes. A remote page must label which machine supplies audio. Leaving a view does not stop its producer.

## Collection and transport

Use one lightweight Python server and plain HTML/CSS/JavaScript, following the existing code. Start with standard-library HTTP and adapters for the protocols already implemented. A server on a laptop can pull from a PC2 sensor endpoint and a local speech endpoint. A server on PC2 can collect local sensors and a configured laptop speech endpoint. An explicit forwarding adapter can push updates where only the producer can initiate the connection; do not require a message broker or automatic discovery.

The browser connects to the hub's origin. The server connects only to configured source addresses, with timeouts, bounded queues and reconnect backoff. Each adapter fails independently. Keep heavy sensor conversion beside its producer. Send JPEGs and bounded binary point snapshots through HTTP; use one SSE connection per page for status and small events. Do not place images or full point clouds in the event history.

Retain the existing payload encodings behind adapters initially. Standardize metadata: schema version, source ID, session/run ID, feed kind, sequence, capture timestamp and clock domain, coordinate frame and units where applicable. Record receiver arrival with a monotonic clock. Network arrival time detects failure without assuming the laptop and robot clocks agree; it does not establish sensor synchronization.

Sensor streams keep the latest sample. Speech and diagnostic events use a bounded history with IDs and explicit gap handling on reconnect. Reset session state on producer restart. Closed views suspend rendering and high-rate polling while central status continues. Multiple browser clients share collectors instead of multiplying hardware readers.

## Offline behavior

Every configured service remains available as an offline view when its feed is absent. Distinguish no source, waiting for first data, live, stale/offline, error and replay. Show the last-update age and a short reason. A connected producer that is waiting for a person to speak is idle, not offline: it needs a heartbeat or session status separate from speech events.

Judge each feed by actual data freshness. A responsive HTTP server does not prove a depth camera is producing images. Start continuous sensor thresholds near the existing two seconds, configurable by feed; use separate transport/session heartbeats for naturally quiet event feeds. Clear or visibly shade stale geometry and images. Preserve conversation history with an offline banner. Empty fresh point clouds remain live.

Replay is explicitly labelled and uses recording time. Pausing it must not make it look like a disconnected live source. Independent recordings do not imply a shared timeline.

## Alternatives researched

- A page containing unrelated remote iframes is a useful integration experiment, but alone cannot provide trustworthy per-feed health or common source selection. It also leaves loopback URLs, offline assets and cross-origin access unresolved.
- Rerun supports iframe embedding and a JavaScript package. Reuse its viewer rather than rebuilding its spatial scene, plots and timeline. Keep the viewer version matched to the pinned `rerun-sdk==0.41.0`. Current online documentation describes a newer version; test the installed version and local assets before choosing the exact embedding/proxy mechanism.
- Rosbridge exposes ROS topics, services and actions through a WebSocket protocol. It would introduce another runtime dependency and would not solve speech, saved captures or the network-isolated simulation path. Existing adapters are the smaller first implementation. A deployment using Foxy would also need a compatible pinned release rather than assuming current ROS documentation applies.
- SSE fits the small one-way status/event channel already used by speech. Browsers impose a low per-origin HTTP/1 connection limit, so use a shared event connection rather than one per panel. Binary snapshots remain separate HTTP requests.

## Implementation sequence

1. Build the shared shell, source configuration and health registry. Demonstrate startup with zero producers, independent feeds, source restarts and reconnects using fixtures.
2. Connect existing sensor HTTP endpoints through adapters. Reuse the three sensor presentations and their processing. Preserve measurements, independent colour/depth status and partial hybrid operation. Install pinned local Three.js assets through the project-approved dependency mechanism.
3. Connect speech status and events. Reuse its display, retain process ownership in the speech environment and expose its existing commands only for an explicitly enabled source. Ensure the hub itself needs no audio or model packages.
4. Embed Rerun and adapt the existing simulation log reader and saved recordings. Test browser access from another machine. Verify configurable URLs, correct version, bounded memory and producer completion status. Support existing capture `.rrd` exports.
5. Add the real-robot ROS feed adapter and export simulated depth through our existing log feed. Supply transforms and source metadata, and support point-cloud-only camera depth without claiming RGB or depth-image measurements are available.
6. Validate combined laptop/robot sources, offline-network assets, load and recovery. Update usage docs and the affected memories with the implementation, plus the architecture guide when processes or connections change.

Keep each step reviewable. Preserve working launch commands until their replacement has feature parity. No changes to HIM, custom ROS messages, general robot command API, new detector overlays or new autonomy features are required for this scope.

## Validation

- Start with no hardware, Docker, speech packages or ROS installed. Home and offline service views must render without startup failure.
- Start, stop and restart each producer independently. Test a reachable server with frozen sensor data, empty fresh clouds, quiet speech, partial colour/depth availability and changed session IDs.
- Exercise two clients, closed views, slow consumers and a burst of events. Confirm bounded memory and no growing frame backlog.
- Run MacBook Docker simulation, native MacBook speech and PC2 sensors concurrently. Stop and restart each separately and verify the others continue. Test simulated and real lidar together, plus distinct MacBook and PC2 speech sessions, without data or history crossing sources. Repeat with fixture producers when hardware is unavailable and distinguish fixture checks from real deployment validation.
- Verify a saved run and `.rrd` capture with replay time and explicit completion/paused state.
- Check a second machine's browser so loopback assumptions and unreachable source addresses are caught.
- Test local assets with internet unavailable, plus desktop and phone layouts.
- Compare one representative simulation course with and without collection. Check policy state age, missed deadlines, CPU and memory; only run broader courses if feed changes affect behavior or results.
- Real sensor validation uses the existing operator procedure and shared topic access. Do not start another device owner as part of UI startup.

## Evidence and open verification

Repository inspection confirms the data paths and constraints above. No robot session, simulation, performance benchmark or embedding prototype was run during this research. The first integration checks must establish Rerun 0.41 embedding/proxy behavior, the optional HTTP forwarding mechanism, and the point-cloud-only depth presentation. Two-way internet hosting and a synchronized cross-machine replay system are outside this local-network viewer proposal.

Primary references:

- [Rerun embedding documentation](https://rerun.io/docs/howto/integrations/embed-web). Iframe and JavaScript options, SDK/viewer version matching.
- [MDN server-sent events](https://developer.mozilla.org/en-US/docs/Web/API/Server-sent_events/Using_server-sent_events). Reconnect behavior, event IDs and HTTP/1 connection limits.
- [Rosbridge protocol](https://github.com/RobotWebTools/rosbridge_suite/blob/ros2/ROSBRIDGE_PROTOCOL.md). Topic, service and action operations and binary encoding options.
- [Python 3.8 HTTP server documentation](https://docs.python.org/3.8/library/http.server.html). Availability of `ThreadingHTTPServer`; appropriate here as a local development service, not a public deployment server.
- [Three.js r160 license](https://raw.githubusercontent.com/mrdoob/three.js/r160/LICENSE). MIT license for the existing sensor rendering dependency.

## HIM viewer embedding feasibility

HIM's local `containers/sim/live.py` serves the viewer without framing-blocking headers. Its `view.py` page polls HTTP `/frames` and posts `/start`; it does not use a WebSocket for live frames. A local iframe can display the existing hosted page. Remote dashboard access would need a proxy for its page, assets and root-relative endpoints. This is feasible without editing or copying HIM. The implemented `him` source embeds the existing local viewer directly, registers a Home button even while offline, and checks availability without starting or controlling HIM. Remote proxying is outside the requested local-only scope.
