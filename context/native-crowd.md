---
name: native-crowd
description: Read before running or tuning native walking G1 crowd trials or interpreting their results
metadata:
  type: reference
  updated: 2026-11-01T08:46:29Z
  covers: [submission/native_g1_sim.py, submission/native_g1_sim.sh, submission/native_crowd_world.py, submission/harp/crowd_motion.py, submission/harp/local_route.py, submission/tests/test_native_g1_sim.py, submission/tests/test_crowd_navigation.py, context/native-g1-result.json]
---

**Runtime:** the retained native Jazzy environment and Unitree model beside the main checkout are required. `native_g1_sim.sh` locates the environment through git's common directory; `HARP_NATIVE_PYTHON` can select another native environment. Historical native simulator/walker source is pinned at `9ee9255`, extracted into ignored `.build/native-g1`. Actual G1 joint dynamics and GR00T ONNX locomotion run on loopback SDK184/ROS185. One trial owns the domains. Parent cleanup terminates its owned child processes. On macOS the GUI restart must preserve `native_crowd_world`; restarting the historical `harp_sim.run_sim` loses the continuous-crowd/LiDAR overrides. The corrected visible launch emits seven-actor `/sim/crowd` frames.

**Control:** native route requests use the current measured LiDAR tracks and a bounded arrival-time grid. Fixed obstacles block cells; moving people add temporal cost. The shared predictive ROS filter scores actual rollout progress toward the goal and produces `/cmd_vel`; native trials do not run Nav2/HIM. The submission image keeps Nav2 as route owner. Simulator truth reaches the pedestrian model and scorer only.

**Walking response:** native probes measured about 0.21 m/s sideways travel for a 0.35 m/s request; forward response was faster. Separate forward/lateral rollout gains represent this asymmetry. Native default request cap is 0.5 m/s with sideways capped at 0.4; these are simulator measurements, not physical G1 limits or calibration.

**Crowd:** seven collidable capsule/head bodies, seeded speed/heading changes, acceleration limits, brief pauses, reciprocal pedestrian avoidance and repeated routes. Mixed mode responds to the G1; adversarial mode has three agents that ignore it. Baseline walking speeds are drawn from 0.35–0.7 m/s with variation; `--flow-speed` scales them. This is synthetic kinematic crowd behavior, not articulated human gait or validated human data. The default goal (6,0) lies beyond circulation; (4,0) remains available as a blocked-goal stress case.

**Evidence:** `native-g1-result.json` stores scored results and source hashes. Each local run also saves immutable source snapshots, world, commands, measured tracks, actual human motion and G1 trajectory. Verdicts require zero contacts, arrival, stable G1, all movers observed moving and valid pedestrian separation. Continuous crowd cases additionally require early first-metre progress, at least 1.5 m forward progress with moving people nearby, and no standstill over 8 s. Thus waiting outside until actors finish cannot pass.

**Current boundary:** with least-intrusion fallback (commit `94c78f6`), suite `runs/crowd-least-intrusion-20261101T083948Z` passes 7/7 with zero contacts: adversarial seed2 three times (26.4, 26.8, 26.6 s), adversarial seed3 (28.1 s), mixed seed1 (35.3 s), mixed seed2 with pillars (43.7 s) and 1.5× flow seed4 (32.3 s). Longest standstill was at most 2.70 s. The previous suite passed 2/5; every traced contact began with the planner frozen in `blocked` while a non-yielding actor walked in. Seven passing synthetic runs do not establish arbitrary-crowd robustness; more seeds, denser flow and occupied goals remain untested.

**Boundary:** [[predictive-avoidance]] describes the prediction and empirical uncertainty limits. Native success does not establish Foxy/Nav2/HIM integration or physical deployment. Repeat seeds, flow speeds and occupied-goal cases before broad claims; a few synthetic passes do not establish arbitrary-crowd robustness.
