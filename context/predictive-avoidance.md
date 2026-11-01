---
name: predictive-avoidance
description: Read before running or changing generic moving-object tracking, predictive avoidance or its alerts
metadata:
  type: reference
  updated: 2026-11-01T06:40:22Z
  covers: [submission/harp/obstacle_tracker.py, submission/harp/predictive_mpc.py, submission/harp/predictive_avoidance.py, submission/harp/avoidance_config.py, submission/harp/SFM_NMPC_REFERENCE.md, submission/test_predictive_sim.py, submission/view_predictive_sim.py, submission/tests/test_predictive*.py, submission/tests/test_obstacle_tracker.py, submission/tests/test_avoidance_config.py]
---

**Selection:** `course.py --avoidance predictive` adds `avoidance` to task JSON. `entrypoint.sh` selects one `/cmd_vel_nav` → `/cmd_vel` filter; default is `approach_stop`. Predictive mode rejects hardware, missing-world and planned-move combinations. Nav2 owns route planning in the image; HIM owns motors.

**Observation:** stamped TF compensates robot/head motion. Input receipt and sensor timestamps must remain fresh. Floor/self filtering precedes voxel grouping, joint Hungarian assignment and motion-seeded separation of merged returns. Every observation remains occupied; capacity overflow stops motion. Tracks report ID, position, velocity, extent, uncertainty and last observation on `/harp/obstacle_tracks`.

**Occlusion:** tracks coast for 2.5 s. Nearby expired occupancy is retained as measured surface cells rather than stopping all directions. Clearance requires multiple rays beyond the former surface, footprint clearing, or transfer into enclosing fresh occupancy. Fresh background receipt alone does not clear an obstacle. This is occupancy memory, not reliable identity through occlusion; ID changes and merging remain limitations.

**Planning:** three velocity blocks over a finite forecast, measured initial motion, delayed acceleration-limited rollout, sampled passing/turning paths and bounded SLSQP refinement. Request ramping uses the preceding request, which lets GR00T build walking speed during gait launch. Actual rollout progress toward the goal, request tracking and continuity compete with stopping; blocked continuations can use a checked short action plus full braking. Imminent forecast conflicts have priority over deeper late overlaps. Existing extra-margin violations can recover over a bounded interval with an alert; body/observed-shape separation stays hard, and new conflicts keep their full margin. `harp/SFM_NMPC_REFERENCE.md` records the independent mathematical adaptation, not an upstream plugin port.

**Model limits:** constant-velocity prediction does not assume human cooperation. Collision inflation caps track uncertainty (`Config.max_collision_uncertainty`); this accepts bounded forecast risk and is not a reachable-set guarantee. Forward and lateral response gains are distinct. Native G1 measurements inform the defaults; other walkers require their own measurements. Zero plus alert cannot prevent an incoming object hitting a stopped robot.

**Integration:** 5 Hz lower-priority filter, 0.15 s planning deadline, zero on stale/malformed input or planning failure. Route success idles; failure latches zero. Goal coordinates bound the predicted approach rather than extrapolating travel through the target. `/harp/obstacle_alert` is a JSON signal, not an installed horn. Existing recorder and planned moves remain separate.

**Checks:** 109 local tests pass with native Jazzy/Python 3.12, including live ROS DDS input-loss/recovery and route failure, merged-track coverage, measured-velocity rollout, braking tail and positive clearance evidence. Four planar MuJoCo scenarios pass; these use ideal odometry and circular footprints. [[native-crowd]] records actual GR00T/G1 results and the remaining crowd failure boundary. Full Docker/Foxy/Nav2/HIM validation requires the external harness and remains unverified on this Mac.
