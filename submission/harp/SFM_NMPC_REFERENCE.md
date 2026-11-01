# Predictive controller reference

`predictive_mpc.py` is an independent, compact NMPC implementation informed by
[PIC4SeR/sfm-nmpc](https://github.com/PIC4SeR/sfm-nmpc/tree/13203ca6bed824f4dd5538d1d665c7014fb5891e),
pinned at `13203ca6bed824f4dd5538d1d665c7014fb5891e`.

The references are its [velocity-block objective](https://github.com/PIC4SeR/sfm-nmpc/blob/13203ca6bed824f4dd5538d1d665c7014fb5891e/docs/mpc_critics_methodology.md)
and [nonlinear planar rollout](https://github.com/PIC4SeR/sfm-nmpc/blob/13203ca6bed824f4dd5538d1d665c7014fb5891e/sfm_nmpc/include/sfm_nmpc/update_state.hpp).
This implementation uses body-frame forward, sideways and angular velocity,
measured initial velocity, acceleration limits, and constant-velocity object
tracks. It does not assume objects react socially to the robot.

The controller evaluates three velocity blocks over four seconds. Candidates are
every direction at three speeds (held, merging back into the route, or stepping
then stopping), turning curves, the previous request (warm start) and standing.
All of them, standing included, are ranked by one soft-constrained cost:

- task terms: actual rollout progress toward the goal, request tracking,
  continuity, a committed passing side and a penalty for retreating;
- predicted entry into an observed body, weighted by its deepest discounted
  penetration rather than its duration (walking toward an actor to shorten an
  encounter is not safer), plus a small integral so an existing overlap is left;
- intrusion into the comfort margins (measurement margin, track uncertainty and a
  forecast spread that grows with time and with the object's own speed), as a
  moderate, tradeable cost;
- a fixed preference for a path that is certifiably clear when one exists.

Intrusion is discounted with a 2 s time constant; in a crowd sweep a longer one
over-weighted speculative far-future overlaps and the robot dithered. This single
ranking replaces an earlier cascade (certified, margin recovery, brake then stop,
least intrusion) whose last-resort branch had become the normal case in crowds,
and whose unconditional zero once froze the robot in front of non-yielding actors.
The previous request is kept unless another is clearly better, so a delayed
walker is not asked to alternate sides each cycle. SLSQP refines a certifiably
clear choice within a time budget; a failed or timed-out solve keeps the checked
sample. Only the first request is executed before replanning. Results report
`margin_recovery` or `least_intrusion` and raise an alert when the chosen path is
not certifiably clear; `blocked` means standing ranked best.

Safety terms are robust to the walker's response. Native G1 runs fit forward
delay ≈0.25 s and gain 0.84, lateral gain 0.60, but the lateral response varies
with gait phase (0.10 m/s rms error on a 0.24 m/s response; one sidestep began
0.7 s late and ended in contact). Every candidate is therefore rolled out with the
nominal model and with a slow-sidestep model (`Config.lateral_pessimism`, 0.6 of
the lateral gain); clearances take the smaller and risk the larger value. A
sidestep that might come late then also cuts closing speed, which is reliable.
To bound planning time, all candidates are ranked with the nominal model and only
the best 24 (plus standing and the previous request) are re-ranked robustly.

The request ramp uses the previous request at every predicted control update,
matching execution during reversals. If stopping predicts a conflict within 2.5 s,
an escape can send its bounded velocity target immediately; urgency comes from
approaching objects or an actual body overlap, never from mere proximity to a wall. Physical rollout
starts from measured velocity and includes acceleration, response delay and
configurable forward/lateral tracking gains.

Moving objects get exact swept-disc checks (closest approach between samples) with
the body envelope, measurement margin and a braking tail. Wall tiles and remembered
surfaces, plus (as a second group with forecast growth) tracks slower than
0.08 m/s, are folded once per cycle into a 5 cm distance field with body and
body-plus-uncertainty layers, so each sampled path
costs a lookup instead of a check against every wall tile; objects that cannot
reach any envelope before the forecast ends are pruned first. Clearance
certification uses the same envelopes; an existing comfort-margin intrusion may
decay over 2 s (a ramped walker's launch) while the observed body stays a hard limit. All of
this is a risk ranking under a constant-velocity forecast with bounded inflation,
not a reachable-set collision guarantee.

The native trial supplies route requests from measured tracks using a bounded
arrival-time route grid. Static obstacles block cells; walking tracks add temporal
costs, since they will not occupy their current position forever. Nav2 retains this
route-planning responsibility in the Foxy image. No simulator truth reaches the
tracker, route planner or MPC; truth is used by the simulated pedestrians and scorer.

This is not the upstream ROS 2 Humble/Ceres plugin, a Foxy port of it, or a
runtime dependency on that repository. No upstream source code is copied.
The original repository distributes its implementation under GPL-3.0.
