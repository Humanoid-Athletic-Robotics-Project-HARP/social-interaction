# Predictive controller reference

`predictive_mpc.py` is an independent, compact NMPC implementation informed by
[PIC4SeR/sfm-nmpc](https://github.com/PIC4SeR/sfm-nmpc/tree/13203ca6bed824f4dd5538d1d665c7014fb5891e),
pinned at `13203ca6bed824f4dd5538d1d665c7014fb5891e`.

The references are its [velocity-block objective](https://github.com/PIC4SeR/sfm-nmpc/blob/13203ca6bed824f4dd5538d1d665c7014fb5891e/docs/mpc_critics_methodology.md)
and [nonlinear planar rollout](https://github.com/PIC4SeR/sfm-nmpc/blob/13203ca6bed824f4dd5538d1d665c7014fb5891e/sfm_nmpc/include/sfm_nmpc/update_state.hpp).
This implementation uses body-frame forward, sideways and angular velocity,
measured initial velocity, acceleration limits, and constant-velocity object
tracks. It does not assume objects react socially to the robot.

The controller evaluates three velocity blocks over four seconds. Its objective
favors actual rollout progress toward the goal, request tracking, stable passing
direction, usable clearance, and motion
when a safe passing trajectory exists. A stop competes with passing rather than
winning automatically. SLSQP refines checked samples; a failed or timed-out solve
keeps a checked sample. Only the first request is executed before replanning.
The request ramp uses the previous request at every predicted control update,
matching execution during reversals. If stopping predicts a conflict within2.5 s,
a checked escape can send its bounded velocity target immediately. Physical
rollout still starts from measured velocity and includes acceleration, response
delay and configurable forward/lateral tracking gains.

Swept-disc checks cover the body envelope, measurement margin and a braking tail.
They use a finite forecast; they do not reserve a pose against an infinite line of
possible pedestrian motion. If a long continuation is infeasible, a recovery checks
one control period followed by braking through the full stopping time. If that is
also infeasible, standing is not assumed safe: it is scored with every sampled
trajectory by time-discounted, integrated intrusion into the body/observed-shape
envelope (dominant) and the comfort margins. The least intrusive trajectory is
executed with an alert; zero plus an alert is chosen only when no movement reduces
predicted intrusion by at least 10%. Previously this case always returned zero,
which froze the robot in front of non-yielding actors that then walked into it.
This fallback is a risk ranking under the same constant-velocity forecast, not a
collision guarantee. An existing violation of the
extra forecast margin may recover over a bounded interval, with an alert, while
body/observed-shape separation remains hard. New conflicts receive no such
relaxation; a stationary margin violation cannot remain indefinitely.
Spatial forecast uncertainty is
reported in tracks; its hard collision inflation is bounded by Config, so this is
a bounded-risk model rather than a reachable-set collision guarantee.

The native trial supplies route requests from measured tracks using a bounded
arrival-time route grid. Static obstacles block cells; walking tracks add temporal
costs, since they will not occupy their current position forever. Nav2 retains this
route-planning responsibility in the Foxy image. No simulator truth reaches the
tracker, route planner or MPC; truth is used by the simulated pedestrians and scorer.

This is not the upstream ROS 2 Humble/Ceres plugin, a Foxy port of it, or a
runtime dependency on that repository. No upstream source code is copied.
The original repository distributes its implementation under GPL-3.0.
