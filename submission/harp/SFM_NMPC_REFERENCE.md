# Predictive controller reference

`predictive_mpc.py` is an independent, compact NMPC implementation informed by
[PIC4SeR/sfm-nmpc](https://github.com/PIC4SeR/sfm-nmpc/tree/13203ca6bed824f4dd5538d1d665c7014fb5891e),
pinned at `13203ca6bed824f4dd5538d1d665c7014fb5891e`.

The references are its [velocity-block objective](https://github.com/PIC4SeR/sfm-nmpc/blob/13203ca6bed824f4dd5538d1d665c7014fb5891e/docs/mpc_critics_methodology.md)
and [nonlinear planar rollout](https://github.com/PIC4SeR/sfm-nmpc/blob/13203ca6bed824f4dd5538d1d665c7014fb5891e/sfm_nmpc/include/sfm_nmpc/update_state.hpp).
This implementation uses body-frame forward, sideways and angular velocity,
measured initial velocity, acceleration limits, and constant-velocity object
tracks. It does not assume objects react socially to the robot.

The controller first checks the nominal command and slower commands, then
checks escape seeds around 360 degrees. SLSQP refines a feasible escape using
one held velocity block over a four-second horizon. Its objective penalizes
speed, departure from the route request, and changes in escape direction.
Holding the block prevents repeatedly postponing the escape to a later block.

Every accepted escape passes swept-disc collision checks, including response
delay, uncertainty expansion and a braking tail. The final stopped position
must also clear each object's continued constant-velocity ray. This prevents
backpedalling that only delays impact beyond the finite horizon. Failed or
expired optimization retains an already checked seed; no feasible seed means
zero plus an alert. These checks depend on the configured model and observed
geometry; they are not a hardware collision guarantee.

This is not the upstream ROS 2 Humble/Ceres plugin, a Foxy port of it, or a
runtime dependency on that repository. No upstream source code is copied.
The original repository distributes its implementation under GPL-3.0.
