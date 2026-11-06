---
name: self-filter
description: Read before changing how the robot's own body is cut out of the lidar (harp/self_filter.py), its margin, or the radii other code still uses to skip the body
metadata:
  type: project
  updated: 2026-11-06T23:41:09Z
  covers: [submission/harp/self_filter.py, submission/harp/lidar.py, submission/harp/lidar_relay.py, submission/tests/test_self_filter.py, submission/tests/test_lidar_relay.py, testground/lidar/self_filter_check.py]
---

**What it does:** `harp/self_filter.py` drops every lidar point inside the robot's body before the cloud is published, so Nav2, `approach_stop`, the predictive tracker and the Rerun view all get a cloud without it. Each robot geom in HIM's G1 model (35 meshes and 33 collision shapes, rubber hands included) is a box around it in its own frame, grown by `MARGIN` (6 cm). Forward kinematics from the 29 joint angles alone places the boxes in `livox_frame` (the lidar is fixed to `torso_link`), so the boxes follow the arms: a wave or any gesture.

**Where it runs:** in the cloud producers, not as its own node. `harp/lidar.py` (sim) filters each 20 ms slice with that state's joints. `harp/lidar_relay.py` (robot) reads `/joint_states` and checks each cloud against up to 4 joint states from its last 0.15 s, since a scan is smeared over 100 ms and a moving arm with it. With no `/joint_states` for 0.5 s the relay publishes whole clouds and says so once.

**Checked:**
- Rays cast like the sim lidar's (`tests/test_self_filter.py`): stand pose, wave hand up, both arms reaching out: no robot point gets through, no floor point or point of a post 0.6 m away is dropped. A scan over a fast raise needs all its poses.
- Real robot (`testground/lidar/self_filter_check.py`, in the image, on the two captures with `rt/lowstate`: 2026-11-01 `lidar/standing-60s`, 11-02 `lidar/visor-off-standing-60s`; different arm poses): about 250 returns a cloud within 0.5 m before, at most 0.03 after at margins of 3 cm or more (2 cm lets about 6 through). The about 35 left within 0.7 m are all straight behind at lidar height, 0.6 m out: the stand, not the robot. Nothing beyond 0.7 m is dropped.
- The margin also absorbs a model offset: real shoulder returns are 2.5–9.5 cm nearer than the model's surface along the same rays, because the lidar measures from its optical centre about 21 mm below the URDF frame `lidar_mount` uses ([[g1-face-visor]]). HIM's G1 model itself is Unitree's: its joints, ranges, mesh files, hands and mounts match `unitree_ros` `g1_29dof_rev_1_0.urdf` exactly (checked 2026-11-06). Keep `MARGIN` at least 4 cm unless the mount moves to the optical centre.
- Sim course (`runs/2026-11-06_22-34-23Z_sim_one_box_self-filter`): passes as before (24.1 s, closest 0.61 m, 0 contacts or recoveries, 2 late states as on main); the published cloud's 10 cm voxels within 0.7 m of the lidar went from 4.2 a frame (`runs/2026-11-02_06-33-17Z_sim_one_box_visor-off`) to 0. No sim run moves the arms with the lidar on yet (the wave task runs `harp_node.py` alone).
- Cost on a Mac: a 20,000-point scan in about 7–15 ms (more with the arms out, where more points fall near the body).

**Still in place, now only for the camera or as margins:** `approach_stop.BODY` (0.45 m), `predictive_avoidance.SELF_RADIUS` (0.45 m) and Nav2's footprint clearing. Shrinking the two radii would let the robot see things near its sides (a shelf edge, a hand held out) but changes when it stands: compare course runs before and after ([[course-runs]]). The depth camera's cloud isn't filtered.

**Not modeled:** anything the robot carries or wears beyond HIM's model (a harness strap, cables): it stays in the cloud. Unitree's issue `unitree_ros#135` reports the URDF's arms closer to the torso than the real ones; the shoulder fit above doesn't show it.

**Rejected:** one fixed box around the stand pose (local branch `self-filter-ros2`, on the old ROS 2 stack): it can't follow the arms.
