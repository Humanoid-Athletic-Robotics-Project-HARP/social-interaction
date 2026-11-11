---
name: self-filter
description: Read before changing how the robot's own body is cut out of the lidar (harp/self_filter.py), its margin, or the radii other code still uses to skip the body
metadata:
  type: project
  updated: 2026-11-11T18:46:42Z
  covers: [submission/harp/self_filter.py, submission/harp/lidar.py, submission/harp/lidar_relay.py, submission/tests/test_self_filter.py, submission/tests/test_lidar_relay.py, testground/lidar/self_filter_check.py]
---

**What it does:** `harp/self_filter.py` drops every lidar point inside the robot's body before the cloud is published, so Nav2, `approach_stop`, the predictive tracker and the Rerun view all get a cloud without it. Each robot geom in HIM's G1 model (35 meshes and 33 collision shapes, rubber hands included) is a box around it in its own frame, grown by `MARGIN` (6 cm). Forward kinematics from the 29 joint angles alone places the boxes in `livox_frame` (the lidar is fixed to `torso_link`), so the boxes follow the arms: a wave or any gesture.

**Where it runs:** in the cloud producers, not as its own node. `harp/lidar.py` (sim) filters each 20 ms slice with that state's joints. `harp/lidar_relay.py` (robot) reads `/joint_states` and checks each cloud against up to 6 joint states (`MAX_POSES`, evenly spread from the newest to the oldest, 40 ms apart at 50 Hz) from its last 0.2 s (`JOINT_WINDOW`): a scan is smeared over 100 ms and a moving arm with it, and the cloud reaches the relay up to about 100 ms after its scan ends (`DELAY`, the relay's estimate for Unitree's `lidar_driver`; unmeasured; raw UDP packets arrive in about 1 ms, [[g1-lidar-measured]]). With 0.15 s, a cloud 100 ms late missed its scan's older half, so a fast arm move left points beside the robot. Measure the driver's delay on the robot and set `DELAY` from it. With no `/joint_states` for 0.5 s the relay publishes whole clouds and says so once.

**Checked:**
- Rays cast like the sim lidar's (`tests/test_self_filter.py`): stand pose, wave hand up, both arms reaching out: no robot point gets through, no floor point or point of a post 0.6 m away is dropped. A scan over a fast raise needs all its poses.
- Real robot (`testground/lidar/self_filter_check.py`, in the image, on the two captures with `rt/lowstate`: 2026-11-01 `lidar/standing-60s`, 11-02 `lidar/visor-off-standing-60s`; different arm poses): about 250 returns a cloud within 0.5 m before, at most 0.03 after at margins of 3 cm or more (2 cm lets about 6 through). The about 35 left within 0.7 m are all straight behind at lidar height, 0.6 m out: the stand, not the robot. Nothing beyond 0.7 m is dropped.
- The margin also absorbs a model offset: real shoulder returns are 2.5–9.5 cm nearer than the model's surface along the same rays, because the lidar measures from its optical centre about 21 mm below the URDF frame `lidar_mount` uses ([[g1-face-visor]]). HIM's G1 model itself is Unitree's: its joints, ranges, mesh files, hands and mounts match `unitree_ros` `g1_29dof_rev_1_0.urdf` exactly (checked 2026-11-06). Keep `MARGIN` at least 4 cm unless the mount moves to the optical centre.
- Sim course (`runs/2026-11-06_22-34-23Z_sim_one_box_self-filter`): passes as before (24.1 s, closest 0.61 m, 0 contacts or recoveries, 2 late states as on main); the published cloud's 10 cm voxels within 0.7 m of the lidar went from 4.2 a frame (`runs/2026-11-02_06-33-17Z_sim_one_box_visor-off`) to 0. No sim run moves the arms with the lidar on yet (the wave task runs `harp_node.py` alone).
- Cost on a Mac: a 20,000-point scan in about 7–15 ms (more with the arms out, where more points fall near the body). Six poses instead of four: 2.0 → 2.9 ms with 250 points near the body, 5.3 → 7.7 ms with 1,000 (Mac, HIM's G1 model, 2026-11-11); PC2 unmeasured.

**Still in place:** `approach_stop.BODY` (0.45 m) and `predictive_avoidance.SELF_RADIUS` (0.45 m), which now only hide real things near the robot's sides, and Nav2's footprint clearing. The depth camera's cloud isn't filtered (it rarely sees the arms).

**Next** (as of 2026-11-08):
- Shrink the two radii toward the footprint (0.4 m), comparing `crossing`, `oncoming`, `one_box` and `building` before and after ([[course-runs]]): they change when the robot stands.
- Move `lidar_mount.POSITION` to the optical centre ([[g1-face-visor]]), rerun `self_filter_check.py`, then consider a 3–4 cm `MARGIN`.
- A sim run with the arms moving and the lidar on, once the behavior loop triggers the wave.
- On the robot: the relay through HARP's harness (timing, PC2's CPU); captures with the arms moving and with an object held near the hand, shoulder and hip, checked with `self_filter_check.py` ([[robot-reference-data]]).

**Not modeled:** anything the robot carries or wears beyond HIM's model (a harness strap, cables): it stays in the cloud. Unitree's issue `unitree_ros#135` reports the URDF's arms closer to the torso than the real ones; the shoulder fit above doesn't show it.

**Rejected:** one fixed box around the stand pose (commit `93058bd`, on the old ROS 2 stack, kept only as the local branch `self-filter-ros2` on the maintainer's Mac): it can't follow the arms.
