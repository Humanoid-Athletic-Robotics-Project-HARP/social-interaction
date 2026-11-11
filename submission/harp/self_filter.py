"""Cut the robot's own body out of the head lidar's cloud, wherever its arms are.

The lidar sees the G1's shoulders, arms and hands (on the robot as near as 0.09 m:
context/g1-lidar-measured.md). Left in, they are an obstacle that moves with the robot,
right beside it, and a raised hand (the wave, harp/wave.py) reaches past the radii that
would otherwise have to stand in for a filter. So each of the robot's geoms in HIM's G1
model (its meshes and collision capsules, the rubber hands included) becomes a box: the
smallest one around it in its own frame, grown by MARGIN. Forward kinematics from the
joint angles alone places the boxes in the lidar's frame, since the lidar is fixed to
torso_link (harp/lidar_mount.py), and every point inside one is dropped.

The cloud producers call it before publishing: harp/lidar.py in the sim, with each
slice's joints, and harp/lidar_relay.py on the robot, with /joint_states from the
scan's last JOINT_WINDOW (a scan is smeared over 100 ms, and so is a moving arm:
scan_poses). testground/lidar/self_filter_check.py checks it on a real capture.
"""

import mujoco
import numpy as np

from harp import lidar_mount

MODEL = "/harp/model/g1_robot.xml"   # HIM's G1, as harp_node.MODEL
ROOT = "pelvis"
NUM_MOTORS = 29
# m around each geom's box: about 3 sigma of the lidar's range noise (2 cm), which can
# put a body point in front of the surface it hit, along the ray.
MARGIN = 0.06
SCAN = 0.1            # s a scan is smeared over (the MID-360's 10 Hz)
DELAY = 0.1           # s at most a cloud reaches the relay after its scan ends (lidar_relay.py)
JOINT_WINDOW = SCAN + DELAY   # s of joint states a whole scan is checked against
MAX_POSES = 6         # joint states a scan is checked against at most: 40 ms apart at 50 Hz


def half_extents(model, g):
    """Centre (in the geom's frame) and half sizes of the smallest box around geom g."""
    kind, size = model.geom_type[g], model.geom_size[g]
    if kind == mujoco.mjtGeom.mjGEOM_MESH:
        mesh = model.geom_dataid[g]
        start = model.mesh_vertadr[mesh]
        vertices = model.mesh_vert[start:start + model.mesh_vertnum[mesh]]
        low, high = vertices.min(axis=0), vertices.max(axis=0)
        return (low + high) / 2, (high - low) / 2
    if kind == mujoco.mjtGeom.mjGEOM_SPHERE:
        half = (size[0],) * 3
    elif kind == mujoco.mjtGeom.mjGEOM_CAPSULE:
        half = (size[0], size[0], size[0] + size[1])
    elif kind == mujoco.mjtGeom.mjGEOM_CYLINDER:
        half = (size[0], size[0], size[1])
    elif kind in (mujoco.mjtGeom.mjGEOM_BOX, mujoco.mjtGeom.mjGEOM_ELLIPSOID):
        half = size
    else:
        raise ValueError(f"geom {g}: no box for type {kind}")
    return np.zeros(3), np.asarray(half, dtype=float)


def scan_poses(times, poses, arrival):
    """The joint angles to check a scan that arrived at `arrival` against: those of the
    joint states (times, rising, same clock) within JOINT_WINDOW before it, at most
    MAX_POSES of them evenly spread, the newest first and the oldest (the scan's start)
    last, both always included; the newest before it if none."""
    recent = [q for t, q in zip(times, poses) if arrival - JOINT_WINDOW <= t <= arrival]
    if not recent:
        before = [q for t, q in zip(times, poses) if t <= arrival]
        return before[-1:]
    if len(recent) > MAX_POSES:
        picks = sorted({round(i * (len(recent) - 1) / (MAX_POSES - 1)) for i in range(MAX_POSES)})
        recent = [recent[i] for i in picks]
    return recent[::-1]


class SelfFilter:
    def __init__(self, model_path=MODEL, margin=MARGIN):
        self.model = mujoco.MjModel.from_xml_path(str(model_path))
        self.data = mujoco.MjData(self.model)
        m = self.model
        root = m.body_rootid[m.body(ROOT).id]
        self.geoms = np.array([g for g in range(m.ngeom) if m.body_rootid[m.geom_bodyid[g]] == root])
        boxes = [half_extents(m, g) for g in self.geoms]
        self.centres = np.array([c for c, _ in boxes])
        self.half = np.array([h for _, h in boxes]) + margin
        self.torso = m.body(lidar_mount.PARENT).id
        self.mount = lidar_mount.rotation()

    def boxes(self, q):
        """Each geom's box in the lidar's frame for joint angles q: centres (G x 3) and
        axes (G x 3 x 3, the box's axes as columns)."""
        d = self.data
        d.qpos[:] = 0.0
        d.qpos[3] = 1.0             # any base pose: only the joints move the body against the lidar
        d.qpos[7:7 + NUM_MOTORS] = np.asarray(q, dtype=float)[:NUM_MOTORS]
        mujoco.mj_kinematics(self.model, d)
        torso = d.xmat[self.torso].reshape(3, 3)
        lidar = torso @ self.mount                       # the lidar's axes in the world
        origin = d.xpos[self.torso] + torso @ lidar_mount.POSITION
        axes = d.geom_xmat[self.geoms].reshape(-1, 3, 3)
        centres = d.geom_xpos[self.geoms] + np.einsum("gij,gj->gi", axes, self.centres)
        return (centres - origin) @ lidar, np.einsum("ji,gjk->gik", lidar, axes)

    def keep(self, points, poses):
        """Which points (N x 3, lidar frame) lie outside the body at every one of the joint
        angle vectors in poses (one is enough when the scan is from one instant)."""
        points = np.asarray(points, dtype=float).reshape(-1, 3)
        body = np.zeros(len(points), bool)
        for q in poses if len(points) else ():
            centres, axes = self.boxes(q)
            # Only points within reach of the body: the rest of the cloud skips the box test.
            reach = np.linalg.norm(self.half, axis=1)
            low = (centres - reach[:, None]).min(axis=0)
            high = (centres + reach[:, None]).max(axis=0)
            near = np.flatnonzero(np.all((points >= low) & (points <= high), axis=1) & ~body)
            if len(near) == 0:
                continue
            # each point in each box's own axes (N x G x 3)
            local = np.einsum("ngi,gij->ngj", points[near, None, :] - centres[None], axes)
            body[near] = np.any(np.all(np.abs(local) <= self.half[None], axis=2), axis=1)
        return ~body
