"""Simulated head depth camera (Intel RealSense D435i), cast by harp/lidar.py with the
same pose, world and movers as the lidar, so both see a person in the same place.

Like the real one (context/g1-head-camera.md): 87 x 58 deg field of view along
its optical axis, pitched 47.6 deg down on the torso (harp/camera_mount.py), so it
sees the floor from about 0.2 m to 2.7 m ahead and things on it up to about 0.8 m
high 1 m ahead; nothing nearer than 0.2 m; range noise growing with the square of
the distance (stereo depth: about 1.6 cm at 2 m); about 15% of pixels without depth.
Unlike it: a sparse even grid of rays (every 8th pixel of 848 x 480) and at the
lidar's 10 Hz, not 15 to 30.

Points are in the optical frame (z forward, x right, y down) on /camera/depth/points,
as a RealSense driver publishes them; harp/state.py publishes the frame's live pose.
"""

import math

import mujoco
import numpy as np

from harp import camera_mount

HFOV, VFOV = 87.0, 58.0          # deg
COLS, ROWS = 106, 60             # rays across and down
MIN_RANGE, MAX_RANGE = 0.2, 6.0  # m along the optical axis
NOISE = 0.004                    # m of depth noise per m^2 of depth, 1 sigma
DROPOUT = 0.15                   # share of pixels without depth (about 19% on the robot)
TOPIC = "/camera/depth/points"


class DepthCamera:
    def __init__(self, model, rng):
        self.model, self.rng = model, rng
        self.torso = model.body(camera_mount.PARENT).id
        self.mount = camera_mount.rotation()
        u = np.tan(np.radians(HFOV / 2)) * np.linspace(-1, 1, COLS)
        v = np.tan(np.radians(VFOV / 2)) * np.linspace(-1, 1, ROWS)
        uu, vv = (a.ravel() for a in np.meshgrid(u, v))
        rays = np.stack([uu, vv, np.ones_like(uu)], axis=1)        # pinhole: z = 1
        self.rays = rays / np.linalg.norm(rays, axis=1, keepdims=True)
        self.z_per_range = self.rays[:, 2]                          # depth = range * this
        self.geomid = np.zeros(len(self.rays), dtype=np.int32)
        self.dist = np.zeros(len(self.rays))

    def cast(self, data):
        """One frame: points with depth, in the optical frame (m). Call after the
        caller's mj_kinematics for this state."""
        m, d = self.model, data
        torso = d.xmat[self.torso].reshape(3, 3)
        origin = d.xpos[self.torso] + torso @ camera_mount.POSITION
        world = torso @ self.mount
        # The head (part of torso_link's geoms) holds the camera: rays start outside it.
        mujoco.mj_multiRay(m, d, origin, (self.rays @ world.T).ravel(), None, True, self.torso,
                           self.geomid, self.dist, len(self.rays), MAX_RANGE / self.z_per_range.min())
        depth = self.dist * self.z_per_range
        depth = depth + self.rng.normal(0.0, 1.0, len(depth)) * NOISE * depth ** 2
        keep = ((self.geomid >= 0) & (depth > MIN_RANGE) & (depth < MAX_RANGE)
                & (self.rng.random(len(depth)) > DROPOUT))
        return self.rays[keep] * (depth[keep] / self.z_per_range[keep])[:, None]


def describe():
    """One line for the log: what the simulated camera is."""
    return (f"Sim depth camera: {COLS}x{ROWS} rays, {HFOV:.0f}x{VFOV:.0f} deg, pitched "
            f"{math.degrees(camera_mount.RPY[1]):.1f} deg down, on {TOPIC}")
