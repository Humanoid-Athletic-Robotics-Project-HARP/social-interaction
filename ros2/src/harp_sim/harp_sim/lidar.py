"""Simulated Livox MID-360, the lidar in the G1's head, for run_sim --lidar.

Casts rays from the lidar's spot in the head (MuJoCo's mj_multiRay) and
publishes where they hit as a ROS 2 PointCloud2, in the lidar's own frame.

Like the real one: 360 deg around and -7 to +52 deg up in its own frame,
mounted upside down (so it looks from 7 deg up to about 50 deg down), 10 Hz,
nothing nearer than 0.1 m or past 40 m, about 2 cm of range noise, a few
points lost. Unlike it: an even grid of rays instead of Livox's wandering
pattern, every surface reflects, and each scan is taken at one instant, so
nothing smears while the robot walks.

The robot's own body is in the cloud, as on the real robot: the shoulders and
arms show up as points and shadow what is behind them. Filtering them out is
the job of whatever reads the cloud. The head shell around the lidar is
skipped.

The rays are cast on a thread with its own copy of the robot's pose and the
moving obstacles' positions (mj_multiRay releases the GIL), so a scan never
delays a physics step.
"""

import threading

import mujoco
import numpy as np
import rclpy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header

from harp_control import lidar_mount

ELEVATION = (-7.0, 52.0)   # deg, in the lidar's frame
H_STEP, V_STEP = 0.75, 1.5   # deg between rays: 480 x 40 = 19,200, about one real frame
MIN_RANGE, MAX_RANGE = 0.1, 40.0   # m
RANGE_NOISE = 0.02   # m, 1 sigma
DROPOUT = 0.05       # share of points lost at random (a guess)
PERIOD = 0.1         # s of sim time between scans
# Unitree's lidar service on the robot publishes rt/utlidar/cloud_livox_mid360
# in livox_frame, so the sim uses the same names.
TOPIC = "/utlidar/cloud_livox_mid360"


class SimLidar:
    """Scans on its own thread whenever the physics loop hands it a pose."""

    def __init__(self, model):
        self.model = model
        self.data = mujoco.MjData(model)
        self.torso = model.body(lidar_mount.PARENT).id
        self.mount = lidar_mount.rotation()
        azimuth = np.radians(np.arange(-180.0, 180.0, H_STEP))
        elevation = np.radians(np.arange(ELEVATION[0], ELEVATION[1] + V_STEP / 2, V_STEP))
        az, el = (a.ravel() for a in np.meshgrid(azimuth, elevation))
        self.rays = np.stack([np.cos(el) * np.cos(az), np.cos(el) * np.sin(az), np.sin(el)], axis=1)
        self.geomid = np.zeros(len(self.rays), dtype=np.int32)
        self.dist = np.zeros(len(self.rays))
        self.rng = np.random.default_rng()

        self.node = rclpy.create_node("sim_lidar")
        self.pub = self.node.create_publisher(PointCloud2, TOPIC, 10)
        self.pose = None
        self.wake = threading.Condition()
        threading.Thread(target=self.run, daemon=True).start()

    def scan(self, data):
        """Called by the physics loop every PERIOD; skipped if the last scan isn't done."""
        with self.wake:
            if self.pose is None:
                self.pose = (data.qpos.copy(), data.mocap_pos.copy())
                self.wake.notify()

    def run(self):
        while rclpy.ok():
            with self.wake:
                self.wake.wait_for(lambda: self.pose is not None)
                self.data.qpos[:], self.data.mocap_pos[:] = self.pose
            try:
                self.publish(self.cast())
            except Exception:
                if rclpy.ok():
                    raise
                return            # ROS shut down while the sim quits
            with self.wake:
                self.pose = None

    def cast(self):
        """One scan: hit points in the lidar's frame."""
        m, d = self.model, self.data
        mujoco.mj_kinematics(m, d)
        torso = d.xmat[self.torso].reshape(3, 3)
        origin = d.xpos[self.torso] + torso @ lidar_mount.POSITION
        lidar = torso @ self.mount
        mujoco.mj_multiRay(m, d, origin, (self.rays @ lidar.T).ravel(), None, True, self.torso,
                           self.geomid, self.dist, None, len(self.rays), MAX_RANGE)
        hit = self.geomid >= 0   # -1: nothing within MAX_RANGE
        dist = self.dist + self.rng.normal(0.0, RANGE_NOISE, len(self.dist))
        keep = hit & (dist > MIN_RANGE) & (self.rng.random(len(dist)) > DROPOUT)
        return self.rays[keep] * dist[keep, None]

    def publish(self, points):
        header = Header(frame_id=lidar_mount.FRAME, stamp=self.node.get_clock().now().to_msg())
        self.pub.publish(point_cloud2.create_cloud_xyz32(header, points))
