"""Simulated Livox MID-360, the lidar in the G1's head, for runs in HIM's simulator,
which has no lidar; it also casts the head depth camera (harp/depth_camera.py) from
the same state. Its own process in a world's image (entrypoint.sh): it casts
rays (MuJoCo's mj_multiRay) through its own copy of the world
(/harp/model/lidar_scene.xml, from submission/sim.sh --world), with the robot
posed from /safety/policy_state (the sim's true pose: this is the sensor, not
odometry), and publishes the hits as a PointCloud2 in the lidar's own frame, and the
same cloud turned 180 deg (lidar_mount.REAR_FRAME) for Nav2's clearing view behind,
which must leave out where its head blinds it (nav2_params.yaml).

Like the real one (measured on our robot: context/g1-lidar-measured.md): 360 deg
around and -7 to +52 deg up in its own frame, mounted upside down (so it looks from
7 deg up to about 50 deg down), 10 Hz, nothing nearer than 0.1 m or past 40 m, about
2 cm of range noise, about 15% of points without an echo. **Its head's shadow**
(lidar_mount.sees, by lidar_mount.VISOR): with the face visor off, a window about
40 deg either side of straight ahead (narrower looking down), then blind out to about
60 deg each side; with it on, nothing within about 60 deg of straight ahead. Its rays
are spread at random over the field of view and cast a slice at each state, about
every 20 ms, so like Livox's wandering pattern no two scans repeat, and like the real
one a scan's points are from 100 ms of motion, smeared while the robot turns (no
deskewing). Unlike it: every surface reflects, and the rays are uniform over the field
of view.

The robot's own body is in the cloud, as on the real robot: the shoulders and
arms show up as points and shadow what is behind them. The head's own geoms (part of
torso_link) are skipped; lidar_mount.sees stands in for them. Moving obstacles (movers.py) exist
only here: scripted, or with the task's "crowd": "aware" (course.py
--crowd aware), giving way to the robot's true pose. The camera casts at its own CAMERA_PERIOD from the same states.

It is a separate process from harp_node because publishing a 190 KB cloud
from inside the policy's process made it miss HIM's 0.2 s command deadline.
Events go to /harp/events: mover_start (mover, starts_at) when a mover sets off. An aware
crowd also prints, for scoring and the replay (not on /harp/events): crowd (its movers) at
start, then movers (t, xy: where each is) every scan.
"""

import array
import json
import os
import sys

import mujoco
import numpy as np
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header, String

from harp import camera_mount, depth_camera, lidar_mount
from harp.events import Events, latched
from harp.movers import AwareMovers, Movers, crowd_event, crowd_mode, path_event

RAYS_PER_S = 200_000         # the real MID-360's points a second, no-echo ones included
MIN_RANGE, MAX_RANGE = 0.1, 40.0   # m
RANGE_NOISE = 0.02   # m, 1 sigma
DROPOUT = 0.15       # share of points without an echo where the head leaves it open (measured)
CAMERA_TOPIC = depth_camera.TOPIC
PERIOD = 0.1         # s of sim time between scans
CAMERA_PERIOD = 1 / 15   # s of sim time between camera frames (the robot's at 15 fps)
# Unitree's lidar service on the robot publishes rt/utlidar/cloud_livox_mid360
# in livox_frame, so the sim uses the same names.
TOPIC = lidar_mount.TOPIC
REAR_TOPIC = lidar_mount.REAR_TOPIC
SCENE = "/harp/model/lidar_scene.xml"
XYZ_FIELDS = [PointField(name=c, offset=4 * i, datatype=PointField.FLOAT32, count=1)
              for i, c in enumerate("xyz")]


class SimLidar:
    """Casts a slice of the scan at each state, publishes a scan every PERIOD of sim
    time and a camera frame every CAMERA_PERIOD."""

    def __init__(self, node, scene):
        self.node = node
        self.model = mujoco.MjModel.from_xml_path(str(scene))
        self.data = mujoco.MjData(self.model)
        aware = crowd_mode(os.environ.get("G1_TASK_COMMAND", "")) == "aware"
        self.movers = (AwareMovers if aware else Movers)(self.model)
        self.aware = aware and bool(self.movers.names)
        if self.aware:
            print(crowd_event(self.movers.names), flush=True)
        self.torso = self.model.body(lidar_mount.PARENT).id
        self.mount = lidar_mount.rotation()
        self.rng = np.random.default_rng()
        self.pub = node.create_publisher(PointCloud2, TOPIC, 10)
        self.rear_pub = node.create_publisher(PointCloud2, REAR_TOPIC, 10)
        self.camera = depth_camera.DepthCamera(self.model, self.rng)
        self.camera_pub = node.create_publisher(PointCloud2, CAMERA_TOPIC, 10)
        print(depth_camera.describe(), flush=True)
        print(f"Sim lidar: face visor {'on' if lidar_mount.VISOR else 'off'}", flush=True)
        self.events = Events(node, "lidar")
        node.create_subscription(String, "/harp/events", self.on_event, latched())
        node.create_subscription(String, "/safety/policy_state", self.on_state,
                                 QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.goal_sent = False
        self.last_slice = None      # sim time of the last state cast
        self.next_scan = self.next_frame = 0.0
        self.pending = []           # this scan's points so far, lidar frame

    def on_event(self, msg):
        if json.loads(msg.data).get("event") == "goal_sent":
            self.goal_sent = True

    def on_state(self, msg):
        state = json.loads(msg.data)
        t = state.get("sim_time")
        if t is None or "position" not in state:
            return
        if self.last_slice is None:
            self.last_slice, self.next_scan, self.next_frame = t, t + PERIOD, t
            return
        dt = t - self.last_slice
        if dt <= 0:
            return
        self.last_slice = t
        for name, starts_at in self.movers.update(np.asarray(state["position"][:2]), t,
                                                  state.get("control_ready", False), self.goal_sent):
            self.events("mover_start", t, mover=name, starts_at=starts_at)
        qpos = np.concatenate([state["position"], state["quat"], state["q"]])
        self.data.qpos[:len(qpos)] = qpos
        self.movers.apply(self.data, t)
        mujoco.mj_kinematics(self.model, self.data)
        self.pending.append(self.cast(int(round(RAYS_PER_S * min(dt, PERIOD)))))
        if t >= self.next_scan:
            points = np.concatenate(self.pending)
            self.pending = []
            self.next_scan = max(self.next_scan + PERIOD, t)
            if self.aware:
                print(path_event(t, self.movers.positions), flush=True)
            self.publish(self.pub, lidar_mount.FRAME, points)
            self.publish(self.rear_pub, lidar_mount.REAR_FRAME, points * (-1.0, -1.0, 1.0))
        if t >= self.next_frame:
            self.next_frame = max(self.next_frame + CAMERA_PERIOD, t)
            self.publish(self.camera_pub, camera_mount.FRAME, self.camera.cast(self.data))

    def cast(self, n):
        """n rays at random over the field of view, now: hit points in the lidar's frame.
        Call after mj_kinematics for this state."""
        m, d = self.model, self.data
        az = self.rng.uniform(-np.pi, np.pi, n)
        el = np.radians(self.rng.uniform(*lidar_mount.ELEVATION, n))
        rays = np.stack([np.cos(el) * np.cos(az), np.cos(el) * np.sin(az), np.sin(el)], axis=1)
        open_ = lidar_mount.sees(rays @ self.mount.T)
        torso = d.xmat[self.torso].reshape(3, 3)
        origin = d.xpos[self.torso] + torso @ lidar_mount.POSITION
        geomid, dist = np.zeros(n, dtype=np.int32), np.zeros(n)
        mujoco.mj_multiRay(m, d, origin, (rays @ (torso @ self.mount).T).ravel(), None, True,
                           self.torso, geomid, dist, n, MAX_RANGE)
        hit = geomid >= 0   # -1: nothing within MAX_RANGE
        dist = dist + self.rng.normal(0.0, RANGE_NOISE, n)
        keep = hit & open_ & (dist > MIN_RANGE) & (self.rng.random(n) > DROPOUT)
        return rays[keep] * dist[keep, None]

    def publish(self, pub, frame, points):
        points = points.astype(np.float32)
        cloud = PointCloud2(
            header=Header(frame_id=frame, stamp=self.node.get_clock().now().to_msg()),
            height=1, width=len(points), fields=XYZ_FIELDS, is_bigendian=False,
            point_step=12, row_step=12 * len(points), is_dense=True)
        cloud.data = array.array("B", points.tobytes())  # array.array, not bytes: Foxy's rclpy checks bytes one by one (124 ms for 800 KB, 1 ms this way)
        pub.publish(cloud)


def main():
    rclpy.init(args=sys.argv)
    node = rclpy.create_node("sim_lidar")
    SimLidar(node, SCENE)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
