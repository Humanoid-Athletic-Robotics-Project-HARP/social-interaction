"""Simulated Livox MID-360, the lidar in the G1's head, for runs in HIM's simulator,
which has no lidar. Its own process in a world's image (entrypoint.sh): it casts
rays (MuJoCo's mj_multiRay) through its own copy of the world
(/harp/model/lidar_scene.xml, from submission/sim.sh --world), with the robot
posed from /safety/policy_state (the sim's true pose: this is the sensor, not
odometry), and publishes the hits as a PointCloud2 in the lidar's own frame.

Like the real one: 360 deg around and -7 to +52 deg up in its own frame,
mounted upside down (so it looks from 7 deg up to about 50 deg down), 10 Hz,
nothing nearer than 0.1 m or past 40 m, about 2 cm of range noise, a few
points lost. Unlike it: an even grid of rays instead of Livox's wandering
pattern, every surface reflects, and each scan is taken at one instant, so
nothing smears while the robot walks.

The robot's own body is in the cloud, as on the real robot: the shoulders and
arms show up as points and shadow what is behind them. The head shell around
the lidar is skipped. Moving obstacles (movers.py) exist only here.

It is a separate process from harp_node because publishing a 190 KB cloud
from inside the policy's process made it miss HIM's 0.2 s command deadline.
Events go to /harp/events: mover_start (mover, starts_at) when a mover sets off.
"""

import json
import sys

import mujoco
import numpy as np
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header, String

from harp import lidar_mount
from harp.movers import Movers

ELEVATION = (-7.0, 52.0)   # deg, in the lidar's frame
H_STEP, V_STEP = 0.75, 1.5   # deg between rays: 480 x 40 = 19,200, about one real frame
MIN_RANGE, MAX_RANGE = 0.1, 40.0   # m
RANGE_NOISE = 0.02   # m, 1 sigma
DROPOUT = 0.05       # share of points lost at random (a guess)
PERIOD = 0.1         # s of sim time between scans
# Unitree's lidar service on the robot publishes rt/utlidar/cloud_livox_mid360
# in livox_frame, so the sim uses the same names.
TOPIC = "/utlidar/cloud_livox_mid360"
SCENE = "/harp/model/lidar_scene.xml"
XYZ_FIELDS = [PointField(name=c, offset=4 * i, datatype=PointField.FLOAT32, count=1)
              for i, c in enumerate("xyz")]


class SimLidar:
    """Scans whenever a state is PERIOD of sim time after the last scan."""

    def __init__(self, node, scene):
        self.node = node
        self.model = mujoco.MjModel.from_xml_path(str(scene))
        self.data = mujoco.MjData(self.model)
        self.movers = Movers(self.model)
        self.torso = self.model.body(lidar_mount.PARENT).id
        self.mount = lidar_mount.rotation()
        azimuth = np.radians(np.arange(-180.0, 180.0, H_STEP))
        elevation = np.radians(np.arange(ELEVATION[0], ELEVATION[1] + V_STEP / 2, V_STEP))
        az, el = (a.ravel() for a in np.meshgrid(azimuth, elevation))
        self.rays = np.stack([np.cos(el) * np.cos(az), np.cos(el) * np.sin(az), np.sin(el)], axis=1)
        self.geomid = np.zeros(len(self.rays), dtype=np.int32)
        self.dist = np.zeros(len(self.rays))
        self.rng = np.random.default_rng()
        self.pub = node.create_publisher(PointCloud2, TOPIC, 10)
        latched = QoSProfile(depth=1000, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.events_pub = node.create_publisher(String, "/harp/events", latched)
        node.create_subscription(String, "/harp/events", self.on_event, latched)
        node.create_subscription(String, "/safety/policy_state", self.on_state,
                                 QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.goal_sent = False
        self.next_scan = 0.0

    def on_event(self, msg):
        if json.loads(msg.data).get("event") == "goal_sent":
            self.goal_sent = True

    def on_state(self, msg):
        state = json.loads(msg.data)
        t = state.get("sim_time")
        if t is None or "position" not in state or t < self.next_scan:
            return
        self.next_scan = t + PERIOD
        for name, starts_at in self.movers.update(np.asarray(state["position"][:2]), t,
                                                  state.get("control_ready", False), self.goal_sent):
            text = json.dumps({"source": "lidar", "event": "mover_start",
                               "mover": name, "starts_at": starts_at})
            self.events_pub.publish(String(data=text))
            print(text, flush=True)
        qpos = np.concatenate([state["position"], state["quat"], state["q"]])
        self.data.qpos[:len(qpos)] = qpos
        self.movers.apply(self.data, t)
        self.publish(self.cast())

    def cast(self):
        """One scan: hit points in the lidar's frame."""
        m, d = self.model, self.data
        mujoco.mj_kinematics(m, d)
        torso = d.xmat[self.torso].reshape(3, 3)
        origin = d.xpos[self.torso] + torso @ lidar_mount.POSITION
        lidar = torso @ self.mount
        mujoco.mj_multiRay(m, d, origin, (self.rays @ lidar.T).ravel(), None, True, self.torso,
                           self.geomid, self.dist, len(self.rays), MAX_RANGE)
        hit = self.geomid >= 0   # -1: nothing within MAX_RANGE
        dist = self.dist + self.rng.normal(0.0, RANGE_NOISE, len(self.dist))
        keep = hit & (dist > MIN_RANGE) & (self.rng.random(len(dist)) > DROPOUT)
        return self.rays[keep] * dist[keep, None]

    def publish(self, points):
        points = points.astype(np.float32)
        cloud = PointCloud2(
            header=Header(frame_id=lidar_mount.FRAME, stamp=self.node.get_clock().now().to_msg()),
            height=1, width=len(points), fields=XYZ_FIELDS, is_bigendian=False,
            point_step=12, row_step=12 * len(points), is_dense=True)
        cloud.data = points.tobytes()
        self.pub.publish(cloud)


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
