"""Stop the robot while something comes at it, and let Nav2 drive again once it has passed.
Its own process between Nav2 and harp_node (entrypoint.sh): Nav2's controller publishes
/cmd_vel_nav (nav2.launch.py), and this passes it on to /cmd_vel, or a zero while stopped.

People step around a robot that stands still; a robot that swerves is hard to read, and
a quick sideways step is where a biped stumbles. So nothing that moves toward the robot
is dodged: the robot stands (GR00T's balance) until it has gone by. Still obstacles,
people standing included, stay Nav2's to plan around.

Each lidar scan (in base_link, which sits on the floor: harp/state.py) gives the nearest
point in a strip ahead, HALF_WIDTH to each side and up to LOOK ahead, between MIN_Z and
MAX_Z above the floor (low enough for the floor's error far out to stay below it), and
outside the robot's own arms (BODY). Its straight-line range, unlike the distance
straight ahead, doesn't change as the robot turns, and for a still point it shrinks no
faster than the robot walks. So something comes at the robot when, in WINDOW scans,
NEEDED of the steps saw the range shrink faster than the robot's own speed by over
APPROACH: a still obstacle entering the strip as the robot turns is one step, not
several. Once standing, it also stays while the nearest point moved faster than MOVING
over the WINDOW (someone walking across the strip, whose range has stopped shrinking),
and stands until neither has held for CLEAR s. It also stands while
no scan has come for LIDAR_TIMEOUT s, once one has (Nav2 on Foxy walks on with a dead
lidar).

Events (policy.log, /harp/events): approach_stop (range m, closing m/s) and
approach_go (stopped_s), and lidar_lost / lidar_back.
"""

import sys
import time
from collections import deque

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from tf2_msgs.msg import TFMessage

from harp import lidar_mount
from harp.events import Events

TOPIC = "/utlidar/cloud_livox_mid360"
HALF_WIDTH = 0.7     # m to each side of the strip: robot_radius 0.4 plus 0.3
LOOK = 6.0           # m ahead it watches
MIN_Z, MAX_Z = 0.3, 1.8   # m above the floor
BODY = 0.45          # m from base_link inside which points are the robot's own arms
APPROACH = 0.3       # m/s faster than the robot's walking that counts as coming at it
WINDOW, NEEDED = 5, 4     # scans looked back over, and steps of them that must agree
MOVING = 0.3         # m/s the nearest point moves over WINDOW to keep a standing robot standing
CLEAR = 1.0          # s with nothing coming or moving before Nav2 drives again
LIDAR_TIMEOUT = 0.5  # s without a scan before the robot stands
PERIOD = 0.1         # s between zero commands while standing


def rotation(q):
    """3x3 matrix of the unit quaternion (x, y, z, w)."""
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def xyz(cloud):
    """N x 3 float array of a PointCloud2's x, y, z, whatever else its points hold."""
    offsets = {f.name: f.offset for f in cloud.fields}
    layout = np.dtype({"names": ["x", "y", "z"], "formats": ["<f4"] * 3,
                       "offsets": [offsets["x"], offsets["y"], offsets["z"]],
                       "itemsize": cloud.point_step})
    points = np.frombuffer(bytes(cloud.data), dtype=layout, count=cloud.width * cloud.height)
    return np.stack([points["x"], points["y"], points["z"]], axis=1).astype(float)


def nearest_ahead(points):
    """(range m, (x, y)) of the nearest point in the strip ahead (base_link), or None."""
    x, y, z = points.T
    rng = np.hypot(x, y)
    keep = ((x > 0) & (np.abs(y) <= HALF_WIDTH) & (x <= LOOK) & (z >= MIN_Z) & (z <= MAX_Z)
            & (rng > BODY))
    if not keep.any():
        return None
    i = np.flatnonzero(keep)[rng[keep].argmin()]
    return float(rng[i]), (float(x[i]), float(y[i]))


def coming(history, speed):
    """(True, closing m/s) if what's nearest ahead came at the robot in most recent steps.
    history: (time s, nearest_ahead or None) per scan, newest last; speed: the robot's, m/s."""
    steps = [(n0[0] - n1[0]) / (t1 - t0) - max(speed, 0.0)
             for (t0, n0), (t1, n1) in zip(history, list(history)[1:])
             if n0 is not None and n1 is not None and t1 > t0]
    fast = [s for s in steps if s > APPROACH]
    return len(fast) >= NEEDED, (max(fast) if fast else 0.0)


def moving(history):
    """True if the nearest point ahead moved faster than MOVING from the first scan in
    history to the last (for a standing robot: someone walking across the strip)."""
    (t0, n0), (t1, n1) = history[0], history[-1]
    if n0 is None or n1 is None or t1 <= t0:
        return False
    return np.hypot(n1[1][0] - n0[1][0], n1[1][1] - n0[1][1]) / (t1 - t0) > MOVING


class ApproachStop:
    def __init__(self, node):
        self.node = node
        self.out = node.create_publisher(Twist, "/cmd_vel", 10)
        self.event = Events(node, "approach")
        node.create_subscription(Twist, "/cmd_vel_nav", self.on_nav, 10)
        node.create_subscription(Odometry, "/odom", self.on_odom, 10)
        node.create_subscription(TFMessage, "/tf", self.on_tf, 50)
        node.create_subscription(PointCloud2, TOPIC, self.on_cloud,
                                 QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        node.create_timer(PERIOD, self.on_timer)
        self.lidar = None           # (rotation, translation): livox_frame in base_link
        self.speed = 0.0            # m/s forward, from /odom
        self.history = deque(maxlen=WINDOW + 1)
        self.last_scan = None       # monotonic s of the last scan; None before the first
        self.lost = False
        self.stopped_at = None      # monotonic s it began standing for an approach
        self.last_coming = None

    def standing(self):
        return self.stopped_at is not None or self.lost

    def on_nav(self, msg):
        self.out.publish(Twist() if self.standing() else msg)

    def on_odom(self, msg):
        self.speed = msg.twist.twist.linear.x

    def on_tf(self, msg):
        for t in msg.transforms:
            if t.header.frame_id == "base_link" and t.child_frame_id == lidar_mount.FRAME:
                q, p = t.transform.rotation, t.transform.translation
                self.lidar = (rotation((q.x, q.y, q.z, q.w)), np.array([p.x, p.y, p.z]))

    def on_cloud(self, cloud):
        now = time.monotonic()
        self.last_scan = now
        if self.lost:
            self.lost = False
            self.event("lidar_back")
        if self.lidar is None:
            return
        turn, shift = self.lidar
        nearest = nearest_ahead(xyz(cloud) @ turn.T + shift)
        self.history.append((now, nearest))
        hit, closing = coming(self.history, self.speed)
        if hit or (self.stopped_at is not None and moving(self.history)):
            self.last_coming = now
            if self.stopped_at is None:
                self.stopped_at = now
                self.event("approach_stop", range=nearest and round(nearest[0], 2),
                           closing=round(closing, 2))
        elif self.stopped_at is not None and now - self.last_coming >= CLEAR:
            self.event("approach_go", stopped_s=round(now - self.stopped_at, 1))
            self.stopped_at = None

    def on_timer(self):
        if (not self.lost and self.last_scan is not None
                and time.monotonic() - self.last_scan > LIDAR_TIMEOUT):
            self.lost = True
            self.event("lidar_lost")
        if self.standing():
            self.out.publish(Twist())


def main():
    rclpy.init(args=sys.argv)
    node = rclpy.create_node("approach_stop")
    ApproachStop(node)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
