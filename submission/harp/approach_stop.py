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
no usable scan, speed feedback or live lidar transform has arrived within
LIDAR_TIMEOUT s, including startup (Nav2 on Foxy walks on with a dead lidar).

The lidar's head blinds it around the front (harp/lidar_mount.py): with the face visor
off it sees the strip from about 1 m out; with it on, nothing of it beyond about 0.6 m.
The head depth camera, pitched down, sees it from about 0.3 to 2.7 m, but only low: a
person's legs, up to about 0.3 m high 2 m ahead. So each scan's nearest point is the
nearer of the lidar's and the latest camera frame's (CAMERA_AGE), the camera's taken
from CAMERA_MIN_Z up. The lidar stays the required input; without the camera the rule
runs on the lidar alone.

Events (policy.log, /harp/events): approach_stop (range m, closing m/s) and
approach_go (stopped_s), and lidar_lost / lidar_back.
"""

import json
import sys
import time
from collections import deque

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage

from harp import camera_mount, lidar_mount
from harp.events import Events, latched

TOPIC = "/utlidar/cloud_livox_mid360"
CAMERA_TOPIC = "/camera/depth/points"
HALF_WIDTH = 0.7     # m to each side of the strip: robot_radius 0.4 plus 0.3
LOOK = 6.0           # m ahead it watches
MIN_Z, MAX_Z = 0.3, 1.8   # m above the floor
CAMERA_MIN_Z = 0.1   # m: the camera sees only legs ahead, and its floor is close and flat
CAMERA_AGE = 0.25    # s a camera frame counts toward a lidar scan
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
    fields = {f.name: f for f in cloud.fields}
    formats, offsets = [], []
    for name in ("x", "y", "z"):
        field = fields.get(name)
        if field is None or field.count != 1 or field.datatype not in (PointField.FLOAT32, PointField.FLOAT64):
            raise ValueError("Cloud needs scalar float x/y/z fields")
        size = 4 if field.datatype == PointField.FLOAT32 else 8
        if field.offset < 0 or field.offset + size > cloud.point_step:
            raise ValueError("Cloud field exceeds point_step")
        formats.append((">" if cloud.is_bigendian else "<") + f"f{size}")
        offsets.append(field.offset)
    if (cloud.point_step <= 0 or cloud.row_step < cloud.width * cloud.point_step
            or len(cloud.data) < cloud.height * cloud.row_step):
        raise ValueError("Cloud has invalid row layout or truncated data")
    layout = np.dtype({"names": ["x", "y", "z"], "formats": formats,
                       "offsets": offsets, "itemsize": cloud.point_step})
    points = np.ndarray((cloud.height, cloud.width), dtype=layout, buffer=bytes(cloud.data),
                        strides=(cloud.row_step, cloud.point_step))
    return np.stack([points[n].ravel() for n in ("x", "y", "z")], axis=1).astype(float)


def nearest_ahead(points, min_z=MIN_Z):
    """(range m, (x, y)) of the nearest point in the strip ahead (base_link), or None."""
    x, y, z = points.T
    rng = np.hypot(x, y)
    keep = ((x > 0) & (np.abs(y) <= HALF_WIDTH) & (x <= LOOK) & (z >= min_z) & (z <= MAX_Z)
            & (rng > BODY))
    if not keep.any():
        return None
    i = np.flatnonzero(keep)[rng[keep].argmin()]
    return float(rng[i]), (float(x[i]), float(y[i]))


def nearer(a, b):
    """The nearer of two nearest_ahead results (either may be None)."""
    if a is None or b is None:
        return a if b is None else b
    return a if a[0] <= b[0] else b


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
        node.create_subscription(String, "/harp/events", self.on_event, latched())
        node.create_subscription(Twist, "/cmd_vel_nav", self.on_nav, 10)
        node.create_subscription(Odometry, "/odom", self.on_odom, 10)
        node.create_subscription(TFMessage, "/tf", self.on_tf, 50)
        node.create_subscription(PointCloud2, TOPIC, self.on_cloud,
                                 QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        node.create_subscription(PointCloud2, CAMERA_TOPIC, self.on_camera,
                                 QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        node.create_timer(PERIOD, self.on_timer)
        self.lidar = None           # (rotation, translation): livox_frame in base_link
        self.lidar_time = None      # receipt of a valid dynamic lidar transform
        self.camera = None          # (rotation, translation): the camera's optical frame in base_link
        self.camera_seen = None     # (monotonic s, nearest_ahead) of the last usable camera frame
        self.odom_time = None       # receipt of valid speed feedback
        self.speed = 0.0            # m/s forward, from /odom
        self.history = deque(maxlen=WINDOW + 1)
        self.last_scan = None       # monotonic s of the last scan; None before the first
        self.lost = True
        self.goal_failed = False   # latch route failure, including late goal acceptance
        self.stopped_at = None      # monotonic s it began standing for an approach
        self.last_coming = None

    def inputs_fresh(self, now):
        return all(t is not None and now - t <= LIDAR_TIMEOUT
                   for t in (self.last_scan, self.lidar_time, self.odom_time))

    def standing(self):
        return (self.goal_failed or self.stopped_at is not None or self.lost
                or not self.inputs_fresh(time.monotonic()))

    def on_event(self, msg):
        try:
            record = json.loads(msg.data)
        except ValueError:
            return
        if (isinstance(record, dict) and record.get("source") == "goal"
                and record.get("event") == "goal_result" and record.get("status") != "SUCCEEDED"):
            self.goal_failed = True
            self.out.publish(Twist())  # A late acceptance/cancel timeout cannot resume this route.

    def on_nav(self, msg):
        self.out.publish(Twist() if self.standing() else msg)

    def on_odom(self, msg):
        speed = msg.twist.twist.linear.x
        if np.isfinite(speed):
            self.speed, self.odom_time = speed, time.monotonic()

    def on_tf(self, msg):
        for t in msg.transforms:
            if t.header.frame_id != "base_link" or t.child_frame_id not in (lidar_mount.FRAME,
                                                                         camera_mount.FRAME):
                continue
            q, p = t.transform.rotation, t.transform.translation
            quat, shift = np.array([q.x, q.y, q.z, q.w]), np.array([p.x, p.y, p.z])
            norm = np.linalg.norm(quat)
            if np.isfinite(shift).all() and np.isfinite(norm) and norm > 1e-6:
                if t.child_frame_id == lidar_mount.FRAME:
                    self.lidar = (rotation(quat / norm), shift)
                    self.lidar_time = time.monotonic()
                else:
                    self.camera = (rotation(quat / norm), shift)

    def on_camera(self, cloud):
        """The nearest point in the strip in a camera frame, kept for the next lidar scan."""
        if cloud.header.frame_id != camera_mount.FRAME or self.camera is None:
            return
        try:
            turn, shift = self.camera
            points = xyz(cloud)
            points = points[np.isfinite(points).all(axis=1)]
            if not len(points):
                return
            self.camera_seen = (time.monotonic(), nearest_ahead(points @ turn.T + shift, CAMERA_MIN_Z))
        except (ValueError, TypeError):
            return

    def on_cloud(self, cloud):
        now = time.monotonic()
        if (cloud.header.frame_id != lidar_mount.FRAME or self.lidar is None
                or any(t is None or now - t > LIDAR_TIMEOUT
                       for t in (self.lidar_time, self.odom_time))):
            return
        try:
            turn, shift = self.lidar
            points = xyz(cloud)
            points = points[np.isfinite(points).all(axis=1)]
            if not len(points):
                return  # An empty/invalid scan cannot establish a clear scene.
            nearest = nearest_ahead(points @ turn.T + shift)
        except (ValueError, TypeError):
            return  # Leave the last usable receipt unchanged; the watchdog holds zero.
        if self.last_scan is None or now - self.last_scan > LIDAR_TIMEOUT:
            self.history.clear()
        self.last_scan = now
        if self.lost:
            self.lost = False
            self.event("lidar_back")
        if self.camera_seen is not None and now - self.camera_seen[0] <= CAMERA_AGE:
            nearest = nearer(nearest, self.camera_seen[1])
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
        if not self.lost and not self.inputs_fresh(time.monotonic()):
            self.lost = True
            self.history.clear()
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
