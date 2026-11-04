"""Mapping and localization for Nav2. entrypoint.sh starts it when the task (G1_TASK_COMMAND)
has "mapping": true (build a map on this walk: course.py --mapping) or "map": a saved one
(course.py --map RUN); Nav2's costmaps then get a static layer on MAP_TOPIC (nav2.launch.py).

build: harp/occupancy.py's map from /harp/static_cloud (the lidar and camera without moving
  objects), published on MAP_TOPIC as it grows, so this walk already remembers every wall it
  saw, and every EVENT_PERIOD as a "map" event, so the run keeps it. Its frame, "map", is this
  walk's odom: map -> odom stays identity.
localize: the saved map on MAP_TOPIC, and map -> odom from harp/localizer.py. The robot is
  taken to start where the mapping walk did (or the task's "map_pose": [x, y, heading] in the
  map, the map-to-odom correction at the start); a search round that finds the start, then a
  match every WORK_PERIOD keeps leg odometry's drift off. "localization" events every
  EVENT_PERIOD say how it is going.
"""
import array
import json
import math
import os
import sys

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener

from harp import events, localizer, occupancy
from harp.approach_stop import rotation, xyz
from harp.lidar_mount import FRAME

STATIC_TOPIC = "/harp/static_cloud"   # harp/predictive_avoidance.py's STATIC_TOPIC
MAP_TOPIC = "/map"
MAP_FRAME = "map"
TF_PERIOD = .05       # s between map -> odom transforms
WORK_PERIOD = .5      # s between scans used
PUBLISH_PERIOD = 3.0  # s between maps published while building
EVENT_PERIOD = 5.0    # s between "map" (building) or "localization" events
BLEND = (.3, .05)     # share of each accepted match taken into the robot's map position, heading
SEARCH_PERIOD = 2.0   # s between start searches until one succeeds


def task_mode(text):
    """("build", None), ("localize", (saved map, start pose)) or (None, None) from the task;
    ValueError for a bad map."""
    try:
        task = json.loads(text or "null")
    except ValueError:
        return None, None
    if not isinstance(task, dict):
        return None, None
    if task.get("map") is not None:
        occupancy.decode(task["map"])
        pose = task.get("map_pose", (0.0, 0.0, 0.0))
        pose = np.array([float(pose[0]), float(pose[1]), math.radians(float(pose[2]))])
        return "localize", (task["map"], pose)
    if task.get("mapping"):
        return "build", None
    return None, None


def transform_points(points, transform):
    q, p = transform.transform.rotation, transform.transform.translation
    quat = np.array([q.x, q.y, q.z, q.w])
    norm = np.linalg.norm(quat)
    if not np.isfinite(norm) or norm < 1e-6:
        raise ValueError("invalid transform")
    return points @ rotation(quat / norm).T + np.array([p.x, p.y, p.z])


def blend(estimate, match, pivot=(0.0, 0.0)):
    """The estimate moved towards a match, as seen at the pivot (the robot): its map position by
    BLEND[0], its heading by BLEND[1] (scan by scan, heading jitters with the torso's sway; turned
    about odom's origin, metres away, it would move the robot)."""
    a, b = localizer.around(estimate, pivot), localizer.around(match, pivot)
    out = a + BLEND[0] * (b - a)
    out[2] = a[2] + BLEND[1] * math.remainder(b[2] - a[2], math.tau)
    return localizer.from_pivot(out, pivot)


def grid_message(grid, origin, resolution, stamp):
    msg = OccupancyGrid()
    msg.header.frame_id = MAP_FRAME
    msg.header.stamp = stamp
    msg.info.resolution = float(resolution)
    msg.info.width, msg.info.height = int(grid.shape[1]), int(grid.shape[0])
    msg.info.origin.position.x, msg.info.origin.position.y = float(origin[0]), float(origin[1])
    msg.info.origin.orientation.w = 1.0
    msg.data = array.array("b", np.ascontiguousarray(grid, dtype=np.int8).tobytes())
    return msg


class Mapper:
    def __init__(self, node, mode, saved=None):
        self.node, self.mode = node, mode
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, node)
        self.broadcast = TransformBroadcaster(node)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.map_pub = node.create_publisher(OccupancyGrid, MAP_TOPIC, latched)
        node.create_subscription(PointCloud2, STATIC_TOPIC, self.on_cloud,
                                 QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.cloud, self.used_stamp = None, None
        self.pose = np.zeros(3)          # map -> odom (x, y, heading)
        self.found = mode == "build"     # localize: the start search succeeded
        self.matches = self.refused = 0
        self.last = None                 # the latest match (pose, inliers, fraction)
        self.next_publish = self.next_event = self.next_search = 0.0
        if mode == "build":
            self.map = occupancy.OccupancyMap()
        else:
            saved_map, self.pose = saved
            grid, origin, resolution = occupancy.decode(saved_map)
            self.grid, self.origin, self.resolution = grid, origin, resolution
            self.localizer = localizer.Localizer(grid, origin, resolution)
            self.start = self.pose.copy()
            self.map_pub.publish(grid_message(grid, origin, resolution, node.get_clock().now().to_msg()))
            events.emit("mapper", "map_loaded", width=int(grid.shape[1]), height=int(grid.shape[0]),
                        occupied=int((grid == 100).sum()), start=[round(float(v), 3) for v in self.pose])
        node.create_timer(TF_PERIOD, self.send_tf)
        node.create_timer(WORK_PERIOD, self.work)

    def on_cloud(self, msg):
        self.cloud = msg

    def send_tf(self):
        t = TransformStamped()
        t.header.stamp = self.node.get_clock().now().to_msg()
        t.header.frame_id, t.child_frame_id = MAP_FRAME, "odom"
        t.transform.translation.x, t.transform.translation.y = float(self.pose[0]), float(self.pose[1])
        t.transform.rotation.z = math.sin(self.pose[2] / 2)
        t.transform.rotation.w = math.cos(self.pose[2] / 2)
        self.broadcast.sendTransform(t)

    def points(self):
        """(odom points, sensor origin) of the latest unused cloud, or None."""
        cloud = self.cloud
        if cloud is None or cloud.header.frame_id != FRAME:
            return None
        stamp = (cloud.header.stamp.sec, cloud.header.stamp.nanosec)
        if stamp == self.used_stamp:
            return None
        try:
            if cloud.width * cloud.height > 200000:
                return None
            local = xyz(cloud)
            local = local[np.isfinite(local).all(axis=1)]
            world = self.tf.lookup_transform("odom", FRAME, Time.from_msg(cloud.header.stamp))
            points = transform_points(local, world)
            origin = transform_points(np.zeros((1, 3)), world)[0]
        except (TransformException, ValueError, TypeError):
            return None
        self.used_stamp = stamp
        return points, origin

    def work(self):
        got = self.points()
        now = self.node.get_clock().now()
        seconds = now.nanoseconds * 1e-9
        if got is not None:
            points, origin = got
            if self.mode == "build":
                self.map.integrate(points, origin)
            else:
                self.localize(points, origin[:2], seconds)
        if self.mode == "build" and seconds >= self.next_publish:
            self.next_publish = seconds + PUBLISH_PERIOD
            grid = self.map.grid()
            self.map_pub.publish(grid_message(grid, self.map.origin, self.map.resolution, now.to_msg()))
        if seconds >= self.next_event:
            self.next_event = seconds + EVENT_PERIOD
            self.report()

    def localize(self, points, robot, seconds):
        walls = points[(points[:, 2] >= localizer.WALL_Z[0]) & (points[:, 2] <= localizer.WALL_Z[1]), :2]
        if not len(walls):
            return
        if not self.found:
            if seconds < self.next_search:
                return
            self.next_search = seconds + SEARCH_PERIOD
            pose, inliers, fraction = self.localizer.search(walls, self.start, robot)
            if inliers >= localizer.MIN_INLIERS and fraction >= localizer.MIN_FRACTION:
                self.pose, self.found = pose, True
                events.emit("mapper", "localized", pose=[round(float(v), 3) for v in pose],
                            inliers=inliers, fraction=round(fraction, 3))
            self.last = (pose, inliers, fraction)
            return
        pose, inliers, fraction = self.localizer.match(walls, self.pose, robot)
        self.last = (pose, inliers, fraction)
        if localizer.accept(pose, self.pose, inliers, fraction, robot):
            self.pose = blend(self.pose, pose, robot)
            self.matches += 1
        else:
            self.refused += 1

    def report(self):
        if self.mode == "build":
            saved = occupancy.encode(self.map.grid(), self.map.origin, self.map.resolution)
            if saved is not None:
                events.emit("mapper", "map", **saved)
            return
        last = self.last
        events.emit("mapper", "localization", found=self.found, pose=[round(float(v), 4) for v in self.pose],
                    matches=self.matches, refused=self.refused,
                    inliers=None if last is None else last[1],
                    fraction=None if last is None else round(last[2], 3))


def main():
    try:
        mode, saved = task_mode(os.environ.get("G1_TASK_COMMAND", ""))
    except ValueError as error:
        events.emit("mapper", "map_refused", reason=str(error))
        return 1
    if mode is None:
        return 0
    rclpy.init(args=sys.argv)
    node = rclpy.create_node("harp_mapper")
    mapper = Mapper(node, mode, saved)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        mapper.report()
    return 0


if __name__ == "__main__":
    sys.exit(main())
