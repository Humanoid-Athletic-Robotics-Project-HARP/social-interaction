"""Opt-in Nav2 command filter: stamped LiDAR tracks -> predictive velocity requests.

This process replaces approach_stop; it never runs beside it. HIM keeps motor
ownership. Alerts are JSON signals, ready for a separate speaker/indicator consumer.
"""
import json
import math
import sys
import time
from dataclasses import asdict

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

from harp.approach_stop import TOPIC, rotation, xyz
from harp.lidar_mount import FRAME
from harp.obstacle_tracker import Tracker
from harp.predictive_mpc import CONTROL_PERIOD, Planner

PERIOD = CONTROL_PERIOD
TIMEOUT = 0.5
MAX_SCAN_AGE = 0.35
MAX_SOLVE_TIME = 0.15


def seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def transform_points(points, transform):
    q, p = transform.transform.rotation, transform.transform.translation
    quat = np.array([q.x, q.y, q.z, q.w])
    shift = np.array([p.x, p.y, p.z])
    norm = np.linalg.norm(quat)
    if not np.isfinite(shift).all() or not np.isfinite(norm) or norm < 1e-6:
        raise ValueError("invalid transform")
    return points @ rotation(quat / norm).T + shift


class PredictiveAvoidance:
    def __init__(self, node, buffer=None):
        self.node = node
        self.tf = buffer if buffer is not None else Buffer(cache_time=Duration(seconds=2.0))
        self.listener = None if buffer is not None else TransformListener(self.tf, node)
        self.tracker, self.planner = Tracker(), Planner()
        self.config = self.planner.config
        self.out = node.create_publisher(Twist, "/cmd_vel", 10)
        latched = QoSProfile(depth=1000, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.events = node.create_publisher(String, "/harp/events", latched)
        self.alerts = node.create_publisher(String, "/harp/obstacle_alert", 10)
        self.tracks_out = node.create_publisher(String, "/harp/obstacle_tracks", 10)
        node.create_subscription(String, "/harp/events", self.on_event, latched)
        node.create_subscription(Twist, "/cmd_vel_nav", self.on_nav, 10)
        node.create_subscription(Odometry, "/odom", self.on_odom, 10)
        node.create_subscription(PointCloud2, TOPIC, self.on_cloud,
                                 QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        node.create_timer(PERIOD, self.on_timer)
        node.create_timer(0.05, self.on_pending)
        self.pending_cloud = None
        self.pose = self.velocity = None
        self.odom_receipt = self.scan_receipt = self.nav_receipt = None
        self.odom_stamp = self.scan_stamp = None
        self.desired = (0.0, 0.0, 0.0)
        self.active = self.failed = False
        self.perception_failed = False
        self.last_tracks, self.tracks_stamp = [], None
        self.mode = None
        self.last_alert = -math.inf
        self.alert_active = False

    def clock(self):
        return self.node.get_clock().now().nanoseconds * 1e-9

    def status(self, mode, alert=False, **fields):
        if mode != self.mode:
            text = json.dumps(dict(source="predictive", event="avoidance", mode=mode, **fields))
            self.events.publish(String(data=text))
            print(text, flush=True)
            self.mode = mode
        now = time.monotonic()
        if alert != self.alert_active or (alert and now - self.last_alert >= 1.0):
            self.alerts.publish(String(data=json.dumps(dict(active=alert, reason=mode,
                                                            stamp=self.clock()))))
            self.alert_active, self.last_alert = alert, now

    def stop(self, reason, alert=False):
        self.out.publish(Twist())
        self.status(reason, alert)

    def on_event(self, msg):
        try:
            value = json.loads(msg.data)
        except (TypeError, ValueError):
            return
        if not isinstance(value, dict) or value.get("source") != "goal":
            return
        event = value.get("event")
        if event in ("goal_sent", "waypoint_sent") and not self.failed:
            self.active = True
        elif event == "goal_result":
            self.active = False
            self.failed |= value.get("status") != "SUCCEEDED"
            self.stop("route_failed" if self.failed else "idle")

    def on_nav(self, msg):
        command = (msg.linear.x, msg.linear.y, msg.angular.z)
        if not np.isfinite(command).all():
            self.nav_receipt = None
            self.stop("invalid_command", True)
            return
        self.desired = command
        self.nav_receipt = time.monotonic()

    def on_odom(self, msg):
        stamp, now = seconds(msg.header.stamp), self.clock()
        p, q, v = msg.pose.pose.position, msg.pose.pose.orientation, msg.twist.twist
        values = (p.x, p.y, q.x, q.y, q.z, q.w, v.linear.x, v.linear.y, v.angular.z)
        norm = math.sqrt(sum(a*a for a in (q.x, q.y, q.z, q.w)))
        if (msg.header.frame_id != "odom" or msg.child_frame_id != "base_link"
                or not np.isfinite(values).all() or norm < 1e-6
                or not 0 <= now - stamp <= TIMEOUT
                or (self.odom_stamp is not None and stamp <= self.odom_stamp)):
            return
        x, y, z, w = (q.x/norm, q.y/norm, q.z/norm, q.w/norm)
        yaw = math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))
        self.pose = (p.x, p.y, yaw)
        self.velocity = (v.linear.x, v.linear.y, v.angular.z)
        self.odom_stamp, self.odom_receipt = stamp, time.monotonic()

    def on_cloud(self, cloud):
        self.pending_cloud = cloud
        self.on_pending()

    def on_pending(self):
        cloud = self.pending_cloud
        if cloud is None:
            return
        stamp, now = seconds(cloud.header.stamp), self.clock()
        if (cloud.header.frame_id != FRAME or not 0 <= now - stamp <= MAX_SCAN_AGE
                or (self.scan_stamp is not None and stamp <= self.scan_stamp)):
            self.pending_cloud = None
            return
        try:
            if cloud.width * cloud.height > 100000:
                raise ValueError("cloud exceeds input limit")
            points = xyz(cloud)
            points = points[np.isfinite(points).all(axis=1)]
            if not len(points):
                raise ValueError("empty cloud")
            at = Time.from_msg(cloud.header.stamp)
            # Both transforms use observation time, including the moving head mount.
            world = self.tf.lookup_transform("odom", FRAME, at)
            body = self.tf.lookup_transform("odom", "base_link", at)
            points = transform_points(points, world)
            base = transform_points(np.zeros((1, 3)), body)[0]
            distance = np.linalg.norm(points[:, :2] - base[:2], axis=1)
            keep = ((points[:, 2] >= 0.15) & (points[:, 2] <= 1.8)
                    & (distance > 0.45) & (distance <= 6.0))
            tracks = self.tracker.update(points[keep, :2], stamp)
        except TransformException:
            # DDS can deliver a scan before its TF. Retry one bounded pending scan,
            # keeping its original timestamp; the watchdog still holds zero if stale.
            return
        except (ValueError, TypeError, OverflowError):
            self.pending_cloud = None
            self.scan_receipt = None
            self.stop("unusable_scan", True)
            return
        self.pending_cloud = None
        observed = {t.id for t in tracks if t.last_seen == stamp}
        self.check_missing(stamp, observed)
        self.last_tracks, self.tracks_stamp = tracks, stamp
        self.scan_stamp, self.scan_receipt = stamp, time.monotonic()
        self.tracks_out.publish(String(data=json.dumps(dict(frame="odom", stamp=stamp,
                                                   tracks=[asdict(t) for t in tracks]))))

    def check_missing(self, stamp, observed=()):
        """Never treat expiration of nearby unseen occupancy as evidence of clearance."""
        if self.pose is None or self.tracks_stamp is None:
            return
        dt = max(0.0, stamp - self.tracks_stamp)
        for track in self.last_tracks:
            if track.id in observed or stamp - track.last_seen <= TIMEOUT:
                continue
            x, y = track.x + track.vx*dt, track.y + track.vy*dt
            uncertainty = track.uncertainty + 0.3*dt + 0.4*dt*dt
            # Whole body plus maximum travel during the planning horizon.
            c = self.config
            reach = c.robot_radius + c.margin + c.speed*c.horizon + track.radius + uncertainty
            if math.hypot(x-self.pose[0], y-self.pose[1]) <= reach:
                self.perception_failed = True
                return  # Restart/reobserve in a new session; receipt freshness cannot clear this.

    def ready(self, now, stamp):
        return (self.pose is not None and all(t is not None and 0 <= now-t <= TIMEOUT
                for t in (self.scan_receipt, self.odom_receipt, self.nav_receipt))
                and all(t is not None and 0 <= stamp-t <= TIMEOUT
                        for t in (self.scan_stamp, self.odom_stamp)))

    def on_timer(self):
        now, stamp = time.monotonic(), self.clock()
        if self.failed or not self.active:
            self.stop("route_failed" if self.failed else "idle")
            return
        self.check_missing(stamp)
        if self.perception_failed:
            self.stop("lost_obstacle", True)
            return
        if not self.ready(now, stamp):
            self.stop("stale_input", True)
            return
        try:
            result = self.planner.plan(self.pose, self.velocity, self.desired,
                                       self.tracker.tracks_at(stamp), stamp)
            # A slow solve cannot renew an obsolete movement request.
            if time.monotonic()-now > MAX_SOLVE_TIME or not self.ready(time.monotonic(), self.clock()):
                self.stop("planning_deadline", True)
                return
            if not np.isfinite(result.command).all():
                raise ValueError("nonfinite plan")
            msg = Twist()
            msg.linear.x, msg.linear.y, msg.angular.z = map(float, result.command)
            self.out.publish(msg)
            self.status(result.mode, result.alert)
        except (ValueError, RuntimeError, FloatingPointError):
            self.stop("planning_failure", True)


def main():
    rclpy.init(args=sys.argv)
    node = rclpy.create_node("predictive_avoidance")
    guard = PredictiveAvoidance(node)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        guard.out.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
