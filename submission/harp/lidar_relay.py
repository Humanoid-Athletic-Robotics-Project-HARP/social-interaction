"""The robot's head lidar into our ROS 2 graph, for Nav2 on the robot (in the sim,
harp/lidar.py stands in for it). entrypoint.sh starts it outside the sim.

On the G1, Unitree's lidar service (lidar_driver, which must be switched on:
context/robot-services.md) publishes rt/utlidar/cloud_livox_mid360 (PointCloud2, about
9.4 Hz, livox_frame) on the robot's DDS domain (ROBOT_DOMAIN, 0)
over its network (ROBOT_INTERFACE, eth0), which HIM's launcher passes to our
image; our ROS 2 runs on HIM's localhost graph instead. This reads the cloud
through Unitree's SDK (its own CycloneDDS beside ROS's Fast DDS) and publishes
it with two changes. The time: the robot stamps it with its own clock, and Nav2
needs stamps that match /tf, so each cloud gets its arrival time (at most about
50-100 ms late). The robot's own body: harp/self_filter.py cuts out the shoulders,
arms and hands, posed by /joint_states over the cloud's last JOINT_WINDOW s (the
lidar's 100 ms, smeared over a moving arm); with none for JOINT_TIMEOUT s (harp_node
isn't running) clouds go out whole. Every other field of a kept point is unchanged. It also publishes the cloud turned 180 deg about the lidar's z
(lidar_mount.REAR_FRAME, +x pointing back), which Nav2's clearing reads so that its
view, which STVL centres on +x, can leave out the blind cone ahead; the sim lidar
publishes both the same way.

  Subscribes  rt/utlidar/cloud_livox_mid360  (DDS, the robot's domain)
              /joint_states                  sensor_msgs/JointState (harp/state.py)
  Publishes   /utlidar/cloud_livox_mid360    sensor_msgs/PointCloud2, restamped, without the body
              /utlidar/cloud_rear            the same in livox_rear
"""

import array
from collections import deque
from contextlib import ExitStack
import math
import os
import signal
import sys
import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, PointCloud2, PointField
from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.idl.sensor_msgs.msg.dds_ import PointCloud2_

from harp import lidar_mount
from harp.self_filter import NUM_MOTORS, SelfFilter

ROBOT_TOPIC = "rt/utlidar/cloud_livox_mid360"   # Unitree's lidar service
TOPIC = lidar_mount.TOPIC                       # what Nav2 marks with
QUIET = 5.0   # s without a cloud before warning
JOINT_WINDOW = 0.15   # s of /joint_states a cloud is checked against: its 100 ms and its delay
JOINT_TIMEOUT = 0.5   # s without /joint_states before clouds go out with the body in them
MAX_POSES = 4         # joint states a cloud is checked against at most, spread over the window


def point_rows(data, point_step):
    """A cloud's point bytes as one row of point_step bytes per point."""
    points = np.frombuffer(bytes(data), np.uint8)
    return points[:len(points) - len(points) % point_step].reshape(-1, point_step)


def xyz(rows, fields, bigendian):
    """Each point's x, y, z (N x 3) from its row of bytes (point_rows)."""
    kind = ">f4" if bigendian else "<f4"
    by_name = {f.name: f for f in fields}
    if any(c not in by_name or by_name[c].datatype != PointField.FLOAT32 for c in "xyz"):
        raise ValueError("x, y and z must be float32")
    return np.stack([rows[:, by_name[c].offset:by_name[c].offset + 4].copy().view(kind)[:, 0]
                     for c in "xyz"], axis=1).astype(float)


def turned(data, fields, point_step, bigendian):
    """A cloud's point bytes with x and y negated (turned 180 deg about z); every other
    field (intensity, ring, time) left as it is."""
    points = point_rows(data, point_step).copy()
    kind = ">f4" if bigendian else "<f4"
    for f in fields:
        if f.name in ("x", "y"):
            if f.datatype != PointField.FLOAT32:
                raise ValueError("x and y must be float32")
            column = points[:, f.offset:f.offset + 4].copy().view(kind)
            # negating gives native byte order: back to the cloud's before the bytes go in
            points[:, f.offset:f.offset + 4] = (-column).astype(kind).view(np.uint8)
    return points.tobytes()


class LidarRelay(Node):
    def __init__(self):
        super().__init__("lidar_relay")
        self.pub = self.create_publisher(PointCloud2, TOPIC, 10)
        self.rear_pub = self.create_publisher(PointCloud2, lidar_mount.REAR_TOPIC, 10)
        self.last = None              # time.monotonic() of the last cloud
        self.quiet_since = time.monotonic()
        self.create_timer(1.0, self.check_quiet)
        self.self_filter = SelfFilter()
        self.joints = deque(maxlen=100)   # (time.monotonic(), joint angles), from /joint_states
        self.joints_lock = threading.Lock()   # clouds come on the SDK's thread
        self.filtering = None         # whether the last cloud was self-filtered, for messages
        self.create_subscription(JointState, "/joint_states", self.on_joints, 10)

    def on_joints(self, msg):
        if len(msg.position) >= NUM_MOTORS:
            with self.joints_lock:
                self.joints.append((time.monotonic(), np.array(msg.position[:NUM_MOTORS])))

    def poses(self):
        """The joint angles to check a cloud arriving now against (none if they're stale)."""
        now = time.monotonic()
        with self.joints_lock:
            joints = list(self.joints)
        if not joints or now - joints[-1][0] > JOINT_TIMEOUT:
            return []
        recent = [q for t, q in joints if t >= now - JOINT_WINDOW] or [joints[-1][1]]
        step = math.ceil(len(recent) / MAX_POSES)
        return recent[::-1][::step]   # the newest first, always included

    def without_body(self, msg, rows):
        """The rows of points outside the robot's body; all of them without joint states."""
        poses = self.poses()
        if not poses:
            self.note(False, f"no /joint_states within {JOINT_TIMEOUT} s: clouds go out with "
                      "the robot's body in them")
            return rows
        try:
            keep = self.self_filter.keep(xyz(rows, msg.fields, msg.is_bigendian), poses)
        except ValueError as e:
            self.note(False, f"not self-filtered: {e}")
            return rows
        self.note(True, "the robot's body is cut out of the cloud (/joint_states)")
        return rows[keep]

    def note(self, filtering, text):
        """Prints text when the cloud starts or stops being self-filtered."""
        if filtering != self.filtering:
            print(f"Lidar: {text}.", flush=True)
            self.filtering = filtering

    def on_cloud(self, msg):
        """From the SDK's reader thread: one cloud, restamped."""
        now = self.get_clock().now()
        out = PointCloud2()
        out.header.stamp = now.to_msg()
        out.header.frame_id = msg.header.frame_id
        out.fields = [PointField(name=f.name, offset=f.offset, datatype=f.datatype, count=f.count)
                      for f in msg.fields]
        out.is_bigendian, out.is_dense = msg.is_bigendian, msg.is_dense
        data = self.without_body(out, point_rows(msg.data, msg.point_step)).tobytes()
        out.height, out.width = 1, len(data) // msg.point_step
        out.point_step, out.row_step = msg.point_step, len(data)
        # array.array, not bytes: Foxy's rclpy checks bytes one by one (about 70 ms a cloud)
        out.data = array.array("B", data)
        self.pub.publish(out)
        rear = PointCloud2(height=out.height, width=out.width, fields=out.fields,
                           is_bigendian=out.is_bigendian, point_step=out.point_step,
                           row_step=out.row_step, is_dense=out.is_dense)
        rear.header.stamp, rear.header.frame_id = out.header.stamp, lidar_mount.REAR_FRAME
        try:
            rear.data = array.array("B", turned(data, out.fields, out.point_step, out.is_bigendian))
            self.rear_pub.publish(rear)
        except ValueError as e:
            print(f"No rear cloud: {e}.", flush=True)
        if self.last is None:
            offset = now.nanoseconds * 1e-9 - (msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)
            print(f"Lidar: {msg.width * msg.height} points in {msg.header.frame_id}, fields "
                  f"{','.join(f.name for f in msg.fields)}; the robot's stamps are {offset:+.3f} s "
                  f"from this clock (replaced by arrival time).", flush=True)
        self.last = time.monotonic()

    def check_quiet(self):
        since = self.last or self.quiet_since
        if time.monotonic() - since > QUIET:
            print(f"No lidar on {ROBOT_TOPIC} for {QUIET:.0f} s.", flush=True)
            self.quiet_since, self.last = time.monotonic(), None


def stop(signum, frame):
    raise KeyboardInterrupt


def main():
    rclpy.init()
    # Undone in reverse, on every way out (a failed Init or spin too): the SDK's reader
    # first, since with it still open the interpreter hangs on the way out.
    with ExitStack() as cleanup:
        cleanup.callback(rclpy.try_shutdown)
        node = LidarRelay()
        cleanup.callback(node.destroy_node)
        domain = int(os.environ.get("ROBOT_DOMAIN", "0"))
        interface = os.environ.get("ROBOT_INTERFACE", "eth0")
        ChannelFactoryInitialize(domain, interface)
        sub = ChannelSubscriber(ROBOT_TOPIC, PointCloud2_)
        cleanup.callback(sub.Close)
        sub.Init(node.on_cloud, 0)
        print(f"Relaying {ROBOT_TOPIC} (domain {domain}, {interface}) to {TOPIC}.", flush=True)
        # Short spins, so SIGTERM (or Ctrl-C) is handled between them.
        signal.signal(signal.SIGTERM, stop)
        try:
            while rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.2)
        except KeyboardInterrupt:
            pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
