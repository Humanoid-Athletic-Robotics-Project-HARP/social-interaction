"""The robot's head lidar into our ROS 2 graph, for Nav2 on the robot (in the sim,
harp/lidar.py stands in for it). entrypoint.sh starts it outside the sim.

On the G1, Unitree's lidar service publishes rt/utlidar/cloud_livox_mid360
(PointCloud2, 10 Hz, livox_frame) on the robot's DDS domain (ROBOT_DOMAIN, 0)
over its network (ROBOT_INTERFACE, eth0), which HIM's launcher passes to our
image; our ROS 2 runs on HIM's localhost graph instead. This reads the cloud
through Unitree's SDK (its own CycloneDDS beside ROS's Fast DDS) and publishes
it unchanged except for the time: the robot stamps it with its own clock, and
Nav2 needs stamps that match /tf, so each cloud gets its arrival time (at most
about 50-100 ms late).

  Subscribes  rt/utlidar/cloud_livox_mid360  (DDS, the robot's domain)
  Publishes   /utlidar/cloud_livox_mid360    sensor_msgs/PointCloud2, restamped
"""

from contextlib import ExitStack
import os
import signal
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.idl.sensor_msgs.msg.dds_ import PointCloud2_

ROBOT_TOPIC = "rt/utlidar/cloud_livox_mid360"   # Unitree's lidar service
TOPIC = "/utlidar/cloud_livox_mid360"           # what Nav2 reads
QUIET = 5.0   # s without a cloud before warning


class LidarRelay(Node):
    def __init__(self):
        super().__init__("lidar_relay")
        self.pub = self.create_publisher(PointCloud2, TOPIC, 10)
        self.last = None              # time.monotonic() of the last cloud
        self.quiet_since = time.monotonic()
        self.create_timer(1.0, self.check_quiet)

    def on_cloud(self, msg):
        """From the SDK's reader thread: one cloud, restamped."""
        now = self.get_clock().now()
        out = PointCloud2()
        out.header.stamp = now.to_msg()
        out.header.frame_id = msg.header.frame_id
        out.height, out.width = msg.height, msg.width
        out.fields = [PointField(name=f.name, offset=f.offset, datatype=f.datatype, count=f.count)
                      for f in msg.fields]
        out.is_bigendian, out.is_dense = msg.is_bigendian, msg.is_dense
        out.point_step, out.row_step = msg.point_step, msg.row_step
        out.data = bytes(msg.data)
        self.pub.publish(out)
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
