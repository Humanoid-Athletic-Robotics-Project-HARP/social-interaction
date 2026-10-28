"""The head lidar into our ROS 2 domain: reads the point cloud the robot
publishes on its own DDS domain and republishes it where Nav2 reads it.

On the G1, Unitree's lidar service publishes rt/utlidar/cloud_livox_mid360
(sensor_msgs PointCloud2, 10 Hz, livox_frame) on domain 0, on the robot's
network. Our ROS 2 runs on domain 2 over loopback, so it never sees it. This
node reads it through unitree_sdk2py, as walker_node reads rt/lowstate, and
publishes it unchanged except for the time: the robot stamps it with its own
clock, which isn't the laptop's, and Nav2 needs stamps that match /tf. So each
cloud is stamped with when it arrived (about 50-100 ms late at most).

run_sim --lidar-dds publishes the sim's lidar the same way, so course tests go
through this node too.

  Subscribes  rt/utlidar/cloud_livox_mid360  (DDS, the robot's domain)
  Publishes   /utlidar/cloud_livox_mid360    sensor_msgs/PointCloud2, restamped

Sim (run_sim --lidar-dds, then):
  ros2 run harp_ros lidar_relay
Real robot (untested):
  ros2 run harp_ros lidar_relay --ros-args -p real:=true -p interface:=en0
"""

from contextlib import ExitStack
import sys
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.idl.sensor_msgs.msg.dds_ import PointCloud2_

ROBOT_TOPIC = "rt/utlidar/cloud_livox_mid360"   # Unitree's lidar service
TOPIC = "/utlidar/cloud_livox_mid360"           # what Nav2 and run_recorder read
QUIET = 5.0   # s without a cloud before warning


class LidarRelay(Node):
    def __init__(self):
        super().__init__("lidar_relay")
        self.declare_parameter("real", False)
        self.declare_parameter("interface", "")
        # DDS domain of the robot's own topics, as in walker_node: -1 is the robot's 0
        # with real:=true, the sim's 1 otherwise.
        self.declare_parameter("domain", -1)
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


def main():
    rclpy.init()
    with ExitStack() as cleanup:
        cleanup.callback(rclpy.try_shutdown)
        node = LidarRelay()
        cleanup.callback(node.destroy_node)
        real = node.get_parameter("real").value
        interface = node.get_parameter("interface").value
        if real and not interface:
            print("Error: real:=true needs interface:=<the one on 192.168.123.x>", file=sys.stderr)
            return 1
        domain = node.get_parameter("domain").value
        if domain < 0:
            domain = 0 if real else 1
        interface = interface or ("lo0" if sys.platform == "darwin" else "lo")
        ChannelFactoryInitialize(domain, interface)
        sub = ChannelSubscriber(ROBOT_TOPIC, PointCloud2_)
        # Close the SDK reader before ROS teardown, including failed Init/spin.
        # Leaving it open can hang Python during interpreter shutdown.
        cleanup.callback(sub.Close)
        sub.Init(node.on_cloud, 0)
        print(f"Relaying {ROBOT_TOPIC} (domain {domain}, {interface}) to {TOPIC}.", flush=True)
        try:
            rclpy.spin(node)
        except (KeyboardInterrupt, ExternalShutdownException):
            pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
