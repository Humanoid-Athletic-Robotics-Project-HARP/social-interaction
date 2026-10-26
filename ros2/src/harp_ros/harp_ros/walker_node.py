"""ROS 2 node around our GR00T walker (harp_control): drive the G1 with
velocity commands on /cmd_vel, and get our leg odometry back on /odom and /tf.

The walking itself is unchanged: Walker still runs the 200 Hz loop and the
50 Hz GR00T policy, and talks to the robot or the sim over rt/lowcmd and
rt/lowstate. This node only hands it the newest velocity command and
publishes where the robot estimates it is.

  Subscribes  /cmd_vel        geometry_msgs/Twist  linear.x, linear.y (m/s), angular.z (rad/s)
  Publishes   /odom           nav_msgs/Odometry    odom -> base_link, 50 Hz while the policy runs
              /tf             odom -> base_link
  Services    /walker/start   std_srvs/Trigger     start the policy (like ] in walk.py)
              /walker/stop    std_srvs/Trigger     damp every joint and quit (like o)

If /cmd_vel goes quiet for cmd_timeout seconds, the command drops to zero and
GR00T balances in place. Start-up is the same as walk.py: wait for the stand
pose, lower the feet to the floor, start, then release the band. In a terminal,
] and o work too; the other walk.py keys don't, since /cmd_vel drives.

Sim (start it first: ros2 run harp_sim run_sim, see README):
  ros2 run harp_ros walker_node
  ros2 service call /walker/start std_srvs/srv/Trigger
  ros2 run teleop_twist_keyboard teleop_twist_keyboard
Real robot (untested; hang it, damp it, then debug mode L2+R2 first):
  ros2 run harp_ros walker_node --ros-args -p real:=true -p interface:=en0
"""

import math
import queue
import sys
import threading
import time

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster
from unitree_sdk2py.core.channel import ChannelFactoryInitialize

from harp_control.gr00t.walk import (CONTROL_DT, MAX_VX, MAX_VY, MAX_YAW_RATE, TerminalKeys,
                                     Walker, check_unitree_controller_off)
from harp_control.leg_odometry import DEFAULT_SCENE, LegOdometry

CMD_LIMITS = np.array([MAX_VX, MAX_VY, MAX_YAW_RATE], dtype=np.float32)
VELOCITY_SMOOTHING = 0.2  # low-pass factor per policy step for the velocity in /odom


class WalkerNode(Node):
    """/cmd_vel into Walker.cmd; leg odometry out on /odom and /tf."""

    def __init__(self):
        super().__init__("walker")
        self.declare_parameter("real", False)
        self.declare_parameter("interface", "")
        self.declare_parameter("scene", DEFAULT_SCENE or "")
        self.declare_parameter("cmd_timeout", 0.5)
        self.cmd_timeout = self.get_parameter("cmd_timeout").value
        self.cmd = np.zeros(3, dtype=np.float32)
        self.cmd_time = -math.inf
        self.requests = queue.Queue()   # ] and o from the services, read by Walker.run
        self.walker = None
        self.odom = None
        self.last = None                # x, y, heading at the previous policy step
        self.velocity = np.zeros(3)     # body-frame vx, vy, yaw rate, smoothed

    def attach(self, walker, odometry):
        """Start talking ROS once the walker exists."""
        self.walker = walker
        self.odom = odometry
        self.create_subscription(Twist, "/cmd_vel", self.on_cmd_vel, 10)
        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self.tf = TransformBroadcaster(self)
        self.create_service(Trigger, "/walker/start", self.on_start)
        self.create_service(Trigger, "/walker/stop", self.on_stop)

    def on_cmd_vel(self, msg):
        self.cmd = np.array([msg.linear.x, msg.linear.y, msg.angular.z], dtype=np.float32)
        self.cmd_time = time.monotonic()

    def on_start(self, request, response):
        if self.walker.policy_on:
            response.success, response.message = True, "The policy is already running."
        elif not self.walker.standing:
            response.success = False
            response.message = "Not yet: wait for the stand pose (about 3 s after start)."
        else:
            self.requests.put("]")
            response.success, response.message = True, "Starting the policy."
        return response

    def on_stop(self, request, response):
        self.requests.put("o")
        response.success, response.message = True, "Stopping: every joint damps and the robot sinks."
        return response

    def read_key(self, terminal):
        """Next ] or o, from the services first, then the terminal if there is one."""
        try:
            return self.requests.get_nowait()
        except queue.Empty:
            pass
        key = terminal() if terminal else None
        return key if key in ("]", "o") else None

    def on_policy_step(self, q, quat):
        self.odom.update(q, quat)
        fresh = time.monotonic() - self.cmd_time < self.cmd_timeout
        cmd = self.cmd if fresh else np.zeros(3, dtype=np.float32)
        self.walker.cmd = np.clip(cmd, -CMD_LIMITS, CMD_LIMITS).astype(np.float32)
        self.publish_odometry()

    def publish_odometry(self):
        # The odom frame is the one the leg odometry integrates in: its origin is
        # where the policy started, its axes follow the IMU's yaw reference.
        x, y = self.odom.position
        yaw, heading = self.odom.yaw, self.odom.heading
        if self.last is not None:
            dx, dy = x - self.last[0], y - self.last[1]
            c, s = math.cos(yaw), math.sin(yaw)
            measured = np.array([c * dx + s * dy, -s * dx + c * dy,
                                 heading - self.last[2]]) / CONTROL_DT
            self.velocity += VELOCITY_SMOOTHING * (measured - self.velocity)
        self.last = (x, y, heading)

        stamp = self.get_clock().now().to_msg()
        rotation = (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x, odom.pose.pose.position.y = float(x), float(y)
        o = odom.pose.pose.orientation
        o.x, o.y, o.z, o.w = rotation
        odom.twist.twist.linear.x, odom.twist.twist.linear.y = map(float, self.velocity[:2])
        odom.twist.twist.angular.z = float(self.velocity[2])
        self.odom_pub.publish(odom)

        tf = TransformStamped()
        tf.header = odom.header
        tf.child_frame_id = "base_link"
        tf.transform.translation.x, tf.transform.translation.y = float(x), float(y)
        r = tf.transform.rotation
        r.x, r.y, r.z, r.w = rotation
        self.tf.sendTransform(tf)


def main():
    # Ctrl-C has to reach the walker loop, which damps the robot on the way out.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = WalkerNode()
    real = node.get_parameter("real").value
    interface = node.get_parameter("interface").value
    scene = node.get_parameter("scene").value
    error = None
    if real and not interface:
        error = "real:=true needs interface:=<the one on 192.168.123.x>"
    elif not scene:
        error = "no G1 model: set the scene parameter, or HARP_G1_SCENE (the ROS 2 environment does)"
    if error:
        print(f"Error: {error}", file=sys.stderr)
        node.destroy_node()
        rclpy.shutdown()
        return 1

    ChannelFactoryInitialize(0 if real else 1, interface or ("lo0" if sys.platform == "darwin" else "lo"))
    if real:
        check_unitree_controller_off()
    walker = Walker(real=real)
    walker.next_hint = ("Drive it with /cmd_vel (geometry_msgs/Twist). "
                        "ros2 service call /walker/stop std_srvs/srv/Trigger (or o here) stops.")
    node.attach(walker, LegOdometry(scene))
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()
    print("Start the policy with ] here or: ros2 service call /walker/start std_srvs/srv/Trigger")

    code = 0
    try:
        if sys.stdin.isatty():
            with TerminalKeys() as keys:
                walker.run(lambda: node.read_key(keys.read), node.on_policy_step)
        else:
            walker.run(lambda: node.read_key(None), node.on_policy_step)
    except KeyboardInterrupt:
        print("Interrupted (Ctrl-C).")
    except Exception as e:
        print(f"Error: {e}", file=sys.stderr)
        code = 1
    walker.damp()
    executor.shutdown()
    node.destroy_node()
    rclpy.shutdown()
    return code


if __name__ == "__main__":
    raise SystemExit(main())
