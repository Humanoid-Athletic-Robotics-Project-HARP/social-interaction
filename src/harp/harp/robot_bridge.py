"""The only node that talks to the robot.

Everything that moves the robot or makes it speak goes through here, so the
speed limits, the command watchdog and dry-run mode all live in one place.
With dry_run on (the default), commands are logged instead of sent.

Subscribes:
  /harp/say      std_msgs/String       text for the robot to speak
  /harp/cmd_vel  geometry_msgs/Twist   body velocity (linear.x m/s, angular.z rad/s)
"""

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String


def clamp(value, limit):
    return max(-limit, min(limit, value))


class RobotBridge(Node):
    def __init__(self):
        super().__init__("robot_bridge")
        self.dry_run = self.declare_parameter("dry_run", True).value
        self.max_vx = self.declare_parameter("max_vx", 0.3).value
        self.max_vyaw = self.declare_parameter("max_vyaw", 0.5).value
        self.cmd_timeout = self.declare_parameter("cmd_timeout", 0.5).value

        if not self.dry_run:
            # TODO: connect to the robot, through the HARP /safety layer if the
            # course requires it, otherwise unitree_sdk2py (AudioClient for speech,
            # LocoClient for walking). Never send rt/lowcmd from here.
            raise RuntimeError("Hardware mode is not implemented yet; set dry_run: true")

        self.moving = False
        self.last_cmd_time = self.get_clock().now()

        self.create_subscription(String, "/harp/say", self.on_say, 10)
        self.create_subscription(Twist, "/harp/cmd_vel", self.on_cmd_vel, 10)
        self.create_timer(0.1, self.check_watchdog)

        self.get_logger().info("Started in DRY RUN mode: commands are logged, not sent")

    def on_say(self, msg):
        self.get_logger().info(f"[dry run] say: {msg.data!r}")

    def on_cmd_vel(self, msg):
        vx = clamp(msg.linear.x, self.max_vx)
        vyaw = clamp(msg.angular.z, self.max_vyaw)
        self.last_cmd_time = self.get_clock().now()
        self.moving = vx != 0.0 or vyaw != 0.0
        self.get_logger().info(
            f"[dry run] move: vx={vx:+.2f} m/s, vyaw={vyaw:+.2f} rad/s",
            throttle_duration_sec=1.0,
        )

    def check_watchdog(self):
        # Stop the robot if whoever was commanding it goes quiet.
        age = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
        if self.moving and age > self.cmd_timeout:
            self.moving = False
            self.get_logger().warn(f"[dry run] no command for {age:.1f}s, stopping")


def main(args=None):
    rclpy.init(args=args)
    node = RobotBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
