"""The social "brain": decides what the robot says and how it moves.

For now: echo back anything it hears, and turn to face the nearest person.
This is where the LLM will eventually plug in.

Subscribes:
  /harp/heard    harp_interfaces/Utterance
  /harp/people   harp_interfaces/PersonArray
Publishes:
  /harp/say      std_msgs/String
  /harp/cmd_vel  geometry_msgs/Twist
"""

import rclpy
from geometry_msgs.msg import Twist
from harp_interfaces.msg import PersonArray, Utterance
from rclpy.node import Node
from std_msgs.msg import String


class Behavior(Node):
    def __init__(self):
        super().__init__("behavior")
        self.turn_gain = self.declare_parameter("turn_gain", 0.8).value
        self.turn_deadband = self.declare_parameter("turn_deadband", 0.1).value

        self.say_pub = self.create_publisher(String, "/harp/say", 10)
        self.cmd_pub = self.create_publisher(Twist, "/harp/cmd_vel", 10)

        self.create_subscription(Utterance, "/harp/heard", self.on_heard, 10)
        self.create_subscription(PersonArray, "/harp/people", self.on_people, 10)

    def on_heard(self, msg):
        # TODO: send to the LLM and speak its reply instead of echoing.
        self.get_logger().info(f"heard: {msg.text!r}")
        self.say_pub.publish(String(data=f"You said: {msg.text}"))

    def on_people(self, msg):
        if not msg.people:
            return
        nearest = min(msg.people, key=lambda p: p.distance)

        cmd = Twist()
        if abs(nearest.bearing) > self.turn_deadband:
            cmd.angular.z = self.turn_gain * nearest.bearing
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = Behavior()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
