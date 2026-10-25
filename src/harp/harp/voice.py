"""Listens to people: microphone -> speech-to-text -> /harp/heard.

For now it publishes a fake utterance every few seconds.

Publishes:
  /harp/heard  harp_interfaces/Utterance
"""

import rclpy
from harp_interfaces.msg import Utterance
from rclpy.node import Node


class Voice(Node):
    def __init__(self):
        super().__init__("voice")
        self.fake_speech = self.declare_parameter("fake_speech", True).value
        period = self.declare_parameter("fake_speech_period", 5.0).value

        self.pub = self.create_publisher(Utterance, "/harp/heard", 10)

        if self.fake_speech:
            self.create_timer(period, self.publish_fake_utterance)
            self.get_logger().info("Publishing FAKE speech on /harp/heard")
        else:
            # TODO: read the G1 mic stream (UDP multicast), run speech-to-text.
            self.get_logger().warn("Real speech input is not implemented yet; publishing nothing")

    def publish_fake_utterance(self):
        msg = Utterance()
        msg.stamp = self.get_clock().now().to_msg()
        msg.text = "Hello robot, how are you?"
        msg.confidence = 1.0
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = Voice()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
