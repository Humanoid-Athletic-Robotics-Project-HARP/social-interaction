"""Turns raw sensor data (lidar, depth camera) into people the robot can see.

For now it only publishes a fake person drifting left and right in front of
the robot, so the rest of the system has something to react to.

Publishes:
  /harp/people  harp_interfaces/PersonArray
"""

import math

import rclpy
from harp_interfaces.msg import Person, PersonArray
from rclpy.node import Node


class Perception(Node):
    def __init__(self):
        super().__init__("perception")
        self.fake_people = self.declare_parameter("fake_people", True).value
        rate = self.declare_parameter("publish_rate", 5.0).value

        self.pub = self.create_publisher(PersonArray, "/harp/people", 10)

        if self.fake_people:
            self.start_time = self.get_clock().now()
            self.create_timer(1.0 / rate, self.publish_fake_people)
            self.get_logger().info("Publishing a FAKE person on /harp/people")
        else:
            # TODO: subscribe to the MID-360 lidar and D435i camera, detect people.
            self.get_logger().warn("Real perception is not implemented yet; publishing nothing")

    def publish_fake_people(self):
        t = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        person = Person()
        person.id = "fake-0"
        person.bearing = 0.6 * math.sin(0.2 * t)
        person.distance = 1.5
        person.facing_robot = True

        msg = PersonArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.people = [person]
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = Perception()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
