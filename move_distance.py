"""Walk the G1 straight forward (or backward) a set distance, then stop.

Uses only Unitree's official ROS 2 interfaces from unitree_ros2:
  - Walking: unitree_api/Request on /api/sport/request, API 7105 (SetVelocity).
    This is the same message the official G1 LocoClient sends
    (unitree_ros2/example/src/include/g1/g1_loco_client.hpp).
  - Position: unitree_go/SportModeState on /lf/odommodestate (20 Hz odometry,
    see the G1 "Odometer Service Interface" doc).

Run with the robot standing in its normal walking mode:
  python3 move_distance.py --ros-args -p distance:=1.0 -p speed:=0.2
"""

import json
import math
import time

import rclpy
from rclpy.node import Node
from unitree_api.msg import Request, Response
from unitree_go.msg import SportModeState

SET_VELOCITY_API_ID = 7105
MAX_SPEED = 0.4           # m/s, hard cap no matter what is passed in
COMMAND_DURATION = 0.5    # s each velocity command stays valid; if this script
                          # dies, the robot stops on its own after this long
COMMAND_PERIOD = 0.1      # s between velocity commands


class MoveDistance(Node):
    def __init__(self):
        super().__init__("move_distance")
        self.distance = self.declare_parameter("distance", 1.0).value
        self.speed = min(abs(self.declare_parameter("speed", 0.2).value), MAX_SPEED)

        self.position = None
        self.create_subscription(SportModeState, "/lf/odommodestate", self.on_odom, 10)
        self.create_subscription(Response, "/api/sport/response", self.on_response, 10)
        self.request_pub = self.create_publisher(Request, "/api/sport/request", 1)

    def on_odom(self, msg):
        self.position = (float(msg.position[0]), float(msg.position[1]))

    def on_response(self, msg):
        if msg.header.identity.api_id == SET_VELOCITY_API_ID and msg.header.status.code != 0:
            self.get_logger().error(
                f"Robot rejected the velocity command (code {msg.header.status.code}). "
                "Is it standing in walking mode?",
                throttle_duration_sec=2.0,
            )

    def send_velocity(self, vx, duration=COMMAND_DURATION):
        req = Request()
        req.header.identity.id = time.monotonic_ns()
        req.header.identity.api_id = SET_VELOCITY_API_ID
        req.parameter = json.dumps({"velocity": [vx, 0.0, 0.0], "duration": duration})
        self.request_pub.publish(req)

    def stop(self):
        try:
            self.send_velocity(0.0, duration=1.0)
            self.get_logger().info("Stop sent")
        except Exception as e:  # e.g. the ROS context is already shut down after Ctrl-C
            self.get_logger().warn(f"Could not send stop ({e}); robot stops within "
                                   f"{COMMAND_DURATION}s anyway")

    def traveled_from(self, start):
        return math.hypot(self.position[0] - start[0], self.position[1] - start[1])

    def spin_for(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def run(self):
        target = abs(self.distance)
        vx = math.copysign(self.speed, self.distance)

        self.get_logger().info("Waiting for odometry on /lf/odommodestate...")
        wait_until = time.monotonic() + 3.0
        while self.position is None:
            if time.monotonic() > wait_until:
                self.get_logger().error("No odometry received. Is setup.sh sourced and the robot on?")
                return
            rclpy.spin_once(self, timeout_sec=0.1)

        start = self.position
        give_up_at = time.monotonic() + 2.0 * target / self.speed + 5.0
        last_sent = 0.0
        self.get_logger().info(f"Walking {self.distance:+.2f} m at {self.speed:.2f} m/s")

        while self.traveled_from(start) < target:
            now = time.monotonic()
            if now > give_up_at:
                self.get_logger().warn("Took too long, stopping early")
                break
            if now - last_sent >= COMMAND_PERIOD:
                self.send_velocity(vx)
                last_sent = now
            rclpy.spin_once(self, timeout_sec=0.02)

        self.stop()
        self.spin_for(1.5)  # let the robot settle, then measure how far it really went
        self.get_logger().info(
            f"Target {target:.2f} m, actually moved {self.traveled_from(start):.2f} m"
        )


def main():
    rclpy.init()
    node = MoveDistance()
    try:
        node.run()
    except KeyboardInterrupt:
        node.get_logger().warn("Interrupted")
        node.stop()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
