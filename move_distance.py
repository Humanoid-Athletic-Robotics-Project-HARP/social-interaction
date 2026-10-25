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
import sys
import time
from collections import deque

import rclpy
from rclpy.node import Node
from unitree_api.msg import Request, Response
from unitree_go.msg import SportModeState

try:
    from rclpy.signals import SignalHandlerOptions
except ImportError:  # Foxy: Ctrl-C doesn't shut down the ROS context, nothing to disable
    SignalHandlerOptions = None

SET_VELOCITY_API_ID = 7105
MAX_SPEED = 0.4           # m/s, hard cap no matter what is passed in
COMMAND_DURATION = 0.5    # requested validity; actual robot expiry needs testing
COMMAND_PERIOD = 0.1      # s between velocity commands
ODOM_TIMEOUT = 0.5        # s without valid position feedback before aborting
STOP_REPEATS = 5          # stop commands sent COMMAND_PERIOD apart, in case some are lost


class MoveDistance(Node):
    def __init__(self):
        super().__init__("move_distance")
        self.distance = self.declare_parameter("distance", 1.0).value
        self.speed = self.declare_parameter("speed", 0.2).value
        if not all(math.isfinite(v) for v in (self.distance, self.speed)) or self.speed <= 0:
            raise ValueError("distance must be finite; speed must be positive and finite")
        self.speed = min(self.speed, MAX_SPEED)

        self.position = None
        self.last_odom = 0.0
        self.request_ids = deque(maxlen=32)
        self.command_error = None
        self.moving = False       # a nonzero velocity was sent and no stop has finished since
        self.create_subscription(SportModeState, "/lf/odommodestate", self.on_odom, 10)
        self.create_subscription(Response, "/api/sport/response", self.on_response, 10)
        self.request_pub = self.create_publisher(Request, "/api/sport/request", 1)

    def on_odom(self, msg):
        position = (float(msg.position[0]), float(msg.position[1]))
        if all(math.isfinite(v) for v in position):
            self.position = position
            self.last_odom = time.monotonic()

    def on_response(self, msg):
        if (msg.header.identity.api_id == SET_VELOCITY_API_ID
                and msg.header.identity.id in self.request_ids
                and msg.header.status.code != 0):
            self.command_error = f"Velocity command rejected (code {msg.header.status.code})"

    def send_velocity(self, vx, duration=COMMAND_DURATION):
        req = Request()
        req.header.identity.id = time.monotonic_ns()
        self.request_ids.append(req.header.identity.id)
        req.header.identity.api_id = SET_VELOCITY_API_ID
        req.parameter = json.dumps({"velocity": [vx, 0.0, 0.0], "duration": duration})
        if vx != 0:
            self.moving = True
        self.request_pub.publish(req)

    def stop(self):
        try:
            for i in range(STOP_REPEATS):
                if i:
                    time.sleep(COMMAND_PERIOD)
                self.send_velocity(0.0, duration=1.0)
        except Exception as e:  # e.g. the ROS context is already shut down
            self.get_logger().warn(f"Could not send stop: {e}")
            return
        self.moving = False
        self.get_logger().info(f"Stop requested ({STOP_REPEATS} commands sent)")

    def traveled_from(self, start):
        return math.hypot(self.position[0] - start[0], self.position[1] - start[1])

    def spin_for(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def run(self):
        target = abs(self.distance)
        vx = math.copysign(self.speed, self.distance)
        if target == 0:
            return

        self.get_logger().info("Waiting for odometry on /lf/odommodestate...")
        wait_until = time.monotonic() + 3.0
        while self.position is None:
            if time.monotonic() > wait_until:
                raise RuntimeError("No position feedback received")
            rclpy.spin_once(self, timeout_sec=0.1)

        start = self.position
        give_up_at = time.monotonic() + 2.0 * target / self.speed + 5.0
        last_sent = 0.0
        self.get_logger().info(f"Walking {self.distance:+.2f} m at {self.speed:.2f} m/s")

        while self.traveled_from(start) < target:
            now = time.monotonic()
            if self.command_error:
                raise RuntimeError(self.command_error)
            if now - self.last_odom > ODOM_TIMEOUT:
                raise RuntimeError("Position feedback timed out")
            if now > give_up_at:
                raise RuntimeError("Movement timed out")
            if now - last_sent >= COMMAND_PERIOD:
                self.send_velocity(vx)
                last_sent = now
            rclpy.spin_once(self, timeout_sec=0.02)

        self.stop()
        self.spin_for(1.5)
        if self.command_error:
            raise RuntimeError(self.command_error)
        if time.monotonic() - self.last_odom > ODOM_TIMEOUT:
            raise RuntimeError("Position feedback lost; final distance unknown")
        self.get_logger().info(
            f"Target {target:.2f} m, estimated displacement {self.traveled_from(start):.2f} m"
        )


def main():
    if SignalHandlerOptions is None:
        rclpy.init()
    else:
        # Galactic and later shut down the ROS context on Ctrl-C, before our
        # cleanup runs, so the stop could never be published. Keep it alive and
        # let Python raise KeyboardInterrupt as usual.
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = None
    try:
        node = MoveDistance()
        node.run()
    except KeyboardInterrupt:
        print("Interrupted", file=sys.stderr)
        return 130
    except Exception as e:
        print(f"Error: {e}", file=sys.stderr)
        return 1
    finally:
        if node is not None:
            if node.moving:
                node.stop()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
