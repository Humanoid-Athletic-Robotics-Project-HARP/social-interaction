"""Walk the G1 straight forward (or backward) a set distance, or turn it in
place by a set angle, then stop. One move per run.

Uses only Unitree's official ROS 2 interfaces from unitree_ros2:
  - Moving: unitree_api/Request on /api/sport/request, API 7105 (SetVelocity).
    This is the same message the official G1 LocoClient sends
    (unitree_ros2/example/src/include/g1/g1_loco_client.hpp).
  - Position and heading: unitree_go/SportModeState on /lf/odommodestate
    (20 Hz odometry, see the G1 "Odometer Service Interface" doc). Heading is
    imu_state.rpy[2].

Run with the robot standing in its normal walking mode:
  python3 move_distance.py --ros-args -p distance:=1.0 -p speed:=0.2
  python3 move_distance.py --ros-args -p angle:=90.0 -p turn_speed:=20.0
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
MAX_TURN_SPEED = 30.0     # deg/s, hard cap no matter what is passed in
COMMAND_DURATION = 0.5    # requested validity; actual robot expiry needs testing
COMMAND_PERIOD = 0.1      # s between velocity commands
ODOM_TIMEOUT = 0.5        # s without valid odometry before aborting
STOP_REPEATS = 5          # stop commands sent COMMAND_PERIOD apart, in case some are lost
WRONG_WAY_LIMIT = math.radians(10.0)  # abort if a turn goes this far the wrong way


def wrap_angle(a):
    """Wrap an angle in radians to [-pi, pi)."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class MoveDistance(Node):
    def __init__(self):
        super().__init__("move_distance")
        self.distance = self.declare_parameter("distance", 0.0).value
        self.speed = self.declare_parameter("speed", 0.2).value
        self.angle = self.declare_parameter("angle", 0.0).value
        self.turn_speed = self.declare_parameter("turn_speed", 20.0).value
        values = (self.distance, self.speed, self.angle, self.turn_speed)
        if not all(math.isfinite(v) for v in values):
            raise ValueError("distance, speed, angle and turn_speed must be finite")
        if self.speed <= 0 or self.turn_speed <= 0:
            raise ValueError("speed and turn_speed must be positive")
        if (self.distance != 0) == (self.angle != 0):
            raise ValueError("set exactly one of distance or angle (one move per run)")
        self.speed = min(self.speed, MAX_SPEED)
        self.turn_speed = min(self.turn_speed, MAX_TURN_SPEED)

        self.position = None
        self.yaw = None           # latest heading, radians, wrapped
        self.heading = 0.0        # unwrapped heading: sum of yaw changes, radians
        self.last_odom = 0.0
        self.request_ids = deque(maxlen=32)
        self.command_error = None
        self.moving = False       # a nonzero velocity was sent and no stop has finished since
        self.create_subscription(SportModeState, "/lf/odommodestate", self.on_odom, 10)
        self.create_subscription(Response, "/api/sport/response", self.on_response, 10)
        self.request_pub = self.create_publisher(Request, "/api/sport/request", 1)

    def on_odom(self, msg):
        position = (float(msg.position[0]), float(msg.position[1]))
        yaw = float(msg.imu_state.rpy[2])
        if not all(math.isfinite(v) for v in (*position, yaw)):
            return
        if self.yaw is not None:
            self.heading += wrap_angle(yaw - self.yaw)
        self.position = position
        self.yaw = yaw
        self.last_odom = time.monotonic()

    def on_response(self, msg):
        if (msg.header.identity.api_id == SET_VELOCITY_API_ID
                and msg.header.identity.id in self.request_ids
                and msg.header.status.code != 0):
            self.command_error = f"Velocity command rejected (code {msg.header.status.code})"

    def send_velocity(self, vx, omega=0.0, duration=COMMAND_DURATION):
        req = Request()
        req.header.identity.id = time.monotonic_ns()
        self.request_ids.append(req.header.identity.id)
        req.header.identity.api_id = SET_VELOCITY_API_ID
        req.parameter = json.dumps({"velocity": [vx, 0.0, omega], "duration": duration})
        if vx != 0 or omega != 0:
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

    def drive(self, vx, omega, progress, target, rate, wrong_way_limit=math.inf):
        """Command (vx, omega) until progress() reaches target, then stop and settle.

        rate is the expected progress per second, used for the timeout.
        """
        give_up_at = time.monotonic() + 2.0 * target / rate + 5.0
        last_sent = 0.0
        while progress() < target:
            now = time.monotonic()
            if self.command_error:
                raise RuntimeError(self.command_error)
            if now - self.last_odom > ODOM_TIMEOUT:
                raise RuntimeError("Odometry timed out")
            if now > give_up_at:
                raise RuntimeError("Movement timed out")
            if progress() < -wrong_way_limit:
                raise RuntimeError("Turning the wrong way; check the sign of omega vs yaw")
            if now - last_sent >= COMMAND_PERIOD:
                self.send_velocity(vx, omega)
                last_sent = now
            rclpy.spin_once(self, timeout_sec=0.02)

        self.stop()
        self.spin_for(1.5)
        if self.command_error:
            raise RuntimeError(self.command_error)
        if time.monotonic() - self.last_odom > ODOM_TIMEOUT:
            raise RuntimeError("Odometry lost; final result unknown")

    def walk(self):
        target = abs(self.distance)
        start = self.position
        self.get_logger().info(f"Walking {self.distance:+.2f} m at {self.speed:.2f} m/s")
        self.drive(math.copysign(self.speed, self.distance), 0.0,
                   lambda: self.traveled_from(start), target, self.speed)
        self.get_logger().info(
            f"Target {target:.2f} m, estimated displacement {self.traveled_from(start):.2f} m"
        )

    def turn(self):
        # Positive angle turns left (counterclockwise seen from above), the
        # ROS convention; Unitree's sign for omega and rpy still needs a robot test.
        target = math.radians(abs(self.angle))
        sign = math.copysign(1.0, self.angle)
        start = self.heading
        self.get_logger().info(
            f"Turning {self.angle:+.1f} deg at {self.turn_speed:.1f} deg/s"
        )
        self.drive(0.0, sign * math.radians(self.turn_speed),
                   lambda: sign * (self.heading - start), target,
                   math.radians(self.turn_speed), WRONG_WAY_LIMIT)
        self.get_logger().info(
            f"Target {self.angle:+.1f} deg, "
            f"estimated turn {math.degrees(self.heading - start):+.1f} deg"
        )

    def run(self):
        self.get_logger().info("Waiting for odometry on /lf/odommodestate...")
        wait_until = time.monotonic() + 3.0
        while self.position is None:
            if time.monotonic() > wait_until:
                raise RuntimeError("No odometry received")
            rclpy.spin_once(self, timeout_sec=0.1)

        if self.angle != 0:
            self.turn()
        else:
            self.walk()


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
