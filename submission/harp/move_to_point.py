"""Walk to a point relative to where the robot stands, in one smooth curve (the
--to move of the old move_distance), by publishing /cmd_vel for harp_node from
our leg odometry (/odom). entrypoint.sh starts it when G1_TASK_COMMAND holds a
target, which reaches our image both in HIM's sim and on the robot:

  G1_TASK_COMMAND='{"to": [X, Y, HEADING], "speed": 0.2, "turn_speed": 20}'

X m forward, Y m left of where the robot stands when the move starts, ending
facing HEADING degrees (positive left; leave it out to face away from the
start). speed (m/s, default 0.2, at most 0.4) and turn_speed (deg/s, default
20, at most 30) are optional.

It starts SETTLE s after HIM hands control to our policy (harp_node's
control_ready event), follows Park & Kuipers' law (smooth_control.py), turns in
place at the end if more than 5 deg off, then stops and exits; GR00T keeps
balancing. If the move takes much longer than expected it stops and reports
move_timeout. Stops, falls and damping are HIM's. Events on /harp/events:
move_start and move_done (target, estimate: x, y forward/left of the start and
heading in degrees, from leg odometry; sim_time when HIM's sim gives one).
"""

import json
import math
import os
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from harp.smooth_control import command, ego_polar, wrap_angle

MAX_SPEED = 0.4           # m/s, hard cap no matter what is passed in
MAX_TURN_SPEED = 30.0     # deg/s, hard cap no matter what is passed in
SETTLE = 2.0              # s after the handoff before the move starts, and after it ends
RATE = 50.0               # Hz of /cmd_vel
CURVE_MIN_SPEED = 0.1     # m/s, keeps the curve walking (GR00T balances below 0.05)
CURVE_ACCEL = 0.4         # m/s^2, ramp for the forward speed
CURVE_TURN_ACCEL = 1.0    # rad/s^2, ramp for the turn rate
POSITION_TOLERANCE = 0.08  # m, the point is reached inside this
HEADING_TOLERANCE = math.radians(5.0)  # the final turn in place stops inside this


def target_from_env():
    """The move from G1_TASK_COMMAND, or None if it holds no "to"."""
    try:
        task = json.loads(os.environ.get("G1_TASK_COMMAND") or "null")
    except ValueError:
        return None
    if not isinstance(task, dict) or "to" not in task:
        return None
    to = [float(v) for v in task["to"]]
    if len(to) not in (2, 3) or not all(math.isfinite(v) for v in to):
        raise SystemExit('G1_TASK_COMMAND "to" takes [X, Y] or [X, Y, HEADING]')
    if math.hypot(to[0], to[1]) < 2 * POSITION_TOLERANCE:
        raise SystemExit("The point is too close to walk to")
    speed = min(float(task.get("speed", 0.2)), MAX_SPEED)
    turn_speed = math.radians(min(float(task.get("turn_speed", 20.0)), MAX_TURN_SPEED))
    if not (speed > 0 and turn_speed > 0):
        raise SystemExit("speed and turn_speed must be positive")
    return to[0], to[1], (to[2] if len(to) == 3 else None), speed, turn_speed


def toward(current, target, max_step):
    return current + max(-max_step, min(max_step, target - current))


class MoveToPoint:
    def __init__(self, node, x, y, heading, speed, turn_speed):
        self.node = node
        self.goal = (x, y, heading)
        self.speed, self.turn_speed = speed, turn_speed
        self.pose = None          # x, y, yaw from /odom
        self.sim_time = None
        self.ready_at = None      # monotonic time of HIM's handoff
        self.phase = "waiting"    # waiting -> moving -> settling -> done
        self.cmd = [0.0, 0.0]     # last forward speed and turn rate sent
        self.pub = node.create_publisher(Twist, "/cmd_vel", 10)
        latched = QoSProfile(depth=1000, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.events_pub = node.create_publisher(String, "/harp/events", latched)
        node.create_subscription(String, "/harp/events", self.on_event, latched)
        node.create_subscription(Odometry, "/odom", self.on_odom, 10)
        node.create_subscription(String, "/safety/policy_state", self.on_state,
                                 QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        node.create_timer(1.0 / RATE, self.step)

    def event(self, name, **fields):
        text = json.dumps({"source": "move_to_point", "event": name, "sim_time": self.sim_time,
                           **fields})
        self.events_pub.publish(String(data=text))
        print(text, flush=True)

    def on_event(self, msg):
        if json.loads(msg.data).get("event") == "control_ready" and self.ready_at is None:
            self.ready_at = time.monotonic()

    def on_state(self, msg):
        self.sim_time = json.loads(msg.data).get("sim_time")

    def on_odom(self, msg):
        p, o = msg.pose.pose.position, msg.pose.pose.orientation
        self.pose = (p.x, p.y, 2.0 * math.atan2(o.z, o.w))

    def begin(self):
        x, y, heading = self.goal
        sx, sy, syaw = self.start = self.pose
        end_heading = math.radians(heading) if heading is not None else math.atan2(y, x)
        c, s = math.cos(syaw), math.sin(syaw)
        self.target = (sx + c * x - s * y, sy + s * x + c * y, syaw + end_heading)
        self.facing = False       # True once at the point, turning to the final heading
        # Generous: the curve is longer than the straight line and slows near the point.
        self.give_up_at = (time.monotonic() + 3.0 * math.hypot(x, y) / self.speed
                           + 2.0 * abs(end_heading) / self.turn_speed + 5.0)
        self.phase = "moving"
        self.event("move_start", target=[x, y, heading], speed=self.speed,
                   turn_speed=round(math.degrees(self.turn_speed), 1))

    def progress(self):
        """Where the robot is relative to the start: x forward, y left, heading (deg)."""
        sx, sy, syaw = self.start
        dx, dy = self.pose[0] - sx, self.pose[1] - sy
        c, s = math.cos(syaw), math.sin(syaw)
        return [round(c * dx + s * dy, 3), round(-s * dx + c * dy, 3),
                round(math.degrees(wrap_angle(self.pose[2] - syaw)), 1)]

    def curve_command(self):
        """Forward speed and turn rate toward the target; None once it is reached."""
        r, _, delta = ego_polar(self.pose, self.target)
        # Reached the point, or passed it (it is close and behind): face the final heading.
        if not self.facing and (r < POSITION_TOLERANCE or (r < 0.3 and abs(delta) > math.pi / 2)):
            self.facing = True
            self.facing_sign = math.copysign(1.0, wrap_angle(self.target[2] - self.pose[2]))
        if self.facing:
            error = wrap_angle(self.target[2] - self.pose[2])
            # Stop inside the tolerance, or once the turn carries past the heading.
            if abs(error) < HEADING_TOLERANCE or error * self.facing_sign < 0:
                return None
            return [0.0, self.facing_sign * self.turn_speed]
        v, w = command(self.pose, self.target, self.speed, self.turn_speed, CURVE_MIN_SPEED)
        # Ramp toward the law's command so the robot doesn't lurch into or out of the curve.
        return [toward(self.cmd[0], v, CURVE_ACCEL / RATE),
                toward(self.cmd[1], w, CURVE_TURN_ACCEL / RATE)]

    def step(self):
        now = time.monotonic()
        if self.phase == "waiting":
            if self.ready_at is not None and self.pose is not None and now > self.ready_at + SETTLE:
                self.begin()
            return
        if self.phase == "moving":
            if now > self.give_up_at:
                self.event("move_timeout", target=list(self.goal), estimate=self.progress())
                self.finish()
                return
            cmd = self.curve_command()
            if cmd is None:
                self.cmd = [0.0, 0.0]
                self.phase, self.settle_until = "settling", now + SETTLE
            else:
                self.cmd = cmd
            self.send()
        elif self.phase == "settling":
            self.send()
            if now > self.settle_until:
                self.event("move_done", target=list(self.goal), estimate=self.progress())
                self.finish()

    def send(self):
        msg = Twist()
        msg.linear.x, msg.angular.z = float(self.cmd[0]), float(self.cmd[1])
        self.pub.publish(msg)

    def finish(self):
        self.cmd = [0.0, 0.0]
        self.send()
        self.phase = "done"


def main():
    move = target_from_env()
    if move is None:
        return 0
    rclpy.init(args=sys.argv)
    node = rclpy.create_node("move_to_point")
    mover = MoveToPoint(node, *move)
    try:
        while rclpy.ok() and mover.phase != "done":
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
