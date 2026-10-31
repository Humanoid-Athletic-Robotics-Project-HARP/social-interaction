"""Planned moves: walk straight set distances, turn in
place by set angles, or walk to a point in one smooth curve, one after another,
by publishing /cmd_vel for harp_node from our leg odometry (/odom).
entrypoint.sh starts it when G1_TASK_COMMAND holds moves, which reaches our
image both in HIM's sim and on the robot:

  G1_TASK_COMMAND='{"moves": [{"walk": 1.0}, {"turn": 90}, {"to": [1, 1, 45]}]}'
  G1_TASK_COMMAND='{"to": [1, 1, 45]}'                     (one curve, the same as a list of one)

  walk: meters straight ahead (negative walks backward), holding the heading it
        started with, ramping up and slowing near the end before requesting zero.
  turn: degrees in place (positive turns left).
  to:   [X, Y] or [X, Y, HEADING]: X m forward, Y m left of where the move
        starts, ending facing HEADING degrees (positive left; leave it out to
        face away from the start). Park & Kuipers' law (smooth_control.py), then
        a turn in place if more than 5 deg off.
  speed (m/s, default 0.2, at most 0.4) and turn_speed (deg/s, default 20, at
  most 30) are optional, beside "moves" or "to".

The first move starts SETTLE s after HIM hands control to our policy
(harp_node's control_ready event), and each later one SETTLE s after the one
before stops; then it exits and GR00T keeps balancing. Each move starts from
wherever the last one ended, so errors carry over. A move that takes much
longer than expected, or a turn that goes over 10 deg the wrong way, stops the
rest. Stops, falls and damping are HIM's. Events on /harp/events: move_start,
move_done (target, estimate: x, y forward/left of the move's start and heading
in degrees, from leg odometry), move_timeout or wrong_way, then all_moves_done;
each with the move's number and sim_time when HIM's sim gives one.
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
SETTLE = 2.0              # s before the first move, between moves and after the last
RATE = 50.0               # Hz of /cmd_vel
WRONG_WAY_LIMIT = math.radians(10.0)  # stop if a turn goes this far the wrong way
HEADING_GAIN = 1.5        # rad/s of turn per rad of heading error, during walks
MAX_HEADING_CORRECTION = 0.3  # rad/s, cap on that correction
WALK_ACCEL = 0.4         # m/s^2, ramp for straight walking's forward/backward speed
WALK_DECEL = 0.4         # m/s^2, braking envelope near the distance target
WALK_MIN_SPEED = 0.1     # m/s, final approach stays above GR00T's 0.05 Balance switch
CURVE_MIN_SPEED = 0.1     # m/s, keeps the curve walking (GR00T balances below 0.05)
CURVE_ACCEL = 0.4         # m/s^2, ramp for the curve's forward speed
CURVE_TURN_ACCEL = 1.0    # rad/s^2, ramp for its turn rate
POSITION_TOLERANCE = 0.08  # m, a curve's point is reached inside this
HEADING_TOLERANCE = math.radians(5.0)  # its final turn in place stops inside this
USAGE = ('G1_TASK_COMMAND takes {"moves": [{"walk": M}, {"turn": DEG}, {"to": [X, Y, HEADING]}], '
         '"speed": M/S, "turn_speed": DEG/S}, or {"to": [X, Y, HEADING]}, all numbers')


def moves_from_env():
    """(moves, speed, turn speed) from G1_TASK_COMMAND, or None if it holds no moves.
    Each move is ("walk", meters), ("turn", radians) or ("to", (x, y, heading deg or None))."""
    try:
        task = json.loads(os.environ.get("G1_TASK_COMMAND") or "null")
    except ValueError:
        return None
    if not isinstance(task, dict) or not ("moves" in task or "to" in task):
        return None
    try:
        items = task["moves"] if "moves" in task else [{"to": task["to"]}]
        moves = [parse_move(item) for item in items]
        speed = min(float(task.get("speed", 0.2)), MAX_SPEED)
        turn_speed = math.radians(min(float(task.get("turn_speed", 20.0)), MAX_TURN_SPEED))
    except (TypeError, ValueError, KeyError, AttributeError):
        raise SystemExit(USAGE)
    if not moves:
        raise SystemExit("No moves: " + USAGE)
    if not (speed > 0 and turn_speed > 0):
        raise SystemExit("speed and turn_speed must be positive")
    return moves, speed, turn_speed


def parse_move(item):
    (kind, value), = item.items()
    if kind == "to":
        to = [float(v) for v in value]
        if len(to) not in (2, 3) or not all(math.isfinite(v) for v in to):
            raise SystemExit('"to" takes [X, Y] or [X, Y, HEADING]')
        if math.hypot(to[0], to[1]) < 2 * POSITION_TOLERANCE:
            raise SystemExit(f"The point {to[:2]} is too close to walk to; turn in place instead")
        return "to", (to[0], to[1], to[2] if len(to) == 3 else None)
    value = float(value)
    if kind not in ("walk", "turn") or not math.isfinite(value) or value == 0:
        raise SystemExit(f"Not a move: {item}. " + USAGE)
    return kind, math.radians(value) if kind == "turn" else value


def toward(current, target, max_step):
    return current + max(-max_step, min(max_step, target - current))


class Moves:
    def __init__(self, node, moves, speed, turn_speed):
        self.node = node
        self.moves = moves
        self.speed, self.turn_speed = speed, turn_speed
        self.index = 0
        self.pose = None          # x, y, yaw from /odom
        self.heading = 0.0        # unwrapped yaw, for turns past 180 deg
        self.sim_time = None
        self.ready_at = None      # monotonic time of HIM's handoff
        self.phase = "waiting"    # waiting -> (moving -> settling) per move -> done
        self.next_at = None       # when the next move may start
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
        text = json.dumps({"source": "moves", "event": name, "sim_time": self.sim_time, **fields})
        self.events_pub.publish(String(data=text))
        print(text, flush=True)

    def on_event(self, msg):
        if json.loads(msg.data).get("event") == "control_ready" and self.ready_at is None:
            self.ready_at = time.monotonic()
            self.next_at = self.ready_at + SETTLE

    def on_state(self, msg):
        self.sim_time = json.loads(msg.data).get("sim_time")

    def on_odom(self, msg):
        p, o = msg.pose.pose.position, msg.pose.pose.orientation
        yaw = 2.0 * math.atan2(o.z, o.w)
        if self.pose is not None:
            self.heading += wrap_angle(yaw - self.pose[2])
        self.pose = (p.x, p.y, yaw)

    def begin(self):
        kind, value = self.moves[self.index]
        self.start, self.start_heading = self.pose, self.heading
        sx, sy, syaw = self.start
        if kind == "to":
            x, y, heading = value
            end_heading = math.radians(heading) if heading is not None else math.atan2(y, x)
            c, s = math.cos(syaw), math.sin(syaw)
            self.target = (sx + c * x - s * y, sy + s * x + c * y, syaw + end_heading)
            self.facing = False   # True once at the point, turning to the final heading
            # Generous: the curve is longer than the straight line and slows near the point.
            duration = 3.0 * math.hypot(x, y) / self.speed + 2.0 * abs(end_heading) / self.turn_speed
        elif kind == "turn":
            duration = 2.0 * abs(value) / self.turn_speed
        else:
            duration = 2.0 * abs(value) / self.speed
        self.give_up_at = time.monotonic() + duration + 5.0
        self.phase = "moving"
        self.event("move_start", move=self.index + 1, kind=kind, target=self.shown(kind, value),
                   speed=self.speed, turn_speed=round(math.degrees(self.turn_speed), 1))

    @staticmethod
    def shown(kind, value):
        return list(value) if kind == "to" else round(math.degrees(value), 1) if kind == "turn" else value

    def progress(self):
        """Where the robot is relative to the move's start: x forward, y left, heading (deg)."""
        sx, sy, syaw = self.start
        dx, dy = self.pose[0] - sx, self.pose[1] - sy
        c, s = math.cos(syaw), math.sin(syaw)
        return [round(c * dx + s * dy, 3), round(-s * dx + c * dy, 3),
                round(math.degrees(self.heading - self.start_heading), 1)]

    def walk_command(self, meters):
        """Ramp ahead/back, brake near the distance target, hold heading; None once there."""
        x, y, _ = self.progress()
        remaining = abs(meters) - math.hypot(x, y)
        if remaining <= 0:
            return None
        # v^2 = 2*a*d bounds the approach speed. Keep a walking floor so the
        # policy doesn't stand before reaching the target; never exceed the
        # requested speed, including requests below WALK_MIN_SPEED.
        target = min(self.speed, max(WALK_MIN_SPEED, math.sqrt(2 * WALK_DECEL * remaining)))
        current = abs(self.cmd[0])
        limit = WALK_ACCEL if target >= current else WALK_DECEL
        speed = toward(current, target, limit / RATE)
        error = self.start_heading - self.heading
        yaw_rate = max(-MAX_HEADING_CORRECTION, min(MAX_HEADING_CORRECTION, HEADING_GAIN * error))
        return [math.copysign(speed, meters), yaw_rate]

    def turn_command(self, angle):
        """In place, at turn_speed; None once turned far enough, "wrong_way" if going the other way."""
        sign = math.copysign(1.0, angle)
        turned = sign * (self.heading - self.start_heading)
        if turned < -WRONG_WAY_LIMIT:
            return "wrong_way"
        if turned >= abs(angle):
            return None
        return [0.0, sign * self.turn_speed]

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
        if self.phase in ("waiting", "settling"):
            self.send()
            if self.next_at is None or now < self.next_at or self.pose is None:
                return
            if self.phase == "settling":
                kind, value = self.moves[self.index]
                self.event("move_done", move=self.index + 1, kind=kind,
                           target=self.shown(kind, value), estimate=self.progress())
                self.index += 1
                if self.index == len(self.moves):
                    self.event("all_moves_done")
                    self.phase = "done"
                    return
            self.begin()
            return
        kind, value = self.moves[self.index]
        if now > self.give_up_at:
            self.stop_all("move_timeout", kind, value)
            return
        cmd = (self.curve_command() if kind == "to" else self.turn_command(value) if kind == "turn"
               else self.walk_command(value))
        if cmd == "wrong_way":
            self.stop_all("wrong_way", kind, value)
            return
        if cmd is None:
            self.cmd = [0.0, 0.0]
            self.phase, self.next_at = "settling", now + SETTLE
        else:
            self.cmd = cmd
        self.send()

    def stop_all(self, reason, kind, value):
        self.cmd = [0.0, 0.0]
        self.send()
        self.event(reason, move=self.index + 1, kind=kind, target=self.shown(kind, value),
                   estimate=self.progress())
        self.phase = "done"

    def send(self):
        msg = Twist()
        msg.linear.x, msg.angular.z = float(self.cmd[0]), float(self.cmd[1])
        self.pub.publish(msg)


def main():
    plan = moves_from_env()
    if plan is None:
        return 0
    rclpy.init(args=sys.argv)
    node = rclpy.create_node("moves")
    moves = Moves(node, *plan)
    try:
        while rclpy.ok() and moves.phase != "done":
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    moves.cmd = [0.0, 0.0]
    moves.send()
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
