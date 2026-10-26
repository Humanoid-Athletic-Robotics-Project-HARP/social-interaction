"""Walk the G1 straight forward (or backward) set distances, turn it in
place by set angles, or walk it to a point in one smooth curve, using our
GR00T walking controller (gr00t/walk.py). Moves run one after another, with
the robot balancing in between.

A curved move (--to X Y [HEADING]) goes to a point relative to where the
robot stands when the move starts, turning while it walks
(smooth_control.py, Park & Kuipers' control law). It ends facing HEADING
(degrees, positive left), or, without it, facing away from where it started.
If it arrives facing more than a few degrees off, it finishes with a turn in
place.

Distance and heading come from our own leg odometry (leg_odometry.py), since
Unitree's odometry is part of the built-in controller we no longer use.

Run against the sim (start it first: ros2 run harp_sim run_sim, see README):
  ros2 run harp_control move_distance --distance 1.0 --speed 0.3
  ros2 run harp_control move_distance --angle 90 --turn-speed 20
  ros2 run harp_control move_distance --distance 2 --angle 90 --distance 2
  ros2 run harp_control move_distance --to 2 1 --to 1 -1 -90
Real robot (untested; hang it, damp it, then debug mode L2+R2 first):
  ros2 run harp_control move_distance --real --interface eth0 --distance 0.5

Keys, typed in this terminal: ] start the policy (balance in place), g start the moves once the robot
stands on its own, o stop (damp) and quit. After the last move the robot
keeps balancing until o.
"""

import argparse
import math
import sys
import time

import numpy as np
from unitree_sdk2py.core.channel import ChannelFactoryInitialize

from harp_control import runlog
from harp_control.gr00t.walk import (CONTROL_DT, TerminalKeys, Walker,
                                     check_unitree_controller_off)
from harp_control.leg_odometry import DEFAULT_SCENE, LegOdometry
from harp_control.smooth_control import command, ego_polar, wrap_angle

MAX_SPEED = 0.4           # m/s, hard cap no matter what is passed in
MAX_TURN_SPEED = 30.0     # deg/s, hard cap no matter what is passed in
SETTLE_TIME = 1.5         # s standing still after each move before reporting it
WRONG_WAY_LIMIT = math.radians(10.0)  # abort if a turn goes this far the wrong way
HEADING_GAIN = 1.5        # rad/s of turn per rad of heading error, during walks
MAX_HEADING_CORRECTION = 0.3  # rad/s, cap on that correction
CURVE_MIN_SPEED = 0.1     # m/s, keeps curved moves walking (GR00T balances below 0.05)
CURVE_ACCEL = 0.4         # m/s^2, ramp for the forward speed of curved moves
CURVE_TURN_ACCEL = 1.0    # rad/s^2, ramp for their turn rate
POSITION_TOLERANCE = 0.08  # m, a curved move has reached its point inside this
HEADING_TOLERANCE = math.radians(5.0)  # its final turn in place stops inside this
ODOM_LOG_PERIOD = 0.1     # s between leg-odometry samples in the run folder (10 Hz)


def rotate(v, angle):
    c, s = math.cos(angle), math.sin(angle)
    return np.array([c * v[0] - s * v[1], s * v[0] + c * v[1]])


def toward(current, target, max_step):
    """current moved toward target by at most max_step."""
    return current + float(np.clip(target - current, -max_step, max_step))


def move_arg(kind):
    """argparse type for one move: a finite, nonzero number tagged with its kind."""
    def number(text):
        value = float(text)
        if not math.isfinite(value) or value == 0:
            raise argparse.ArgumentTypeError(f"{text!r} must be a finite, nonzero number")
        return kind, value
    return number


class PointArg(argparse.Action):
    """--to X Y [HEADING]: appends ("curve", (x, y, heading or None)) to the moves."""

    def __call__(self, parser, namespace, values, option_string=None):
        if len(values) not in (2, 3):
            parser.error(f"{option_string} takes X Y or X Y HEADING, got {len(values)} numbers")
        try:
            numbers = [float(v) for v in values]
        except ValueError:
            parser.error(f"{option_string} takes numbers, got {' '.join(values)}")
        if not all(math.isfinite(n) for n in numbers):
            parser.error(f"{option_string} takes finite numbers")
        if math.hypot(numbers[0], numbers[1]) < 2 * POSITION_TOLERANCE:
            parser.error(f"{option_string} {' '.join(values)} is too close; "
                         "use --angle to turn in place")
        heading = numbers[2] if len(numbers) == 3 else None
        moves = getattr(namespace, self.dest) or []
        moves.append(("curve", (numbers[0], numbers[1], heading)))
        setattr(namespace, self.dest, moves)


class Moves:
    """Waits for g, then runs each walk or turn until the estimate reaches it."""

    def __init__(self, walker, odometry, moves, speed, turn_speed):
        self.walker = walker
        self.odom = odometry
        self.moves = moves
        self.speed = min(speed, MAX_SPEED)
        self.turn_speed = math.radians(min(turn_speed, MAX_TURN_SPEED))
        self.index = 0
        self.phase = "waiting"    # waiting -> (moving -> settling) per move -> done
        self.start_position = None
        self.start_heading = 0.0
        self.give_up_at = self.settle_until = 0.0
        self.next_log = 0.0

    def read_key(self, read):
        """Pass ], g and o through; ignore the walking keys."""
        key = read()
        if key == "g" and self.phase == "waiting":
            if not self.walker.policy_on:
                print("Not yet: press ] first to start the policy, then g.")
            else:
                self.begin()
            return None
        return key if key in ("]", "o") else None

    def begin(self):
        kind, value = self.moves[self.index]
        self.start_position = self.odom.position.copy()
        self.start_heading = self.odom.heading
        step = f"[{self.index + 1}/{len(self.moves)}]"
        if kind == "turn":
            duration = 2.0 * math.radians(abs(value)) / self.turn_speed
            print(f"{step} Turning {value:+.1f} deg at {math.degrees(self.turn_speed):.1f} deg/s")
        elif kind == "curve":
            x, y, heading = value
            end_heading = math.radians(heading) if heading is not None else math.atan2(y, x)
            # The point and heading in the odometry frame, which IMU yaw is measured in.
            self.start_yaw = self.odom.yaw
            self.target = (*(self.odom.position + rotate((x, y), self.start_yaw)),
                           self.start_yaw + end_heading)
            self.facing = False     # True once at the point, turning to the final heading
            facing = f"{heading:+.0f} deg" if heading is not None else "away from the start"
            print(f"{step} Walking to {x:+.2f} m forward, {y:+.2f} m left, ending facing "
                  f"{facing}, at up to {self.speed:.2f} m/s")
            # Generous: the curve is longer than the straight line and slows near the point.
            duration = 3.0 * math.hypot(x, y) / self.speed + 2.0 * abs(end_heading) / self.turn_speed
        else:
            duration = 2.0 * abs(value) / self.speed
            print(f"{step} Walking {value:+.2f} m at {self.speed:.2f} m/s")
        self.started_at = time.monotonic()
        self.give_up_at = self.started_at + duration + 5.0
        self.phase = "moving"
        runlog.event("move_distance", "move_start", move=self.index + 1, kind=kind,
                     target=list(value) if kind == "curve" else value)

    def traveled(self):
        return float(np.hypot(*(self.odom.position - self.start_position)))

    def turned(self):
        return self.odom.heading - self.start_heading

    def pose(self):
        return (*self.odom.position, self.odom.yaw)

    def curve_command(self):
        """Velocity command for a curved move; None once it is done."""
        r, _, delta = ego_polar(self.pose(), self.target)
        # Reached the point, or passed it (it is close and behind): face the final heading.
        if not self.facing and (r < POSITION_TOLERANCE or (r < 0.3 and abs(delta) > math.pi / 2)):
            self.facing = True
            self.facing_sign = math.copysign(1.0, wrap_angle(self.target[2] - self.odom.yaw))
        if self.facing:
            error = wrap_angle(self.target[2] - self.odom.yaw)
            # Stop inside the tolerance, or once the turn carries past the heading.
            if abs(error) < HEADING_TOLERANCE or error * self.facing_sign < 0:
                return None
            return [0.0, 0.0, self.facing_sign * self.turn_speed]
        v, w = command(self.pose(), self.target, self.speed, self.turn_speed, CURVE_MIN_SPEED)
        # Ramp toward the law's command so the robot doesn't lurch into or out of the curve.
        vx, _, yaw_rate = self.walker.cmd
        return [toward(vx, v, CURVE_ACCEL * CONTROL_DT), 0.0,
                toward(yaw_rate, w, CURVE_TURN_ACCEL * CONTROL_DT)]

    def curve_progress(self):
        """Where the robot is relative to the curved move's start: x forward, y left, heading."""
        x, y = rotate(self.odom.position - self.start_position, -self.start_yaw)
        return x, y, math.degrees(wrap_angle(self.odom.yaw - self.start_yaw))

    def on_policy_step(self, q, quat):
        self.odom.update(q, quat)
        now = time.monotonic()
        if now >= self.next_log:
            x, y = self.odom.position
            vx, vy, wz = self.walker.cmd
            runlog.sample("leg_odom", t=time.time(), x=float(x), y=float(y), yaw=float(self.odom.yaw),
                          cmd_vx=float(vx), cmd_vy=float(vy), cmd_wz=float(wz))
            self.next_log = now + ODOM_LOG_PERIOD
        if self.phase == "moving":
            kind, value = self.moves[self.index]
            if now > self.give_up_at:
                if kind == "turn":
                    progress = f"turned an estimated {abs(math.degrees(self.turned())):.0f} of {abs(value):.0f} deg"
                elif kind == "curve":
                    x, y, _ = self.curve_progress()
                    progress = (f"reached an estimated ({x:+.2f}, {y:+.2f}) m "
                                f"of ({value[0]:+.2f}, {value[1]:+.2f}) m")
                else:
                    progress = f"walked an estimated {self.traveled():.2f} of {abs(value):.2f} m"
                runlog.event("move_distance", "move_timeout", move=self.index + 1, progress=progress)
                raise RuntimeError(
                    f"Move {self.index + 1} timed out: {progress} in {now - self.started_at:.0f} s. "
                    "If the robot stepped in place, something was holding it (in the sim, the "
                    "band: press 9 in the MuJoCo window).")
            if kind == "turn":
                # Positive angle turns left (counterclockwise seen from above).
                sign = math.copysign(1.0, value)
                progress = sign * self.turned()
                if progress < -WRONG_WAY_LIMIT:
                    runlog.event("move_distance", "wrong_way", move=self.index + 1,
                                 degrees=round(math.degrees(-progress), 1))
                    raise RuntimeError(
                        f"Turning the wrong way: {math.degrees(-progress):.0f} deg opposite to the command.")
                done = progress >= math.radians(abs(value))
                cmd = [0.0, 0.0, sign * self.turn_speed]
            elif kind == "curve":
                cmd = self.curve_command()
                done = cmd is None
            else:
                done = self.traveled() >= abs(value)
                # Steer back toward the heading the walk started with.
                error = self.start_heading - self.odom.heading
                yaw_rate = float(np.clip(HEADING_GAIN * error,
                                         -MAX_HEADING_CORRECTION, MAX_HEADING_CORRECTION))
                cmd = [math.copysign(self.speed, value), 0.0, yaw_rate]
            if done:
                cmd = [0.0, 0.0, 0.0]
                self.phase, self.settle_until = "settling", now + SETTLE_TIME
            self.walker.cmd = np.array(cmd, dtype=np.float32)
        elif self.phase == "settling" and now > self.settle_until:
            kind, value = self.moves[self.index]
            if kind == "turn":
                estimate = math.degrees(self.turned())
                print(f"  Target {value:+.1f} deg, estimated turn {estimate:+.1f} deg")
            elif kind == "curve":
                x, y, heading = self.curve_progress()
                gx, gy, _ = value
                target_heading = math.degrees(wrap_angle(self.target[2] - self.start_yaw))
                estimate = [x, y, heading]
                print(f"  Target ({gx:+.2f}, {gy:+.2f}) m facing {target_heading:+.0f} deg, "
                      f"estimated ({x:+.2f}, {y:+.2f}) m facing {heading:+.1f} deg")
            else:
                estimate = self.traveled()
                print(f"  Target {abs(value):.2f} m, "
                      f"estimated displacement {estimate:.2f} m")
            runlog.event("move_distance", "move_done", move=self.index + 1, kind=kind,
                         target=list(value) if kind == "curve" else value, estimate=estimate)
            self.index += 1
            if self.index < len(self.moves):
                self.begin()
            else:
                print("All moves done. GR00T keeps balancing; support the robot, then press o "
                      "to stop (it will sink).")
                runlog.event("move_distance", "all_moves_done")
                self.phase = "done"


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    # Both flags append to one list, so repeated moves run in command-line order.
    parser.add_argument("--distance", dest="moves", action="append", type=move_arg("walk"),
                        help="walk this many meters (negative walks backward); repeatable")
    parser.add_argument("--angle", dest="moves", action="append", type=move_arg("turn"),
                        help="turn this many degrees (positive turns left); repeatable")
    parser.add_argument("--to", dest="moves", action=PointArg, nargs="+", metavar="NUM",
                        help="X Y [HEADING]: walk in one smooth curve to X m forward, Y m left "
                             "(from where the move starts), ending facing HEADING deg (positive "
                             "left; default: away from the start); repeatable")
    parser.add_argument("--speed", type=float, default=0.2, help=f"m/s, capped at {MAX_SPEED}")
    parser.add_argument("--turn-speed", type=float, default=20.0,
                        help=f"deg/s, capped at {MAX_TURN_SPEED:.0f}")
    parser.add_argument("--real", action="store_true",
                        help="the real robot (DDS domain 0) instead of the sim (domain 1)")
    parser.add_argument("--interface", help="network interface; default lo0/lo in the sim")
    parser.add_argument("--scene", default=DEFAULT_SCENE,
                        help="G1 MuJoCo model, for leg odometry (default: $HARP_G1_SCENE)")
    args = parser.parse_args()

    if not args.scene:
        parser.error("no G1 model: pass --scene, or set HARP_G1_SCENE (the ROS 2 environment does)")

    if not args.moves:
        parser.error("give at least one --distance, --angle or --to")
    if not (math.isfinite(args.speed) and math.isfinite(args.turn_speed)) \
            or args.speed <= 0 or args.turn_speed <= 0:
        parser.error("speed and turn-speed must be positive")
    if args.real and not args.interface:
        parser.error("--real needs --interface (the one on 192.168.123.x)")
    interface = args.interface or ("lo0" if sys.platform == "darwin" else "lo")

    ChannelFactoryInitialize(0 if args.real else 1, interface)
    if args.real:
        check_unitree_controller_off()
    walker = Walker(real=args.real)
    walker.next_hint = "Then press g here to start the moves (o stops)."
    plan = Moves(walker, LegOdometry(args.scene), args.moves, args.speed, args.turn_speed)
    runlog.event("move_distance", "start", argv=sys.argv[1:])
    try:
        with TerminalKeys() as keys:
            walker.run(lambda: plan.read_key(keys.read), plan.on_policy_step)
    except KeyboardInterrupt:
        print("Interrupted (Ctrl-C).")
    except Exception as e:
        print(f"Error: {e}", file=sys.stderr)
        runlog.event("move_distance", "error", message=str(e))
        walker.damp()
        return 1
    walker.damp()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
