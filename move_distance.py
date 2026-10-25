"""Walk the G1 straight forward (or backward) set distances and turn it in
place by set angles, using our GR00T walking controller (gr00t/walk.py).
Moves run one after another, with the robot balancing in between.

Distance and heading come from our own estimate, since Unitree's odometry is
part of the built-in controller we no longer use:
  - Heading: yaw from the pelvis IMU quaternion.
  - Position: leg odometry. Joint angles and the IMU orientation give each
    foot's position relative to the pelvis (forward kinematics of the G1
    model in MuJoCo). The lower foot is taken as planted, so the pelvis moves
    by minus that foot's motion.

Run against the sim (start sim/run_sim.py first, see README):
  ../sim-venv/bin/python move_distance.py --distance 1.0 --speed 0.3
  ../sim-venv/bin/python move_distance.py --angle 90 --turn-speed 20
  ../sim-venv/bin/python move_distance.py --distance 2 --angle 90 --distance 2
Real robot (untested; hang it, damp it, then debug mode L2+R2 first):
  python3 move_distance.py --real --interface eth0 --distance 0.5

Keys: ] start the policy (balance in place), g start the moves once the robot
stands on its own, o stop (damp) and quit. After the last move the robot
keeps balancing until o.
"""

import argparse
import math
import sys
import time
from pathlib import Path

import mujoco
import numpy as np
from unitree_sdk2py.core.channel import ChannelFactoryInitialize

from gr00t.walk import NUM_MOTORS, TerminalKeys, Walker, check_unitree_controller_off

DEFAULT_SCENE = (Path(__file__).resolve().parent.parent
                 / "unitree_mujoco/unitree_robots/g1/scene_29dof.xml")
FEET = ("left_ankle_roll_link", "right_ankle_roll_link")
MAX_SPEED = 0.4           # m/s, hard cap no matter what is passed in
MAX_TURN_SPEED = 30.0     # deg/s, hard cap no matter what is passed in
SETTLE_TIME = 1.5         # s standing still after each move before reporting it
WRONG_WAY_LIMIT = math.radians(10.0)  # abort if a turn goes this far the wrong way
HEADING_GAIN = 1.5        # rad/s of turn per rad of heading error, during walks
MAX_HEADING_CORRECTION = 0.3  # rad/s, cap on that correction


def yaw_of(quat):
    w, x, y, z = quat
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def wrap_angle(a):
    """Wrap an angle in radians to [-pi, pi)."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def move_arg(kind):
    """argparse type for one move: a finite, nonzero number tagged with its kind."""
    def number(text):
        value = float(text)
        if not math.isfinite(value) or value == 0:
            raise argparse.ArgumentTypeError(f"{text!r} must be a finite, nonzero number")
        return kind, value
    return number


class LegOdometry:
    """Pelvis position on the floor and unwrapped heading, from joints and IMU."""

    def __init__(self, scene):
        self.model = mujoco.MjModel.from_xml_path(str(scene))
        self.data = mujoco.MjData(self.model)
        self.feet = [self.model.body(name).id for name in FEET]
        self.position = np.zeros(2)
        self.heading = 0.0        # sum of yaw changes, radians
        self.yaw = None
        self.stance = None
        self.stance_foot = None   # planted foot relative to the pelvis, world-aligned

    def update(self, q, quat):
        yaw = yaw_of(quat)
        if self.yaw is not None:
            self.heading += wrap_angle(yaw - self.yaw)
        self.yaw = yaw

        d = self.data
        d.qpos[:3] = 0.0          # pelvis at the origin, so foot positions are relative to it
        d.qpos[3:7] = quat
        d.qpos[7:7 + NUM_MOTORS] = q
        mujoco.mj_kinematics(self.model, d)
        feet = [d.xpos[f].copy() for f in self.feet]
        stance = 0 if feet[0][2] < feet[1][2] else 1
        if stance == self.stance:
            self.position -= feet[stance][:2] - self.stance_foot[:2]
        self.stance, self.stance_foot = stance, feet[stance]


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

    def read_key(self, read):
        """Pass ], g and o through; ignore the walking keys."""
        key = read()
        if key == "g" and self.phase == "waiting":
            if not self.walker.policy_on:
                print("Press ] first to start the policy.")
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
            target, rate = math.radians(abs(value)), self.turn_speed
            print(f"{step} Turning {value:+.1f} deg at {math.degrees(rate):.1f} deg/s")
        else:
            target, rate = abs(value), self.speed
            print(f"{step} Walking {value:+.2f} m at {self.speed:.2f} m/s")
        self.give_up_at = time.monotonic() + 2.0 * target / rate + 5.0
        self.phase = "moving"

    def traveled(self):
        return float(np.hypot(*(self.odom.position - self.start_position)))

    def turned(self):
        return self.odom.heading - self.start_heading

    def on_policy_step(self, q, quat):
        self.odom.update(q, quat)
        now = time.monotonic()
        if self.phase == "moving":
            kind, value = self.moves[self.index]
            if now > self.give_up_at:
                raise RuntimeError("Movement timed out")
            if kind == "turn":
                # Positive angle turns left (counterclockwise seen from above).
                sign = math.copysign(1.0, value)
                progress = sign * self.turned()
                if progress < -WRONG_WAY_LIMIT:
                    raise RuntimeError("Turning the wrong way")
                done = progress >= math.radians(abs(value))
                cmd = [0.0, 0.0, sign * self.turn_speed]
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
                print(f"  Target {value:+.1f} deg, "
                      f"estimated turn {math.degrees(self.turned()):+.1f} deg")
            else:
                print(f"  Target {abs(value):.2f} m, "
                      f"estimated displacement {self.traveled():.2f} m")
            self.index += 1
            if self.index < len(self.moves):
                self.begin()
            else:
                print("Done. The robot keeps balancing; support it, then press o to stop.")
                self.phase = "done"


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    # Both flags append to one list, so repeated moves run in command-line order.
    parser.add_argument("--distance", dest="moves", action="append", type=move_arg("walk"),
                        help="walk this many meters (negative walks backward); repeatable")
    parser.add_argument("--angle", dest="moves", action="append", type=move_arg("turn"),
                        help="turn this many degrees (positive turns left); repeatable")
    parser.add_argument("--speed", type=float, default=0.2, help=f"m/s, capped at {MAX_SPEED}")
    parser.add_argument("--turn-speed", type=float, default=20.0,
                        help=f"deg/s, capped at {MAX_TURN_SPEED:.0f}")
    parser.add_argument("--real", action="store_true",
                        help="the real robot (DDS domain 0) instead of the sim (domain 1)")
    parser.add_argument("--interface", help="network interface; default lo0/lo in the sim")
    parser.add_argument("--scene", default=str(DEFAULT_SCENE),
                        help="G1 MuJoCo model, for leg odometry")
    args = parser.parse_args()

    if not args.moves:
        parser.error("give at least one --distance or --angle")
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
    plan = Moves(walker, LegOdometry(args.scene), args.moves, args.speed, args.turn_speed)
    try:
        with TerminalKeys() as keys:
            print("Press ] to start the policy, then g to move once the robot stands on its own.")
            walker.run(lambda: plan.read_key(keys.read), plan.on_policy_step)
    except KeyboardInterrupt:
        print("Interrupted")
    except Exception as e:
        print(f"Error: {e}", file=sys.stderr)
        walker.damp()
        return 1
    walker.damp()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
