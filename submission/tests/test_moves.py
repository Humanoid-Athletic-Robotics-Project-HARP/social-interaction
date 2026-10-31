"""Straight-speed profile and move termination; no robot or simulator is involved.

Imports the application's ROS message types, but the command/state tests do not
create a ROS node or publish. The point-mass loop is a command model, not GR00T.
"""

import math
import unittest
from unittest.mock import patch

from harp.moves import Moves, RATE, SETTLE, WALK_ACCEL, WALK_DECEL, WALK_MIN_SPEED


def walking(distance=1.0, speed=0.4):
    moves = Moves.__new__(Moves)
    moves.moves, moves.index = [("walk", distance)], 0
    moves.speed, moves.turn_speed = speed, 0.3
    moves.pose = moves.start = (0.0, 0.0, 0.0)
    moves.heading = moves.start_heading = 0.0
    moves.cmd = [0.0, 0.0]
    moves.phase, moves.give_up_at = "moving", 1000.0
    moves.sent, moves.events = [], []
    moves.send = lambda: moves.sent.append(moves.cmd[:])
    moves.event = lambda name, **fields: moves.events.append((name, fields))
    return moves


class StraightSpeed(unittest.TestCase):
    def test_ramps_up_in_both_directions_and_respects_requested_cap(self):
        for sign in (-1, 1):
            with self.subTest(sign=sign):
                moves = walking(distance=sign * 2.0)
                for _ in range(100):
                    previous = abs(moves.cmd[0])
                    moves.cmd = moves.walk_command(sign * 2.0)
                    self.assertGreater(sign * moves.cmd[0], 0)
                    self.assertLessEqual(abs(moves.cmd[0]), moves.speed)
                    self.assertLessEqual(abs(moves.cmd[0]) - previous, WALK_ACCEL / RATE + 1e-12)
                self.assertAlmostEqual(abs(moves.cmd[0]), 0.4)

    def test_brakes_with_bounded_steps_and_does_not_stall_short_of_target(self):
        for sign in (-1, 1):
            with self.subTest(sign=sign):
                moves = walking(distance=sign)
                moves.pose, moves.cmd = (sign * 0.99, 0.0, 0.0), [sign * 0.4, 0.0]
                for _ in range(60):
                    previous = abs(moves.cmd[0])
                    moves.cmd = moves.walk_command(sign)
                    self.assertLessEqual(previous - abs(moves.cmd[0]), WALK_DECEL / RATE + 1e-12)
                    self.assertGreaterEqual(abs(moves.cmd[0]), WALK_MIN_SPEED)
                self.assertAlmostEqual(abs(moves.cmd[0]), WALK_MIN_SPEED)

    def test_low_requested_speed_is_not_raised_to_approach_floor(self):
        moves = walking(speed=0.08)
        moves.pose = (0.99, 0.0, 0.0)
        for _ in range(40):
            moves.cmd = moves.walk_command(1.0)
            self.assertLessEqual(moves.cmd[0], 0.08)

    def test_heading_correction_is_retained(self):
        moves = walking()
        moves.heading = 0.1
        self.assertAlmostEqual(moves.walk_command(1.0)[1], -0.15)
        moves.heading = -2.0
        self.assertAlmostEqual(moves.walk_command(1.0)[1], 0.3)

    def test_short_and_long_command_models_reach_the_target(self):
        for sign in (-1, 1):
            for distance in (0.01, 0.5, 1.0, 2.0):
                with self.subTest(sign=sign, distance=distance):
                    target = sign * distance
                    moves = walking(distance=target)
                    peak, speed_at_end = 0.0, 0.0
                    for _ in range(5000):
                        cmd = moves.walk_command(target)
                        if cmd is None:
                            break
                        previous = abs(moves.cmd[0])
                        speed = abs(cmd[0])
                        self.assertLessEqual(speed - previous, WALK_ACCEL / RATE + 1e-12)
                        self.assertLessEqual(previous - speed, WALK_DECEL / RATE + 1e-12)
                        peak = max(peak, speed)
                        speed_at_end = speed
                        moves.cmd = cmd
                        # Synthetic response only: 85% of the velocity request.
                        moves.pose = (moves.pose[0] + 0.85 * cmd[0] / RATE, 0.0, 0.0)
                    else:
                        self.fail("Command model never reached the target")
                    self.assertLess(abs(abs(moves.pose[0]) - distance), 0.01)
                    if distance >= 0.5:
                        self.assertGreater(peak, speed_at_end)
                        self.assertLessEqual(speed_at_end, WALK_MIN_SPEED + 0.01)


class MoveTermination(unittest.TestCase):
    def test_reaching_or_passing_target_requests_zero_immediately(self):
        for position in (1.0, 1.1):
            with self.subTest(position=position):
                moves = walking()
                moves.pose, moves.cmd = (position, 0.0, 0.0), [0.4, 0.3]
                with patch("harp.moves.time.monotonic", return_value=10.0):
                    moves.step()
                self.assertEqual(moves.sent, [[0.0, 0.0]])
                self.assertEqual(moves.phase, "settling")
                self.assertEqual(moves.next_at, 10.0 + SETTLE)

    def test_timeout_bypasses_smoothing_and_ends_the_plan(self):
        moves = walking()
        moves.cmd, moves.give_up_at = [0.4, 0.3], 9.0
        with patch("harp.moves.time.monotonic", return_value=10.0):
            moves.step()
        self.assertEqual(moves.sent, [[0.0, 0.0]])
        self.assertEqual(moves.phase, "done")
        self.assertEqual(moves.events[0][0], "move_timeout")

    def test_next_straight_move_starts_a_fresh_ramp(self):
        moves = walking()
        moves.moves.append(("walk", -1.0))
        moves.pose, moves.cmd = (1.01, 0.0, 0.0), [0.3, 0.0]
        with patch("harp.moves.time.monotonic", return_value=10.0):
            moves.step()
        with patch("harp.moves.time.monotonic", return_value=13.0):
            moves.step()
            moves.step()
        self.assertEqual(moves.index, 1)
        self.assertAlmostEqual(moves.sent[-1][0], -WALK_ACCEL / RATE)


if __name__ == "__main__":
    unittest.main()
