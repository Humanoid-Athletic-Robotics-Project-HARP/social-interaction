"""Straight-speed profile and move termination; no robot or simulator is involved.

Imports the application's ROS message types, but the command/state tests do not
create a ROS node or publish. The point-mass loop is a command model, not GR00T.
"""

import math
import unittest
from unittest.mock import patch
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from unittest.mock import Mock
from harp.moves import main

from harp.moves import (Moves, RATE, SETTLE, WALK_ACCEL, WALK_DECEL, WALK_MIN_SPEED,
                        TURN_ACCEL, TURN_DECEL, TURN_MIN_SPEED, HEADING_TOLERANCE)


def walking(distance=1.0, speed=0.4):
    moves = Moves.__new__(Moves)
    moves.moves, moves.index = [("walk", distance)], 0
    moves.speed, moves.turn_speed = speed, 0.3
    moves.pose = moves.start = (0.0, 0.0, 0.0)
    moves.odom_time = 10.0
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
            moves.odom_time = 13.0
            moves.step()
            moves.step()
        self.assertEqual(moves.index, 1)
        self.assertAlmostEqual(moves.sent[-1][0], -WALK_ACCEL / RATE)


class ProgramExit(unittest.TestCase):
    def test_shared_events_ignore_malformed_or_nonobject_json(self):
        moves = walking()
        moves.ready_at = None
        for value in ("hello", "[]", "null", "42"):
            moves.on_event(String(data=value))
        self.assertIsNone(moves.ready_at)
        with patch("harp.moves.time.monotonic", return_value=10.0):
            moves.on_event(String(data='{"event":"control_ready"}'))
        self.assertEqual(moves.ready_at, 10.0)

    def run_main(self, moves, spin_error=None):
        node = Mock()
        with patch("harp.moves.moves_from_env", return_value=([("walk", 1.0)], 0.2, 0.3)), \
             patch("harp.moves.rclpy.init"), patch("harp.moves.rclpy.ok", return_value=True), \
             patch("harp.moves.rclpy.create_node", return_value=node), \
             patch("harp.moves.Moves", return_value=moves), \
             patch("harp.moves.rclpy.spin_once", side_effect=spin_error), \
             patch("harp.moves.rclpy.shutdown") as shutdown:
            try:
                return main()
            finally:
                self.assertEqual(moves.sent[-1], [0.0, 0.0])
                node.destroy_node.assert_called_once()
                shutdown.assert_called_once()

    def test_fault_returns_nonzero_and_success_returns_zero(self):
        for completed, expected in ((0, 1), (1, 0)):
            moves = walking()
            moves.phase, moves.index = "done", completed
            self.assertEqual(self.run_main(moves), expected)

    def test_callback_failure_still_sends_zero_and_cleans_up(self):
        moves = walking()
        moves.cmd = [0.4, 0.3]
        with self.assertRaisesRegex(RuntimeError, "callback"):
            self.run_main(moves, RuntimeError("callback"))


class TurnSpeed(unittest.TestCase):
    def test_turns_ramp_in_both_directions_and_respect_cap(self):
        for sign in (-1, 1):
            moves = walking()
            for _ in range(60):
                previous = moves.cmd[1]
                moves.cmd = moves.turn_command(sign * math.pi / 2)
                self.assertGreater(sign * moves.cmd[1], 0)
                self.assertLessEqual(abs(moves.cmd[1]), moves.turn_speed)
                self.assertLessEqual(abs(moves.cmd[1] - previous), TURN_ACCEL / RATE + 1e-12)

    def test_turns_slow_near_angle_without_stalling(self):
        for sign in (-1, 1):
            moves = walking()
            moves.heading, moves.cmd = sign * (math.pi / 2 - 0.0001), [0.0, sign * 0.3]
            for _ in range(60):
                previous = abs(moves.cmd[1])
                moves.cmd = moves.turn_command(sign * math.pi / 2)
                self.assertLessEqual(previous - abs(moves.cmd[1]), TURN_DECEL / RATE + 1e-12)
                self.assertGreaterEqual(abs(moves.cmd[1]), TURN_MIN_SPEED)
            self.assertAlmostEqual(abs(moves.cmd[1]), TURN_MIN_SPEED)

    def test_low_requested_turn_rate_is_not_raised_to_floor(self):
        moves = walking()
        moves.turn_speed = 0.08
        for _ in range(40):
            moves.cmd = moves.turn_command(0.01)
            self.assertLessEqual(moves.cmd[1], 0.08)

    def test_synthetic_turns_finish_including_past_180_degrees(self):
        for sign in (-1, 1):
            for degrees in (1, 30, 90, 180, 450):
                with self.subTest(sign=sign, degrees=degrees):
                    moves = walking()
                    angle = sign * math.radians(degrees)
                    for _ in range(5000):
                        cmd = moves.turn_command(angle)
                        if cmd is None:
                            break
                        self.assertIsInstance(cmd, list)
                        moves.cmd = cmd
                        moves.heading += 0.85 * cmd[1] / RATE
                    else:
                        self.fail("Synthetic turn never reached its angle")
                    self.assertLess(abs(moves.heading - angle), math.radians(0.5))

    def test_final_curve_alignment_ramps_even_after_opposite_rotation(self):
        moves = walking()
        moves.target, moves.pose = (1.0, 1.0, math.pi / 2), (1.0, 1.0, 0.0)
        moves.facing, moves.cmd = False, [0.2, -0.3]
        for _ in range(60):
            previous = moves.cmd[1]
            moves.cmd = moves.curve_command()
            self.assertEqual(moves.cmd[0], 0.0)
            self.assertLessEqual(abs(moves.cmd[1] - previous), max(TURN_ACCEL, TURN_DECEL) / RATE + 1e-12)
        self.assertGreater(moves.cmd[1], 0)
        moves.pose = (1.0, 1.0, math.pi / 2 - HEADING_TOLERANCE - 0.0001)
        for _ in range(60):
            moves.cmd = moves.curve_command()
        self.assertAlmostEqual(moves.cmd[1], TURN_MIN_SPEED)
        moves.pose = moves.target
        self.assertIsNone(moves.curve_command())

    def test_target_and_wrong_way_turns_bypass_smoothing(self):
        for sign in (-1, 1):
            for heading, phase in ((sign * math.pi / 2, "settling"), (-sign * 0.3, "done")):
                moves = walking()
                moves.moves = [("turn", sign * math.pi / 2)]
                moves.heading, moves.cmd = heading, [0.0, sign * 0.3]
                with patch("harp.moves.time.monotonic", return_value=10.0):
                    moves.step()
                self.assertEqual(moves.sent, [[0.0, 0.0]])
                self.assertEqual(moves.phase, phase)


class StraightDirection(unittest.TestCase):
    def test_sideways_distance_does_not_complete_a_forward_or_backward_target(self):
        for sign in (-1, 1):
            moves = walking(distance=sign)
            moves.pose = (sign * 0.99, 0.15, 0.0)
            cmd = moves.walk_command(sign)
            self.assertIsInstance(cmd, list)
            self.assertGreater(sign * cmd[0], 0)

    def test_progress_uses_the_start_heading(self):
        for sign in (-1, 1):
            moves = walking(distance=sign)
            moves.start = (2.0, 3.0, math.pi / 2)
            moves.pose = (2.0, 3.0 + sign * 1.01, math.pi / 2)
            moves.heading = moves.start_heading = math.pi / 2
            self.assertIsNone(moves.walk_command(sign))

    def test_drift_and_wrong_direction_request_zero_and_abort(self):
        for sign in (-1, 1):
            for x, y, reason in ((-sign, 0.0, "wrong_way"),
                                 (sign * 0.6, 0.8, "walk_drift"),
                                 (sign * 1.1, 0.3, "walk_drift")):
                with self.subTest(sign=sign, x=x, y=y):
                    moves = walking(distance=sign)
                    moves.pose, moves.cmd = (x, y, 0.0), [sign * 0.4, 0.3]
                    with patch("harp.moves.time.monotonic", return_value=10.0):
                        moves.step()
                    self.assertEqual(moves.sent, [[0.0, 0.0]])
                    self.assertEqual(moves.phase, "done")
                    self.assertEqual(moves.events[0][0], reason)

    def test_small_wrong_way_displacement_is_not_progress(self):
        for sign in (-1, 1):
            moves = walking(distance=sign * 0.01)
            moves.pose = (-sign * 0.02, 0.0, 0.0)
            self.assertGreater(sign * moves.walk_command(sign * 0.01)[0], 0)


class FeedbackWatchdog(unittest.TestCase):
    def test_start_waits_for_fresh_feedback(self):
        for received in (None, 9.0):
            moves = walking()
            moves.phase, moves.next_at, moves.odom_time = "waiting", 0.0, received
            with patch("harp.moves.time.monotonic", return_value=10.0):
                moves.step()
            self.assertEqual(moves.sent, [[0.0, 0.0]])
            self.assertEqual(moves.phase, "waiting")

    def test_lost_feedback_stops_every_move_type_without_ramping(self):
        for move in (("walk", 1.0), ("turn", 1.0), ("to", (1.0, 1.0, 45.0))):
            with self.subTest(move=move):
                moves = walking()
                moves.moves, moves.cmd = [move], [0.4, 0.3]
                with patch("harp.moves.time.monotonic", return_value=10.51):
                    moves.step()
                self.assertEqual(moves.sent, [[0.0, 0.0]])
                self.assertEqual(moves.phase, "done")
                self.assertEqual(moves.events[0][0], "odom_timeout")

    def test_lost_feedback_during_settling_blocks_the_next_move(self):
        moves = walking()
        moves.moves.append(("walk", -1.0))
        moves.phase, moves.next_at = "settling", 10.1
        with patch("harp.moves.time.monotonic", return_value=11.0):
            moves.step()
        self.assertEqual(moves.index, 0)
        self.assertEqual(moves.phase, "done")
        self.assertEqual(moves.events[0][0], "odom_timeout")

    def test_invalid_feedback_does_not_refresh_or_change_pose(self):
        for x, z, w in ((float("nan"), 0.0, 1.0), (0.0, float("inf"), 1.0), (0.0, 0.0, 0.0)):
            moves, msg = walking(), Odometry()
            msg.pose.pose.position.x = x
            msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = z, w
            with patch("harp.moves.time.monotonic", return_value=10.4):
                moves.on_odom(msg)
            self.assertEqual(moves.odom_time, 10.0)
            self.assertEqual(moves.pose, (0.0, 0.0, 0.0))

    def test_valid_feedback_refreshes_receipt_and_fault_does_not_resume(self):
        moves, msg = walking(), Odometry()
        msg.pose.pose.orientation.w = 1.0
        with patch("harp.moves.time.monotonic", return_value=10.4):
            moves.on_odom(msg)
            moves.step()
        self.assertGreater(moves.sent[-1][0], 0)
        with patch("harp.moves.time.monotonic", return_value=11.0):
            moves.step()
            moves.on_odom(msg)
            moves.step()
        self.assertEqual(moves.sent[-1], [0.0, 0.0])
        self.assertEqual(moves.phase, "done")


if __name__ == "__main__":
    unittest.main()
