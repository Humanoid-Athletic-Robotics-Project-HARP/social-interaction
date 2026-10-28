"""Offline callbacks and startup checks; no ROS node or DDS channel is created."""

import math
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from geometry_msgs.msg import Twist
import numpy as np

from harp_ros import walker_node


class WalkerNodeInputTests(unittest.TestCase):
    def make_node(self):
        return SimpleNamespace(cmd=np.array([0.3, 0.0, 0.0], dtype=np.float32),
                               cmd_time=10.0, get_logger=lambda: Mock())

    def test_invalid_velocity_clears_the_previous_motion_request(self):
        for field in ("x", "y", "yaw"):
            for value in (float("nan"), float("inf"), -float("inf")):
                with self.subTest(field=field, value=value):
                    node = self.make_node()
                    msg = Twist()
                    if field == "yaw":
                        msg.angular.z = value
                    else:
                        setattr(msg.linear, field, value)
                    walker_node.WalkerNode.on_cmd_vel(node, msg)

                    np.testing.assert_array_equal(node.cmd, np.zeros(3))
                    self.assertEqual(node.cmd_time, -math.inf)

    def test_large_finite_velocity_is_clipped_before_float32_conversion(self):
        node = self.make_node()
        msg = Twist()
        msg.linear.x, msg.linear.y, msg.angular.z = 1e300, -1e300, 1e300
        with patch.object(walker_node.time, "monotonic", return_value=20.0):
            walker_node.WalkerNode.on_cmd_vel(node, msg)

        np.testing.assert_allclose(node.cmd, walker_node.CMD_LIMITS * [1, -1, 1])
        self.assertTrue(np.isfinite(node.cmd).all())
        self.assertEqual(node.cmd_time, 20.0)

    def test_malformed_or_unrelated_events_do_not_break_auto_start_callback(self):
        node = SimpleNamespace(feet_down=False)
        for text in ("not JSON", "[]", "null", "42", "{}", '{"event":"remote"}'):
            with self.subTest(text=text):
                walker_node.WalkerNode.on_event(node, SimpleNamespace(data=text))
                self.assertFalse(node.feet_down)
        walker_node.WalkerNode.on_event(
            node, SimpleNamespace(data='{"source":"sim","event":"feet_down"}'))
        self.assertTrue(node.feet_down)


class WalkerNodeStartupTests(unittest.TestCase):
    def make_node(self, **overrides):
        values = {"real": False, "auto_start": False, "cmd_timeout": 0.5}
        values.update(overrides)
        with patch.object(walker_node.Node, "__init__", return_value=None), \
                patch.object(walker_node.WalkerNode, "declare_parameter"), \
                patch.object(walker_node.WalkerNode, "get_parameter",
                             side_effect=lambda name: SimpleNamespace(value=values[name])):
            return walker_node.WalkerNode()

    def test_auto_start_is_rejected_on_the_real_robot(self):
        with self.assertRaises(ValueError):
            self.make_node(real=True, auto_start=True)

    def test_command_timeout_must_be_positive_and_finite(self):
        for timeout in (0.0, -0.1, float("nan"), float("inf")):
            with self.subTest(timeout=timeout):
                with self.assertRaises(ValueError):
                    self.make_node(cmd_timeout=timeout)

    def test_sim_auto_start_and_manual_real_start_remain_available(self):
        self.assertTrue(self.make_node(auto_start=True).auto_start)
        self.assertFalse(self.make_node(real=True).auto_start)


if __name__ == "__main__":
    unittest.main()
