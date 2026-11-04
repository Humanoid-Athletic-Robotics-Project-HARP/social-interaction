"""harp_node.py's command limits and harp/state.py's truth events, without HIM or ROS nodes.

HIM's client (/example/client.py in the image) is replaced by a stub class.
"""

import math
import sys
import types
import unittest
from unittest.mock import Mock, patch

sys.modules.setdefault("client", types.SimpleNamespace(PolicyNode=object))
import harp_node  # noqa: E402
from harp.state import StatePublisher  # noqa: E402


class CommandLimits(unittest.TestCase):
    def test_commands_are_clipped_to_the_limits(self):
        self.assertEqual(harp_node.clipped([1.2, -0.9, 3.0]), [0.6, -0.4, 1.0])
        self.assertEqual(harp_node.clipped([0.2, 0.0, -0.5]), [0.2, 0.0, -0.5])

    def test_malformed_or_non_finite_commands_are_refused(self):
        for cmd in ([0.1, 0.0], [0.1, 0.0, 0.0, 0.0], [math.nan, 0.0, 0.0], [0.0, math.inf, 0.0],
                    ["a", 0.0, 0.0], 0.3, None):
            with self.subTest(cmd=cmd):
                self.assertIsNone(harp_node.clipped(cmd))


def publisher():
    out = StatePublisher.__new__(StatePublisher)
    out.legs, out.drift, out.next_truth = Mock(), None, 0.0
    out.node = Mock()
    out.from_legs = lambda: (0.0, 0.0, 0.0, (0.0, 0.0, 0.0))
    out.publish_odometry = out.publish_joints_imu = lambda *args: None
    return out


class TruthEvents(unittest.TestCase):
    def test_robot_state_with_a_base_position_but_no_sim_time(self):
        # HIM adds "position" on the robot when G1_BASE_STATE_TOPIC is set; it has no sim_time.
        with patch("harp.state.events.emit") as emit:
            publisher().publish({"q": [], "quat": [1.0, 0.0, 0.0, 0.0], "position": [1.0, 2.0, 0.8]})
        emit.assert_not_called()

    def test_sim_state_logs_truth(self):
        with patch("harp.state.events.emit") as emit:
            publisher().publish({"q": [], "quat": [1.0, 0.0, 0.0, 0.0], "position": [1.0, 2.0, 0.8],
                                 "sim_time": 3.0})
        self.assertEqual(emit.call_args[0][:3], ("policy", "odom", 3.0))


if __name__ == "__main__":
    unittest.main()
