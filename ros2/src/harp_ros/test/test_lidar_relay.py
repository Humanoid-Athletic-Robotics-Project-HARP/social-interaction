"""Offline lifecycle checks; no ROS context or DDS reader is started."""

import contextlib
import io
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from harp_ros import lidar_relay


class RelayCleanupTests(unittest.TestCase):
    def run_relay(self, *, spin_error=None, init_error=None, factory_error=None,
                  close_error=None, real=False, interface=""):
        self.node, self.sub = Mock(), Mock()
        values = {"real": real, "interface": interface, "domain": -1}
        self.node.get_parameter.side_effect = lambda name: SimpleNamespace(value=values[name])
        self.sub.Init.side_effect = init_error
        self.closed = []
        self.sub.Close.side_effect = lambda: self.close_reader(close_error)
        self.node.destroy_node.side_effect = lambda: self.closed.append("node")
        with contextlib.ExitStack() as stack:
            stack.enter_context(contextlib.redirect_stdout(io.StringIO()))
            stack.enter_context(contextlib.redirect_stderr(io.StringIO()))
            stack.enter_context(patch.object(lidar_relay.rclpy, "init"))
            stack.enter_context(patch.object(lidar_relay, "LidarRelay", return_value=self.node))
            self.factory = stack.enter_context(patch.object(
                lidar_relay, "ChannelFactoryInitialize", side_effect=factory_error))
            stack.enter_context(patch.object(lidar_relay, "ChannelSubscriber", return_value=self.sub))
            stack.enter_context(patch.object(lidar_relay.rclpy, "spin", side_effect=spin_error))
            stack.enter_context(patch.object(lidar_relay.rclpy, "shutdown"))
            stack.enter_context(patch.object(lidar_relay.rclpy, "try_shutdown",
                                            side_effect=lambda: self.closed.append("context")))
            return lidar_relay.main()

    def close_reader(self, error):
        self.closed.append("reader")
        if error:
            raise error

    def test_normal_exit_closes_reader_before_node_and_context(self):
        self.assertEqual(self.run_relay(), 0)
        self.assertEqual(self.closed, ["reader", "node", "context"])
        self.factory.assert_called_once_with(1, "lo0" if lidar_relay.sys.platform == "darwin" else "lo")

    def test_spin_failure_is_propagated_after_cleanup(self):
        with self.assertRaisesRegex(RuntimeError, "executor failed"):
            self.run_relay(spin_error=RuntimeError("executor failed"))
        self.assertEqual(self.closed, ["reader", "node", "context"])

    def test_failed_subscriber_initialization_also_closes_reader(self):
        with self.assertRaisesRegex(RuntimeError, "reader failed"):
            self.run_relay(init_error=RuntimeError("reader failed"))
        self.assertEqual(self.closed, ["reader", "node", "context"])

    def test_failed_factory_still_destroys_ros_resources(self):
        with self.assertRaisesRegex(RuntimeError, "DDS unavailable"):
            self.run_relay(factory_error=RuntimeError("DDS unavailable"))
        self.assertEqual(self.closed, ["node", "context"])
        self.sub.Close.assert_not_called()

    def test_failed_reader_close_still_destroys_ros_resources(self):
        with self.assertRaisesRegex(RuntimeError, "close failed"):
            self.run_relay(close_error=RuntimeError("close failed"))
        self.assertEqual(self.closed, ["reader", "node", "context"])

    def test_missing_real_interface_cleans_up_without_opening_dds(self):
        self.assertEqual(self.run_relay(real=True), 1)
        self.assertEqual(self.closed, ["node", "context"])
        self.factory.assert_not_called()


if __name__ == "__main__":
    unittest.main()
