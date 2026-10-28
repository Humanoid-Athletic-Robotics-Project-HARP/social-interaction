"""Inspect replay launch actions without starting ROS processes or reading a bag."""

import importlib.util
from pathlib import Path
import unittest
from unittest.mock import patch

from launch import LaunchContext
from launch.actions import ExecuteProcess, SetEnvironmentVariable, TimerAction
from launch_ros.actions import Node


PATH = Path(__file__).parents[1] / "launch" / "replay.launch.py"
SPEC = importlib.util.spec_from_file_location("harp_eval_replay_launch", PATH)
replay = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(replay)


class ReplayDomainTests(unittest.TestCase):
    def context(self, domain="100", record="false"):
        context = LaunchContext()
        context.environment["ROS_DOMAIN_ID"] = "2"
        context.launch_configurations.update({
            "run": "source-run", "bag": "", "params_file": "/test/nav2.yaml",
            "rate": "1.0", "rviz": "true", "record": record, "tag": "offline",
            "domain": domain,
        })
        return context

    def actions(self, context):
        with patch.object(replay, "find_bag", return_value=Path("/test/replay")), \
                patch.object(replay, "get_package_share_directory", return_value="/test/harp_ros"):
            return replay.actions(context)

    def test_default_domain_is_distinct_from_manual_and_course_domains(self):
        with patch.object(replay, "get_package_share_directory", return_value="/test/harp_ros"):
            description = replay.generate_launch_description()
        arguments = description.get_launch_arguments()
        domain = next(arg for arg in arguments if arg.name == "domain")
        context = LaunchContext()
        domain.execute(context)

        self.assertEqual(context.launch_configurations["domain"], "100")

    def test_player_nav2_and_optional_recorder_follow_isolated_environment(self):
        context = self.context(record="true")
        actions = self.actions(context)

        self.assertIsInstance(actions[0], SetEnvironmentVariable)
        self.assertEqual(context.environment["ROS_DOMAIN_ID"], "2")
        actions[0].execute(context)
        self.assertEqual(context.environment["ROS_DOMAIN_ID"], "100")
        self.assertTrue(any(isinstance(action, ExecuteProcess) for action in actions[1:]))
        self.assertTrue(any(isinstance(action, TimerAction) for action in actions[1:]))
        self.assertTrue(any(isinstance(action, Node) for action in actions[1:]))

    def test_explicit_domain_overrides_inherited_domain(self):
        context = self.context(domain="101")
        actions = self.actions(context)
        self.assertIsInstance(actions[0], SetEnvironmentVariable)
        actions[0].execute(context)

        self.assertEqual(context.environment["ROS_DOMAIN_ID"], "101")

    def test_invalid_domain_is_rejected_before_processes_are_built(self):
        for domain in ("", "-1", "233", "not-a-domain", "1.5"):
            with self.subTest(domain=domain):
                with self.assertRaises(ValueError):
                    self.actions(self.context(domain=domain))


if __name__ == "__main__":
    unittest.main()
