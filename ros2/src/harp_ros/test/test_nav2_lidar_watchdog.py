"""Check configured deadlines against the actual built STVL buffer, without DDS."""

from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest
import xml.etree.ElementTree as ET

import yaml


SOURCE = Path(__file__).resolve().parents[2]


class LidarWatchdogTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if not shutil.which("cmake"):
            raise unittest.SkipTest("needs the built Pixi/ROS environment and CMake")
        cls.directory = tempfile.TemporaryDirectory(prefix="harp-stvl-watchdog-")
        cls.addClassCleanup(cls.directory.cleanup)
        build = Path(cls.directory.name)
        commands = [
            ["cmake", "-S", str(SOURCE / "harp_ros/test/stvl_watchdog"), "-B", str(build),
             f"-DSTVL_SOURCE={SOURCE / 'spatio_temporal_voxel_layer/spatio_temporal_voxel_layer'}",
             "-DCMAKE_IGNORE_PREFIX_PATH=/opt/homebrew"],
            ["cmake", "--build", str(build), "--parallel", "2"],
        ]
        for command in commands:
            result = subprocess.run(command, capture_output=True, text=True, check=False)
            if result.returncode:
                raise RuntimeError(result.stdout + result.stderr)
        cls.binary = build / "stvl_watchdog"
        cls.config = yaml.safe_load((SOURCE / "harp_ros/config/nav2_params.yaml").read_text())

    def test_costmaps_expire_and_resume_with_the_configured_marking_deadline(self):
        for name in ("local_costmap", "global_costmap"):
            with self.subTest(costmap=name):
                layer = self.config[name][name]["ros__parameters"]["stvl_layer"]
                marking = layer["lidar_marking"]
                result = subprocess.run([str(self.binary), str(marking["expected_update_rate"])],
                                        capture_output=True, text=True, check=False)
                self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

    def test_clearing_shares_the_guarded_cloud_without_a_false_staleness_deadline(self):
        # This STVL version returns before refreshing a clearing-only buffer's time.
        # Guard the same cloud through marking instead of making clearing stale forever.
        for name in ("local_costmap", "global_costmap"):
            with self.subTest(costmap=name):
                layer = self.config[name][name]["ros__parameters"]["stvl_layer"]
                self.assertEqual(layer["lidar_marking"]["topic"], layer["lidar_clearing"]["topic"])
                self.assertEqual(layer["lidar_clearing"]["expected_update_rate"], 0.0)

    def test_enabled_recovery_actions_are_nonmoving_and_match_the_tree(self):
        behaviors = self.config["behavior_server"]["ros__parameters"]
        enabled = {behaviors[name]["plugin"] for name in behaviors["behavior_plugins"]}
        self.assertEqual(enabled, {"nav2_behaviors::Wait"})

        tree = ET.parse(SOURCE / "harp_ros/behavior_trees/navigate.xml")
        recovery = tree.find(".//ReactiveFallback[@name='RecoveryFallback']")
        self.assertIsNotNone(recovery)
        # GoalUpdated lets a new goal interrupt recovery. Every action leaf
        # after it must be a nonmoving behavior provided by the enabled server.
        actions = {node.tag for node in recovery.iter()
                   if len(node) == 0 and node.tag != "GoalUpdated"}
        self.assertEqual(actions, {"Wait"})
        self.assertEqual(enabled, {"nav2_behaviors::" + action for action in actions})
        # Prevent a motion recovery from being moved to another branch.
        self.assertFalse(any(node.tag in {"Spin", "BackUp", "DriveOnHeading", "AssistedTeleop"}
                             for node in tree.iter()))


if __name__ == "__main__":
    unittest.main()
