"""Exercise the interactive recorder with real ROS messages on a local test domain.

Run with ROS_LOCALHOST_ONLY=1 and a dedicated ROS_DOMAIN_ID, never a robot's domain.
The publisher below is synthetic odometry; this test does not drive a robot.
"""

import math
import os
import queue
import subprocess
import sys
import tempfile
import threading
import time
import unittest
from pathlib import Path

import rclpy
import yaml
from nav_msgs.msg import Odometry


@unittest.skipUnless(os.environ.get("ROS_LOCALHOST_ONLY") == "1"
                     and os.environ.get("ROS_DOMAIN_ID", "0") != "0",
                     "requires an isolated local ROS test domain")
class InteractiveRecorder(unittest.TestCase):
    def setUp(self):
        rclpy.init()
        self.node = rclpy.create_node("test_waypoint_source")
        self.publisher = self.node.create_publisher(Odometry, "/odom", 10)
        self.position = (1.0, 2.0, 90.0)
        self.folder = tempfile.TemporaryDirectory()
        self.output = Path(self.folder.name) / "recorded route.yaml"
        self.process = subprocess.Popen(
            [sys.executable, "-u", "-m", "harp.record_waypoints", "--output", str(self.output)],
            stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
            text=True, bufsize=1)
        self.lines = queue.Queue()
        self.reader = threading.Thread(target=self.read_lines, daemon=True)
        self.reader.start()

    def read_lines(self):
        for line in self.process.stdout:
            self.lines.put(line)

    def tearDown(self):
        if self.process.poll() is None:
            self.process.terminate()
        self.process.wait(timeout=5)
        self.reader.join(timeout=1)
        self.process.stdin.close()
        self.process.stdout.close()
        self.node.destroy_node()
        rclpy.shutdown()
        self.folder.cleanup()

    def publish(self):
        msg = Odometry()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id, msg.child_frame_id = "odom", "base_link"
        x, y, heading = self.position
        msg.pose.pose.position.x, msg.pose.pose.position.y = x, y
        angle = math.radians(heading) / 2
        msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = math.sin(angle), math.cos(angle)
        self.publisher.publish(msg)
        rclpy.spin_once(self.node, timeout_sec=0.05)

    def wait_for_line(self, token, timeout=10):
        deadline = time.monotonic() + timeout
        seen = []
        while time.monotonic() < deadline:
            self.publish()
            while not self.lines.empty():
                line = self.lines.get_nowait()
                seen.append(line)
                if token in line:
                    return
            if self.process.poll() is not None:
                break
        self.fail(f"Did not receive {token!r}: {''.join(seen)}")

    def command(self, text):
        self.process.stdin.write(text + "\n")
        self.process.stdin.flush()

    def settle(self):
        deadline = time.monotonic() + 0.5
        while time.monotonic() < deadline:
            self.publish()

    def test_capture_undo_save_and_load(self):
        self.wait_for_line("Odometry ready;")
        self.command("r")
        self.wait_for_line("Recorded 1:")
        self.position = (3.0, 2.0, 0.0)
        self.settle()
        self.command("r")
        self.wait_for_line("Recorded 2:")
        self.command("u")
        self.wait_for_line("Removed:")
        self.position = (4.0, -1.0, -45.0)
        self.settle()
        self.command("r")
        self.wait_for_line("Recorded 2:")
        self.command("s")
        self.wait_for_line("Saved 2 waypoints")
        self.assertEqual(self.process.wait(timeout=5), 0)
        data = yaml.safe_load(self.output.read_text())
        self.assertEqual(data, {"frame": "odom", "waypoints": [[1.0, 2.0, 90.0], [4.0, -1.0, -45.0]],
                                "laps": 1})

    def test_quit_writes_no_file(self):
        self.wait_for_line("Enter/r:")
        self.command("q")
        self.wait_for_line("Recording discarded")
        self.assertEqual(self.process.wait(timeout=5), 1)
        self.assertFalse(self.output.exists())


if __name__ == "__main__":
    unittest.main()
