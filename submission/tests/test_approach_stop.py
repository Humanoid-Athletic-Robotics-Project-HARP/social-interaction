"""harp/approach_stop.py's rule for something coming at the robot. Needs ROS 2 Foxy and
numpy, so it runs in our image:
  docker run --rm -v "$PWD/submission:/src:ro" -w /src --entrypoint bash harp-submission:local \
    -c 'source /opt/ros/foxy/setup.bash && PYTHONPATH=/src:$PYTHONPATH python3 -m unittest discover tests'
"""

import unittest
from types import SimpleNamespace
from unittest.mock import patch
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String

import numpy as np
from sensor_msgs.msg import PointCloud2, PointField

from harp.approach_stop import ApproachStop, WINDOW, coming, moving, nearest_ahead, xyz

SCAN = 0.1   # s between scans


def scans(ranges):
    """History of a point straight ahead at each range (None: nothing in the strip)."""
    return [(i * SCAN, None if r is None else (r, (r, 0.0)))
            for i, r in enumerate(ranges[-(WINDOW + 1):])]


def at(points):
    """History of the nearest point at each (x, y)."""
    return [(i * SCAN, (float(np.hypot(*p)), p)) for i, p in enumerate(points)]


class Coming(unittest.TestCase):
    def test_still_obstacle_while_walking(self):
        self.assertFalse(coming(scans([4.0 - 0.03 * i for i in range(6)]), 0.3)[0])

    def test_person_walking_at_the_robot(self):
        hit, closing = coming(scans([5.0 - 0.11 * i for i in range(6)]), 0.3)
        self.assertTrue(hit)
        self.assertAlmostEqual(closing, 0.8, places=5)

    def test_person_walking_at_a_standing_robot(self):
        self.assertTrue(coming(scans([4.0 - 0.08 * i for i in range(6)]), 0.0)[0])

    def test_still_obstacle_entering_the_strip(self):
        # A pole swings into the strip as the robot turns: one jump, then still.
        self.assertFalse(coming(scans([None, None, 5.0, 2.0, 2.0, 2.0]), 0.0)[0])
        self.assertFalse(coming(scans([5.0, 5.0, 2.0, 2.0, 2.0, 2.0]), 0.0)[0])

    def test_walking_backward_counts_as_standing(self):
        # Backing away makes a still point recede; that's no approach.
        self.assertFalse(coming(scans([2.0 + 0.02 * i for i in range(6)]), -0.2)[0])


class Moving(unittest.TestCase):
    def test_person_walking_across(self):
        # 0.9 m/s sideways, 1.1 m ahead: the range barely changes, the point does.
        self.assertTrue(moving(at([(1.1, 0.5 - 0.09 * i) for i in range(6)])))

    def test_still_point_with_lidar_noise(self):
        jitter = [(2.0, 0.1), (2.02, 0.12), (1.99, 0.09), (2.01, 0.11), (2.0, 0.1), (2.02, 0.12)]
        self.assertFalse(moving(at(jitter)))

    def test_nothing_ahead(self):
        self.assertFalse(moving(scans([None] * 6)))


class NearestAhead(unittest.TestCase):
    def test_only_the_strip_counts(self):
        points = np.array([
            [3.0, 0.0, 0.05],    # floor
            [0.3, 0.1, 1.0],     # the robot's own arm
            [-2.0, 0.0, 1.0],    # behind
            [2.0, 1.5, 1.0],     # off to the side
            [7.0, 0.0, 1.0],     # past LOOK
            [2.5, 0.4, 2.5],     # above MAX_Z
            [3.0, -0.6, 1.0],    # in the strip
        ])
        rng, (x, y) = nearest_ahead(points)
        self.assertAlmostEqual(rng, np.hypot(3.0, -0.6))
        self.assertEqual((x, y), (3.0, -0.6))
        self.assertIsNone(nearest_ahead(points[:6]))

    def test_turning_keeps_the_range(self):
        point = np.array([[3.0, 0.2, 1.0]])
        for yaw in (0.05, 0.1, -0.1):
            c, s = np.cos(yaw), np.sin(yaw)
            turned = point @ np.array([[c, s, 0], [-s, c, 0], [0, 0, 1]])
            self.assertAlmostEqual(nearest_ahead(turned)[0], nearest_ahead(point)[0])


class Xyz(unittest.TestCase):
    def test_points_with_more_fields(self):
        fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
                  for i, n in enumerate(("x", "y", "z", "intensity"))]
        data = np.array([[1, 2, 3, 9], [4, 5, 6, 9]], dtype="<f4")
        cloud = PointCloud2(height=1, width=2, fields=fields, point_step=16, row_step=32)
        cloud.data = data.tobytes()
        np.testing.assert_array_equal(xyz(cloud), [[1, 2, 3], [4, 5, 6]])


class InputGate(unittest.TestCase):
    def make_filter(self):
        output = []
        node = SimpleNamespace(create_publisher=lambda *a: SimpleNamespace(publish=output.append),
                               create_subscription=lambda *a: None, create_timer=lambda *a: None)
        gate = ApproachStop(node)
        gate.event = lambda *a, **k: None
        return gate, output

    def usable_cloud(self):
        fields = [PointField(name=n, offset=4*i, datatype=PointField.FLOAT32, count=1)
                  for i, n in enumerate(("x", "y", "z"))]
        cloud = PointCloud2(height=1, width=1, fields=fields, point_step=12, row_step=12)
        cloud.header.frame_id = "livox_frame"
        cloud.data = np.array([[2, 0, 1]], dtype="<f4").tobytes()
        return cloud

    def test_valid_inputs_release_and_each_stale_input_blocks_immediately(self):
        for stale in ("last_scan", "lidar_time", "odom_time"):
            gate, output = self.make_filter()
            gate.lidar = (np.eye(3), np.zeros(3))
            gate.lidar_time = gate.odom_time = 10.0
            command = Twist()
            command.linear.x = 0.2
            with patch("harp.approach_stop.time.monotonic", return_value=10.0):
                gate.on_cloud(self.usable_cloud())
                gate.on_nav(command)
                self.assertEqual(output[-1].linear.x, 0.2)
                setattr(gate, stale, 9.0)
                gate.on_nav(command)
                self.assertEqual(output[-1].linear.x, 0.0)
                gate.on_timer()
                self.assertEqual(len(gate.history), 0)
                gate.lidar_time = gate.odom_time = 10.0
                gate.on_cloud(self.usable_cloud())
                gate.on_nav(command)
                self.assertEqual(output[-1].linear.x, 0.2)

    def test_bad_clouds_do_not_refresh_usable_scan_time(self):
        for bad in ("frame", "truncated", "nan", "missing_field"):
            gate, _ = self.make_filter()
            gate.lidar = (np.eye(3), np.zeros(3))
            gate.lidar_time = gate.odom_time = 10.0
            cloud = self.usable_cloud()
            if bad == "frame":
                cloud.header.frame_id = "other_frame"
            elif bad == "truncated":
                cloud.data = b"x"
            elif bad == "nan":
                cloud.data = np.array([[float("nan"), 0, 1]], dtype="<f4").tobytes()
            else:
                cloud.fields = []
            with patch("harp.approach_stop.time.monotonic", return_value=10.0):
                gate.on_cloud(cloud)
            self.assertIsNone(gate.last_scan)
            self.assertTrue(gate.lost)

    def camera_cloud(self, point):
        cloud = self.usable_cloud()
        cloud.header.frame_id = "d435_depth_optical_frame"
        cloud.data = np.array([point], dtype="<f4").tobytes()
        return cloud

    def test_camera_sees_legs_ahead_the_lidar_misses(self):
        # The lidar's nearest is 2 m ahead; the camera's, a leg 1.2 m ahead and 0.15 m up,
        # lower than the lidar's MIN_Z: the camera's counts while its frame is fresh.
        for camera_at, expected in ((9.9, 1.2), (9.5, 2.0)):
            gate, _ = self.make_filter()
            gate.lidar = gate.camera = (np.eye(3), np.zeros(3))
            gate.lidar_time = gate.odom_time = 10.0
            with patch("harp.approach_stop.time.monotonic", return_value=camera_at):
                gate.on_camera(self.camera_cloud([1.2, 0.0, 0.15]))
            with patch("harp.approach_stop.time.monotonic", return_value=10.0):
                gate.on_cloud(self.usable_cloud())
            self.assertAlmostEqual(gate.history[-1][1][0], expected, places=5)

    def test_camera_alone_never_releases_gate(self):
        gate, output = self.make_filter()
        gate.camera = (np.eye(3), np.zeros(3))
        gate.odom_time = 10.0
        with patch("harp.approach_stop.time.monotonic", return_value=10.0):
            gate.on_camera(self.camera_cloud([1.2, 0.0, 0.15]))
            gate.on_nav(Twist())
        self.assertIsNone(gate.last_scan)
        self.assertEqual(output[-1].linear.x, 0.0)

    def test_invalid_speed_does_not_refresh_odometry(self):
        gate, _ = self.make_filter()
        msg = Odometry()
        msg.twist.twist.linear.x = float("nan")
        gate.on_odom(msg)
        self.assertIsNone(gate.odom_time)

    def test_route_failure_latches_zero_despite_fresh_inputs_and_late_commands(self):
        gate, output = self.make_filter()
        gate.lidar = (np.eye(3), np.zeros(3))
        gate.lidar_time = gate.odom_time = 10.0
        command = Twist()
        command.linear.x = 0.2
        with patch("harp.approach_stop.time.monotonic", return_value=10.0):
            gate.on_cloud(self.usable_cloud())
            gate.on_nav(command)
            self.assertEqual(output[-1].linear.x, 0.2)
            for value in ("bad", "[]", '{"source":"other","event":"goal_result","status":"FAILED"}'):
                gate.on_event(String(data=value))
            self.assertFalse(gate.goal_failed)
            gate.on_event(String(data='{"source":"goal","event":"goal_result","status":"TIMEOUT"}'))
            self.assertEqual(output[-1].linear.x, 0.0)
            gate.on_cloud(self.usable_cloud())
            gate.on_nav(command)
            self.assertEqual(output[-1].linear.x, 0.0)
            self.assertTrue(gate.goal_failed)

    def test_startup_without_scan_or_transform_holds_zero(self):
        gate, output = self.make_filter()
        command = Twist()
        command.linear.x = 0.2
        gate.on_nav(command)
        self.assertEqual(output[-1].linear.x, 0.0)

    def test_untransformable_cloud_never_releases_gate(self):
        gate, output = self.make_filter()
        with patch("harp.approach_stop.time.monotonic", return_value=10.0):
            gate.on_cloud(PointCloud2())
            command = Twist()
            command.linear.x = 0.2
            gate.on_nav(command)
        self.assertEqual(output[-1].linear.x, 0.0)
        self.assertIsNone(gate.last_scan)


class CloudLayout(unittest.TestCase):
    def test_float64_coordinates(self):
        fields = [PointField(name=n, offset=8*i, datatype=PointField.FLOAT64, count=1)
                  for i, n in enumerate(("x", "y", "z"))]
        cloud = PointCloud2(height=1, width=1, fields=fields, point_step=24, row_step=24)
        cloud.data = np.array([[1.5, 2.5, 3.5]], dtype="<f8").tobytes()
        np.testing.assert_array_equal(xyz(cloud), [[1.5, 2.5, 3.5]])

    def test_big_endian_and_row_padding(self):
        fields = [PointField(name=n, offset=4*i, datatype=PointField.FLOAT32, count=1)
                  for i, n in enumerate(("x", "y", "z"))]
        for endian in ("<", ">"):
            cloud = PointCloud2(height=2, width=1, fields=fields, point_step=12,
                                row_step=16, is_bigendian=endian == ">")
            cloud.data = b"".join(np.array(p, dtype=endian+"f4").tobytes() + b"PAD!"
                                  for p in ((1, 2, 3), (4, 5, 6)))
            np.testing.assert_array_equal(xyz(cloud), [[1, 2, 3], [4, 5, 6]])


if __name__ == "__main__":
    unittest.main()
