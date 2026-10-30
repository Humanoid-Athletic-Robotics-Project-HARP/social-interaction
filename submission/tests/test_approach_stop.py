"""harp/approach_stop.py's rule for something coming at the robot. Needs ROS 2 Foxy and
numpy, so it runs in our image:
  docker run --rm -v "$PWD/submission:/src:ro" -w /src --entrypoint bash harp-submission:local \
    -c 'source /opt/ros/foxy/setup.bash && PYTHONPATH=/src:$PYTHONPATH python3 -m unittest discover tests'
"""

import unittest

import numpy as np
from sensor_msgs.msg import PointCloud2, PointField

from harp.approach_stop import WINDOW, coming, moving, nearest_ahead, xyz

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


if __name__ == "__main__":
    unittest.main()
