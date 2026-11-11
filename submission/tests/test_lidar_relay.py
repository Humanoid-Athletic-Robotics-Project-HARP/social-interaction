"""harp/lidar_relay.py: the rear cloud (the robot's lidar turned 180 deg about z for Nav2's
clearing view) and the self-filter's plumbing (the sim never runs the relay, so only this
checks it; harp/self_filter.py itself: test_self_filter.py)."""

from collections import deque
import threading
import time
from types import SimpleNamespace
import unittest

import numpy as np
from sensor_msgs.msg import PointField

from harp.lidar_relay import JOINT_TIMEOUT, LidarRelay, point_rows, turned, xyz
from harp.self_filter import JOINT_WINDOW, MAX_POSES

# Unitree's lidar_driver layout: x y z intensity (float32), ring (uint16), time (float32)
FIELDS = [PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
          PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
          PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
          PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
          PointField(name="ring", offset=16, datatype=PointField.UINT16, count=1),
          PointField(name="time", offset=18, datatype=PointField.FLOAT32, count=1)]
POINT = np.dtype({"names": ["x", "y", "z", "intensity", "ring", "time"],
                  "formats": ["<f4", "<f4", "<f4", "<f4", "<u2", "<f4"],
                  "offsets": [0, 4, 8, 12, 16, 18], "itemsize": 22})


class Turned(unittest.TestCase):
    def test_negates_x_and_y_only(self):
        cloud = np.zeros(2, POINT)
        cloud["x"], cloud["y"], cloud["z"] = (1.5, -2.0), (0.25, 3.0), (1.2, -0.4)
        cloud["intensity"], cloud["ring"], cloud["time"] = (7.0, 9.0), (3, 65535), (0.001, 0.099)
        out = np.frombuffer(turned(cloud.tobytes(), FIELDS, 22, False), POINT)
        np.testing.assert_array_equal(out["x"], -cloud["x"])
        np.testing.assert_array_equal(out["y"], -cloud["y"])
        for name in ("z", "intensity", "ring", "time"):
            np.testing.assert_array_equal(out[name], cloud[name])

    def test_big_endian(self):
        cloud = np.array([(1.0, 2.0, 3.0)], dtype=[("x", ">f4"), ("y", ">f4"), ("z", ">f4")])
        out = np.frombuffer(turned(cloud.tobytes(), FIELDS[:3], 12, True), cloud.dtype)
        self.assertEqual(tuple(out[0]), (-1.0, -2.0, 3.0))

    def test_non_float_x_is_refused(self):
        fields = [PointField(name="x", offset=0, datatype=PointField.INT32, count=1)]
        with self.assertRaises(ValueError):
            turned(bytes(12), fields, 12, False)


class DropsX(object):
    """A stand-in self-filter: the 'body' is every point with x below 0."""
    def __init__(self):
        self.poses = None

    def keep(self, points, poses):
        self.poses = poses
        return points[:, 0] >= 0


def relay(joints=()):
    """A LidarRelay without ROS or the SDK: the self-filter's plumbing alone."""
    r = LidarRelay.__new__(LidarRelay)
    r.self_filter, r.joints, r.joints_lock, r.filtering = DropsX(), deque(joints), threading.Lock(), None
    return r


class WithoutBody(unittest.TestCase):
    def cloud(self):
        cloud = np.zeros(3, POINT)
        cloud["x"], cloud["y"], cloud["z"] = (1.0, -0.2, 2.0), (0.0, 0.1, -1.0), (0.5, 0.3, 0.1)
        cloud["intensity"], cloud["ring"], cloud["time"] = (5.0, 6.0, 7.0), (1, 2, 3), (0.01, 0.02, 0.03)
        return cloud

    def test_drops_the_body_and_keeps_every_field(self):
        now = time.monotonic()
        r = relay([(now - 0.01, np.zeros(29))])
        msg = SimpleNamespace(fields=FIELDS, is_bigendian=False)
        rows = r.without_body(msg, point_rows(self.cloud().tobytes(), 22))
        out = np.frombuffer(rows.tobytes(), POINT)
        np.testing.assert_array_equal(out, self.cloud()[[0, 2]])
        self.assertTrue(r.filtering)

    def test_without_fresh_joints_the_cloud_goes_out_whole(self):
        msg = SimpleNamespace(fields=FIELDS, is_bigendian=False)
        for joints in ([], [(time.monotonic() - JOINT_TIMEOUT - 0.1, np.zeros(29))]):
            r = relay(joints)
            rows = r.without_body(msg, point_rows(self.cloud().tobytes(), 22))
            np.testing.assert_array_equal(np.frombuffer(rows.tobytes(), POINT), self.cloud())
            self.assertFalse(r.filtering)

    def test_poses_cover_the_window_newest_first(self):
        now = time.monotonic()
        joints = [(now - 0.3 + 0.02 * i, np.full(29, float(i))) for i in range(15)]   # 50 Hz
        poses = relay(joints).poses()
        self.assertLessEqual(len(poses), MAX_POSES)
        self.assertEqual(poses[0][0], 14.0)                       # the newest
        self.assertGreaterEqual(min(p[0] for p in poses), 15 - JOINT_WINDOW / 0.02 - 1)

    def test_poses_reach_back_over_a_late_cloud_scan(self):
        # A cloud 100 ms late was scanned 0.1-0.2 s ago: a pose from then must be checked.
        now = time.monotonic()
        joints = [(now - 0.3 + 0.02 * i, np.full(29, 0.3 - 0.02 * i)) for i in range(16)]   # value: age
        ages = [p[0] for p in relay(joints).poses()]
        self.assertGreaterEqual(max(ages), 0.17)
        self.assertLessEqual(max(np.diff(sorted(ages))), 0.041)   # as close as with 0.15 s and 4

    def test_rows_leave_out_each_rows_padding(self):
        cloud = np.zeros(4, POINT)
        cloud["x"], cloud["ring"] = (1.0, 2.0, 3.0, 4.0), (1, 2, 3, 4)
        two_rows = cloud[:2].tobytes() + bytes(6) + cloud[2:4].tobytes() + bytes(6)   # row_step 50
        out = np.frombuffer(point_rows(two_rows, 22, width=2, height=2, row_step=50).tobytes(), POINT)
        np.testing.assert_array_equal(out, cloud[:4])

    def test_xyz_reads_the_layout(self):
        points = xyz(point_rows(self.cloud().tobytes(), 22), FIELDS, False)
        np.testing.assert_allclose(points[:, 0], self.cloud()["x"])
        np.testing.assert_allclose(points[:, 2], self.cloud()["z"], rtol=1e-6)


if __name__ == "__main__":
    unittest.main()
