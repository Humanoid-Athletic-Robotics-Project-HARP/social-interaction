"""harp/lidar_relay.py's rear cloud: the robot's lidar turned 180 deg about z for Nav2's
clearing view (the sim never runs the relay, so only this checks it)."""

import unittest

import numpy as np
from sensor_msgs.msg import PointField

from harp.lidar_relay import turned

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


if __name__ == "__main__":
    unittest.main()
