"""harp/lidar_mount.py's head shadow, with and without the face visor, and Nav2's clearing
views following it (nav2/nav2.launch.py): a sim run would only show a wrong one as worse
avoidance."""

import importlib.util
import math
import os
import unittest
from unittest.mock import patch

import numpy as np

from harp import lidar_mount

LAUNCH = os.path.join(os.path.dirname(__file__), "..", "nav2", "nav2.launch.py")


def direction(azimuth, elevation):
    """A direction in torso_link from azimuth (deg, + left) and elevation (deg, + up)."""
    a, e = math.radians(azimuth), math.radians(elevation)
    return [math.cos(e) * math.cos(a), math.cos(e) * math.sin(a), math.sin(e)]


class Sees(unittest.TestCase):
    def test_visor_off_window_and_wedges(self):
        d = [direction(a, -10) for a in (0, 30, -30, 50, -50, 80, -80, 180)]
        np.testing.assert_array_equal(lidar_mount.sees(d, visor=False),
                                      [True, True, True, False, False, True, True, True])

    def test_visor_on_blind_ahead(self):
        d = [direction(a, -10) for a in (0, 30, -50, 80, 180)]
        np.testing.assert_array_equal(lidar_mount.sees(d, visor=True), [False, False, False, True, True])

    def test_window_narrows_looking_down_and_the_camera_band_blocks_the_top(self):
        self.assertTrue(lidar_mount.sees([direction(30, -20)], visor=False)[0])
        self.assertFalse(lidar_mount.sees([direction(30, -48)], visor=False)[0])
        self.assertTrue(lidar_mount.sees([direction(0, -48)], visor=False)[0])
        self.assertFalse(lidar_mount.sees([direction(0, 4)], visor=False)[0])


class Nav2Clearing(unittest.TestCase):
    def setUp(self):
        spec = importlib.util.spec_from_file_location("nav2_launch", LAUNCH)
        self.launch = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(self.launch)

    def sources(self, path):
        with open(path) as f:
            return [line.split(":", 1)[1].split() for line in f if "observation_sources:" in line]

    def test_front_clearing_only_with_the_visor_off(self):
        with patch.object(lidar_mount, "VISOR", False):
            off = self.sources(self.launch.parameters_for())
        with patch.object(lidar_mount, "VISOR", True):
            on = self.sources(self.launch.parameters_for())
        self.assertEqual(len(off), 2)
        self.assertTrue(all("lidar_front_clearing" in s for s in off))
        self.assertEqual(len(on), 2)
        self.assertTrue(all("lidar_front_clearing" not in s and "lidar_clearing" in s for s in on))


if __name__ == "__main__":
    unittest.main()
