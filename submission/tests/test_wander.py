"""harp/behavior.py's wander points: in front of the robot, within the area around its
start. Needs ROS 2 Foxy, so it runs in our image (how: tests/test_approach_stop.py)."""

import math
import random
import unittest

from harp.behavior import FRONT, MAX_LEG, MIN_LEG, wander_point


def bearing(pose, point):
    """Degrees from where the robot faces to the point."""
    a = math.atan2(point[1] - pose[1], point[0] - pose[0]) - pose[2]
    return math.degrees(math.atan2(math.sin(a), math.cos(a)))


class WanderPoint(unittest.TestCase):
    def points(self, pose, home=(0.0, 0.0), radius=2.5, n=200):
        rng = random.Random(1)
        return [wander_point(pose, home, radius, rng) for _ in range(n)]

    def test_in_front_from_the_start(self):
        for yaw in (0.0, 2.0, -2.5):
            pose = (0.0, 0.0, yaw)
            for p in self.points(pose):
                self.assertLessEqual(abs(bearing(pose, p)), FRONT + 1e-6)
                self.assertTrue(MIN_LEG <= math.dist(p, pose[:2]) <= MAX_LEG + 1e-6)

    def test_stays_within_the_area(self):
        pose = (1.5, 1.0, 0.7)
        for p in self.points(pose):
            self.assertLessEqual(math.dist(p, (0.0, 0.0)), 2.5)
            self.assertGreaterEqual(math.dist(p, pose[:2]), MIN_LEG)

    def test_turns_at_the_edge_facing_out(self):
        pose = (2.4, 0.0, 0.0)
        points = self.points(pose)
        for p in points:
            self.assertLessEqual(math.dist(p, (0.0, 0.0)), 2.5)
        self.assertTrue(all(abs(bearing(pose, p)) > FRONT for p in points))

    def test_heads_home_from_outside(self):
        self.assertEqual(wander_point((6.0, 0.0, 0.0), (0.0, 0.0), 2.5, random.Random(1)),
                         (0.0, 0.0))


if __name__ == "__main__":
    unittest.main()
