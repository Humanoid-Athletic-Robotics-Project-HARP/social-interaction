"""harp/behavior.py: wander points in front of the robot, within the area around its
start; goals Nav2 calls reached too early sent again. Needs ROS 2 Foxy, so it runs in our
image (how: tests/test_approach_stop.py)."""

import math
import random
import time
import unittest
from types import SimpleNamespace
from unittest.mock import patch

from harp.behavior import ARRIVED, FRONT, MAX_LEG, MIN_LEG, Behavior, wander_point


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


class Go(unittest.TestCase):
    def go(self, ends, statuses):
        """Behavior.go with Nav2 answering statuses in turn, the robot at each of ends after
        each; the statuses it returned, and how many goals it sent."""
        robot = SimpleNamespace(node=None, nav=None, pose=None, events=[])
        robot.event = lambda name, **fields: robot.events.append(name)
        answers = iter(zip(ends, statuses))

        def go_to(node, nav, goal, deadline, interrupt=None, handoff=None):
            robot.pose, status = next(answers)
            return status, None

        with patch("harp.behavior.go_to", go_to):
            status = Behavior.go(robot, (2.0, 0.0, 0.0), time.monotonic() + 10)
        return status, len(robot.events) + 1

    def test_reached(self):
        self.assertEqual(self.go([(1.8, 0.1, 0.1)], ["SUCCEEDED"]), ("SUCCEEDED", 1))

    def test_ended_at_once_far_away_is_sent_again(self):
        self.assertEqual(self.go([(0.5, 0.0, 0.0), (2.0, 0.0, 0.0)], ["SUCCEEDED"] * 2),
                         ("SUCCEEDED", 2))

    def test_wrong_heading_is_sent_again(self):
        self.assertEqual(self.go([(2.0, 0.0, math.pi), (2.0, 0.0, 0.2)], ["SUCCEEDED"] * 2),
                         ("SUCCEEDED", 2))

    def test_failure_is_not_sent_again(self):
        self.assertEqual(self.go([(0.5, 0.0, 0.0)], ["ABORTED"]), ("ABORTED", 1))

    def test_arrived_margin(self):
        self.assertEqual(self.go([(2.0 - ARRIVED + 0.01, 0.0, 0.0)], ["SUCCEEDED"]), ("SUCCEEDED", 1))


if __name__ == "__main__":
    unittest.main()
