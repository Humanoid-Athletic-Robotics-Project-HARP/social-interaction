"""Making way (harp/make_way.py): out of the office corner HIM's robot stood stuck in."""
from types import SimpleNamespace
import math
import unittest

import numpy as np

from harp.make_way import COOLDOWN, STALL_TIME, MakeWay, toward

# The office's corner: corridor 1 along x (|y| < 1.1), corridor 2 up from it (6.2 < x < 8.4).
WALLS = [((-1.8, -1.1), (8.4, -1.1)), ((-1.8, 1.1), (6.2, 1.1)), ((6.2, 1.1), (6.2, 8.4)),
         ((8.4, -1.1), (8.4, 8.4))]
# Standing there, so in the planner's fixed field: the plant, a person, the cart that stopped.
STANDING = [(6.45, 1.4, .28), (7.8, 2.6, .25), (7.0, 2.9, .45)]


def clearance(points):
    points = np.asarray(points, dtype=float).reshape(-1, 2)
    best = np.full(len(points), np.inf)
    for a, b in WALLS:
        a, b = np.array(a), np.array(b)
        f = np.clip((points - a) @ (b - a) / ((b - a) @ (b - a)), 0, 1)
        best = np.minimum(best, np.linalg.norm(points - (a + f[:, None] * (b - a)), axis=1))
    for x, y, r in STANDING:
        best = np.minimum(best, np.linalg.norm(points - [x, y], axis=1) - r)
    return best


def person(x, y, ident=1):
    return SimpleNamespace(id=ident, x=x, y=y, radius=.25)


class MakeWayTest(unittest.TestCase):
    GOAL = (7.3, 7.0)

    def stall(self, way, pose, seconds, mode="evade", tracks=()):
        spot = None
        for k in range(int(seconds * 5) + 1):
            spot = way.update(k * .2, pose, self.GOAL, mode, list(tracks), clearance)
        return spot

    def test_stuck_in_the_corner_it_steps_back_where_it_is_wider(self):
        spot = self.stall(MakeWay(), (7.0, 2.1, math.pi / 2), STALL_TIME + 1)
        self.assertIsNotNone(spot)
        self.assertLess(spot[1], 1.1)   # back in the junction, out of corridor 2
        self.assertGreater(clearance(spot)[0], clearance([(7.0, 2.1)])[0] + .3)

    def test_it_waits_away_from_people_too(self):
        # Someone in corridor 1: it steps back into the junction on the far side from them.
        walker = person(5.6, 0.2)
        spot = self.stall(MakeWay(), (7.0, 2.1, 0.), STALL_TIME + 1, tracks=[walker])
        self.assertIsNotNone(spot)
        self.assertLess(spot[1], 1.1)
        self.assertGreater(math.hypot(spot[0] - walker.x, spot[1] - walker.y), 1.5)

    def test_with_people_where_it_would_go_it_stays(self):
        crowd = [person(7.0, 0.4), person(7.8, -0.4, 2), person(6.6, -0.2, 3)]
        self.assertIsNone(self.stall(MakeWay(), (7.0, 2.1, 0.), STALL_TIME + 1, tracks=crowd))

    def test_not_while_making_progress_near_the_goal_or_following(self):
        way = MakeWay()
        for k in range(int((STALL_TIME + 1) * 5)):   # walking up corridor 2
            self.assertIsNone(way.update(k * .2, (7.0, 2.0 + .1 * k, 0.), self.GOAL, "evade", [], clearance))
        self.assertIsNone(self.stall(MakeWay(), (7.3, 6.5, 0.), STALL_TIME + 1))   # by the goal
        self.assertIsNone(self.stall(MakeWay(), (7.0, 2.1, 0.), STALL_TIME + 1, mode="follow"))

    def test_it_gives_up_at_the_spot_then_waits_out_its_cooldown(self):
        way = MakeWay()
        spot = self.stall(way, (7.0, 2.1, 0.), STALL_TIME + 1)
        t = (STALL_TIME + 1) + .2
        self.assertIsNone(way.update(t, (spot[0], spot[1], 0.), self.GOAL, "evade", [], clearance))
        self.assertIsNone(way.update(t + COOLDOWN - 1, (7.0, 2.1, 0.), self.GOAL, "evade", [], clearance))

    def test_toward_is_a_body_frame_request(self):
        vx, vy, wz = toward((0., 0., math.pi / 2), (0., 2.), .3)   # facing +y, the spot ahead
        self.assertAlmostEqual(vx, .3)
        self.assertAlmostEqual(vy, 0.)
        self.assertEqual(wz, 0.)


if __name__ == "__main__":
    unittest.main()
