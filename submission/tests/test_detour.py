import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from harp.detour import PATIENCE, DetourPatience, path_length  # noqa: E402


class Detour(unittest.TestCase):
    def test_path_length(self):
        self.assertAlmostEqual(path_length([(0, 0), (3, 0), (3, 4)]), 7.0)
        self.assertEqual(path_length([(1, 1)]), 0.0)

    def test_waits_for_the_short_way_and_resumes_when_it_is_back(self):
        # HIM office: 3 m to the goal, then a waiting cart made Nav2's path 11-16 m.
        d = DetourPatience()
        for t, length in ((0, 4.), (1, 3.5), (2, 3.)):
            self.assertIsNone(d.update(t, length))
        self.assertFalse(d.waiting(2))
        self.assertEqual(d.update(3, 11.), "wait")
        self.assertTrue(d.waiting(3.5))
        self.assertIsNone(d.update(4, 16.))
        self.assertTrue(d.waiting(4))
        self.assertEqual(d.update(8, 3.4), "resume")
        self.assertFalse(d.waiting(8))

    def test_takes_the_detour_after_patience(self):
        d = DetourPatience()
        d.update(0, 3.)
        self.assertEqual(d.update(1, 12.), "wait")
        self.assertFalse(d.waiting(1+PATIENCE))
        self.assertEqual(d.update(1+PATIENCE, 12.), "give_up")
        # The detour is now the way: following it, its paths shrink, no new wait.
        for k in range(5):
            self.assertIsNone(d.update(2+PATIENCE+k, 11.5-k))
            self.assertFalse(d.waiting(2+PATIENCE+k))

    def test_a_route_that_is_long_from_the_start_or_grows_slowly_is_no_detour(self):
        d = DetourPatience()
        for t in range(20):
            self.assertIsNone(d.update(t, 20.+.3*t))   # no jump: never waits
        self.assertFalse(d.waiting(20))

    def test_small_jumps_are_no_detour(self):
        d = DetourPatience()
        d.update(0, 2.)
        self.assertIsNone(d.update(1, 4.5))    # over 1.5x but only 2.5 m longer
        d.reset()
        d.update(0, 10.)
        self.assertIsNone(d.update(1, 14.))    # 4 m longer but under 1.5x

    def test_a_new_goal_forgets_old_paths_and_bad_lengths_are_ignored(self):
        d = DetourPatience()
        d.update(0, 2.)
        d.reset()
        self.assertIsNone(d.update(1, 12.))
        self.assertIsNone(d.update(2, float("nan")))
        self.assertIsNone(d.update(3, 0.))
        self.assertFalse(d.waiting(3))

    def test_old_paths_drop_out_of_the_reference(self):
        d = DetourPatience()
        d.update(0, 3.)
        self.assertIsNone(d.update(11, 12.))   # the 3 m path is over MEMORY s old


if __name__ == "__main__":
    unittest.main()
