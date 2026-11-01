"""Safety and behavioral properties of the prediction/controller boundary."""
import math
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np
from harp.predictive_mpc import Config, Planner


def obstacle(x, y, vx=0, vy=0, radius=.2, uncertainty=.03):
    return SimpleNamespace(x=x, y=y, vx=vx, vy=vy, radius=radius,
                           uncertainty=uncertainty, last_seen=0., hits=5)


class PredictiveMpc(unittest.TestCase):
    def test_clear_route_passes_acceleration_limited_request(self):
        planner = Planner()
        result = planner.plan((0, 0, 0), (0, 0, 0), (.3, 0, .6), [], 0)
        self.assertEqual(result.mode, 'follow')
        self.assertAlmostEqual(result.command[0], .08)
        self.assertLessEqual(abs(result.command[2]), .2+1e-9)

    def test_diagonal_speed_and_acceleration_use_vector_norm(self):
        planner = Planner()
        result = planner.plan((0, 0, 0), (0, 0, 0), (.3, .3, 0), [], 0)
        self.assertLessEqual(np.linalg.norm(result.command[:2]), .08+1e-9)
        result = planner.plan((0, 0, 0), (.2, .2, 0), (.3, .3, 0), [], 0)
        self.assertLessEqual(np.linalg.norm(result.command[:2]), .3+1e-9)

    def test_crossing_yields_then_resumes(self):
        planner = Planner()
        result = planner.plan((0, 0, 0), (.25, 0, 0), (.25, 0, 0),
                              [obstacle(1.3, -1.5, vy=.4)], 0)
        self.assertEqual(result.mode, 'yield')
        self.assertGreaterEqual(result.clearance, 0)
        self.assertLess(result.command[0], .25)
        result = planner.plan((0, 0, 0), (0, 0, 0), (.25, 0, 0),
                              [obstacle(1.3, 2, vy=.4)], 0)
        self.assertEqual(result.mode, 'follow')

    def test_head_on_escape_clears_continued_obstacle_path(self):
        planner = Planner()
        item = obstacle(3, 0, vx=-.5)
        result = planner.plan((0, 0, 0), (.25, 0, 0), (.25, 0, 0), [item], 0)
        self.assertEqual(result.mode, 'evade')
        self.assertGreater(abs(result.command[1]), .02)
        self.assertGreaterEqual(result.clearance, 0)
        self.assertFalse(result.alert)

    def test_head_on_terminal_check_rejects_merely_postponing_impact(self):
        planner = Planner()
        objects = np.array([[3., 0, -.5, 0, .2, .03]])
        backward = np.tile([-.3, 0., 0.], (3, 1))
        clear, _ = planner._evaluate(backward, np.zeros(3), np.zeros(3), objects)
        self.assertLess(clear[0], 0)

    def test_trapped_scene_requests_zero_and_alert(self):
        planner = Planner()
        # Existing overlap is deliberately infeasible: no predicted escape may certify it.
        result = planner.plan((0, 0, 0), (.1, 0, 0), (.25, 0, 0), [obstacle(.5, 0)], 0)
        self.assertEqual(result.command, (0., 0., 0.))
        self.assertEqual(result.mode, 'blocked')
        self.assertTrue(result.alert)
        self.assertLess(result.clearance, 0)

    def test_swept_collision_is_detected_between_time_samples(self):
        planner = Planner()
        path = np.zeros((1, len(planner.times), 2))
        # Fast object crosses the robot between .0 and .1 second samples.
        objects = np.array([[0., -5., 0., 100., .1, 0.]])
        self.assertLess(planner._clearances(path, objects)[0], 0)

    def test_solver_failure_keeps_only_checked_seed(self):
        planner = Planner()
        with patch('harp.predictive_mpc.minimize', side_effect=TimeoutError):
            result = planner.plan((0, 0, 0), (.25, 0, 0), (.25, 0, 0),
                                  [obstacle(3, 0, vx=-.5)], 0)
        self.assertEqual(result.mode, 'evade')
        self.assertEqual(result.solver, 'feasible_seed_budget')
        self.assertGreaterEqual(result.clearance, 0)
        self.assertLessEqual(np.linalg.norm(result.command[:2]), .3+1e-9)

    def test_invalid_and_excessive_inputs_fail_closed(self):
        cases = [([obstacle(math.nan, 0)], (0, 0, 0)),
                 ([obstacle(0, 0, radius=-1)], (0, 0, 0)),
                 ([obstacle(10, 0)]*65, (0, 0, 0)),
                 ([], (math.inf, 0, 0))]
        for tracks, pose in cases:
            result = Planner().plan(pose, (0, 0, 0), (.25, 0, 0), tracks, 0)
            self.assertEqual(result.mode, 'blocked')
            self.assertTrue(result.alert)
            self.assertEqual(result.command, (0., 0., 0.))

    def test_invalid_configuration_rejected(self):
        with self.assertRaises(ValueError):
            Planner(Config(acceleration=0))


if __name__ == '__main__':
    unittest.main()
