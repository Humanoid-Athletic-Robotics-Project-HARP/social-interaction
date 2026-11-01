"""Verdicts must reject contacts, instability and missing obstacle motion."""
import math
import unittest
from native_g1_sim import Trial


class Verdict(unittest.TestCase):
    def trial(self):
        trial = Trial.__new__(Trial)
        trial.truth = [30., 4., 0., .74]
        trial.first_truth = [0., 0.]
        trial.status = 'SUCCEEDED'
        trial.contacts = 0
        trial.movers = trial.obstacle_count = 7
        trial.min_height = .73
        trial.max_tilt = .12
        trial.modes = {'follow': 1, 'yield': 1}
        return trial

    def test_success_requires_every_obstacle_and_stable_contact_free_walking(self):
        self.assertTrue(self.trial().report()['passed'])
        for key, value in [('contacts', 1), ('movers', 6), ('min_height', .4),
                           ('max_tilt', math.radians(50)), ('status', 'TIMEOUT')]:
            with self.subTest(key=key):
                trial = self.trial()
                setattr(trial, key, value)
                self.assertFalse(trial.report()['passed'])
