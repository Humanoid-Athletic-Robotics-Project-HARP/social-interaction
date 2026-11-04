"""wave_check.py's pass rules and tilt; no run folder or simulator is involved."""

import math
import unittest

import wave_check

GOOD = dict(status="within_limits", thrash_mode="enforce", weighted_rms=0.3, pelvis_drift_m=0.01,
            tilt_change_deg=1.0, foot_move_m=0.005, reply_s=0.005, wave_done=True)


class Tilt(unittest.TestCase):
    def test_tilt_of_upright_and_tipped(self):
        self.assertAlmostEqual(wave_check.tilt_deg((1.0, 0.0, 0.0, 0.0)), 0.0, places=6)
        half = math.radians(10.0) / 2
        self.assertAlmostEqual(wave_check.tilt_deg((math.cos(half), 0.0, math.sin(half), 0.0)), 10.0, places=4)


class PeakScore(unittest.TestCase):
    def test_peak_over_the_window(self):
        thrash = [{"metrics": {"weighted_rms": 0.9}}, None, {"metrics": {"weighted_rms": 0.5}},
                  {"metrics": {"weighted_rms": 0.2}}]
        self.assertEqual(wave_check.peak_score(thrash, [1, 2, 3]), 0.5)

    def test_no_telemetry_is_none(self):
        # course.py --prune strips the telemetry from all but the newest runs
        self.assertIsNone(wave_check.peak_score(None, [0, 1]))
        self.assertIsNone(wave_check.peak_score([None, None], [0, 1]))


class Failures(unittest.TestCase):
    def test_a_good_run_has_no_failures(self):
        self.assertEqual(wave_check.failures(GOOD), [])

    def test_each_check_fails_on_its_own(self):
        for key, value, name in (("status", "violation", "status"), ("thrash_mode", "monitor", "thrash_mode"),
                                 ("weighted_rms", 0.8, "weighted_rms"), ("pelvis_drift_m", 0.04, "pelvis_drift_m"),
                                 ("tilt_change_deg", 6.0, "tilt_change_deg"), ("foot_move_m", 0.03, "foot_move_m"),
                                 ("reply_s", 0.03, "reply_s"), ("wave_done", False, "wave_done"),
                                 ("weighted_rms", None, "thrash_telemetry")):
            with self.subTest(key=key, value=value):
                self.assertEqual(wave_check.failures(dict(GOOD, **{key: value})), [name])


if __name__ == "__main__":
    unittest.main()
