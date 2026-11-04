"""The right-arm wave (harp/wave.py) as a function of time; no robot or simulator is involved.

The forward-kinematics checks load HIM's G1 model (/harp/model in the image) and only
compute poses (mj_kinematics); they are skipped where the model or MuJoCo is missing.
"""

import unittest
from pathlib import Path

import numpy as np

from harp import wave

MODEL = Path("/harp/model/g1_robot.xml")
START = 10.0


def samples(w, step=0.001):
    """(t, q, dq, phase) every step s from the wave's start to 1 s after it ends."""
    for t in np.arange(START, START + w.duration + 1.0, step):
        q, dq, phase = w.at(float(t))
        yield float(t), q, dq, phase


class Motion(unittest.TestCase):
    def test_starts_at_the_given_pose(self):
        start = (0.05, -0.04, 0.03, 0.02, 0.0, 0.01, -0.01)
        q, dq, phase = wave.Wave(START, start).at(START)
        np.testing.assert_allclose(q, start)
        np.testing.assert_allclose(dq, 0.0, atol=1e-9)
        self.assertEqual(phase, "raise")

    def test_hand_up_after_the_raise(self):
        q, _, _ = wave.Wave(START, wave.NEUTRAL).at(START + wave.RAISE_S)
        np.testing.assert_allclose(q, wave.HAND_UP, atol=1e-9)

    def test_ends_at_neutral_and_stays(self):
        w = wave.Wave(START, wave.NEUTRAL)
        for t in (START + w.duration, START + w.duration + 5.0):
            q, dq, phase = w.at(t)
            np.testing.assert_allclose(q, wave.NEUTRAL, atol=1e-9)
            np.testing.assert_allclose(dq, 0.0, atol=1e-9)
            self.assertEqual(phase, "done")

    def test_targets_stay_inside_joint_ranges(self):
        low, high = np.array(wave.LOW) + 0.1, np.array(wave.HIGH) - 0.1
        for t, q, _, _ in samples(wave.Wave(START, wave.NEUTRAL)):
            self.assertTrue(np.all(q >= low) and np.all(q <= high), f"t={t}: {q}")

    def test_position_and_velocity_are_continuous(self):
        w, h = wave.Wave(START, wave.NEUTRAL), 1e-4
        last = None
        for t, q, dq, _ in samples(w):
            numeric = (w.at(t + h)[0] - w.at(t - h)[0]) / (2 * h)
            self.assertLess(np.max(np.abs(numeric - dq)), 0.02, f"t={t}")
            if last is not None:
                self.assertLess(np.max(np.abs(dq - last)), 0.1, f"velocity jump at t={t}")
            last = dq

    def test_speeds_stay_far_below_him_limits(self):
        for t, _, dq, _ in samples(wave.Wave(START, wave.NEUTRAL)):
            self.assertLessEqual(np.max(np.abs(dq)), 8.0, f"t={t}")
            self.assertEqual(dq[5], 0.0)    # wrist pitch: 5 Nm, about 0.11 rad of tracking room
            self.assertEqual(dq[6], 0.0)    # wrist yaw: the same

    def test_wave_has_the_set_cycles(self):
        signs = [np.sign(q[3] - wave.HAND_UP[3]) for _, q, _, phase in samples(wave.Wave(START, wave.NEUTRAL))
                 if phase == "wave" and abs(q[3] - wave.HAND_UP[3]) >= 1e-4]
        changes = sum(1 for a, b in zip(signs, signs[1:]) if a != b)
        self.assertEqual(changes, 2 * wave.CYCLES - 1)

    def test_phases_in_order(self):
        phases = []
        for _, _, _, phase in samples(wave.Wave(START, wave.NEUTRAL)):
            if not phases or phases[-1] != phase:
                phases.append(phase)
        self.assertEqual(phases, ["raise", "wave", "lower", "done"])


@unittest.skipUnless(MODEL.exists(), "HIM's G1 model is only in the image")
class Pose(unittest.TestCase):
    """Where the hand is, from HIM's G1 model (pelvis 0.79 m up, upright)."""

    @classmethod
    def setUpClass(cls):
        import mujoco
        cls.mujoco = mujoco
        cls.model = mujoco.MjModel.from_xml_path(str(MODEL))
        cls.data = mujoco.MjData(cls.model)

    def position(self, arm, body):
        d = self.data
        d.qpos[:] = 0.0
        d.qpos[0:7] = (0.0, 0.0, 0.79, 1.0, 0.0, 0.0, 0.0)
        d.qpos[29:36] = arm
        self.mujoco.mj_kinematics(self.model, d)
        return d.xpos[self.model.body(body).id].copy()

    def hand(self, arm, relative_to):
        return self.position(arm, "right_wrist_yaw_link") - self.position(arm, relative_to)

    def test_hand_up_is_beside_the_head(self):
        x, y, z = self.hand(wave.HAND_UP, "right_shoulder_pitch_link")
        self.assertGreaterEqual(z, 0.15)
        self.assertLessEqual(y, -0.15)
        self.assertLessEqual(abs(x), 0.2)

    def test_swing_moves_the_hand_sideways(self):
        up = np.array(wave.HAND_UP)
        centre = self.position(up, "right_wrist_yaw_link")
        for swing in (wave.SWING, -wave.SWING):
            arm = up.copy()
            arm[3] += swing
            d = self.position(arm, "right_wrist_yaw_link") - centre
            self.assertGreaterEqual(np.linalg.norm(d), 0.05)
            self.assertGreaterEqual(abs(d[1]), 0.9 * np.linalg.norm(d))

    def test_raise_path_stays_clear_of_the_torso(self):
        up = np.array(wave.HAND_UP)
        for s in np.linspace(0.0, 1.0, 21):
            self.assertLessEqual(self.hand(up * s, "torso_link")[1], -0.14, f"s={s}")


if __name__ == "__main__":
    unittest.main()
