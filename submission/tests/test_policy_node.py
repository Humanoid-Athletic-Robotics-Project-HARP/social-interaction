"""harp_node.py's command limits and harp/state.py's truth events, without HIM or ROS nodes.

HIM's client (/example/client.py in the image) is replaced by a stub class.
"""

import math
import sys
import types
import unittest
from unittest.mock import Mock, patch

import numpy as np

sys.modules.setdefault("client", types.SimpleNamespace(PolicyNode=object))
from harp import wave  # noqa: E402  (before harp_node, which puts the image's /harp first)
import harp_node  # noqa: E402
from harp.state import StatePublisher  # noqa: E402


class CommandLimits(unittest.TestCase):
    def test_commands_are_clipped_to_the_limits(self):
        self.assertEqual(harp_node.clipped([1.2, -0.9, 3.0]), [0.6, -0.4, 1.0])
        self.assertEqual(harp_node.clipped([0.2, 0.0, -0.5]), [0.2, 0.0, -0.5])

    def test_malformed_or_non_finite_commands_are_refused(self):
        for cmd in ([0.1, 0.0], [0.1, 0.0, 0.0, 0.0], [math.nan, 0.0, 0.0], [0.0, math.inf, 0.0],
                    ["a", 0.0, 0.0], 0.3, None):
            with self.subTest(cmd=cmd):
                self.assertIsNone(harp_node.clipped(cmd))


def publisher():
    out = StatePublisher.__new__(StatePublisher)
    out.legs, out.drift, out.next_truth = Mock(), None, 0.0
    out.node = Mock()
    out.from_legs = lambda: (0.0, 0.0, 0.0, (0.0, 0.0, 0.0))
    out.publish_odometry = out.publish_joints_imu = lambda *args: None
    return out


class TruthEvents(unittest.TestCase):
    def test_robot_state_with_a_base_position_but_no_sim_time(self):
        # HIM adds "position" on the robot when G1_BASE_STATE_TOPIC is set; it has no sim_time.
        with patch("harp.state.events.emit") as emit:
            publisher().publish({"q": [], "quat": [1.0, 0.0, 0.0, 0.0], "position": [1.0, 2.0, 0.8]})
        emit.assert_not_called()

    def test_sim_state_logs_truth(self):
        with patch("harp.state.events.emit") as emit:
            publisher().publish({"q": [], "quat": [1.0, 0.0, 0.0, 0.0], "position": [1.0, 2.0, 0.8],
                                 "sim_time": 3.0})
        self.assertEqual(emit.call_args[0][:3], ("policy", "odom", 3.0))



class WaveTask(unittest.TestCase):
    def test_wave_task_detection(self):
        self.assertTrue(harp_node.wave_task({"wave": {}}))
        for task in ({"to": [1, 1]}, {"moves": []}, [0.0, 0.0, 0.0], None):
            with self.subTest(task=task):
                self.assertFalse(harp_node.wave_task(task))


class Walker:
    """HIM's walker stand-in: every target 0.1, fresh lists each call."""

    def command(self, state):
        return {"session_id": "s", "sequence": 0, "observation_id": 0, "q": [0.1] * 29,
                "dq": [0.0] * 29, "kp": [40.0] * 29, "kd": [1.0] * 29, "tau": [0.0] * 29}


READY = 5.0                      # s: the first control_ready state
START = READY + wave.SETTLE_S    # s: when the wave starts


def state(t, ready=True, arm=wave.NEUTRAL, sim=True):
    """A policy state at t; in the sim received_at_secs is another clock, so only sim_time is t."""
    q = [0.0] * 29
    q[22:29] = list(arm)
    out = {"q": q, "control_ready": ready, "received_at_secs": t + 1000.0 if sim else t}
    if sim:
        out["sim_time"] = t
    return out


class WavingPolicyTests(unittest.TestCase):
    def setUp(self):
        self.emit = Mock()
        self.policy = harp_node.WavingPolicy(Walker(), self.emit)
        self.walker = Walker().command(None)

    def test_frame_unchanged_before_control_ready(self):
        self.assertEqual(self.policy.command(state(START + 1.0, ready=False)), self.walker)
        self.emit.assert_not_called()

    def test_frame_unchanged_during_settle(self):
        self.policy.command(state(READY))
        self.assertEqual(self.policy.command(state(START - 0.01)), self.walker)
        self.emit.assert_not_called()

    def test_wave_starts_from_the_walkers_arm_targets(self):
        # Not the measured pose: the arm sags below its targets, and commanding the sag drops it.
        self.policy.command(state(READY))
        frame = self.policy.command(state(START, arm=[0.05] * 7))
        np.testing.assert_allclose(frame["q"][22:29], [0.1] * 7, atol=1e-9)
        np.testing.assert_allclose(frame["dq"][22:29], [0.0] * 7, atol=1e-9)
        self.emit.assert_called_with("wave_start", START, pose=[0.1] * 7)

    def test_only_the_right_arm_changes(self):
        self.policy.command(state(READY))
        self.policy.command(state(START))
        t = START + wave.RAISE_S + 0.3
        frame = self.policy.command(state(t))
        q, dq, _ = wave.Wave(START, [0.1] * 7).at(t)
        self.assertEqual(frame["q"][:22], [0.1] * 22)
        self.assertEqual(frame["dq"][:22], [0.0] * 22)
        np.testing.assert_allclose(frame["q"][22:29], q)
        np.testing.assert_allclose(frame["dq"][22:29], dq)

    def test_robot_clock_without_sim_time(self):
        self.policy.command(state(READY, sim=False))
        self.assertEqual(self.policy.command(state(START - 0.01, sim=False)), self.walker)
        self.policy.command(state(START, sim=False))
        self.emit.assert_called_with("wave_start", None, pose=[0.1] * 7)
        frame = self.policy.command(state(START + wave.RAISE_S + 0.3, sim=False))
        self.assertNotEqual(frame["q"][22:29], self.walker["q"][22:29])

    def test_frame_values_are_lists_of_29_floats(self):
        self.policy.command(state(READY))
        self.policy.command(state(START))
        frame = self.policy.command(state(START + wave.RAISE_S + 0.3))
        for key in ("q", "dq"):
            self.assertIs(type(frame[key]), list)
            self.assertEqual(len(frame[key]), 29)
            self.assertTrue(all(type(v) is float for v in frame[key]), key)

    def test_wave_runs_once(self):
        frames = []
        for t in np.arange(READY, START + wave.Wave(START, wave.NEUTRAL).duration + 5.0, 0.02):
            frames.append((len(self.emit.call_args_list), self.policy.command(state(float(t)))))
        names = [c.args[0] for c in self.emit.call_args_list]
        self.assertEqual(names.count("wave_start"), 1)
        self.assertEqual(names.count("wave_done"), 1)
        self.assertEqual([c.kwargs["phase"] for c in self.emit.call_args_list if c.args[0] == "wave_phase"],
                         ["wave", "lower"])
        done = names.index("wave_done")
        after = [frame for calls, frame in frames if calls > done]
        self.assertTrue(after)
        self.assertTrue(all(frame == self.walker for frame in after))


if __name__ == "__main__":
    unittest.main()
