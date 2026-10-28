"""Offline safety regressions; no DDS channels, policy models or robot are opened."""

from collections import deque
from contextlib import redirect_stderr, redirect_stdout
import io
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import numpy as np
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_

from harp_control import remote
from harp_control.gr00t import walk


class Clock:
    def __init__(self):
        self.elapsed = 0.0

    def monotonic(self):
        return self.elapsed

    def sleep(self, seconds):
        self.elapsed += seconds


def offline_walker(real=False):
    walker = walk.Walker.__new__(walk.Walker)
    walker.real = real
    walker.state_timeout, walker.state_time = 0.5, 0.0
    walker.state = SimpleNamespace(
        mode_machine=5, wireless_remote=[0] * 40,
        motor_state=[SimpleNamespace(q=0.0, dq=0.0) for _ in range(walk.NUM_MOTORS)],
        imu_state=SimpleNamespace(quaternion=[1.0, 0.0, 0.0, 0.0], gyroscope=[0.0] * 3),
    )
    walker.cmd = np.zeros(3, dtype=np.float32)
    walker.height = walk.DEFAULT_HEIGHT
    walker.quit = walker.standing = walker.policy_on = False
    walker.on_policy_output = None
    walker.low_cmd = unitree_hg_msg_dds__LowCmd_()
    walker.crc = SimpleNamespace(Crc=lambda command: 0)
    walker.cmd_pub = Mock()
    walker.hold = Mock()
    return walker


class WalkerSafetyTests(unittest.TestCase):
    def test_nonfinite_motor_targets_or_gains_never_publish(self):
        for argument in range(3):
            for value in (float("nan"), float("inf"), -float("inf")):
                with self.subTest(argument=argument, value=value):
                    walker = offline_walker()
                    values = [np.zeros(walk.NUM_MOTORS), walk.KP.copy(), walk.KD.copy()]
                    values[argument][4] = value
                    with self.assertRaises(ValueError):
                        walker.send(*values)
                    walker.cmd_pub.Write.assert_not_called()

    def test_finite_targets_still_clip_to_joint_limits(self):
        walker = offline_walker()
        targets = np.full(walk.NUM_MOTORS, 100.0)

        walker.send(targets, walk.KP, walk.KD)

        walker.cmd_pub.Write.assert_called_once()
        actual = [motor.q for motor in walker.low_cmd.motor_cmd[:walk.NUM_MOTORS]]
        np.testing.assert_allclose(actual, walk.Q_UPPER)

    def test_nonfinite_state_stops_before_a_normal_motor_command(self):
        for field in ("q", "dq", "quaternion", "gyroscope"):
            with self.subTest(field=field):
                walker = offline_walker()
                if field in ("q", "dq"):
                    setattr(walker.state.motor_state[0], field, float("nan"))
                else:
                    getattr(walker.state.imu_state, field)[0] = float("nan")
                walker.damp = Mock()
                # A pending quit keeps the uncorrected loop finite for this regression.
                with patch.object(walk, "time", Clock()), \
                        patch.object(walk.runlog, "event"), \
                        redirect_stdout(io.StringIO()), redirect_stderr(io.StringIO()):
                    reason = walker.run(lambda: "o")

                self.assertEqual(reason, "error")
                walker.damp.assert_called_once()
                walker.cmd_pub.Write.assert_not_called()

    def test_logging_failure_cannot_skip_error_damping(self):
        walker = offline_walker()
        walker.loop = Mock(side_effect=RuntimeError("test control failure"))
        walker.damp = Mock()
        with patch.object(walk.runlog, "event", side_effect=OSError("disk full")), \
                redirect_stdout(io.StringIO()), redirect_stderr(io.StringIO()):
            reason = walker.run(lambda: None)

        self.assertEqual(reason, "error")
        walker.damp.assert_called_once()

    def test_logging_failure_cannot_skip_damping_or_real_hold(self):
        walker = offline_walker(real=True)
        walker.send_damping = Mock()
        with patch.object(walk, "time", Clock()), \
                patch.object(walk.runlog, "event", side_effect=OSError("publisher unavailable")), \
                redirect_stdout(io.StringIO()), redirect_stderr(io.StringIO()):
            walker.stop("remote_estop", "test e-stop")

        self.assertGreaterEqual(walker.send_damping.call_count, 1)
        walker.hold.assert_called_once()
        self.assertFalse(walker.policy_on)

    def test_remote_estop_still_damps_before_processing_movement_keys(self):
        walker = offline_walker()
        walker.damp = Mock()
        keys = Mock(return_value="w")
        remote.encode(remote.ESTOP, walker.state.wireless_remote)
        with patch.object(walk, "time", Clock()), patch.object(walk.runlog, "event"), \
                redirect_stdout(io.StringIO()), redirect_stderr(io.StringIO()):
            reason = walker.run(keys)

        self.assertEqual(reason, "remote_estop")
        walker.damp.assert_called_once()
        keys.assert_not_called()
        walker.cmd_pub.Write.assert_not_called()

    def test_failed_telemetry_preserves_the_original_safety_stop_reason(self):
        for cause in ("remote_estop", "state_lost", "fall"):
            with self.subTest(cause=cause):
                walker = offline_walker()
                walker.damp = Mock()
                if cause == "remote_estop":
                    remote.encode(remote.ESTOP, walker.state.wireless_remote)
                elif cause == "state_lost":
                    walker.state_time = -1.0
                else:
                    walker.policy_on = True
                    walker.state.imu_state.quaternion = [0.0, 1.0, 0.0, 0.0]
                with patch.object(walk, "time", Clock()), \
                        patch.object(walk.runlog, "event", side_effect=OSError("disk full")), \
                        redirect_stdout(io.StringIO()), redirect_stderr(io.StringIO()):
                    reason = walker.run(lambda: None)

                self.assertEqual(reason, cause)
                walker.damp.assert_called_once()
                walker.cmd_pub.Write.assert_not_called()

    def test_closed_terminal_cannot_skip_damping_or_real_hold(self):
        walker = offline_walker(real=True)
        walker.send_damping = Mock()
        with patch.object(walk, "time", Clock()), patch.object(walk.runlog, "event"), \
                patch("builtins.print", side_effect=BrokenPipeError("terminal closed")):
            walker.stop("quit", "test stop")

        self.assertGreaterEqual(walker.send_damping.call_count, 1)
        walker.hold.assert_called_once()

    def test_closed_python_streams_allow_full_stop_damping_and_hold(self):
        walker = offline_walker(real=True)
        walker.send_damping = Mock()
        walker.read_key = Mock(return_value="o")
        walker.hold = walk.Walker.hold.__get__(walker, walk.Walker)
        closed = io.StringIO()
        closed.close()
        clock = Clock()
        with patch.object(walk, "time", clock), patch.object(walk.runlog, "event"), \
                patch.object(walk.sys, "stdout", closed), patch.object(walk.sys, "stderr", closed):
            walker.stop("error", "test stop with closed output streams")

        self.assertGreaterEqual(clock.elapsed, walk.STOP_TIME)
        self.assertGreaterEqual(walker.send_damping.call_count, 2)
        walker.read_key.assert_called_once()
        self.assertFalse(walker.policy_on)


class PolicyNumericTests(unittest.TestCase):
    def make_policy(self, output=None):
        policy = walk.Gr00tPolicy.__new__(walk.Gr00tPolicy)
        policy.action = np.zeros(walk.NUM_ACTIONS, dtype=np.float32)
        policy.history = deque([np.zeros(walk.OBS_DIM, dtype=np.float32)] * walk.HISTORY_LEN,
                               maxlen=walk.HISTORY_LEN)
        session = Mock()
        session.get_inputs.return_value = [SimpleNamespace(name="observation")]
        session.run.return_value = [np.array([np.zeros(walk.NUM_ACTIONS)
                                              if output is None else output])]
        policy.balance = policy.walk = session
        return policy

    def test_nonfinite_observation_never_reaches_onnx(self):
        for field in range(6):
            with self.subTest(field=field):
                policy = self.make_policy()
                values = [np.zeros(29), np.zeros(29), np.array([1.0, 0.0, 0.0, 0.0]),
                          np.zeros(3), np.zeros(3), 0.74]
                if field == 5:
                    values[field] = float("nan")
                else:
                    values[field][0] = float("nan")
                with self.assertRaises(ValueError):
                    policy.step(*values)
                policy.balance.run.assert_not_called()

    def test_nonfinite_policy_output_is_rejected(self):
        output = np.zeros(walk.NUM_ACTIONS)
        output[2] = float("nan")
        policy = self.make_policy(output)
        with self.assertRaises(ValueError):
            policy.step(np.zeros(29), np.zeros(29), np.array([1.0, 0.0, 0.0, 0.0]),
                        np.zeros(3), np.zeros(3), 0.74)

    def test_finite_policy_output_keeps_action_scaling(self):
        policy = self.make_policy(np.ones(walk.NUM_ACTIONS))
        targets = policy.step(np.zeros(29), np.zeros(29), np.array([1.0, 0.0, 0.0, 0.0]),
                              np.zeros(3), np.zeros(3), 0.74)
        np.testing.assert_allclose(targets, walk.STAND_POSE[:walk.NUM_ACTIONS] + walk.ACTION_SCALE)


if __name__ == "__main__":
    unittest.main()
