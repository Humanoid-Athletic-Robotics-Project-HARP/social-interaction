"""Offline navigation checks: route order, failures, and confirmed cancellation."""

from contextlib import redirect_stdout
import io
import json
import math
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from builtin_interfaces.msg import Time
from nav2_simple_commander.robot_navigator import TaskResult

from harp_eval import course_goal
from harp_eval.waypoint_route import WaypointRoute


class FakeClock:
    """Advance during sleeps and bounded future waits; no wall-clock waits."""

    def __init__(self):
        self.elapsed = 0.0

    def monotonic(self):
        return self.elapsed

    def sleep(self, seconds):
        self.elapsed += seconds

    def spin_until_future_complete(self, node, future, timeout_sec):
        future.wait(timeout_sec)


class FakeFuture:
    """Complete after a scripted delay; None leaves the response unanswered."""

    def __init__(self, clock, value, delay=0.0):
        self.clock, self.value, self.delay = clock, value, delay
        self.ready = delay == 0.0
        self.waits = []

    def done(self):
        return self.ready

    def result(self):
        return self.value if self.ready else None

    def wait(self, timeout):
        if timeout is None or not math.isfinite(timeout) or timeout < 0:
            raise AssertionError("navigation response waits must have a finite deadline")
        self.waits.append(timeout)
        if self.ready:
            return
        if self.delay is None or self.delay > timeout:
            self.clock.sleep(timeout)
            if self.delay is not None:
                self.delay -= timeout
        else:
            self.clock.sleep(self.delay)
            self.ready = True


class FakeGoalHandle:
    def __init__(self, nav):
        self.nav = nav
        self.accepted = nav.accepted
        self.result_future = FakeFuture(nav.clock, None, delay=None)

    def get_result_async(self):
        return self.result_future

    def cancel_goal_async(self):
        nav = self.nav
        nav.cancel_calls += 1
        nav.cancel_requested = True
        nav.calls.append(("cancel", len(nav.goals)))
        future = FakeFuture(nav.clock, SimpleNamespace(goals_canceling=[self]),
                            delay=nav.scenario.get("cancel_after", 0.0))
        nav.cancel_responses.append(future)
        return future


class FakeActionClient:
    def __init__(self, nav):
        self.nav = nav

    def wait_for_server(self, timeout_sec):
        nav = self.nav
        nav.server_waits.append(timeout_sec)
        if len(nav.goals) >= len(nav.scenarios):
            raise AssertionError("an unexpected later waypoint was requested")
        if nav.scenarios[len(nav.goals)].get("server_available", True):
            return True
        nav.clock.sleep(timeout_sec)
        return False

    def send_goal_async(self, request, feedback_callback):
        nav = self.nav
        if nav.feedback is not None or nav.result_future is not None or nav.goal_handle is not None:
            raise AssertionError("the previous waypoint's client state was not cleared")
        nav.scenario = nav.scenarios[len(nav.goals)]
        nav.goals.append(request.pose)
        nav.calls.append(("send", len(nav.goals)))
        nav.accepted = nav.scenario.get("accepted", True)
        nav.feedback_callback = feedback_callback
        nav.complete = False
        nav.polls = nav.cancel_checks = 0
        nav.cancel_requested = nav.cancel_confirmed = False
        nav.interrupted = False
        future = FakeFuture(nav.clock, FakeGoalHandle(nav),
                            delay=nav.scenario.get("accept_after", 0.0))
        nav.acceptance_responses.append(future)
        return future


class FakeNavigator:
    """A scripted action client that retains stale results after rejection.

    Requests expose actual async clients, handles, and futures. A scenario
    controls server availability, response delays, terminal result/polls,
    cancellation polls, interruption, and recovery feedback. The previous
    result remains after rejection; reading it is a test failure.
    """

    def __init__(self, scenarios, clock, control=None):
        self.scenarios = scenarios
        self.clock = clock
        self.goals = []
        self.calls = []
        self.feedback = object()  # the adapter must clear feedback before each send
        self.goal_handle = object()
        self.result_future = object()
        self.result = TaskResult.SUCCEEDED
        self.scenario = None
        self.accepted = False
        self.complete = False
        self.polls = self.cancel_checks = self.cancel_calls = 0
        self.cancel_requested = self.cancel_confirmed = False
        self.interrupted = False
        self.result_reads = []
        self.server_waits = []
        self.acceptance_responses = []
        self.cancel_responses = []
        self.nav_to_pose_client = FakeActionClient(self)
        self.control = {} if control is None else control

    def get_clock(self):
        return SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: Time()))

    def isTaskComplete(self):
        if not self.accepted:
            raise AssertionError("the rejected request inspected the previous task")
        if self.scenario.get("interrupt") and not self.interrupted and not self.cancel_requested:
            self.interrupted = True
            raise KeyboardInterrupt
        if self.cancel_requested:
            self.cancel_checks += 1
            needed = self.scenario.get("cancel_polls", 2)
            if needed is None or self.cancel_checks < needed:
                return False
            self.cancel_confirmed = self.complete = True
            self.result = TaskResult.CANCELED
            self.calls.append(("cancel_complete", len(self.goals)))
            return True
        self.polls += 1
        needed = self.scenario.get("running_polls", 1)
        if needed is None or self.polls <= needed:
            self.feedback_callback(SimpleNamespace(feedback=SimpleNamespace(
                number_of_recoveries=self.scenario.get("recoveries", 0),
                distance_remaining=0.75,
                current_pose=SimpleNamespace(pose=SimpleNamespace(
                    position=SimpleNamespace(x=0.0, y=0.0))),
            )))
            if self.polls == self.scenario.get("manual_at_poll"):
                self.control["manual"] = True
            return False
        self.complete = True
        self.result = self.scenario.get("result", TaskResult.SUCCEEDED)
        return True

    def getFeedback(self):
        return self.feedback

    def getResult(self):
        if not self.accepted or not self.complete:
            raise AssertionError("a result was read without a completed accepted request")
        self.result_reads.append(len(self.goals))
        return self.result

class FollowRouteTests(unittest.TestCase):
    POINTS = ((1.0, 0.0, 0.0), (1.0, 1.0, 90.0), (0.0, 0.0, -90.0))

    def follow(self, scenarios, *, laps=1, points=None, timeout=2.0, waypoint_events=True,
               during_pause=None):
        route = WaypointRoute(self.POINTS if points is None else points, laps=laps)
        clock = FakeClock()
        control = {"manual": False, "stopped": False}
        nav = FakeNavigator(scenarios, clock, control)

        def spin_once(node, timeout_sec):
            clock.sleep(timeout_sec)
            if during_pause is None:
                raise AssertionError("manual pause needs a scripted operator response")
            during_pause(nav, route, clock, control)

        with patch.object(course_goal, "time", clock), \
                patch.object(course_goal.rclpy, "spin_until_future_complete",
                             side_effect=clock.spin_until_future_complete), \
                patch.object(course_goal.rclpy, "spin_once", side_effect=spin_once), \
                patch.object(course_goal.runlog, "event") as record, \
                redirect_stdout(io.StringIO()):
            result = course_goal.follow_route(nav, route, timeout,
                                              waypoint_events=waypoint_events, control=control)
        events = [(call.args[1], call.kwargs) for call in record.call_args_list]
        self.assertTrue(all(call.args[0] == "goal" for call in record.call_args_list))
        self.assertEqual(sum(name == "goal_sent" for name, _ in events), 1)
        self.assertEqual(sum(name == "goal_result" for name, _ in events), 1)
        self.assertEqual(events[-1], ("goal_result", result))
        return result, nav, route, events

    def assert_sent_poses(self, nav, expected):
        self.assertEqual(len(nav.goals), len(expected))
        for pose, (x, y, yaw) in zip(nav.goals, expected):
            self.assertEqual(pose.header.frame_id, "odom")
            self.assertEqual((pose.pose.position.x, pose.pose.position.y), (x, y))
            self.assertAlmostEqual(pose.pose.orientation.z, math.sin(math.radians(yaw) / 2))
            self.assertAlmostEqual(pose.pose.orientation.w, math.cos(math.radians(yaw) / 2))

    def test_two_laps_send_ordered_poses_and_one_terminal_result(self):
        scenarios = [{"recoveries": n} for n in (1, 0, 2, 0, 0, 0)]
        result, nav, route, events = self.follow(scenarios, laps=2)

        self.assert_sent_poses(nav, self.POINTS * 2)
        self.assertEqual(result["status"], "SUCCEEDED")
        self.assertEqual(result["recoveries"], 3)
        self.assertEqual(result["completed_waypoints"], 6)
        self.assertEqual(result["completed_laps"], 2)
        self.assertIsNone(route.current())
        self.assertEqual(nav.cancel_calls, 0)
        expected = [(lap, point) for lap in (1, 2) for point in (1, 2, 3)]
        for name in ("waypoint_sent", "waypoint_result"):
            self.assertEqual([(d["lap"], d["waypoint"]) for n, d in events if n == name],
                             expected)
        self.assertTrue(all(d["status"] == "SUCCEEDED"
                            for name, d in events if name == "waypoint_result"))
        self.assertEqual(events[0][1]["laps"], 2)
        self.assertEqual((events[0][1]["x"], events[0][1]["y"], events[0][1]["yaw"]),
                         self.POINTS[-1])

    def test_rejection_after_success_does_not_read_or_advance_stale_result(self):
        result, nav, route, events = self.follow([{}, {"accepted": False}])

        self.assert_sent_poses(nav, self.POINTS[:2])
        self.assertEqual(nav.result_reads, [1])
        self.assertEqual(result["status"], "REJECTED")
        self.assertEqual(result["completed_waypoints"], 1)
        self.assertEqual(result["completed_laps"], 0)
        self.assertEqual(route.current(), self.POINTS[1])
        self.assertIsNone(result["remaining"])
        self.assertEqual(nav.cancel_calls, 0)
        self.assertEqual([d["status"] for name, d in events if name == "waypoint_result"],
                         ["SUCCEEDED", "REJECTED"])

    def test_aborted_goal_stops_route_at_failed_waypoint(self):
        result, nav, route, _ = self.follow([{}, {"result": TaskResult.FAILED}])

        self.assert_sent_poses(nav, self.POINTS[:2])
        self.assertEqual(result["status"], "FAILED")
        self.assertEqual(result["completed_waypoints"], 1)
        self.assertEqual(route.current(), self.POINTS[1])
        self.assertEqual(nav.result_reads, [1, 2])
        self.assertEqual(nav.cancel_calls, 0)

    def test_per_goal_timeout_waits_for_cancellation_without_advancing(self):
        result, nav, route, events = self.follow(
            [{}, {"running_polls": None, "cancel_polls": 4}], timeout=0.25)

        self.assert_sent_poses(nav, self.POINTS[:2])
        self.assertEqual(result["status"], "TIMEOUT")
        self.assertEqual(result["completed_waypoints"], 1)
        self.assertEqual(route.current(), self.POINTS[1])
        self.assertEqual(nav.cancel_calls, 1)
        self.assertEqual(nav.cancel_checks, 4)
        self.assertTrue(nav.cancel_confirmed)
        self.assertEqual(nav.calls[-1], ("cancel_complete", 2))
        self.assertEqual(nav.result_reads, [1])
        self.assertEqual([d["status"] for name, d in events if name == "waypoint_result"],
                         ["SUCCEEDED", "TIMEOUT"])

    def test_unconfirmed_cancellation_ends_without_sending_later_goal(self):
        result, nav, route, _ = self.follow(
            [{"running_polls": None, "cancel_polls": None}], timeout=0.1)

        self.assert_sent_poses(nav, self.POINTS[:1])
        self.assertEqual(result["status"], "CANCEL_TIMEOUT")
        self.assertEqual(result["completed_waypoints"], 0)
        self.assertEqual(route.current(), self.POINTS[0])
        self.assertEqual(nav.cancel_calls, 1)
        self.assertGreater(nav.cancel_checks, 1)
        self.assertFalse(nav.cancel_confirmed)
        self.assertEqual(nav.result_reads, [])

    def test_keyboard_interrupt_cancels_active_goal_and_stops_route(self):
        result, nav, route, _ = self.follow([{"interrupt": True, "cancel_polls": 3}])

        self.assert_sent_poses(nav, self.POINTS[:1])
        self.assertEqual(result["status"], "CANCELED")
        self.assertEqual(result["completed_waypoints"], 0)
        self.assertEqual(route.current(), self.POINTS[0])
        self.assertEqual(nav.cancel_calls, 1)
        self.assertEqual(nav.cancel_checks, 3)
        self.assertTrue(nav.cancel_confirmed)
        self.assertEqual(nav.result_reads, [])

    def test_unanswered_acceptance_expires_without_reading_previous_result(self):
        result, nav, route, _ = self.follow([{}, {"accept_after": None}], timeout=0.25)

        self.assert_sent_poses(nav, self.POINTS[:2])
        self.assertEqual(result["status"], "TIMEOUT")
        self.assertEqual(result["completed_waypoints"], 1)
        self.assertEqual(route.current(), self.POINTS[1])
        self.assertEqual(nav.result_reads, [1])
        self.assertEqual(nav.cancel_calls, 0)
        unanswered = nav.acceptance_responses[-1]
        self.assertFalse(unanswered.done())
        self.assertEqual(len(unanswered.waits), 1)
        self.assertAlmostEqual(unanswered.waits[0], 0.25)
        self.assertGreaterEqual(nav.clock.elapsed, 0.25)
        self.assertLess(nav.clock.elapsed, 0.5)

    def test_unavailable_server_expires_before_sending_next_waypoint(self):
        result, nav, route, _ = self.follow([{}, {"server_available": False}], timeout=0.25)

        self.assert_sent_poses(nav, self.POINTS[:1])
        self.assertEqual(result["status"], "TIMEOUT")
        self.assertEqual(result["completed_waypoints"], 1)
        self.assertEqual(route.current(), self.POINTS[1])
        self.assertEqual(nav.result_reads, [1])
        self.assertEqual(nav.cancel_calls, 0)
        self.assertIsNone(nav.goal_handle)
        self.assertIsNone(nav.result_future)
        self.assertTrue(all(0 <= wait <= 0.2 for wait in nav.server_waits))
        self.assertGreaterEqual(nav.clock.elapsed, 0.25)
        self.assertLess(nav.clock.elapsed, 0.5)

    def test_unanswered_cancel_response_expires_before_terminal_polling(self):
        result, nav, route, _ = self.follow(
            [{"running_polls": None, "cancel_after": None}], timeout=0.1)

        self.assert_sent_poses(nav, self.POINTS[:1])
        self.assertEqual(result["status"], "CANCEL_TIMEOUT")
        self.assertEqual(result["completed_waypoints"], 0)
        self.assertEqual(route.current(), self.POINTS[0])
        self.assertEqual(nav.cancel_calls, 1)
        self.assertEqual(nav.cancel_checks, 0)
        self.assertFalse(nav.cancel_confirmed)
        self.assertEqual(nav.result_reads, [])
        response = nav.cancel_responses[0]
        self.assertFalse(response.done())
        self.assertEqual(response.waits, [course_goal.CANCEL_TIMEOUT])
        self.assertGreaterEqual(nav.clock.elapsed, course_goal.CANCEL_TIMEOUT + 0.1)
        self.assertLess(nav.clock.elapsed, course_goal.CANCEL_TIMEOUT + 0.3)

    def test_legacy_single_goal_keeps_event_fields(self):
        result, nav, route, events = self.follow(
            [{}], points=self.POINTS[:1], waypoint_events=False)

        self.assert_sent_poses(nav, self.POINTS[:1])
        self.assertEqual([name for name, _ in events], ["goal_sent", "goal_result"])
        self.assertEqual(events[0][1], {"x": 1.0, "y": 0.0, "yaw": 0.0})
        self.assertEqual(set(result), {"status", "recoveries", "seconds", "remaining"})
        self.assertEqual(result["status"], "SUCCEEDED")
        self.assertIsNone(route.current())

    def test_long_manual_pause_resumes_same_waypoint_and_finishes_two_laps(self):
        paused = []

        def operator(nav, route, clock, control):
            self.assertEqual(route.current(), self.POINTS[0])
            self.assertEqual(route.completed_laps, 0)
            self.assertEqual(len(nav.goals), 1)
            self.assertTrue(nav.cancel_confirmed)
            paused.append(clock.elapsed)
            if len(paused) == 30:
                control["manual"] = False

        result, nav, route, events = self.follow(
            [{"manual_at_poll": 1, "running_polls": None, "recoveries": 2},
             *[{} for _ in range(6)]], laps=2, timeout=0.5, during_pause=operator)

        self.assert_sent_poses(nav, (self.POINTS[0], *self.POINTS * 2))
        self.assertGreater(paused[-1] - paused[0], 0.5)
        self.assertEqual(result["status"], "SUCCEEDED")
        self.assertEqual(result["completed_waypoints"], 6)
        self.assertEqual(result["completed_laps"], 2)
        self.assertEqual(result["recoveries"], 2)
        self.assertEqual(nav.cancel_calls, 1)
        self.assertNotIn(1, nav.result_reads)
        self.assertIsNone(route.current())
        self.assertEqual(sum(name == "waypoint_paused" for name, _ in events), 1)
        self.assertEqual(sum(name == "waypoint_resumed" for name, _ in events), 1)
        self.assertEqual(sum(name == "waypoint_result" for name, _ in events), 6)
        self.assertEqual(sum(name == "waypoint_sent" for name, _ in events), 6)
        self.assertGreater(result["paused_seconds"], 0.5)

    def test_resume_preserves_remaining_execution_budget(self):
        spins = []

        def operator(nav, route, clock, control):
            spins.append(clock.elapsed)
            if len(spins) == 20:
                control["manual"] = False

        result, nav, route, _ = self.follow(
            [{"manual_at_poll": 4, "running_polls": None}, {"running_polls": None}],
            timeout=0.5, during_pause=operator)

        self.assert_sent_poses(nav, (self.POINTS[0], self.POINTS[0]))
        self.assertEqual(result["status"], "TIMEOUT")
        self.assertEqual(result["completed_waypoints"], 0)
        self.assertEqual(nav.cancel_calls, 2)
        self.assertLess(result["seconds"] - result["paused_seconds"], 0.75)
        self.assertEqual(route.current(), self.POINTS[0])

    def test_failed_manual_cancellation_ends_without_resuming_or_advancing(self):
        result, nav, route, events = self.follow(
            [{"manual_at_poll": 1, "running_polls": None, "cancel_after": None}], timeout=0.5)

        self.assert_sent_poses(nav, self.POINTS[:1])
        self.assertEqual(result["status"], "CANCEL_TIMEOUT")
        self.assertEqual(result["completed_waypoints"], 0)
        self.assertEqual(nav.cancel_calls, 1)
        self.assertEqual(route.current(), self.POINTS[0])
        self.assertFalse(any(name == "waypoint_resumed" for name, _ in events))

    def test_estop_or_interrupt_during_manual_pause_ends_without_resending(self):
        for cause in ("estop", "interrupt"):
            with self.subTest(cause=cause):
                def operator(nav, route, clock, control):
                    if cause == "interrupt":
                        raise KeyboardInterrupt
                    course_goal.observe_event(
                        SimpleNamespace(data='{"source":"walker","event":"estop"}'),
                        [], [], [0.0], control)

                result, nav, route, events = self.follow(
                    [{"manual_at_poll": 1, "running_polls": None}], during_pause=operator)

                self.assert_sent_poses(nav, self.POINTS[:1])
                self.assertEqual(result["status"], "CANCELED")
                self.assertEqual(result["completed_waypoints"], 0)
                self.assertEqual(nav.cancel_calls, 1)
                self.assertEqual(route.current(), self.POINTS[0])
                self.assertFalse(any(name == "waypoint_resumed" for name, _ in events))


class SharedEventTests(unittest.TestCase):
    def test_malformed_shared_events_do_not_change_session_state(self):
        control = {"manual": False, "stopped": False}
        policy, released, lags = [], [], [0.0]
        for text in ("bad JSON", "[]", "null", "42", "{}", '{"source":"walker"}',
                     '{"source":"sim","event":"band_released","t":null}',
                     '{"source":"sim","event":"lag","t":"bad"}',
                     '{"source":"sim","event":"lag","t":NaN}'):
            with self.subTest(text=text):
                course_goal.observe_event(SimpleNamespace(data=text), policy, released, lags, control)

        self.assertEqual(control, {"manual": False, "stopped": False})
        self.assertEqual((policy, released, lags), ([], [], [0.0]))

    def test_operator_and_valid_sim_events_update_the_existing_subscription_state(self):
        control = {"manual": False, "stopped": False}
        policy, released, lags = [], [], [0.0]
        for source, name, stamp in (("walker", "policy_on", 1), ("sim", "band_released", 2),
                                    ("sim", "lag", 3), ("walker", "manual", 4)):
            course_goal.observe_event(SimpleNamespace(data=json.dumps(
                {"source": source, "event": name, "t": stamp})), policy, released, lags, control)

        self.assertTrue(control["manual"])
        self.assertEqual((len(policy), len(released), lags), (1, 1, [3]))
        course_goal.observe_event(SimpleNamespace(data='{"source":"walker","event":"hand_back"}'),
                                  policy, released, lags, control)
        self.assertFalse(control["manual"])


if __name__ == "__main__":
    unittest.main()
