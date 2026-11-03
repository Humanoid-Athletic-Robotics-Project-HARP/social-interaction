"""Course verdict checks, without a simulator or recorded data."""
import unittest
from unittest.mock import Mock, patch
import course
from harp import course_goal
from course import check
from harp.route import route_from_data


class CourseChecks(unittest.TestCase):
    def test_nonfinite_measurements_cannot_pass(self):
        summary = dict(outcome="succeeded", seconds=1, closest_m=1, contacts=0,
                       speed_near_people_mps=0, recoveries=0, end_error_m=0)
        for field, criterion in (("seconds", "max_time_s"), ("closest_m", "min_clearance_m"),
                                 ("contacts", "max_contacts"), ("end_error_m", "max_end_error_m")):
            for value in (float("nan"), float("inf"), -float("inf")):
                with self.subTest(field=field, value=value):
                    bad = dict(summary, **{field: value})
                    self.assertEqual(check(bad, {criterion: 1}), (False, [criterion]))

    def test_first_talk_person_and_time(self):
        talk = dict(distance_m=1.1, facing_deg=5.0, ended="time", seconds=30.0)
        summary = dict(outcome="succeeded", seconds=12.0, closest_m=1, contacts=0,
                       speed_near_people_mps=0, recoveries=None, end_error_m=None,
                       loop={"talks": [dict(talk, person="ahead"), dict(talk, person="wedge")]})
        self.assertEqual(check(summary, {"first_talk": "ahead", "max_first_talk_s": 20}), (True, []))
        self.assertEqual(check(summary, {"first_talk": "wedge", "max_first_talk_s": 10}),
                         (False, ["max_first_talk_s", "first_talk"]))
        none = dict(summary, seconds=None, loop={"talks": []})
        self.assertEqual(check(none, {"first_talk": "ahead", "max_first_talk_s": 20}),
                         (False, ["max_first_talk_s", "first_talk"]))

    def test_not_people_found_and_not_talked_to(self):
        talk = dict(distance_m=1.1, facing_deg=5.0, ended="time", seconds=30.0, person="person")
        summary = dict(outcome="succeeded", seconds=12.0, closest_m=1, contacts=0,
                       speed_near_people_mps=0, recoveries=None, end_error_m=None,
                       loop={"talks": [talk], "not_people": ["pillar"]})
        self.assertEqual(check(summary, {"not_people": ["pillar"]}), (True, []))
        missed = dict(summary, loop={"talks": [talk], "not_people": []})
        self.assertEqual(check(missed, {"not_people": ["pillar"]}), (False, ["not_people"]))
        talked = dict(summary, loop={"talks": [talk, dict(talk, person="pillar")],
                                     "not_people": ["pillar"]})
        self.assertEqual(check(talked, {"not_people": ["pillar"]}), (False, ["not_people"]))

    def test_route_accepts_the_speed_criterion_used_by_course_scoring(self):
        route = route_from_data({"frame": "odom", "waypoints": [[1, 0]],
                                 "criteria": {"max_speed_near_people_mps": 0.1}})
        self.assertEqual(route.criteria, {"max_speed_near_people_mps": 0.1})


class GoalStartup(unittest.TestCase):
    def test_missing_control_handoff_is_bounded_and_reports_failure(self):
        node, publisher = Mock(), Mock()
        node.create_publisher.return_value = publisher
        with patch.object(course_goal, "route_from_env", return_value=None), \
             patch.object(course_goal, "goal_site", return_value=(1.0, 0.0)), \
             patch.object(course_goal, "wait_for_active", return_value=True), \
             patch.object(course_goal, "spin_until", return_value=False) as spin:
            self.assertEqual(course_goal.run(node), 1)
        self.assertEqual(spin.call_args.args[2], course_goal.NAV2_START)
        self.assertIn("CONTROL_NOT_READY", publisher.publish.call_args.args[0].data)


@unittest.skipUnless(__import__("os").environ.get("ROS_LOCALHOST_ONLY") == "1"
                     and __import__("os").environ.get("ROS_DOMAIN_ID", "0") != "0",
                     "requires an isolated local ROS test domain")
class LifecycleWait(unittest.TestCase):
    """wait_for_active against a fake lifecycle node: Foxy's Fast DDS refuses a reply to a client
    whose reply channel the server hasn't discovered, and Nav2's bt_navigator died of it at start-up
    (HIM, 2026-11-02/03), so a fresh client must not call get_state at once."""
    def setUp(self):
        import rclpy
        from lifecycle_msgs.msg import State, TransitionEvent
        from lifecycle_msgs.srv import GetState
        self.rclpy, self.TransitionEvent = rclpy, TransitionEvent
        rclpy.init()
        self.server = rclpy.create_node("fake_bt")
        self.calls, self.label = [], "unconfigured"
        def get_state(request, response):
            self.calls.append(__import__("time").monotonic())
            response.current_state = State(label=self.label)
            return response
        self.server.create_service(GetState, "fake_bt/get_state", get_state)
        self.events = self.server.create_publisher(TransitionEvent, "fake_bt/transition_event", 10)
        self.client = rclpy.create_node("waiter")
        self.addCleanup(rclpy.shutdown)

    def wait(self, limit, labels, during=lambda t: None):
        import threading, time
        stop = threading.Event()
        from rclpy.executors import SingleThreadedExecutor
        executor = SingleThreadedExecutor()   # the server's own: wait_for_active spins the global one
        executor.add_node(self.server)
        def serve():
            start = time.monotonic()
            while not stop.is_set():
                executor.spin_once(timeout_sec=.05)
                during(time.monotonic() - start)
        thread = threading.Thread(target=serve); thread.start()
        try:
            return course_goal.wait_for_active(self.client, "fake_bt", limit, labels)
        finally:
            stop.set(); thread.join()

    def test_a_transition_event_answers_without_calling_the_service(self):
        from lifecycle_msgs.msg import State
        def configure(t):
            if 1.0 < t and not getattr(self, "sent", False):
                self.sent = True
                self.events.publish(self.TransitionEvent(goal_state=State(label="inactive")))
        self.assertTrue(self.wait(5., ("inactive", "active"), configure))
        self.assertEqual(self.calls, [])

    def test_without_events_it_asks_only_once_the_service_has_settled(self):
        import time
        self.label = "active"
        start = time.monotonic()
        self.assertTrue(self.wait(10., ("active",)))
        self.assertEqual(len(self.calls), 1)
        self.assertGreaterEqual(self.calls[0] - start, course_goal.SERVICE_SETTLE - .2)
