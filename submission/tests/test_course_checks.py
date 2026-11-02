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
