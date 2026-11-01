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


def _feedback(x, y):
    from types import SimpleNamespace
    pose = SimpleNamespace(position=SimpleNamespace(x=x, y=y))
    return SimpleNamespace(current_pose=SimpleNamespace(pose=pose), number_of_recoveries=0,
                           distance_remaining=0.)


class StaleSuccess(unittest.TestCase):
    def test_a_success_far_from_the_waypoint_is_resent_not_accepted(self):
        # Foxy Nav2 once followed the previous goal's path and reported the next
        # waypoint reached at once, 2 m away (HIM run 2026-11-01_23-23-20Z).
        route = route_from_data({"frame": "odom", "waypoints": [[0, 2], [0, 0]], "laps": 1})
        replies = [("SUCCEEDED", _feedback(0., 2.)),      # waypoint 1, genuinely there
                   ("SUCCEEDED", _feedback(0., 2.)),      # waypoint 2: stale, still at 1
                   ("SUCCEEDED", _feedback(.05, .1))]     # waypoint 2 after resending
        events = []
        with patch.object(course_goal, "go_to", side_effect=replies) as go:
            status = course_goal.follow(None, None, route,
                                        lambda name, **f: events.append((name, f)), True)
        self.assertEqual(status, "SUCCEEDED")
        self.assertEqual(go.call_count, 3)
        stale = [f for name, f in events if name == "stale_success"]
        self.assertEqual(len(stale), 1)
        self.assertAlmostEqual(stale[0]["distance"], 2.)
        result = [f for name, f in events if name == "goal_result"][0]
        self.assertEqual(result["completed_waypoints"], 2)

    def test_a_success_without_a_reported_pose_is_trusted(self):
        route = route_from_data({"frame": "odom", "waypoints": [[1, 0]], "laps": 1})
        with patch.object(course_goal, "go_to", return_value=("SUCCEEDED", None)) as go:
            status = course_goal.follow(None, None, route, lambda name, **f: None, True)
        self.assertEqual((status, go.call_count), ("SUCCEEDED", 1))
