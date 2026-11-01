"""Command ownership and course option compatibility."""
import json
import unittest

from course import course_task
from harp.avoidance_config import select
from harp.route import route_from_data


class Selection(unittest.TestCase):
    def test_default_and_opt_in_select_one_filter(self):
        self.assertEqual(select(""), "approach_stop")
        self.assertEqual(select('{"avoidance":"predictive"}'), "predictive_avoidance")
        self.assertEqual(select('{"moves":[{"walk":1}]}'), "approach_stop")

    def test_unparsable_task_keeps_the_default_filter(self):
        # Other task readers tolerate bad JSON; startup must not stop on it.
        self.assertEqual(select("not json", "real", False), "approach_stop")

    def test_unsupported_combinations_fail_before_startup(self):
        task = '{"avoidance":"predictive"}'
        for text, mode, world in ((task, "real", True), (task, "sim", False),
                                   ('{"avoidance":"typo"}', "sim", True),
                                   ('{"avoidance":"predictive","to":[1,0]}', "sim", True)):
            with self.assertRaises(ValueError):
                select(text, mode, world)

    def test_route_and_predictive_option_survive_together(self):
        route = route_from_data({"frame":"odom", "waypoints":[[1,2]], "laps":2})
        task = json.loads(course_task('{"avoidance":"predictive"}', route))
        self.assertEqual(task['avoidance'], 'predictive')
        self.assertEqual(task['route']['laps'], 2)
        self.assertEqual(task['route']['waypoints'][0][:2], [1,2])
        self.assertEqual(json.loads(course_task(json.dumps(task), avoidance='reactive'))['route'],
                         task['route'])

    def test_explicit_route_replaces_inherited_planned_move(self):
        route = route_from_data({"frame":"odom", "waypoints":[[1,2]]})
        for mode in ('reactive', 'predictive'):
            task = json.loads(course_task('{"to":[1,0],"moves":[{"walk":1}]}', route, mode))
            self.assertNotIn('to', task)
            self.assertNotIn('moves', task)
            self.assertIn('route', task)
            self.assertEqual(task['avoidance'], mode)


if __name__ == '__main__':
    unittest.main()
