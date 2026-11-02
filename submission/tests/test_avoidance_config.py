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

    def test_an_aware_crowd_is_for_simulator_worlds_only(self):
        from harp.movers import crowd_mode
        self.assertEqual(crowd_mode(""), "scripted")
        self.assertEqual(crowd_mode('{"crowd":"aware"}'), "aware")
        self.assertEqual(select('{"crowd":"aware","avoidance":"predictive"}'), "predictive_avoidance")
        for text, mode, world in (('{"crowd":"typo"}', "sim", True),
                                  ('{"crowd":"aware"}', "real", True),
                                  ('{"crowd":"aware"}', "sim", False)):
            with self.assertRaises(ValueError):
                select(text, mode, world)

    def test_course_adds_the_crowd_and_keeps_the_other_options(self):
        route = route_from_data({"frame":"odom", "waypoints":[[1,2]]})
        task = json.loads(course_task('{"avoidance":"predictive"}', route, crowd="aware"))
        self.assertEqual((task["crowd"], task["avoidance"]), ("aware", "predictive"))
        self.assertEqual(task["route"]["waypoints"][0][:2], [1, 2])
        self.assertEqual(json.loads(course_task(json.dumps(task), crowd="scripted"))["crowd"],
                         "scripted")
        with self.assertRaises(ValueError):
            course_task("", crowd="typo")

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



class Nav2Overrides(unittest.TestCase):
    def launch(self):
        import importlib.util
        from pathlib import Path
        try:
            spec = importlib.util.spec_from_file_location(
                "nav2_launch", Path(__file__).resolve().parents[1] / "nav2/nav2.launch.py")
            launch = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(launch)
        except ImportError:
            self.skipTest("requires ROS 2 launch")
        return launch

    def test_only_predictive_avoidance_changes_nav2(self):
        import yaml
        from harp.predictive_avoidance import STATIC_TOPIC
        launch = self.launch()
        for other in ("approach_stop", None):
            self.assertEqual(launch.parameters_for(other), launch.PARAMS)
        with open(launch.parameters_for("predictive_avoidance")) as source:
            params = yaml.safe_load(source)
        self.assertTrue(params["controller_server"]["ros__parameters"]["FollowPath"]
                        ["follow_when_blocked"])
        self.assertEqual(launch.STATIC_TOPIC, STATIC_TOPIC)
        for costmap in ("local_costmap", "global_costmap"):
            layer = params[costmap][costmap]["ros__parameters"]["stvl_layer"]
            # Marks from the cloud without moving objects; clearing from the raw lidar.
            self.assertEqual(layer["lidar_marking"]["topic"], STATIC_TOPIC)
            self.assertEqual(layer["lidar_clearing"]["topic"], "/utlidar/cloud_livox_mid360")
        # Never set in the shared file: on Foxy its controller_server entry beats a
        # launch override (a /** entry), as follow_when_blocked: false there did in HIM.
        with open(launch.PARAMS) as source:
            shared = yaml.safe_load(source)
        self.assertNotIn("follow_when_blocked",
                         shared["controller_server"]["ros__parameters"]["FollowPath"])

if __name__ == '__main__':
    unittest.main()
