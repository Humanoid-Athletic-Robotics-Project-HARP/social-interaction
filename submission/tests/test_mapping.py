"""The map a mapping walk builds (harp/occupancy.py) and localization on it (harp/localizer.py)."""
import json
import math
import unittest

import numpy as np

from harp import localizer, occupancy
from harp.localizer import Localizer, accept, compose


def wall_points(segments, heights=(.3, .8, 1.4), step=.03):
    """Returns along each wall segment ((x0, y0), (x1, y1)) at these heights."""
    out = []
    for (x0, y0), (x1, y1) in segments:
        n = max(2, int(math.hypot(x1 - x0, y1 - y0) / step))
        for f in np.linspace(0, 1, n):
            for z in heights:
                out.append((x0 + f * (x1 - x0), y0 + f * (y1 - y0), z))
    return np.array(out)


def floor_points(x0, x1, y0, y1, step=.1):
    xs, ys = np.meshgrid(np.arange(x0, x1, step), np.arange(y0, y1, step))
    return np.stack([xs.ravel(), ys.ravel(), np.zeros(xs.size)], axis=1)


ROOM = [((0, 0), (6, 0)), ((6, 0), (6, 4)), ((6, 4), (0, 4)), ((0, 4), (0, 0))]
PILLAR = [((3, 2), (3.2, 2)), ((3.2, 2), (3.2, 2.2)), ((3.2, 2.2), (3, 2.2)), ((3, 2.2), (3, 2))]
SENSOR = np.array([1.0, 1.0, 1.2])


def room_map(scans=5):
    m = occupancy.OccupancyMap(size=16)
    points = np.vstack([wall_points(ROOM + PILLAR), floor_points(.2, 5.8, .2, 3.8)])
    for _ in range(scans):
        m.integrate(points, SENSOR)
    return m


def cell(m, x, y):
    rows, cols, inside = m.index(np.array([[x, y]]))
    assert inside[0]
    return m.grid()[rows[0], cols[0]]


class TestOccupancyMap(unittest.TestCase):
    def test_walls_are_occupied_the_floor_free_and_beyond_unknown(self):
        m = room_map()
        self.assertEqual(cell(m, 3.0, 0.0), 100)
        self.assertEqual(cell(m, 3.1, 2.0), 100)
        self.assertEqual(cell(m, 4.5, 3.0), 0)
        self.assertEqual(cell(m, 4.0, -2.0), -1)

    def test_a_low_box_under_high_rays_stays(self):
        m = occupancy.OccupancyMap(size=16)
        box = wall_points([((3, 1), (3, 1.4))], heights=(.15, .25))
        wall = wall_points([((5, -1), (5, 3))], heights=(1.0,))
        for _ in range(20):
            m.integrate(np.vstack([box, wall]), SENSOR)
        self.assertEqual(cell(m, 3.0, 1.2), 100)

    def test_an_object_gone_is_cleared_by_the_floor_seen_through_its_place(self):
        m = occupancy.OccupancyMap(size=16)
        box = wall_points([((3, 1), (3, 1.4))], heights=(.5,))
        for _ in range(3):
            m.integrate(box, SENSOR)
        self.assertEqual(cell(m, 3.0, 1.2), 100)
        floor = floor_points(2.5, 4.0, .8, 1.6, step=.05)
        for _ in range(10):
            m.integrate(floor, SENSOR)
        self.assertEqual(cell(m, 3.0, 1.2), 0)

    def test_saved_maps_round_trip_and_bad_ones_are_refused(self):
        m = room_map()
        saved = occupancy.encode(m.grid(), m.origin, m.resolution)
        grid, origin, resolution = occupancy.decode(saved)
        self.assertEqual(resolution, m.resolution)
        rows, cols, _ = m.index(np.array([[3.0, 0.0]]))
        r = int(round((m.origin[1] + rows[0] * m.resolution - origin[1]) / resolution))
        c = int(round((m.origin[0] + cols[0] * m.resolution - origin[0]) / resolution))
        self.assertEqual(grid[r, c], 100)
        self.assertLess(grid.size, m.grid().size)
        for bad in ({}, dict(saved, width=saved["width"] + 1), dict(saved, cells="!!"),
                    dict(saved, resolution=0)):
            with self.assertRaises(ValueError):
                occupancy.decode(bad)
        self.assertIsNone(occupancy.encode(np.full((5, 5), -1, np.int8), (0, 0), .05))
        self.assertTrue(occupancy.pgm(grid).startswith(b"P5\n"))


class TestLocalizer(unittest.TestCase):
    def setUp(self):
        m = room_map()
        self.loc = Localizer(m.grid(), m.origin, m.resolution)
        self.scan = wall_points(ROOM + PILLAR, heights=(1.0,))[:, :2]

    def odom_scan(self, pose):
        """The scan in an odom frame that is pose (map-to-odom) off the map."""
        c, s = math.cos(pose[2]), math.sin(pose[2])
        return (self.scan - pose[:2]) @ np.array([[c, -s], [s, c]])

    def test_odometry_drift_is_corrected(self):
        # A wall on a cell's edge is mapped at the cell's centre, so the fit at the truth is
        # off by up to half a cell; every match must land on that same fit.
        fit, _, _ = self.loc.match(self.odom_scan(np.zeros(3)), np.zeros(3))
        self.assertLess(np.hypot(*fit[:2]), .04)
        for truth in ([.08, -.06, math.radians(1.5)], [-.1, .1, math.radians(-2)]):
            truth = np.array(truth)
            pose, inliers, fraction = self.loc.match(self.odom_scan(truth), np.zeros(3))
            self.assertLess(np.hypot(*(pose[:2] - truth[:2] - fit[:2])), .01)
            self.assertLess(abs(pose[2] - truth[2] - fit[2]), math.radians(.2))
            self.assertGreater(fraction, .9)
            self.assertTrue(accept(pose, np.zeros(3), inliers, fraction))

    def test_the_start_search_finds_a_larger_offset(self):
        fit, _, _ = self.loc.match(self.odom_scan(np.zeros(3)), np.zeros(3))
        truth = np.array([-.4, .3, math.radians(-8)])
        pose, _, fraction = self.loc.search(self.odom_scan(truth), np.zeros(3))
        self.assertLess(np.hypot(*(pose[:2] - truth[:2] - fit[:2])), .01)
        self.assertGreater(fraction, .9)

    def test_along_a_corridor_the_prior_is_kept(self):
        walls = [((-10, 0), (10, 0)), ((-10, 2), (10, 2))]
        m = occupancy.OccupancyMap(size=24)
        for _ in range(5):
            m.integrate(np.vstack([wall_points(walls), floor_points(-9, 9, .2, 1.8)]), np.array([0, 1, 1.2]))
        loc = Localizer(m.grid(), m.origin, m.resolution)
        scan = wall_points([((-4, 0), (4, 0)), ((-4, 2), (4, 2))], heights=(1.0,))[:, :2]
        prior = np.array([.5, 0.0, 0.0])        # odometry thinks we're 0.5 m along
        truth_across = .12
        pose, _, _ = loc.match(scan - (0, truth_across), prior)
        self.assertAlmostEqual(pose[1], truth_across, delta=.04)
        self.assertAlmostEqual(pose[0], .5, delta=.05)

    def test_a_scan_off_the_map_or_too_sparse_is_refused(self):
        far = self.scan + 20.0
        pose, inliers, fraction = self.loc.match(far, np.zeros(3))
        self.assertFalse(accept(pose, np.zeros(3), inliers, fraction))
        few = self.odom_scan(np.zeros(3))[:20]
        pose, inliers, fraction = self.loc.match(few, np.zeros(3))
        self.assertFalse(accept(pose, np.zeros(3), inliers, fraction))
        self.assertFalse(accept(np.array([.5, 0, 0]), np.zeros(3), 500, 1.0))

    def test_a_map_without_walls_is_refused(self):
        with self.assertRaises(ValueError):
            Localizer(np.zeros((10, 10), np.int8), (0, 0), .05)



class TestWiring(unittest.TestCase):
    """How a map reaches Nav2, the goal and the scorer."""

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

    def test_a_map_puts_a_static_layer_first_in_both_costmaps(self):
        import yaml
        from harp.mapper import MAP_TOPIC
        launch = self.launch()
        self.assertTrue(launch.uses_map('{"mapping": true}'))
        self.assertTrue(launch.uses_map('{"map": {"cells": ""}}'))
        self.assertFalse(launch.uses_map('{"avoidance": "predictive"}'))
        self.assertFalse(launch.uses_map("not json"))
        self.assertEqual(launch.MAP_TOPIC, MAP_TOPIC)
        with open(launch.parameters_for("predictive_avoidance", mapped=True)) as source:
            params = yaml.safe_load(source)
        for costmap in ("local_costmap", "global_costmap"):
            p = params[costmap][costmap]["ros__parameters"]
            self.assertEqual(p["plugins"][0], "static_layer")
            self.assertEqual(p["plugins"][1:], ["stvl_layer", "inflation_layer"])
            self.assertEqual(p["static_layer"]["plugin"], "nav2_costmap_2d::StaticLayer")
            self.assertEqual(p["map_topic"], MAP_TOPIC)
            self.assertEqual(p["global_frame"], "odom")   # the map joins through map -> odom
        with open(launch.parameters_for("predictive_avoidance")) as source:
            plain = yaml.safe_load(source)
        self.assertNotIn("static_layer", plain["global_costmap"]["global_costmap"]["ros__parameters"]["plugins"])

    def test_goals_in_the_map_go_to_nav2_in_odom(self):
        from types import SimpleNamespace
        from harp.course_goal import to_odom
        turn = math.radians(10)
        transform = SimpleNamespace(transform=SimpleNamespace(
            translation=SimpleNamespace(x=.3, y=-.2, z=0.0),
            rotation=SimpleNamespace(x=0.0, y=0.0, z=math.sin(turn / 2), w=math.cos(turn / 2))))
        x, y, heading = to_odom((2.0, 1.0, 90.0), transform)
        self.assertAlmostEqual(x, .3 + 2 * math.cos(turn) - math.sin(turn))
        self.assertAlmostEqual(y, -.2 + 2 * math.sin(turn) + math.cos(turn))
        self.assertAlmostEqual(heading, 100.0)

    def test_the_task_picks_the_mapper_mode(self):
        from harp.mapper import blend, task_mode
        self.assertEqual(task_mode('{"mapping": true}'), ("build", None))
        self.assertEqual(task_mode('{"avoidance": "predictive"}'), (None, None))
        self.assertEqual(task_mode(""), (None, None))
        saved = occupancy.encode(room_map().grid(), room_map().origin, .05)
        mode, (got, pose) = task_mode(json.dumps({"map": saved, "map_pose": [1, 2, 90]}))
        self.assertEqual(mode, "localize")
        np.testing.assert_allclose(pose, [1, 2, math.pi / 2])
        with self.assertRaises(ValueError):
            task_mode(json.dumps({"map": dict(saved, width=3)}))
        from harp.mapper import BLEND
        mid = blend(np.array([0, 0, math.radians(179)]), np.array([.2, 0, math.radians(-179)]))
        self.assertAlmostEqual(mid[0], .2 * BLEND[0])
        self.assertAlmostEqual(math.degrees(mid[2]), 179 + 2 * BLEND[1])
        # Seen at a pivot far from odom's origin, a heading change doesn't move the robot.
        pivot = (8.0, 0.0)
        turned = blend(np.zeros(3), np.array([0.0, 0.0, math.radians(2)]), pivot)
        from harp.localizer import around
        moved = around(turned, pivot)[:2] - np.array(pivot)
        self.assertLess(np.hypot(*moved), .3 * 8 * math.radians(2) + 1e-9)

    def test_no_crowd_parks_every_mover_out_of_reach(self):
        from harp import movers
        self.assertEqual(movers.crowd_mode('{"crowd": "none"}'), "none")
        names = ["a", "b"]
        event = json.loads(movers.crowd_event(names, "none"))
        got_names, times, positions = movers.logged_path([event])
        self.assertEqual(got_names, names)
        self.assertGreater(np.linalg.norm(positions, axis=-1).min(), 900)
        self.assertIsNone(movers.logged_path([]))

    def test_course_adds_the_map_to_the_task(self):
        from course import course_task, crowd_of
        task = json.loads(course_task('{"avoidance": "predictive"}', crowd="none", extra={"mapping": True}))
        self.assertEqual(task, {"avoidance": "predictive", "crowd": "none", "mapping": True})
        self.assertEqual(crowd_of([{"event": "crowd", "mode": "none"}]), "none")
        self.assertEqual(crowd_of([]), "scripted")

    def test_drifting_odometry_scales_and_turns_each_step(self):
        from harp.state import Drift, odom_drift
        self.assertIsNone(odom_drift('{"odom_drift": [0.05, 6]}', "hardware"))
        scale, rate = odom_drift('{"odom_drift": [0.05, 6]}', "sim")
        self.assertAlmostEqual(rate, math.radians(.1))
        with self.assertRaises(ValueError):
            odom_drift('{"odom_drift": [0.9, 0]}', "sim")
        d = Drift(.05, 0.0)
        d.apply(0.0, 0.0, 0.0, (0, 0, 0))
        x, y, yaw, v = d.apply(1.0, 0.0, 0.0, (1, 0, 0))
        self.assertAlmostEqual(x, 1.05)
        self.assertAlmostEqual(v[0], 1.05)



class TestMapperNode(unittest.TestCase):
    """harp/mapper.py on a node, with the transform from a stand-in TF buffer."""

    def setUp(self):
        import rclpy
        self.rclpy = rclpy
        rclpy.init()
        self.node = rclpy.create_node("mapper_test")

    def tearDown(self):
        self.node.destroy_node()
        self.rclpy.shutdown()

    def cloud(self, points, stamp):
        from std_msgs.msg import Header
        from harp.lidar_mount import FRAME
        from harp.predictive_avoidance import cloud_message
        header = Header(frame_id=FRAME)
        header.stamp.sec = stamp
        return cloud_message(header, np.asarray(points, dtype=np.float32))

    def stand_in_tf(self, mapper, x, y, z):
        """odom <- lidar: a lidar at (x, y, z) in odom, level and facing +x."""
        from geometry_msgs.msg import TransformStamped
        t = TransformStamped()
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = x, y, z
        t.transform.rotation.w = 1.0
        mapper.tf = type("TF", (), {"lookup_transform": lambda self, *a, **k: t})()

    def test_a_mapping_walk_maps_and_reports(self):
        from unittest.mock import patch
        from harp import mapper as module
        m = module.Mapper(self.node, "build")
        self.stand_in_tf(m, 1.0, 1.0, 1.2)
        local = np.vstack([wall_points(ROOM + PILLAR), floor_points(.2, 5.8, .2, 3.8)]) - (1, 1, 1.2)
        with patch.object(module.events, "emit") as emit:
            for stamp in range(1, 6):
                m.on_cloud(self.cloud(local, stamp))
                m.next_event = 0.0
                m.work()
        self.assertEqual(cell(m.map, 3.0, 0.0), 100)
        saved = [c for c in emit.call_args_list if c.args[:2] == ("mapper", "map")]
        self.assertTrue(saved)
        grid, origin, resolution = occupancy.decode(saved[-1].kwargs)
        self.assertGreater((grid == 100).sum(), 300)
        np.testing.assert_allclose(m.pose, 0)

    def test_a_walk_on_a_saved_map_finds_its_start_and_corrects_drift(self):
        from unittest.mock import patch
        from harp import mapper as module
        built = room_map()
        saved = occupancy.encode(built.grid(), built.origin, built.resolution)
        _, args = module.task_mode(json.dumps({"map": saved}))
        with patch.object(module.events, "emit"):
            m = module.Mapper(self.node, "localize", args)
        # Odometry off the map by (0.3, -0.2, 5 deg): the walls seen in odom.
        truth = np.array([.3, -.2, math.radians(5)])
        c, s = math.cos(truth[2]), math.sin(truth[2])
        walls = wall_points(ROOM + PILLAR, heights=(1.0,))
        odom_xy = (walls[:, :2] - truth[:2]) @ np.array([[c, -s], [s, c]])
        odom = np.column_stack([odom_xy, walls[:, 2]])
        self.stand_in_tf(m, 0.0, 0.0, 0.0)
        with patch.object(module.events, "emit"):
            for stamp in range(1, 12):
                m.on_cloud(self.cloud(odom, stamp))
                m.next_search = 0.0
                m.work()
        self.assertTrue(m.found)
        fit, _, _ = m.localizer.match(walls[:, :2], np.zeros(3))
        self.assertLess(np.hypot(*(m.pose[:2] - truth[:2] - fit[:2])), .02)
        self.assertLess(abs(m.pose[2] - truth[2]), math.radians(.5))


if __name__ == "__main__":
    unittest.main()
