"""Walkers that see the robot (course.py --crowd aware) in HIM runs' simulated lidar."""
import io
import json
import os
import tempfile
import unittest
from contextlib import redirect_stdout
from pathlib import Path
from unittest.mock import Mock, patch

import mujoco
import numpy as np

import course
from harp import lidar_mount
from harp.movers import AwareMovers, Movers, logged_path


def world(extra="", walker="0 0 0", route="6 0"):
    """A robot (its free joint first, as HIM's G1) and one walker along route in 10 s."""
    return f"""<mujoco><custom>
        <numeric name="p_duration" data="10"/><numeric name="p_route" data="{route}"/>
        </custom><worldbody>
        <body name="{lidar_mount.PARENT}"><freejoint/><geom type="cylinder" size=".3 .7"/></body>
        <body name="p" mocap="true" pos="{walker}"><geom type="cylinder" size=".25 .85"/></body>
        {extra}</worldbody></mujoco>"""


def walk(xml, movers_class, robot_xy, seconds):
    """The walker's position every lidar scan (0.1 s), the robot standing at robot_xy."""
    model = mujoco.MjModel.from_xml_string(xml)
    data, movers = mujoco.MjData(model), movers_class(model)
    data.qpos[:2] = robot_xy
    path = []
    for t in np.arange(0., seconds + 1e-9, .1):
        movers.apply(data, t)
        path.append((t, data.mocap_pos[0][:2].copy()))
    return path


class AwareWalkers(unittest.TestCase):
    def test_a_walker_steps_around_a_robot_standing_on_its_route_and_rejoins_it(self):
        path = walk(world(), AwareMovers, (3., 0.), 12.)
        self.assertGreater(min(np.linalg.norm(xy - (3., 0.)) for _, xy in path), .6)
        self.assertGreater(max(abs(xy[1]) for _, xy in path), .3)    # around, not waiting
        self.assertTrue(any(xy[0] > 5.6 and abs(xy[1]) < .15 for t, xy in path if t >= 9.))

    def test_far_from_the_robot_a_walker_keeps_its_route_and_pace(self):
        aware = walk(world(), AwareMovers, (3., 20.), 23.)
        scripted = walk(world(), Movers, (3., 20.), 23.)
        self.assertLess(max(abs(xy[1]) for _, xy in aware), .02)
        end = next(t for t, xy in aware if xy[0] > 5.85)
        back = next(t for t, xy in aware if t > end and xy[0] < .15)
        self.assertLess(abs(end - 10.), 1.)
        self.assertLess(abs(back - 20.), 1.5)
        # Out of step only by starting from standing; turning back, it slows as people do.
        self.assertLess(max(np.linalg.norm(a - s) for (t, a), (_, s) in zip(aware, scripted)
                            if t < 9.5), .25)

    def test_a_walker_with_no_trips_stands_where_it_is_in_both_crowds(self):
        standing = world(extra="").replace('<numeric name="p_route" data="6 0"/>',
                                           '<numeric name="p_route" data="6 0"/><numeric name="p_trips" data="0"/>')
        for kind in (Movers, AwareMovers):
            self.assertLess(max(np.linalg.norm(xy) for _, xy in walk(standing, kind, (3., 20.), 5.)), .01)

    def test_a_bigger_mover_keeps_its_gap_from_its_own_edge(self):
        # A trolley (0.9 x 0.6 m) came within 0.1 m of the robot when sized as a person.
        xml = world().replace('<geom type="cylinder" size=".25 .85"/>',
                              '<geom type="box" size=".45 .3 .5"/>')
        path = walk(xml, AwareMovers, (3., 0.), 14.)
        reach = np.hypot(.45, .3)
        self.assertGreater(min(np.linalg.norm(xy - (3., 0.)) for _, xy in path), reach + .35 + .1)
        self.assertTrue(any(xy[0] > 5.6 for _, xy in path))

    def test_stepping_aside_never_takes_a_walker_into_a_wall(self):
        # The robot is on the walker's right, a wall on its left: it steps toward the wall.
        path = walk(world('<geom name="wall0" type="box" size="3 .075 1" pos="3 .6 1"/>'),
                    AwareMovers, (3., -.8), 14.)
        self.assertGreater(min(.525 - xy[1] for _, xy in path), .2)
        self.assertGreater(min(np.linalg.norm(xy - (3., -.8)) for _, xy in path), .6)
        self.assertTrue(any(xy[0] > 5.6 for _, xy in path))


    def test_aware_walkers_give_way_to_each_other_and_never_overlap(self):
        # Head-on along one line: they walked through each other (and the trolley) before.
        xml = f"""<mujoco><custom>
            <numeric name="p_duration" data="10"/><numeric name="p_route" data="6 0"/>
            <numeric name="q_duration" data="10"/><numeric name="q_route" data="0 0"/>
            </custom><worldbody>
            <body name="{lidar_mount.PARENT}"><freejoint/><geom type="cylinder" size=".3 .7"/></body>
            <body name="p" mocap="true" pos="0 0 0"><geom type="cylinder" size=".25 .85"/></body>
            <body name="q" mocap="true" pos="6 0 0"><geom type="cylinder" size=".25 .85"/></body>
            </worldbody></mujoco>"""
        model = mujoco.MjModel.from_xml_string(xml)
        data, movers = mujoco.MjData(model), AwareMovers(model)
        data.qpos[:2] = (3., 20.)   # the robot far away
        gaps, ends = [], set()
        for t in np.arange(0., 14., .1):
            movers.apply(data, t)
            gaps.append(np.linalg.norm(movers.positions[0] - movers.positions[1]))
            ends |= {i for i, end in ((0, 6.), (1, 0.)) if abs(movers.positions[i][0] - end) < .2}
        self.assertGreater(min(gaps), .5)        # 0.25 + 0.25: their bodies never overlap
        self.assertEqual(ends, {0, 1})           # and both get past each other


class CheckedDistance(unittest.TestCase):
    """course.geom_distance: MuJoCo 3.3's capsule-to-cylinder distance sometimes came back 0
    with nearest points off both geoms (a foot 0.66 m from an atrium pillar scored a contact)."""
    XML = """<mujoco><worldbody><geom name="pillar" type="cylinder" size="0.3 1.0" pos="6.5 2.2 1.0"/>
        <body name="foot" pos="5.66 1.73 0.04" quat="0.704 -0.009 -0.710 -0.001">
        <geom type="capsule" size="0.01 0.093"/></body>
        <body name="ball" pos="6.5 2.2 2.5"><geom type="sphere" size="0.1"/></body>
        <body name="box" pos="0 0 0"><geom type="box" size="0.1 0.1 0.1"/></body></worldbody></mujoco>"""

    def setUp(self):
        self.model = mujoco.MjModel.from_xml_string(self.XML)
        self.data = mujoco.MjData(self.model)
        mujoco.mj_kinematics(self.model, self.data)

    def test_the_exact_distance_matches_the_geometry(self):
        # The foot lies along x at 0.04 m; its nearer end (5.753, 1.73) is 0.8826 m from the
        # pillar's axis: 0.8826 - 0.3 - 0.01 (MuJoCo 3.3.7 agrees; 3.2.3 says 0.393).
        for g, want in ((1, .5733), (2, .4)):    # the ball 0.4 m above the pillar's top
            self.assertAlmostEqual(course.rounded_to_cylinder(self.model, self.data, g, 0), want, places=3)
            self.assertAlmostEqual(course.geom_distance(self.model, self.data, g, 0, 5.), want, places=3)
        self.assertIsNone(course.rounded_to_cylinder(self.model, self.data, 3, 0))           # a box

    def test_a_short_positive_with_nearest_points_off_the_geom_is_replaced(self):
        def short(model, data, g, o, limit, fromto):
            fromto[:] = [5.908, 1.851, .333, 6.216, 2.045, .481]   # MuJoCo 3.2.3, this pair
            return .393
        exact = course.rounded_to_cylinder(self.model, self.data, 1, 0)
        with patch.object(course.mujoco, "mj_geomDistance", short):
            self.assertAlmostEqual(course.geom_distance(self.model, self.data, 1, 0, 5.), exact)

    def test_a_distance_at_the_limit_is_kept(self):
        with patch.object(course.mujoco, "mj_geomDistance", lambda m, d, g, o, limit, ft: limit):
            self.assertEqual(course.geom_distance(self.model, self.data, 1, 0, .2), .2)

    def test_a_zero_with_nearest_points_off_the_geoms_is_replaced(self):
        def broken(model, data, g, o, limit, fromto):
            fromto[:] = [11.52, 3.48, .08, 12.49, 4.08, .04]   # as seen in HIM's atrium
            return 0.
        exact = course.rounded_to_cylinder(self.model, self.data, 1, 0)
        with patch.object(course.mujoco, "mj_geomDistance", broken):
            self.assertAlmostEqual(course.geom_distance(self.model, self.data, 1, 0, 2.), exact)
            self.assertEqual(course.geom_distance(self.model, self.data, 3, 0, 2.), 0.)  # other shapes kept
        self.assertGreater(exact, .5)

    def test_a_zero_whose_nearest_points_are_apart_is_replaced(self):
        foot = self.data.geom_xpos[1].copy()
        def apart(model, data, g, o, limit, fromto):
            fromto[:] = [*foot, 6.25, 2.04, .04]   # on both geoms, but far apart: not 0
            return 0.
        exact = course.rounded_to_cylinder(self.model, self.data, 1, 0)
        with patch.object(course.mujoco, "mj_geomDistance", apart):
            self.assertAlmostEqual(course.geom_distance(self.model, self.data, 1, 0, 2.), exact)


class LoggedPaths(unittest.TestCase):
    EVENTS = [{"source": "lidar", "event": "crowd", "mode": "aware", "movers": ["p"]},
              {"source": "lidar", "event": "movers", "sim_time": 0., "xy": [[0., 0.]]},
              {"source": "lidar", "event": "movers", "sim_time": 2., "xy": [[1., 2.]]}]

    def test_movers_follow_a_logged_path(self):
        model = mujoco.MjModel.from_xml_string(world())
        data, movers = mujoco.MjData(model), Movers(model, path=logged_path(self.EVENTS))
        for t, xy in ((-1., (0., 0.)), (1., (.5, 1.)), (2., (1., 2.)), (5., (1., 2.))):
            movers.apply(data, t)
            np.testing.assert_allclose(data.mocap_pos[0][:2], xy)
        self.assertIsNone(logged_path(self.EVENTS[1:]))   # no crowd event: scripted movers

    def test_contacts_are_counted_where_the_logged_walker_was(self):
        # Scripted, the walker stays near (5, 5); logged, it stood against the robot.
        with tempfile.TemporaryDirectory() as folder:
            xml = Path(folder) / "world.xml"
            xml.write_text(world(walker="5 5 0", route="6 5"))
            trace = {"time": np.arange(0., 1.01, .02),
                     "qpos": np.tile([0., 0., .8, 1., 0., 0., 0.], (51, 1))}
            logged = (["p"], np.array([0., 1.]), np.array([[[.3, 0.]], [[.3, 0.]]]))
            self.assertEqual(course.clearance(xml, trace, {})[1], 0)
            self.assertEqual(course.clearance(xml, trace, {}, logged)[1], 1)

    def test_scoring_a_run_counts_contacts_on_the_logged_paths(self):
        # crossing.xml's person walks x = 2.5, well clear of a robot standing at the origin;
        # an aware crowd's log has it standing against the robot instead.
        crowd = [{"source": "lidar", "event": "crowd", "mode": "aware", "movers": ["person"]},
                 {"source": "lidar", "event": "movers", "sim_time": 0., "xy": [[.3, 0.]]},
                 {"source": "lidar", "event": "movers", "sim_time": 2., "xy": [[.3, 0.]]}]
        for events, contacts, mode in (([], 0, "scripted"), (crowd, 1, "aware")):
            with tempfile.TemporaryDirectory() as folder:
                folder = Path(folder)
                g1, run, scenes = folder / "g1", folder / "run", folder / "scenes"
                for path in (g1, run):
                    path.mkdir()
                (g1 / "g1_robot.xml").write_text("""<mujoco><worldbody><body name="pelvis">
                    <freejoint/><geom type="cylinder" size=".3 .7"/></body></worldbody></mujoco>""")
                (run / "run.json").write_text(json.dumps({"world": "crossing"}))
                (run / "report.json").write_text(json.dumps({"status": "within_limits"}))
                (run / "policy.log").write_text("".join("submission-1  | " + json.dumps(e) + "\n" for e in [
                    {"source": "goal", "event": "goal_result", "status": "SUCCEEDED", "seconds": 2.,
                     "recoveries": 0}] + events))
                np.savez(run / "trace.npz", time=np.arange(0., 2.01, .02),
                         qpos=np.tile([0., 0., .8, 1., 0., 0., 0.], (101, 1)))
                with patch.object(course, "MODEL", g1), patch.object(course, "SCENES", scenes):
                    summary = course.score(run)
            self.assertEqual((summary["contacts"], summary["crowd"]), (contacts, mode))

    def test_the_replay_walks_an_aware_crowd_where_the_lidar_logged_it(self):
        with tempfile.TemporaryDirectory() as folder:
            folder = Path(folder)
            g1, run, scenes = folder / "g1", folder / "run", folder / "scenes"
            for path in (g1, run):
                path.mkdir()
            (g1 / "g1_robot.xml").write_text("""<mujoco><worldbody><body name="pelvis">
                <freejoint/><geom type="sphere" size=".1"/></body></worldbody></mujoco>""")
            (run / "run.json").write_text(json.dumps({"world": "crossing"}))
            (run / "policy.log").write_text("".join(
                "submission-1  | " + json.dumps(e) + "\n" for e in (
                    {"source": "lidar", "event": "crowd", "mode": "aware", "movers": ["person"]},
                    {"source": "lidar", "event": "mover_start", "mover": "person", "starts_at": 4.},
                    {"source": "lidar", "event": "movers", "sim_time": 0., "xy": [[2.5, -3.]]},
                    {"source": "lidar", "event": "movers", "sim_time": 10., "xy": [[1.5, 2.]]})))
            qpos = np.tile([0., 0., .8, 1., 0., 0., 0.], (2, 1))
            with patch.object(course, "MODEL", g1), patch.object(course, "SCENES", scenes):
                _, poses = course.replay_movers(run, qpos, np.array([5., 10.]))
        np.testing.assert_allclose(poses[:, 7:9], [[2., -.5], [1.5, 2.]])


class SimLidarCrowd(unittest.TestCase):
    def lidar(self, task):
        from harp.lidar import SimLidar
        node = Mock()
        with tempfile.TemporaryDirectory() as folder:
            scene = Path(folder) / "scene.xml"
            scene.write_text(world())
            out = io.StringIO()
            with patch.dict(os.environ, {"G1_TASK_COMMAND": task}), redirect_stdout(out):
                lidar = SimLidar(node, scene)
                # Not the rays (the image's MuJoCo 3.2.3 casts them): the movers and the log.
                lidar.camera = Mock()
                with patch.object(SimLidar, "cast", return_value=np.zeros((0, 3))), \
                        patch.object(SimLidar, "publish"):
                    for t in (0., .1, .2):
                        lidar.on_state(Mock(data=json.dumps(dict(
                            sim_time=t, position=[3., 0., .8], quat=[1., 0., 0., 0.], q=[],
                            control_ready=True))))
        lines = out.getvalue().splitlines()
        self.assertTrue(lines)
        return lidar, [json.loads(line) for line in lines if line.startswith("{")]

    def test_an_aware_crowd_logs_where_its_walkers_go(self):
        lidar, printed = self.lidar('{"crowd":"aware"}')
        crowd = {k: v for k, v in printed[0].items() if k != "wall"}   # harp/events.py's head
        self.assertEqual(crowd, {"source": "lidar", "event": "crowd", "mode": "aware",
                                 "movers": ["p"]})
        paths = [e for e in printed if e["event"] == "movers"]
        self.assertEqual([e["sim_time"] for e in paths], [.1, .2])   # with each 100 ms scan
        np.testing.assert_allclose(paths[-1]["xy"], lidar.movers.positions, atol=1e-3)

    def test_scripted_movers_log_no_paths(self):
        _, printed = self.lidar("")
        self.assertFalse([e for e in printed if e["event"] in ("crowd", "movers")])


if __name__ == "__main__":
    unittest.main()
