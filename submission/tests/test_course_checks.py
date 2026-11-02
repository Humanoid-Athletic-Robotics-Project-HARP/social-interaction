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


class RouteMovers(unittest.TestCase):
    def test_a_route_mover_walks_its_points_and_turns_back(self):
        import mujoco
        import numpy as np
        from harp.movers import Movers
        model = mujoco.MjModel.from_xml_string("""<mujoco><custom>
            <numeric name="p_duration" data="4"/><numeric name="p_route" data="2 0 2 2"/>
            </custom><worldbody><body name="p" mocap="true" pos="0 0 0">
            <geom type="sphere" size=".2"/></body></worldbody></mujoco>""")
        data, movers = mujoco.MjData(model), Movers(model)
        self.assertEqual(movers.started, [0.0])   # no delay: walks from the start
        # 4 m of route in 4 s: the corner at 2 s, the end at 4 s, back at the corner at 6 s.
        for t, xy in ((1, (1, 0)), (2, (2, 0)), (3, (2, 1)), (4, (2, 2)), (6, (2, 0)), (8, (0, 0))):
            movers.apply(data, t)
            np.testing.assert_allclose(data.mocap_pos[0][:2], xy, atol=1e-9)

    def test_the_building_world_loads_with_people_and_moving_objects(self):
        import mujoco
        from pathlib import Path
        from harp.movers import Movers
        world = Path(__file__).resolve().parents[1]/'worlds/building.xml'
        movers = Movers(mujoco.MjModel.from_xml_path(str(world)))
        self.assertEqual(sorted(movers.names), sorted(
            [f"person{i}" for i in range(7)] + ["trolley", "delivery"]))
        self.assertTrue(all(route is not None for route in movers.routes))

    def test_the_replay_walks_movers_where_scoring_places_them(self):
        import json
        import tempfile
        from pathlib import Path
        import numpy as np
        with tempfile.TemporaryDirectory() as folder:
            folder = Path(folder)
            g1, run, scenes = folder / "g1", folder / "run", folder / "scenes"
            for path in (g1, run):
                path.mkdir()
            # A stand-in for HIM's G1: its root free joint first, as in the real model.
            (g1 / "g1_robot.xml").write_text("""<mujoco><worldbody><body name="pelvis">
                <freejoint/><geom type="sphere" size=".1"/></body></worldbody></mujoco>""")
            (run / "run.json").write_text(json.dumps({"world": "crossing"}))
            (run / "policy.log").write_text('submission-1  | ' + json.dumps(
                {"event": "mover_start", "mover": "person", "starts_at": 4.0}) + "\n")
            qpos = np.tile([0., 0., .8, 1., 0., 0., 0.], (3, 1))
            with patch.object(course, "MODEL", g1), patch.object(course, "SCENES", scenes):
                model, poses = course.replay_movers(run, qpos, np.array([0., 6.5, 9.]))
        self.assertEqual(model.nq, 14)
        np.testing.assert_array_equal(poses[:, :7], qpos)
        # crossing.xml: 6 m from y=-3 in 5 s, once, starting at 4 s.
        np.testing.assert_allclose(poses[:, 7:10], [[2.5, -3, 0], [2.5, 0, 0], [2.5, 3, 0]])
        np.testing.assert_array_equal(poses[:, 10], 1.)


class Pruning(unittest.TestCase):
    def test_a_trace_cut_short_is_left_alone_and_pruning_goes_on(self):
        # HIM's sim stopping mid-write (plant_error) left a trace that isn't a zip; pruning
        # every run after each course then failed on it.
        import tempfile
        from pathlib import Path
        with tempfile.TemporaryDirectory() as folder:
            run = Path(folder)
            (run / "trace.npz").write_bytes(b"PK\x03\x04 cut short")
            self.assertIsNone(course.saved_model(run))
            self.assertTrue(course.strip(run))
            self.assertEqual((run / "trace.npz").read_bytes(), b"PK\x03\x04 cut short")


class ReplayScene(unittest.TestCase):
    def test_the_replay_marks_the_goal_and_shows_the_floor_plan_through_walls(self):
        import tempfile
        import xml.etree.ElementTree as ET
        from pathlib import Path
        import stage_world
        with tempfile.TemporaryDirectory() as folder:
            folder = Path(folder)
            (folder / "g1").mkdir()
            stage_world.main("building", folder / "g1", folder / "out")
            replay = ET.parse(folder / "out" / "building_replay_world.xml").getroot()
            plain = ET.parse(folder / "out" / "building_fixed.xml").getroot()
        geoms = {g.get("name"): g for g in replay.find("worldbody").iter("geom")}
        for marker in ("goal_area", "goal_pole"):
            self.assertEqual((geoms[marker].get("contype"), geoms[marker].get("conaffinity")), ("0", "0"))
        self.assertEqual(geoms["wall0"].get("rgba").split()[3], str(stage_world.SEE_THROUGH))
        # Only the replay: HIM's scene, scoring and the lidar keep solid walls and no markers.
        names = {g.get("name") for g in plain.find("worldbody").iter("geom")}
        self.assertNotIn("goal_pole", names)


    def test_the_replay_page_starts_over_the_whole_world(self):
        import tempfile
        from pathlib import Path
        import mujoco
        model = mujoco.MjModel.from_xml_string("""<mujoco><worldbody>
            <geom type="plane" size="0 0 .05"/>
            <geom type="box" size="1 .1 1" pos="0 2 1"/><geom type="box" size="1 .1 1" pos="8 -2 1"/>
            </worldbody></mujoco>""")
        with tempfile.TemporaryDirectory() as folder:
            page = Path(folder) / "index.html"
            page.write_text("a\n" + "\n".join(course.VIEWER_CAMERA) + "\nb\n")
            course.overview_camera(page, model, "run")
            text = page.read_text()
        self.assertIn("controls.target.set(4.00, 0.00, 0);", text)   # the world's centre
        self.assertNotIn(course.VIEWER_CAMERA[0], text)
