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


class FailedGoal(unittest.TestCase):
    def test_a_failed_goal_is_sent_again_while_time_remains(self):
        # Walkers waiting in the only doorway leave Nav2 no path; once they move on it finds one.
        route = route_from_data({"frame": "odom", "waypoints": [[2, 0]], "laps": 1})
        replies = [("FAILED", _feedback(0., 0.)), ("FAILED", _feedback(.5, 0.)),
                   ("SUCCEEDED", _feedback(2., 0.))]
        events = []
        with patch.object(course_goal, "go_to", side_effect=replies) as go, \
                patch.object(course_goal, "pause") as pause:
            status = course_goal.follow(None, None, route,
                                        lambda name, **f: events.append((name, f)), True)
        self.assertEqual(status, "SUCCEEDED")
        self.assertEqual(go.call_count, 3)
        self.assertEqual(pause.call_count, 2)
        self.assertEqual(sum(name == "goal_retry" for name, _ in events), 2)

    def test_a_goal_localization_moves_is_sent_again_where_it_now_is(self):
        # Leg odometry drifts; once map -> odom has moved the point 0.1 m, Nav2 gets it again.
        route = route_from_data({"frame": "odom", "waypoints": [[2, 0]], "laps": 1})
        shift = [0.0]
        change = lambda p: (p[0] + shift[0], p[1], p[2])
        sent = []

        def go_to(node, nav, point, deadline, interrupt=None, handoff=None):
            sent.append(point)
            if len(sent) == 1:
                shift[0] = .3                       # localization corrects 0.3 m meanwhile
                self.assertTrue(interrupt() and handoff())
                return "HANDED_OFF", None
            self.assertFalse(interrupt())
            return "SUCCEEDED", _feedback(2.3, 0.)
        with patch.object(course_goal, "go_to", side_effect=go_to):
            status = course_goal.follow(None, None, route, lambda name, **f: None, True, change)
        self.assertEqual(status, "SUCCEEDED")
        self.assertEqual([p[0] for p in sent], [2.0, 2.3])

    def test_without_time_left_a_failure_ends_the_route(self):
        route = route_from_data({"frame": "odom", "waypoints": [[2, 0]], "laps": 1})
        with patch.object(course_goal, "go_to", return_value=("FAILED", None)) as go, \
                patch.object(course_goal, "TIMEOUT", 1.0), patch.object(course_goal, "pause"):
            status = course_goal.follow(None, None, route, lambda name, **f: None, True)
        self.assertEqual((status, go.call_count), ("FAILED", 1))


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

    def test_a_cut_off_trace_is_reported_not_scored(self):
        # HIM writes trace.npz last, under its report deadline; on a Mac short of memory it
        # came out cut off after a route had finished, and scoring crashed the whole course.
        import tempfile
        from pathlib import Path
        from types import SimpleNamespace
        with tempfile.TemporaryDirectory() as folder:
            run = Path(folder)
            (run / "trace.npz").write_bytes(b"PK\x03\x04 cut short")
            with patch.object(course, "score") as score:
                self.assertFalse(course.run_and_score(run, SimpleNamespace(score=True), None))
            score.assert_not_called()


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
