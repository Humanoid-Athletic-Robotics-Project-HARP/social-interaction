"""harp/behavior.py: wander points in front of the robot, within the area around its
start; goals Nav2 calls reached too early sent again; the talk through speech's bridge. Needs ROS 2 Foxy, so it runs in our
image (how: tests/test_approach_stop.py)."""

import json
import math
import random
import time
import unittest
from types import SimpleNamespace
from unittest.mock import patch

from harp.behavior import (ARRIVED, FRONT, LONGEST, MAX_LEG, MIN_LEG, STOP_WAIT, TALK, Behavior,
                           wander_point)


def bearing(pose, point):
    """Degrees from where the robot faces to the point."""
    a = math.atan2(point[1] - pose[1], point[0] - pose[0]) - pose[2]
    return math.degrees(math.atan2(math.sin(a), math.cos(a)))


class WanderPoint(unittest.TestCase):
    def points(self, pose, home=(0.0, 0.0), radius=2.5, n=200):
        rng = random.Random(1)
        return [wander_point(pose, home, radius, rng) for _ in range(n)]

    def test_in_front_from_the_start(self):
        for yaw in (0.0, 2.0, -2.5):
            pose = (0.0, 0.0, yaw)
            for p in self.points(pose):
                self.assertLessEqual(abs(bearing(pose, p)), FRONT + 1e-6)
                self.assertTrue(MIN_LEG <= math.dist(p, pose[:2]) <= MAX_LEG + 1e-6)

    def test_stays_within_the_area(self):
        pose = (1.5, 1.0, 0.7)
        for p in self.points(pose):
            self.assertLessEqual(math.dist(p, (0.0, 0.0)), 2.5)
            self.assertGreaterEqual(math.dist(p, pose[:2]), MIN_LEG)

    def test_turns_at_the_edge_facing_out(self):
        pose = (2.4, 0.0, 0.0)
        points = self.points(pose)
        for p in points:
            self.assertLessEqual(math.dist(p, (0.0, 0.0)), 2.5)
        self.assertTrue(all(abs(bearing(pose, p)) > FRONT for p in points))

    def test_heads_home_from_outside(self):
        self.assertEqual(wander_point((6.0, 0.0, 0.0), (0.0, 0.0), 2.5, random.Random(1)),
                         (0.0, 0.0))


class Go(unittest.TestCase):
    def go(self, ends, statuses):
        """Behavior.go with Nav2 answering statuses in turn, the robot at each of ends after
        each; the statuses it returned, and how many goals it sent."""
        robot = SimpleNamespace(node=None, nav=None, pose=None, events=[])
        robot.event = lambda name, **fields: robot.events.append(name)
        answers = iter(zip(ends, statuses))

        def go_to(node, nav, goal, deadline, interrupt=None, handoff=None):
            robot.pose, status = next(answers)
            return status, None

        with patch("harp.behavior.go_to", go_to):
            status = Behavior.go(robot, (2.0, 0.0, 0.0), time.monotonic() + 10)
        return status, len(robot.events) + 1

    def test_reached(self):
        self.assertEqual(self.go([(1.8, 0.1, 0.1)], ["SUCCEEDED"]), ("SUCCEEDED", 1))

    def test_ended_at_once_far_away_is_sent_again(self):
        self.assertEqual(self.go([(0.5, 0.0, 0.0), (2.0, 0.0, 0.0)], ["SUCCEEDED"] * 2),
                         ("SUCCEEDED", 2))

    def test_wrong_heading_is_sent_again(self):
        self.assertEqual(self.go([(2.0, 0.0, math.pi), (2.0, 0.0, 0.2)], ["SUCCEEDED"] * 2),
                         ("SUCCEEDED", 2))

    def test_failure_is_not_sent_again(self):
        self.assertEqual(self.go([(0.5, 0.0, 0.0)], ["ABORTED"]), ("ABORTED", 1))

    def test_arrived_margin(self):
        self.assertEqual(self.go([(2.0 - ARRIVED + 0.01, 0.0, 0.0)], ["SUCCEEDED"]), ("SUCCEEDED", 1))


class Talk(unittest.TestCase):
    def talk(self, answers=(), leaves=math.inf, prepared=None, earlier=None):
        """Behavior.talk on a fake clock, speech's bridge answering each (s, event, fields)
        in answers when that time comes (through on_event, for the talk it started), the
        person leaving at leaves s; earlier: what it had heard for the prepared talk before.
        What it sent on /harp/talk ((what, talk, reason, s)) and its talk_done event."""
        clock = [0.0]
        robot = SimpleNamespace(node=None, talk_id=prepared, talked={}, sent=[], events=[],
                                ready=False, speech={prepared: dict(earlier or {})} if prepared else {})
        robot.tell = lambda what, person, talk, **f: robot.sent.append(
            (what, talk, f.get("reason"), round(clock[0], 1)))
        robot.event = lambda name, **f: robot.events.append((name, f))
        robot.where = lambda person: (1.0, 0.0)
        robot.distance = lambda xy: 1.1
        robot.left = lambda person: clock[0] >= leaves
        pending = sorted(answers, key=lambda a: a[0])

        def spin_until(node, done, limit=math.inf):
            deadline = clock[0] + limit
            while not done() and clock[0] < deadline - 1e-9:
                clock[0] = round(clock[0] + 0.1, 6)   # no float drift past a time
                while pending and pending[0][0] <= clock[0]:
                    _, name, fields = pending.pop(0)
                    record = dict(fields, source="speech", event=name, talk=robot.sent[0][1])
                    Behavior.on_event(robot, SimpleNamespace(data=json.dumps(record)))
            return done()

        with patch("harp.behavior.spin_until", spin_until), \
                patch("harp.behavior.time", SimpleNamespace(monotonic=lambda: clock[0])):
            Behavior.talk(robot, "person_a")
        self.assertEqual(robot.speech, {})
        return robot.sent, robot.events[-1][1]

    def test_no_speech_is_the_placeholder(self):
        sent, done = self.talk()
        self.assertEqual([s[:1] + s[2:3] for s in sent], [("start", None), ("stop", "time")])
        self.assertEqual((done["ended"], done["speech"], done["seconds"]), ("time", False, TALK))

    def test_no_speech_and_they_leave(self):
        sent, done = self.talk(leaves=10.0)
        self.assertEqual((sent[-1][2], done["ended"], done["seconds"]), ("left", "left", 10.0))

    def test_speech_ends_it_with_a_goodbye(self):
        sent, done = self.talk([(0.3, "talk_start", {}), (50.0, "talk_done", {"reason": "bye"})])
        self.assertEqual([s[0] for s in sent], ["start"])
        self.assertEqual((done["ended"], done["speech"], done["seconds"]), ("bye", True, 50.0))

    def test_speech_that_answers_late_still_counts(self):
        _, done = self.talk([(8.0, "talk_start", {}), (60.0, "talk_done", {"reason": "goodbye"})])
        self.assertEqual((done["ended"], done["seconds"]), ("goodbye", 60.0))

    def test_they_leave_mid_talk(self):
        sent, done = self.talk([(0.3, "talk_start", {}), (22.0, "talk_done", {"reason": "stopped"})],
                               leaves=20.0)
        self.assertEqual(sent[-1][::2], ("stop", "left"))
        self.assertEqual((done["ended"], done["seconds"]), ("left", 22.0))

    def test_speech_that_never_ends_is_stopped(self):
        sent, done = self.talk([(0.3, "talk_start", {})])
        self.assertEqual(sent[-1][::2] + (sent[-1][3],), ("stop", "time", LONGEST))
        self.assertEqual(done["seconds"], LONGEST + STOP_WAIT)

    def test_speech_failing_to_start_falls_back(self):
        _, done = self.talk([(0.5, "talk_done", {"reason": "failed", "error": "no LLM"})])
        self.assertEqual((done["ended"], done["speech"], done["error"], done["seconds"]),
                         ("time", False, "no LLM", TALK))

    def test_its_prepared_sessions_end_before_the_start_is_ignored(self):
        """The prepared session ended on the way (Stop on the page); the start began a new one."""
        _, done = self.talk([(0.3, "talk_start", {}), (40.0, "talk_done", {"reason": "bye"})],
                            prepared="abc", earlier={"talk_done": {"reason": "stopped"}})
        self.assertEqual((done["ended"], done["seconds"]), ("bye", 40.0))

    def test_a_prepared_talk_keeps_its_id(self):
        sent, _ = self.talk(prepared="abc")
        self.assertEqual({s[1] for s in sent}, {"abc"})

    def approach(self, status):
        """Behavior.approach to a confirmed person, Nav2 answering status; what it sent."""
        robot = SimpleNamespace(people={"person_a"}, pose=(0.0, 0.0, 0.0), talk_id=None,
                                speech={}, talked={}, sent=[], node=None)
        robot.where = lambda person: (3.0, 0.0)
        robot.distance = lambda xy: math.dist(xy, robot.pose[:2])
        robot.event = lambda name, **f: None
        robot.tell = lambda what, person, talk, **f: robot.sent.append((what, talk, f.get("reason")))
        robot.go = lambda goal, deadline, interrupt=None, handoff=None: status
        return Behavior.approach(robot, "person_a"), robot

    def test_a_walk_up_prepares_speech(self):
        arrived, robot = self.approach("SUCCEEDED")
        self.assertTrue(arrived)
        self.assertEqual([s[0] for s in robot.sent], ["prepare"])
        self.assertEqual(robot.talk_id, robot.sent[0][1])   # the talk that follows uses it
        self.assertIn(robot.talk_id, robot.speech)

    def test_a_failed_walk_up_cancels_it(self):
        arrived, robot = self.approach("ABORTED")
        self.assertFalse(arrived)
        self.assertEqual([(s[0], s[2]) for s in robot.sent], [("prepare", None), ("stop", "cancelled")])
        self.assertEqual(robot.sent[0][1], robot.sent[1][1])
        self.assertEqual((robot.talk_id, robot.speech), (None, {}))

    def test_only_speech_events_of_its_talks_are_kept(self):
        robot = SimpleNamespace(ready=False, speech={"t1": {}})
        for record in ({"source": "speech", "event": "talk_start", "talk": "t1"},
                       {"source": "speech", "event": "talk_done", "talk": "old"},
                       {"source": "behavior", "event": "talk_done", "talk": "t1"}):
            Behavior.on_event(robot, SimpleNamespace(data=json.dumps(record)))
        Behavior.on_event(robot, SimpleNamespace(data="not json"))
        self.assertEqual(list(robot.speech), ["t1"])
        self.assertEqual(list(robot.speech["t1"]), ["talk_start"])


if __name__ == "__main__":
    unittest.main()
