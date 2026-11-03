"""harp/game_planner.py: the game-theoretic core (HARP_PREDICTIVE_PLANNER=game)."""
import math
import os
import sys
import time
import unittest

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from harp import game_planner as game  # noqa: E402
from harp.game_planner import GamePlanner  # noqa: E402
from harp.obstacle_tracker import STRUCTURE_ID, Track  # noqa: E402
from harp.predictive_mpc import Config, Responsiveness  # noqa: E402

BODY = .4 + .25   # robot disc and a person


def planner():
    return GamePlanner(Config(speed=.5, tracking_gain=.85, lateral_tracking_gain=.6))


def person(x, y, vx, vy, t, ident=1):
    return Track(ident, x, y, vx, vy, .25, .05, t, 20)


def episode(p, start, velocity, desired, seconds=10., goal=(8., 0.)):
    """Closed loop: the robot walks each request (HIM's walker gains), the person walks
    straight on and never reacts. Returns (closest centre distance, robot pose)."""
    pose, vel = np.zeros(3), np.zeros(3)
    (px, py), (pvx, pvy) = start, velocity
    resp, closest = Responsiveness(), math.inf
    for k in range(int(seconds/.2)):
        t = k*.2
        tracks = [person(px, py, pvx, pvy, t)]
        p.credits = resp.update(tracks, tuple(pose), tuple(vel))
        p.stubborn = resp.stubborn
        cmd = np.array(p.plan(tuple(pose), tuple(vel), desired, tracks, t, goal).command)
        for _ in range(2):
            vel = cmd*np.array([.85, .6, 1.])
            c, s = math.cos(pose[2]), math.sin(pose[2])
            pose += np.array([c*vel[0]-s*vel[1], s*vel[0]+c*vel[1], vel[2]])*.1
            px, py = px+pvx*.1, py+pvy*.1
            closest = min(closest, math.hypot(px-pose[0], py-pose[1]))
    return closest, pose


class GameCore(unittest.TestCase):
    def test_with_nobody_around_it_follows_nav2(self):
        p = planner()
        r = p.plan((0., 0., 0.), (.4, 0., 0.), (.4, 0., .2), [], 0., None)
        self.assertEqual(r.mode, 'follow')
        self.assertAlmostEqual(r.command[0], .4, delta=.05)
        self.assertAlmostEqual(r.command[2], .2, delta=.05)

    def test_a_turn_on_the_spot_stays_a_turn_while_someone_walks_past(self):
        # The MPC answered a third of Nav2's turn requests in HIM's office by walking on.
        p = planner()
        for k in range(5):
            tracks = [person(-1.+.8*k*.2, 2.5, .8, 0., k*.2)]
            r = p.plan((0., 0., 0.), (0., 0., .5), (0., 0., .5), tracks, k*.2, (0., -3.))
        self.assertLess(abs(r.command[0]), .1)
        self.assertGreater(r.command[2], .3)

    def test_a_head_on_walker_who_never_reacts_is_passed_without_contact(self):
        closest, pose = episode(planner(), (6., .1), (-.8, 0.), (.5, 0., 0.))
        self.assertGreater(closest, BODY)
        self.assertGreater(pose[0], 2.5)          # and it kept walking on

    def test_a_crossing_walker_is_passed_without_contact(self):
        closest, pose = episode(planner(), (4., -3.), (0., .9), (.5, 0., 0.))
        self.assertGreater(closest, BODY)
        self.assertGreater(pose[0], 3.)

    def test_who_plays(self):
        p = planner()
        objects = np.array([[0., 0., 1., 0., .25, .05, 0.], [0., 0., 1., 0., .25, .05, .9],
                            [0., 0., 1., 0., .25, .05, 0.]])
        p.object_ids = np.array([1, 2, 3])
        p.stubborn = {3: 3}
        self.assertEqual(p._reactivity(objects).tolist(), [game.REACT_PRIOR, .9, 0.])

    def test_in_the_equilibrium_a_person_who_plays_makes_room_and_one_who_doesnt_does_not(self):
        # Robot: straight on or step left; person: straight on or step aside. Each straight
        # pair collides. A playing person takes some of the avoiding; a stubborn one none.
        cost = np.array([[[1., 0.], [0., 0.]]])          # (person, robot sample, person sample)
        robot_log = np.log(np.array([.7, .3]))
        people_log = np.log(np.array([[.7, .3]]))
        p = planner()
        r1, h1, _ = p._equilibrium(robot_log, people_log, cost, np.array([1.]))
        r0, h0, _ = p._equilibrium(robot_log, people_log, cost, np.array([0.]))
        self.assertGreater(h1[0, 1], .3)                   # the playing person steps aside more
        self.assertAlmostEqual(h0[0, 1], .3, places=6)     # the stubborn one stays as forecast
        self.assertGreater(r0[1], r1[1])                   # so the robot does more of it

    def test_it_never_walks_into_a_wall(self):
        p = planner()
        wall = [Track(STRUCTURE_ID+i, x, .9, 0., 0., .2, .05, 0., 9) for i, x in enumerate(np.arange(-1., 4., .3))]
        r = p.plan((0., 0., 0.), (.3, 0., 0.), (.35, .35, 0.), wall, 0., None)
        paths, _ = p._rollout(np.tile(r.command, (3, 1))[None], np.zeros(3), np.array([.3, 0., 0.]))
        self.assertGreaterEqual(p._safe(paths, np.zeros((0, 7)))[0], 0.)

    def test_starting_too_close_to_something_it_still_moves_away(self):
        # HIM: a few cm inside the clearance of a standing person or a remembered object, every
        # path counted as unsafe and the robot stood there for the rest of the run.
        p = planner()
        still = [Track(STRUCTURE_ID, 0., .35, 0., 0., .05, .05, 0., 9)]   # its centre 0.35 m away: inside the 0.40 m disc
        for k in range(3):
            r = p.plan((0., 0., 0.), (0., 0., 0.), (.4, 0., 0.), still, k*.2, None)
        self.assertGreater(r.command[0], .1)            # walks on along the wall, not frozen
        paths, _ = p._rollout(np.tile(r.command, (3, 1))[None], np.zeros(3), np.zeros(3))
        self.assertGreaterEqual(p._safe(paths, np.zeros((0, 7)))[0], 0.)

    def test_results_are_plain_python_for_the_filters_json_events(self):
        # A numpy bool in Result.alert crashed the filter in HIM (json.dumps) the first time
        # an alert came; the robot then stood for the rest of the run.
        import json
        p = planner()
        for k in range(8):   # a walker straight at it: alerts come
            r = p.plan((0., 0., 0.), (.4, 0., 0.), (.4, 0., 0.), [person(1.2-.8*k*.2, 0., -.8, 0., k*.2)], k*.2, None)
            self.assertIs(type(r.alert), bool)
            json.dumps(dict(mode=r.mode, alert=r.alert, solver=r.solver, clearance=r.clearance,
                            command=list(r.command)))

    def test_planning_time_with_eight_people(self):
        p = planner()
        rng = np.random.default_rng(1)
        times = []
        for k in range(10):
            tracks = [person(*rng.uniform(-3, 3, 2), *rng.uniform(-1, 1, 2), 0., ident=i) for i in range(8)]
            started = time.perf_counter()
            p.plan((0., 0., 0.), (.3, 0., 0.), (.4, 0., 0.), tracks, 0., (5., 0.))
            times.append(time.perf_counter()-started)
        self.assertLess(float(np.median(times)), .1)   # the filter's deadline is 0.15 s


if __name__ == '__main__':
    unittest.main()
