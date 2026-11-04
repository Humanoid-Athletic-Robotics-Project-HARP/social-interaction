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
    straight on and never reacts. Returns (closest centre distance, robot pose, person x)."""
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
    return closest, pose, px


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
        closest, pose, walker = episode(planner(), (6., .1), (-.8, 0.), (.5, 0., 0.))
        self.assertGreater(closest, BODY)
        self.assertLess(walker, pose[0])          # the walker is past it: no freeze in front
        self.assertGreater(pose[0], 1.5)

    def test_a_crossing_walker_is_passed_without_contact(self):
        closest, pose, _ = episode(planner(), (4., -3.), (0., .9), (.5, 0., 0.))
        self.assertGreater(closest, BODY)
        self.assertGreater(pose[0], 3.)

    def test_cvar_is_the_mean_of_the_worst_share(self):
        cost = np.array([[[0., 1., 2., 3.]]])
        w = np.array([[.25, .25, .25, .25]])
        self.assertAlmostEqual(GamePlanner._cvar(cost, w, 1.)[0], 1.5)
        self.assertAlmostEqual(GamePlanner._cvar(cost, w, .25)[0], 3.)
        self.assertAlmostEqual(GamePlanner._cvar(cost, w, .5)[0], 2.5)
        self.assertAlmostEqual(GamePlanner._cvar(cost, w, .375)[0], (3*.25+2*.125)/.375)

    def test_who_plays(self):
        p = planner()
        objects = np.array([[0., 0., 1., 0., .25, .05, 0.], [0., 0., 1., 0., .25, .05, .9],
                            [0., 0., 1., 0., .25, .05, 0.]])
        p.object_ids = np.array([1, 2, 3])
        p.stubborn = {3: 3}
        self.assertEqual(p._reactivity(objects).tolist(), [game.REACT_PRIOR, .9, 0.])

    def test_a_stubborn_player_is_recognised_while_someone_far_away_does_not_play(self):
        # object_ids covers every moving object; the far one doesn't play, so the lengths differ.
        p = planner()
        p.stubborn, p.stubborn_players = {3: 3}, True
        seen = []
        reactivity = p._reactivity
        p._reactivity = lambda objects, ids=None: seen.append(reactivity(objects, ids)) or seen[-1]
        tracks = [person(2.5, 0., -1., 0., 0., ident=3), person(0., 5.4, 0., -1., 0., ident=9)]
        p.plan((0., 0., 0.), (.4, 0., 0.), (.4, 0., 0.), tracks, 0., (8., 0.))
        self.assertEqual(len(p.object_ids), 2)
        self.assertEqual(seen[-1].tolist(), [0.])
        p.stubborn_players = False                   # the default: as tuned (unrecognised)
        p.plan((0., 0., 0.), (.4, 0., 0.), (.4, 0., 0.), tracks, .2, (8., 0.))
        self.assertGreater(seen[-1][0], 0.)

    def test_the_safe_horizon_follows_safe_time_as_set_at_run_time(self):
        # The benchmark's --set changes module constants after import; a default argument bound
        # at import had kept SAFE_TIME at 1.2 whatever was set.
        p = planner()
        p.field = None
        path = np.zeros((1, len(p.times), 2))                         # standing at the origin
        person = np.array([[3., 0., -1., 0., .25, .05, 0.]])          # 3 m off, walking at it
        old = game.SAFE_TIME
        try:
            game.SAFE_TIME = .5                                       # arrives after 2.35 s
            self.assertGreater(p._safe(path, person)[0], 0.)
            game.SAFE_TIME = 3.
            self.assertLess(p._safe(path, person)[0], 0.)
        finally:
            game.SAFE_TIME = old

    def test_the_mode_mean_keeps_its_size_when_every_weight_is_tiny(self):
        seeds = np.array([[[.4, 0., 0.]], [[.2, .2, 0.]], [[0., 0., 0.]]])
        near = np.array([True, True, False])
        mean = GamePlanner._mode_mean(np.array([1e-14, 1e-14, .9]), seeds, near, 0)
        self.assertTrue(np.allclose(mean, [[.3, .1, 0.]]))
        self.assertTrue(np.allclose(GamePlanner._mode_mean(np.zeros(3), seeds, near, 1), seeds[1]))

    def test_in_the_equilibrium_a_person_who_plays_makes_room_and_one_who_doesnt_does_not(self):
        # Robot: straight on or step left; person: straight on or step aside. Each straight
        # pair collides. A playing person takes some of the avoiding; a stubborn one none.
        cost = np.array([[[1., 0.], [0., 0.]]])          # (person, robot sample, person sample)
        robot_log = np.log(np.array([.7, .3]))
        people_log = np.log(np.array([[.7, .3]]))
        p = planner()
        risk, game.RISK_ALPHA = game.RISK_ALPHA, 1.   # the property of the plain average
        try:
            r1, h1, _ = p._equilibrium(robot_log, people_log, cost, np.array([1.]))
            r0, h0, _ = p._equilibrium(robot_log, people_log, cost, np.array([0.]))
        finally:
            game.RISK_ALPHA = risk
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


class SlowComputer(unittest.TestCase):
    """Plans that run slow (PC2's cores) step down to cheaper LEVELS, and back up when fast."""

    def test_slow_plans_step_down_a_level_at_a_time_and_fast_ones_back(self):
        p = planner()
        for _ in range(3):
            p._budget(2*game.BUDGET)
        self.assertEqual((p.level, p.samples, p.rounds), (1,) + game.LEVELS[1])
        for _ in range(3):
            p._budget(2*game.BUDGET)
        self.assertEqual(p.level, 2)
        for _ in range(9):
            p._budget(2*game.BUDGET)
        self.assertEqual(p.level, len(game.LEVELS)-1)   # never past the cheapest
        for _ in range(3*game.JUDGE):
            p._budget(.1*game.BUDGET)
        self.assertEqual((p.level, p.samples, p.rounds), (0, game.SAMPLES, game.ROUNDS))

    def test_a_cheaper_level_still_plans_round_a_walker(self):
        p = planner()
        p.level, (p.samples, p.rounds) = 2, game.LEVELS[2]
        result = p.plan((0., 0., 0.), (.3, 0., 0.), (.5, 0., 0.), [person(2.5, 0., -1., 0., 0.)], 0.)
        self.assertTrue(np.isfinite(result.command).all())
        self.assertLess(result.command[0], .5)

    def test_the_prior_follows_the_speed_cap(self):
        # Requests off Nav2's by the same share of the speed cap cost the same at 0.3 and 0.5 m/s.
        logs = []
        for speed in (.3, .5):
            p = GamePlanner(Config(speed=speed, tracking_gain=.85, lateral_tracking_gain=.6))
            seeds = np.array([[[speed, 0., 0.]]*3, [[.5*speed, 0., 0.]]*3])
            paths = np.zeros((2, len(p.times), 3))
            log = p._robot_prior(seeds, paths, np.array([speed, 0., 0.]))
            logs.append(log[1]-log[0])
        self.assertAlmostEqual(logs[0], logs[1], places=6)
