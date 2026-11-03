"""Safety and behavioral properties of the prediction/controller boundary."""
import math
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np
from harp.predictive_mpc import Config, Planner


def obstacle(x, y, vx=0, vy=0, radius=.2, uncertainty=.03):
    return SimpleNamespace(x=x, y=y, vx=vx, vy=vy, radius=radius,
                           uncertainty=uncertainty, last_seen=0., hits=5)


class CooperativeContingency(unittest.TestCase):
    """Credit for objects seen giving way, never for the next YIELD_DELAY seconds."""
    def plan(self, walker, credit):
        planner = Planner(Config(speed=.5, tracking_gain=.85, lateral_tracking_gain=.6))
        walker.id = 7
        planner.credits = {7: credit}
        return planner.plan((0, 0, 0), (.45, 0, 0), (.45, 0, 0), [walker], 0., (6., 0.))

    def test_a_walker_seen_giving_way_is_not_detoured_as_if_it_would_walk_into_the_robot(self):
        # Walking down the next lane, 0.9 m over: on a straight line it passes inside the
        # forecast margins, but a person who gives way widens that.
        self.assertNotEqual(self.plan(obstacle(3., .9, vx=-.5, radius=.25), 0.).mode, 'follow')
        self.assertEqual(self.plan(obstacle(3., .9, vx=-.5, radius=.25), 1.).mode, 'follow')

    def test_credit_never_relaxes_the_next_moments(self):
        from harp.predictive_mpc import YIELD_DELAY, yield_allowance
        self.assertEqual(float(np.max(yield_allowance(np.linspace(0., YIELD_DELAY, 13)))), 0.)
        result = self.plan(obstacle(1.2, 0., vx=-.5, radius=.25), 1.)   # contact within 1 s
        self.assertNotEqual(result.mode, 'follow')
        self.assertTrue(result.alert)


class Chattering(unittest.TestCase):
    def jumps(self, smoothing, seed):
        """Command changes over 0.1 m/s while two people pass in the next lanes, their
        tracks noisy (5 cm, 5 cm/s), the robot walking toward its goal."""
        rng = np.random.default_rng(seed)
        planner = Planner(Config(speed=.5, tracking_gain=.85, lateral_tracking_gain=.6,
                                 smoothing=smoothing, cooperative=False))
        pose, velocity, commands = np.zeros(3), np.array([.4, 0., 0.]), []
        for tick in range(30):
            t = tick*.2
            people = [obstacle(4.-.5*t, .9+.05*rng.normal(), vx=-.5+.05*rng.normal(),
                               vy=.05*rng.normal(), radius=.25, uncertainty=.05),
                      obstacle(6.-.5*t, -.8+.05*rng.normal(), vx=-.5+.05*rng.normal(),
                               vy=.05*rng.normal(), radius=.25, uncertainty=.05)]
            commands.append(planner.plan(pose, velocity, (.45, 0, 0), people, t, (9., 0.)).command)
            velocity = np.array(commands[-1])*[.85, .6, 1.]
            pose[:2] += velocity[:2]*.2
        return int(np.sum(np.linalg.norm(np.diff(np.array(commands)[:, :2], axis=0), axis=1) > .1))

    def test_a_still_clear_manoeuvre_is_not_swapped_every_cycle(self):
        smooth = sum(self.jumps(True, seed) for seed in range(4))
        self.assertLess(smooth, sum(self.jumps(False, seed) for seed in range(4)))


class Heading(unittest.TestCase):
    """HIM building, Nov 2: in evade the filter withheld Nav2's turn (62 of 103 commands)
    or turned against it, because a turn also swung its body-frame escape step."""
    def test_a_held_escape_step_keeps_its_direction_while_turning(self):
        planner = Planner(Config(speed=.5))
        controls = np.tile([0., .3, .3], (1, 3, 1))
        paths, _ = planner._rollout(controls, np.zeros(3), np.zeros(3), body=np.zeros((1, 3), bool))
        self.assertLess(abs(paths[0, -1, 0]), .05)
        self.assertGreater(paths[0, -1, 1], .5)
        curved, _ = planner._rollout(controls, np.zeros(3), np.zeros(3))
        self.assertLess(curved[0, -1, 0], -.2)   # without holding, the step swings round

    def turns(self, heading, seed):
        """(planner turn, Nav2 turn, last turn) avoiding two noisy passers-by, Nav2 asking to turn."""
        rng = np.random.default_rng(seed)
        planner = Planner(Config(speed=.5, tracking_gain=.85, lateral_tracking_gain=.6,
                                 cooperative=False, heading=heading))
        pose, velocity, out, last = np.zeros(3), np.array([.4, 0., 0.]), [], 0.
        for tick in range(30):
            t = tick*.2
            people = [obstacle(4.-.5*t, .9+.05*rng.normal(), vx=-.5+.05*rng.normal(),
                               vy=.05*rng.normal(), radius=.25, uncertainty=.05),
                      obstacle(6.-.5*t, -.8+.05*rng.normal(), vx=-.5+.05*rng.normal(),
                               vy=.05*rng.normal(), radius=.25, uncertainty=.05)]
            desired = (.45, 0., .3 if tick < 15 else -.3)
            result = planner.plan(pose, velocity, desired, people, t, (9., 0.))
            if result.mode in ('yield', 'evade'):
                out.append((result.command[2], desired[2], last))
            last = result.command[2]
            velocity = np.array(result.command)*[.85, .6, 1.]
            co, si = math.cos(pose[2]), math.sin(pose[2])
            pose += np.array([co*velocity[0]-si*velocity[1], si*velocity[0]+co*velocity[1],
                              velocity[2]])*.2
        return out

    def test_avoiding_keeps_turning_as_nav2_asks(self):
        for heading, check in ((True, self.assertEqual), (False, self.assertGreater)):
            turns = [pair for seed in range(4) for pair in self.turns(heading, seed)]
            self.assertGreater(len(turns), 20)
            # Wrong: withheld or against Nav2, unless still ramping to Nav2's own new turn.
            wrong = sum((w*d <= 0 or abs(w) < .05) and abs(w-d) >= abs(last-d) for w, d, last in turns)
            check(wrong, 0, (heading, wrong, len(turns)))


class PeopleMargin(unittest.TestCase):
    def test_moving_objects_get_the_extra_margin_and_walls_do_not(self):
        plain, spaced = Planner(Config(speed=.5)), Planner(Config(speed=.5, people_margin=.15))
        controls = np.tile([.4, 0., 0.], (1, 3, 1))
        person = np.array([[2., .9, -.3, 0., .25, .03, 0.]])
        for planner in (plain, spaced):
            planner.paths, _ = planner._rollout(controls, np.zeros(3), np.zeros(3))
        gap = plain._clearances(plain.paths, person)[0] - spaced._clearances(spaced.paths, person)[0]
        self.assertAlmostEqual(gap, .15, places=6)
        for planner in (plain, spaced):
            planner._build_field(np.array([[2., .9, 0., 0., .2, .03, 0.]]), np.zeros(3), 3.)
        self.assertAlmostEqual(plain._clearances(plain.paths, np.zeros((0, 7)))[0],
                               spaced._clearances(spaced.paths, np.zeros((0, 7)))[0], places=6)

    def test_invalid_people_margin_rejected(self):
        with self.assertRaises(ValueError):
            Planner(Config(people_margin=-.1))


class Responsiveness_(unittest.TestCase):
    def scans(self, credit, walk, steps=8):
        """A walker 3.5 m ahead of a standing robot, its velocity at each scan from walk."""
        from harp.predictive_mpc import Responsiveness
        judge = Responsiveness()
        if credit is not None:
            judge.credit[3] = credit
        x, y, out = 3.5, .1, []
        for k in range(steps):
            vx, vy = walk(k)
            out.append(judge.update([SimpleNamespace(id=3, x=x, y=y, vx=vx, vy=vy)], (0., 0., 0.),
                                    (0., 0., 0.))[3])
            x, y = x + .1*vx, y + .1*vy
        return out

    def test_unknown_walkers_get_no_credit(self):
        self.assertEqual(self.scans(None, lambda k: (-.5, 0.), steps=1), [0.])

    def test_a_walker_turning_aside_earns_credit(self):
        credits = self.scans(None, lambda k: (-.5, 0.) if k < 2 else (-.45, .25), steps=12)
        self.assertGreaterEqual(credits[-1], .5)
        self.assertEqual(credits[3], 0.)   # a single turn (a route corner) earns nothing yet

    def test_a_walker_holding_a_near_course_loses_credit_fast(self):
        credits = self.scans(1., lambda k: (-1.2, 0.), steps=12)   # closest pass inside 3 s
        self.assertEqual(credits[-1], 0.)


class PredictiveMpc(unittest.TestCase):
    def test_forecast_matches_executed_request_ramps_during_reversal(self):
        config = Config(speed=.5,response_delay=0.,lateral_tracking_gain=.6)
        forecast,execution = Planner(config),Planner(config)
        for planner in (forecast,execution):
            planner.previous[:] = (0.,.4,0.)
            planner.initialized = True
        target = np.array([0.,-.4,0.])
        pose,velocity = np.zeros(3),np.array([0.,.24,0.])
        paths,_ = forecast._rollout(np.tile(target,(3,1)),pose,velocity)
        actual = [pose[:2].copy()]
        for tick in range(10):
            cmd = np.array(execution.plan(pose,velocity,target,[],tick*.2).command)
            cmd[1] *= config.lateral_tracking_gain
            for _ in range(2):
                velocity += np.clip(cmd-velocity,-config.acceleration*.1,config.acceleration*.1)
                pose += velocity*.1
                actual.append(pose[:2].copy())
        np.testing.assert_allclose(paths[0,:len(actual)],actual,atol=1e-9)

    def test_incoming_collision_requests_a_sustained_lateral_target_promptly(self):
        planner = Planner(Config(speed=.5,tracking_gain=.85,lateral_tracking_gain=.6))
        result = planner.plan((0,0,0),(0,0,0),(.45,0,0),
                              [obstacle(2.,0.,vx=-.5,radius=.25,uncertainty=.05)],0.,(6.,0.))
        self.assertEqual(result.mode,'evade')
        self.assertGreater(abs(result.command[1]),.25)
        self.assertLessEqual(abs(result.command[1]),.4)
        # A walker's forecast spreads with its speed, so this escape need not be
        # certifiable; it must then be signalled. Physical separation is
        # checked in closed loop by test_does_not_freeze_in_front_of_an_incoming_object.
        self.assertEqual(result.alert, result.clearance < 0)
        # A target step never means teleporting to the target physical speed.
        paths,_ = planner._rollout(np.tile(result.command,(3,1)),np.zeros(3),np.zeros(3))
        speed = np.diff(paths[0],axis=0)/planner.config.dt
        np.testing.assert_allclose(speed[:planner.delay_steps],0.)
        acceleration = np.linalg.norm(np.diff(speed,axis=0),axis=1)/planner.config.dt
        self.assertLessEqual(acceleration.max(),planner.config.acceleration+1e-9)

    def test_existing_forecast_margin_violation_can_escape_but_cannot_remain(self):
        item = obstacle(.75,0.,uncertainty=.15)
        planner = Planner()
        result = planner.plan((0,0,0),(0,0,0),(.25,0,0),[item],0.)
        self.assertIn('margin_recovery',result.solver)
        self.assertTrue(result.alert)
        self.assertGreater(np.linalg.norm(result.command[:2]),.05)
        self.assertLess(result.command[0],0.)
        objects = np.array([[item.x,item.y,0.,0.,item.radius,item.uncertainty]])
        standing = np.zeros((1,len(planner.times),2))
        self.assertLess(planner._clearances(standing,objects,recover_margin=True)[0],0.)

    def test_margin_recovery_does_not_relax_a_new_incoming_conflict(self):
        planner = Planner()
        objects = np.array([[2.,0.,-.5,0.,.2,.15]])
        standing = np.zeros((1,len(planner.times),2))
        np.testing.assert_array_equal(planner._clearances(standing,objects),
            planner._clearances(standing,objects,recover_margin=True))
        objects[0,:4] = [.5,0.,0.,0.]
        self.assertLess(planner._clearances(standing,objects,recover_margin=True)[0],0.)

    def test_all_evasive_rollouts_start_from_measured_velocity(self):
        # Candidate generation must not overwrite the robot's actual starting
        # motion, including sideways travel and yaw during a passing manoeuvre.
        measured = np.array([.12,-.17,.25])
        for item in (obstacle(1.2,0),obstacle(.5,0)):
            planner = Planner()
            with patch.object(planner,'_rollout',wraps=planner._rollout) as rollout:
                planner.plan((0,0,0),measured,(.25,0,0),[item],0.)
            self.assertGreater(rollout.call_count,1)
            for call in rollout.call_args_list:
                np.testing.assert_array_equal(call.args[2],measured)

    def test_request_ramp_builds_even_while_gait_launch_velocity_is_zero(self):
        planner = Planner()
        requests = [planner.plan((0,0,0),(0,0,0),(.3,0,0),[],i*.2).command[0]
                    for i in range(4)]
        np.testing.assert_allclose(requests,[.08,.16,.24,.3])

    def test_passing_a_stationary_blocker_competes_with_stopping(self):
        result = Planner().plan((0,0,0),(0,0,0),(.25,0,0),[obstacle(1.2,0)],0.)
        self.assertGreater(np.linalg.norm(result.command[:2]),.05)
        self.assertGreaterEqual(result.clearance,0.)

    def test_clear_route_passes_acceleration_limited_request(self):
        planner = Planner()
        result = planner.plan((0, 0, 0), (0, 0, 0), (.3, 0, .6), [], 0)
        self.assertEqual(result.mode, 'follow')
        self.assertAlmostEqual(result.command[0], .08)
        self.assertLessEqual(abs(result.command[2]), .2+1e-9)

    def test_diagonal_speed_and_acceleration_use_vector_norm(self):
        planner = Planner()
        result = planner.plan((0, 0, 0), (0, 0, 0), (.3, .3, 0), [], 0)
        self.assertLessEqual(np.linalg.norm(result.command[:2]), .08+1e-9)
        result = planner.plan((0, 0, 0), (.2, .2, 0), (.3, .3, 0), [], 0)
        self.assertLessEqual(np.linalg.norm(result.command[:2]), .3+1e-9)

    def test_crossing_yields_then_resumes(self):
        planner = Planner()
        result = planner.plan((0, 0, 0), (.25, 0, 0), (.25, 0, 0),
                              [obstacle(1.3, -1.5, vy=.4)], 0)
        self.assertIn(result.mode, ('yield','evade'))
        self.assertGreaterEqual(result.clearance, 0)
        self.assertLess(result.command[0], .25)
        result = planner.plan((0, 0, 0), (0, 0, 0), (.25, 0, 0),
                              [obstacle(1.3, 2, vy=.4)], 0)
        self.assertEqual(result.mode, 'follow')

    def test_head_on_escape_clears_continued_obstacle_path(self):
        planner = Planner()
        item = obstacle(3, 0, vx=-.5)
        result = planner.plan((0, 0, 0), (.25, 0, 0), (.25, 0, 0), [item], 0)
        # Clear without alarm, and responding now: slowing (cutting the closing
        # speed while a sidestep may come late) or sidestepping. The closed-loop
        # test below checks that it actually passes.
        self.assertGreaterEqual(result.clearance, 0)
        self.assertFalse(result.alert)
        self.assertTrue(result.command[0] < .25-1e-6 or abs(result.command[1]) > .02)

    def test_head_on_closed_loop_passes_instead_of_postponing_by_backpedalling(self):
        planner = Planner()
        pose, velocity = np.zeros(3), np.zeros(3)
        minimum, lateral = math.inf, 0.
        for step in range(35):
            obj = obstacle(3.-.5*step*.2,0.,vx=-.5)
            result = planner.plan(pose,velocity,(.25,0,0),[obj],step*.2)
            delta = np.asarray(result.command)-velocity
            delta[:2] *= min(1.,.08/max(np.linalg.norm(delta[:2]),1e-9))
            velocity += delta
            pose[:2] += .2*velocity[:2]
            minimum = min(minimum,math.hypot(pose[0]-obj.x,pose[1]-obj.y))
            lateral = max(lateral,abs(pose[1]))
        self.assertGreater(minimum,.6)
        self.assertGreater(lateral,.6)

    def test_trapped_scene_requests_zero_and_alert(self):
        planner = Planner()
        # Boxed in: every step deepens an intrusion that standing only grazes.
        box = [obstacle(.72*math.cos(a),.72*math.sin(a))
               for a in np.linspace(0,2*math.pi,10,endpoint=False)]
        result = planner.plan((0, 0, 0), (0, 0, 0), (.25, 0, 0), box, 0)
        self.assertEqual(result.command, (0., 0., 0.))
        self.assertEqual(result.mode, 'blocked')
        self.assertTrue(result.alert)
        self.assertLess(result.clearance, 0)

    def test_existing_overlap_is_left_with_an_alert_rather_than_held(self):
        planner = Planner()
        item = obstacle(.5, 0)
        # No predicted escape can certify this overlap, but standing keeps it forever.
        result = planner.plan((0, 0, 0), (.1, 0, 0), (.25, 0, 0), [item], 0)
        self.assertEqual(result.mode, 'evade')
        self.assertEqual(result.solver, 'least_intrusion')
        self.assertTrue(result.alert)
        self.assertLess(result.clearance, 0)
        paths,_ = planner._rollout(np.tile(result.command,(3,1)),np.zeros(3),np.array([.1,0,0]))
        separation = np.linalg.norm(paths[0]-[item.x,item.y],axis=1)
        self.assertGreater(separation[int(2/planner.config.dt)], .5)

    def test_does_not_freeze_in_front_of_an_incoming_object(self):
        # Recorded crowd contacts: nothing was certifiably clear, so the old
        # fallback stood still while a non-yielding actor walked into the robot.
        config = Config(speed=.5,tracking_gain=.85,lateral_tracking_gain=.6)
        planner = Planner(config)
        pose,velocity,person = np.zeros(3),np.zeros(3),np.array([0.,1.3,0.,-.55])
        pending = [np.zeros(3)]*int(round(config.response_delay/.1))
        minimum, modes = math.inf, []
        for tick in range(25):
            track = obstacle(*person,radius=.33,uncertainty=.05)
            result = planner.plan(pose,velocity,(.5,0,0),[track],tick*.2,(6.,0.))
            modes.append(result.mode)
            gain = np.array([config.tracking_gain,config.lateral_tracking_gain,1.])
            for _ in range(2):
                pending.append(np.array(result.command)*gain)
                velocity += np.clip(pending.pop(0)-velocity,-config.acceleration*.1,config.acceleration*.1)
                pose[:2] += velocity[:2]*.1
                person[:2] += person[2:]*.1
                minimum = min(minimum,math.hypot(*(person[:2]-pose[:2])))
        # Moves at once, while the actor approaches. Standing would let the
        # actor's centre pass within 0.0 m of the robot's; contact needs under
        # 0.5 m (G1 half-width with arms ~0.3 m, plus a 0.2 m person).
        self.assertEqual(modes[:8],['evade']*8)
        self.assertGreater(minimum,.5)

    def test_unreachable_far_objects_do_not_change_the_plan(self):
        near = [obstacle(1.6,-1.,vy=.4,radius=.25)]
        wall = [obstacle(9.+.36*i,4.,radius=.23,uncertainty=.05) for i in range(120)]
        plain = Planner().plan((0,0,0),(.2,0,0),(.3,0,0),near,0.,(4.,0.))
        crowded = Planner().plan((0,0,0),(.2,0,0),(.3,0,0),near+wall,0.,(4.,0.))
        self.assertEqual(plain.mode,crowded.mode)
        np.testing.assert_allclose(plain.command,crowded.command,atol=1e-9)

    def test_static_distance_field_matches_exact_clearance(self):
        rng = np.random.default_rng(3)
        angle, reach = rng.uniform(0,2*np.pi,14), rng.uniform(1.,2.6,14)
        objects = np.column_stack((reach*np.cos(angle), reach*np.sin(angle), np.zeros((14,2)),
                                   rng.uniform(.1,.3,14), rng.uniform(.03,.2,14)))
        # Exact swept checks include forecast growth; fixed geometry needs none.
        planner = Planner(Config(uncertainty_rate=0.))
        paths,_ = planner._rollout(np.array([np.tile([.3*np.cos(a),.3*np.sin(a),0.],(3,1))
                                             for a in np.linspace(0,6.2,12)]), np.zeros(3), np.zeros(3))
        exact = planner._clearances(paths, objects)
        planner._build_field(objects, np.zeros(3), 3.)
        field = planner._clearances(paths, np.zeros((0,6)))
        np.testing.assert_allclose(field, exact, atol=.04)
        # Conservative wherever it decides anything (not deep inside a body,
        # where the distance cone's apex is smoothed by interpolation).
        near = exact > -.1
        self.assertTrue(near.any())
        self.assertTrue(np.all(field[near] <= exact[near]+1e-9))

    def test_risk_ranks_penetration_depth_not_encounter_duration(self):
        planner = Planner()
        person = np.array([[2.,0.,-.5,0.,.3,.05]])
        toward,_ = planner._rollout(np.tile([.3,0.,0.],(3,1)),np.zeros(3),np.zeros(3))
        aside,_ = planner._rollout(np.tile([0.,.3,0.],(3,1)),np.zeros(3),np.zeros(3))
        # Walking into an oncoming actor shortens the overlap but deepens it.
        risk = planner._risk(np.vstack((toward,aside)), person)
        self.assertLess(risk[1], risk[0])

    def test_symmetric_head_on_commits_to_one_side(self):
        planner = Planner(Config(speed=.5,tracking_gain=.85,lateral_tracking_gain=.6))
        pose, velocity, sides = np.zeros(3), np.array([.3,0.,0.]), []
        for tick in range(8):
            person = obstacle(3.-.7*tick*.2,0.,vx=-.5,radius=.27,uncertainty=.05)
            result = planner.plan(pose,velocity,(.45,0,0),[person],tick*.2,(6.,0.))
            if abs(result.command[1]) > .05:
                sides.append(np.sign(result.command[1]))
            velocity = np.array(result.command)*[.85,.6,1.]
            pose[:2] += velocity[:2]*.2
        self.assertTrue(sides)
        self.assertEqual(len(set(sides)), 1)

    def test_swept_collision_is_detected_between_time_samples(self):
        planner = Planner()
        path = np.zeros((1, len(planner.times), 2))
        # Fast object crosses the robot between .0 and .1 second samples.
        objects = np.array([[0., -5., 0., 100., .1, 0.]])
        self.assertLess(planner._clearances(path, objects)[0], 0)

    def test_solver_failure_keeps_only_checked_seed(self):
        planner = Planner()
        with patch('harp.predictive_mpc.minimize', side_effect=TimeoutError):
            result = planner.plan((0, 0, 0), (.25, 0, 0), (.25, 0, 0),
                                  [obstacle(3, 0, vx=-.5)], 0)
        self.assertEqual(result.mode, 'evade')
        self.assertEqual(result.solver, 'feasible_seed_budget')
        self.assertGreaterEqual(result.clearance, 0)
        self.assertLessEqual(np.linalg.norm(result.command[:2]), .3+1e-9)

    def test_invalid_and_excessive_inputs_fail_closed(self):
        cases = [([obstacle(math.nan, 0)], (0, 0, 0)),
                 ([obstacle(0, 0, radius=-1)], (0, 0, 0)),
                 ([obstacle(10, 0)]*1281, (0, 0, 0)),
                 # Moving objects each need a swept check; fixed geometry goes into the
                 # field and has its own, larger limit.
                 ([obstacle(1.5, 0, vx=-.5)]*161, (0, 0, 0)),
                 ([obstacle(1.5, 0)]*641, (0, 0, 0)),
                 ([], (math.inf, 0, 0))]
        for tracks, pose in cases:
            result = Planner().plan(pose, (0, 0, 0), (.25, 0, 0), tracks, 0)
            self.assertEqual(result.mode, 'blocked')
            self.assertTrue(result.alert)
            self.assertEqual(result.command, (0., 0., 0.))

    def test_postponing_an_unavoidable_hit_is_not_rated_safe(self):
        # A walker 4 m ahead comes straight at the robot faster than it can back
        # away. Within the horizon nothing touches, so backing away and standing
        # in the walker's path looked as safe as stepping out of it.
        planner = Planner(Config(tracking_gain=.85, lateral_tracking_gain=.6))
        walker = np.array([[4., 0., -.55, 0., .25, .05]])
        back, aside, stand = (np.tile(v, (3, 1)) for v in ([-.3, 0, 0], [0, .3, 0], [0, 0, 0]))
        paths, _ = planner._rollout(np.array([back, aside, stand]), np.zeros(3), np.zeros(3))
        risk = planner._risk(paths, walker)
        self.assertLess(risk[1], .1)
        self.assertGreater(risk[0], risk[1]+.5)
        self.assertGreater(risk[2], risk[1]+.5)

    def test_walls_do_not_count_against_the_moving_object_limit(self):
        from harp.obstacle_tracker import STRUCTURE_ID
        wall = []
        for i in range(200):
            tile = obstacle(1.5+.05*i, 1.2)
            tile.id = STRUCTURE_ID+i
            wall.append(tile)
        result = Planner().plan((0, 0, 0), (0, 0, 0), (.25, 0, 0), wall+[obstacle(3, -.5, vx=-.4)], 0)
        self.assertNotEqual(result.solver, 'track_capacity')

    def test_invalid_configuration_rejected(self):
        with self.assertRaises(ValueError):
            Planner(Config(acceleration=0))


if __name__ == '__main__':
    unittest.main()
