"""Verdicts must reject contacts, instability and missing obstacle motion."""
import math
import unittest
from native_g1_sim import Trial


class Verdict(unittest.TestCase):
    def trial(self):
        trial = Trial.__new__(Trial)
        trial.truth = [30., 4., 0., .74]
        trial.first_truth = [0., 0.]
        trial.status = 'SUCCEEDED'
        trial.contacts = 0
        trial.movers = trial.obstacle_count = 7
        trial.min_height = .73
        trial.max_tilt = .12
        trial.modes = {'follow': 1, 'yield': 1}
        trial.first_meter = 5.
        trial.crowd_progress = 2.
        trial.longest_wait = 0.
        trial.started, trial.finished = 1., 20.
        trial.minimum_people_separation = 1.
        trial.goal = (4.,0.)
        return trial

    def test_success_requires_every_obstacle_and_stable_contact_free_walking(self):
        self.assertTrue(self.trial().report()['passed'])
        for key, value in [('contacts', 1), ('movers', 6), ('min_height', .4),
                           ('max_tilt', math.radians(50)), ('status', 'TIMEOUT')]:
            with self.subTest(key=key):
                trial = self.trial()
                setattr(trial, key, value)
                self.assertFalse(trial.report()['passed'])

    def test_waiting_outside_the_crowd_cannot_pass(self):
        trial = self.trial()
        trial.crowd = [{}]
        trial.crowd_progress = .1
        self.assertFalse(trial.report()['passed'])
        trial.crowd_progress = 2.
        trial.longest_wait = 12.
        self.assertFalse(trial.report()['passed'])


class Building(unittest.TestCase):
    def rectangles(self):
        from native_g1_sim import BUILDING_BLOCKS, BUILDING_WALLS
        walls = [(((x0+x1)/2,(y0+y1)/2),(max(abs(x1-x0)/2,.075),max(abs(y1-y0)/2,.075)))
                 for x0,y0,x1,y1 in BUILDING_WALLS]
        return walls+[((x,y),(hx,hy)) for x,y,hx,hy,_ in BUILDING_BLOCKS]

    def gap(self, point):
        # Distance from a point to the nearest wall or block surface.
        best = math.inf
        for (cx,cy),(hx,hy) in self.rectangles():
            dx,dy = max(abs(point[0]-cx)-hx,0.),max(abs(point[1]-cy)-hy,0.)
            best = min(best,math.hypot(dx,dy))
        return best

    def test_walker_routes_never_pass_through_walls_or_blocks(self):
        from native_g1_sim import BUILDING_ROUTES
        for index,(_,route) in enumerate(BUILDING_ROUTES):
            for (x0,y0),(x1,y1) in zip(route,route[1:]):
                steps = max(2,int(math.dist((x0,y0),(x1,y1))/.05))
                for k in range(steps+1):
                    point = (x0+(x1-x0)*k/steps, y0+(y1-y0)*k/steps)
                    with self.subTest(walker=index,point=point):
                        # A 0.20 m capsule must fit; wall repulsion adds margin.
                        self.assertGreater(self.gap(point),.2)

    def test_start_and_goal_leave_room_for_the_robot(self):
        for point in ((0.,0.),(9.,0.)):
            self.assertGreater(self.gap(point),1.)
