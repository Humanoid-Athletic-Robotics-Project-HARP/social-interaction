"""Crowd regressions: route progress and positive visibility evidence."""
import math
from types import SimpleNamespace
import unittest
import numpy as np
from harp.local_route import route_direction
from harp.predictive_avoidance import observed_free
from harp.predictive_mpc import Planner, Config


class CrowdNavigation(unittest.TestCase):
    def test_goal_cost_scores_actual_progress_not_only_matching_velocities(self):
        planner = Planner()
        controls = np.tile([.3,0.,.6],(2,3,1))
        paths = np.zeros((2,len(planner.times),2))
        paths[0,:,0] = np.linspace(0.,1.,len(planner.times))
        paths[1,:,0] = np.linspace(0.,-1.,len(planner.times))
        planner.goal = np.array([6.,0.])
        costs = planner._costs(controls,np.array([.3,0.,.6]),paths)
        self.assertLess(costs[0],costs[1])

    def test_risk_prioritizes_imminent_threat_over_a_later_deeper_overlap(self):
        planner = Planner()
        objects = np.array([[0.,0.,0.,0.,.2,.03]])
        paths = np.zeros((2,len(planner.times),2))
        paths[:,:,0] = 5.
        paths[0,10,0] = .45
        paths[1,40,0] = 0.
        risk = planner._risk(paths,objects)
        self.assertGreater(risk[0],risk[1])

    def test_prediction_brakes_at_the_goal_instead_of_walking_past_it(self):
        planner = Planner()
        controls = np.tile([.3,0.,0.],(3,1))
        without,_ = planner._rollout(controls,np.zeros(3),np.zeros(3))
        planner.goal = np.array([.6,0.])
        with_goal,_ = planner._rollout(controls,np.zeros(3),np.zeros(3))
        self.assertLess(with_goal[0,-1,0],.65)
        self.assertGreater(without[0,-1,0],1.)
    def test_route_goes_beside_a_blocker_instead_of_requesting_motion_into_it(self):
        t = SimpleNamespace(x=1.,y=0.,radius=.25,uncertainty=.08)
        direction = route_direction((0,0),(4,0),[t])
        self.assertGreater(abs(direction[1]), .5)
        self.assertGreater(direction[0], 0.)

    def test_occupied_goal_does_not_disable_detours_around_a_nearer_blocker(self):
        tracks = [SimpleNamespace(x=x,y=0.,radius=.25,uncertainty=.08) for x in (1.,4.)]
        direction = route_direction((0,0),(4,0),tracks)
        self.assertGreater(abs(direction[1]),.5)

    def test_a_departing_person_is_not_treated_as_a_permanent_wall(self):
        t = SimpleNamespace(x=1.,y=0.,vx=0.,vy=.7,radius=.25,uncertainty=.05)
        direction = route_direction((0,0),(4,0),[t])
        self.assertGreater(direction[0],.8)

    def test_sparse_or_occluded_rays_do_not_clear_a_lost_obstacle(self):
        t = SimpleNamespace(x=2.,y=0.,radius=.25,uncertainty=2.)
        self.assertFalse(observed_free(t,np.array([0,0,1.2]),np.array([[3,0,0]])))
        rays = self.free_rays(t)
        rays = np.vstack([rays,[1.8,0,1.]])
        self.assertFalse(observed_free(t,np.array([0,0,1.2]),rays))

    def free_rays(self,t):
        origin = np.array([0.,0.,1.2])
        points = []
        for z in (.5,1.):
            for dx,dy in [(0,0),(.25,0),(-.25,0),(0,.25),(0,-.25)]:
                direction = np.array([t.x+dx,t.y+dy,z])-origin
                direction /= np.linalg.norm(direction)
                for jitter in (-.003,0,.003):
                    points.append(origin+4.*(direction+[0,jitter,0]))
        return np.array(points)

    def test_dense_rays_through_former_occupancy_prove_it_cleared(self):
        t = SimpleNamespace(x=2.,y=0.,radius=.25,uncertainty=2.)
        self.assertTrue(observed_free(t,np.array([0,0,1.2]),self.free_rays(t)))

    def test_route_prefers_seen_space_to_an_equal_unseen_detour(self):
        blocker = [SimpleNamespace(x=2.,y=y,radius=.2,uncertainty=.05,vx=0.,vy=0.)
                   for y in np.arange(-.6,.61,.3)]
        occluder = [SimpleNamespace(x=1.,y=1.1,radius=.3,uncertainty=.05,vx=0.,vy=0.),
                    SimpleNamespace(x=1.,y=-2.6,radius=.3,uncertainty=.05,vx=0.,vy=0.)]
        # Both detours around the blocker are equally long; the one above lies
        # in the occluder's shadow, so it is not known to be free.
        direction = route_direction((0.,0.),(4.,0.),blocker+occluder)
        self.assertLess(direction[1],0.)

    def test_recovery_checks_the_entire_braking_tail(self):
        planner = Planner(Config(speed=.4))
        controls = np.zeros((3,3))
        controls[0] = (.4,0.,.6)
        paths,_ = planner._rollout(controls,np.zeros(3),np.zeros(3),brake_after=planner.config.period)
        c = planner.config
        stop_time = c.response_delay+c.period+c.speed/c.acceleration+2*c.dt
        stop_index = int(math.ceil(stop_time/c.dt))
        np.testing.assert_allclose(paths[:,stop_index:],
                                   np.broadcast_to(paths[:,-1:],paths[:,stop_index:].shape),atol=1e-6)


if __name__ == '__main__':
    unittest.main()
