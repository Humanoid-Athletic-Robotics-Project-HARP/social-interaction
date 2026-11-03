"""A game-theoretic core for predictive avoidance (HARP_PREDICTIVE_PLANNER=game).

The sampling MPC (predictive_mpc.Planner) forecasts every person walking straight on, never
reacting. In a crowd that makes standing, turning and walking all look like collisions (the
freezing-robot problem, Trautman & Krause 2010): in HIM the robot dodged against Nav2's heading
on about a third of its turn requests, and on eight held-out worlds it did no better than the
plain stop rule. People do react, and each person's reaction depends on what the robot does.

This planner treats the robot and the people near it as players of one game, after BRNE
(Sun, Baldini, Hughes, Trautman & Murphey, "Mixed strategy Nash equilibrium for crowd
navigation", IJRR 2024; written from the paper, not their code). Each player has a cloud of
possible trajectories with prior weights (the robot's: what it can walk near Nav2's request;
a person's: their tracked walk with smooth changes of pace and direction). Each round, every
player reweights its own cloud by the collisions it expects with the others' current clouds:

    w_i(x) ∝ prior_i(x) · exp(-β ρ_i E_{x_j ~ w_j, j≠i}[collision(x, x_j)])

The fixed point is a mixed-strategy Nash equilibrium of the game in which each player trades
collisions against staying close to its prior (the Bayesian update is that best response).
The robot then walks the equilibrium's most likely manoeuvre.

ρ_i says how much a person plays: 1 for the robot, REACT_PRIOR for someone unknown, more once
they are seen giving way (Responsiveness credit), 0 once seen keeping course into a conflict
(Responsiveness stubborn), when they become a fixed forecast, as the MPC assumed for everyone.

Safety doesn't rest on the equilibrium (contingency planning, as in contingency MPC): the
chosen first request must keep the robot's body clear of every object's straight-line forecast
for SAFE_TIME, before anyone can react, and must leave a plan B, some trajectory starting
with about the same request that keeps the comfort distance (COMFORT_GAP past the body: the
0.30 m clearance rule) from those forecasts for CONTINGENCY_TIME, should nobody react at all. The equilibrium's most likely request with a plan B is taken; else the
most likely one safe for SAFE_TIME; else the least intrusive. Walls count throughout. Walls,
remembered surfaces and slow objects come from the MPC's distance field (Planner._prepare).
"""

import math

import numpy as np

from harp.predictive_mpc import Planner, Result

SAMPLES = 32             # trajectories per person
PLAYERS = 8              # nearest moving objects that play; the rest are fixed forecasts
ENGAGE = 5.0             # m: farther objects are fixed forecasts
ROUNDS = 6               # rounds of best responses (converged in tests by about 4)
BETA = 6.                # weight of an expected collision against the prior (nats per unit)
REACT_PRIOR = .6         # how much someone not yet seen either way is expected to play
COMFORT_GAP = .26        # m past the robot's 0.40 m disc: about 0.30 m from its arms (0.355 m)
HARD = 4.                # collision cost of body overlap per metre, beyond the comfort part
RISK_TIME = 2.           # s: later predicted overlaps count less (prediction grows unsure)
SAFE_TIME = 1.2          # s nobody is assumed to react in (the MPC's YIELD_DELAY)
CONTINGENCY_TIME = 2.5   # s a plan B must stay clear for if people don't react at all
PERSON_SPREAD = (.35, .25)   # m/s velocity change and m/s^2 turn spread of a person's cloud
PRIOR_TEMPERATURE = .08  # request cost (m/s)^2 per nat of robot prior: 2σ², σ = 0.2 m/s
SIDEWAYS = 3.            # a sideways request costs this much more than forward (slow sidestep)
TURNING = .3             # turn-rate deviation weight (a round footprint doesn't turn)
SWITCHING = 1.           # cost of changing the request from the last one, per (m/s)^2
MODE_RADIUS = .12        # m/s: requests this close to the chosen one are averaged with it
STRIDE = 2               # collision costs every 2nd rollout step (0.2 s), for planning time


class GamePlanner(Planner):
    """plan(pose, velocity, desired, tracks, now, goal): as Planner.plan, a game-theoretic core."""

    def __init__(self, config=None, seed=0):
        super().__init__(config)
        self.rng = np.random.default_rng(seed)
        self.stubborn = {}       # track id -> scans seen keeping course into a conflict
        self.equilibrium = {}    # last plan's diagnostics (players, rounds, expected collision)

    # ----- the players' clouds -----
    def _robot_seeds(self, desired):
        """Request blocks (n, 3, 3) near Nav2's request: Nav2's own, standing, turning on the
        spot, every direction at three speeds (held, merging back, or stepping then stopping),
        curves, and random perturbations; every one turns as Nav2 asks unless it says otherwise."""
        c = self.config
        turn = float(desired[2])
        stop = np.array([0., 0., turn])
        seeds = [np.tile(desired, (3, 1)), np.zeros((3, 3)), np.tile(stop, (3, 1))]
        for speed in (c.speed, .65*c.speed, .3*c.speed):
            for angle in np.arange(16)*2*math.pi/16:
                step = np.array([speed*math.cos(angle), speed*math.sin(angle), turn])
                seeds += [np.tile(step, (3, 1)), np.array([step, step, desired]),
                          np.array([step, stop, stop])]
            for share in (-1., -.5, -.25, .25, .5, 1.):
                curve = np.array([speed, 0., share*c.yaw_speed])
                seeds.append(np.array([curve, curve, desired]))
        noise = self.rng.normal(0., 1., (48, 3, 3))*np.array([.15, .08, .3])
        seeds += list(desired[None, None, :]+noise)
        seeds.append(np.tile(self.previous, (3, 1)))
        seeds = np.clip(np.asarray(seeds), -self.limits, self.limits)
        return self._linear_limit(seeds, c.speed)

    def _robot_prior(self, seeds, paths, desired):
        """Log prior weight of each robot trajectory: close to Nav2's request (sideways costlier),
        little change from the last request, off walls. Nav2's request is the robot's whole
        intention: a pull toward the goal in a straight line (as the MPC had) walked the robot
        against Nav2's route, which knows the walls."""
        error = seeds-desired[None, None, :]
        track = np.mean(error[..., 0]**2+SIDEWAYS*error[..., 1]**2+TURNING*error[..., 2]**2, axis=1)
        switch = SWITCHING*np.sum((seeds[:, 0, :2]-self.previous[:2])**2, axis=-1)
        log = -(track+switch)/PRIOR_TEMPERATURE
        if self.field is not None:
            c = self.config
            wall = self._field_at(paths[:, 1:], 1, self.times[1:])-c.robot_radius
            body = self._field_at(paths[:, 1:], 0, self.times[1:])-c.robot_radius
            log = log-BETA*np.max(np.maximum(0., c.margin-wall)/c.margin, axis=1)
            log = np.where(np.min(body, axis=1) < 0., -np.inf, log)
        return log

    def _people(self, objects):
        """Each moving object's cloud (m, SAMPLES, steps, 2): its tracked walk, with one
        unchanged sample and the rest changing pace and direction smoothly; log prior weights
        (m, SAMPLES): off walls."""
        t = self.times[None, None, :, None]
        dv = self.rng.normal(0., PERSON_SPREAD[0], (len(objects), SAMPLES, 1, 2))
        da = self.rng.normal(0., PERSON_SPREAD[1], (len(objects), SAMPLES, 1, 2))
        dv[:, 0], da[:, 0] = 0., 0.
        base = objects[:, None, None, :2]+objects[:, None, None, 2:4]*t
        clouds = base+dv*t+.5*da*t*t
        log = np.zeros((len(objects), SAMPLES))
        if self.field is not None and len(objects):
            inside = self._field_at(clouds.reshape(-1, len(self.times), 2), 0)
            inside = inside.reshape(len(objects), SAMPLES, -1)-objects[:, None, None, 4]
            log = np.where(np.min(inside, axis=-1) < 0., -BETA, 0.)
            log[:, 0] = 0.   # keep the tracked walk itself
        return clouds, log

    def _collisions(self, paths, clouds, objects):
        """Expected-collision matrix per person (m, robot samples, person samples): the worst
        discounted overlap of the comfort disc, plus body overlap weighted HARD."""
        c = self.config
        weight = np.exp(-self.times[::STRIDE]/RISK_TIME)
        body = c.robot_radius+objects[:, 4]
        comfort = body+COMFORT_GAP
        d = np.linalg.norm(paths[None, :, None, ::STRIDE, :]-clouds[:, None, :, ::STRIDE, :], axis=-1)
        soft = (np.maximum(0., comfort[:, None, None, None]-d)/COMFORT_GAP)**2
        hard = HARD*np.maximum(0., body[:, None, None, None]-d)
        return np.max((soft+hard)*weight, axis=-1)

    def _reactivity(self, objects):
        ids = self.object_ids if len(self.object_ids) == len(objects) else np.zeros(len(objects), int)
        out = np.empty(len(objects))
        for k, ident in enumerate(ids):
            if self.stubborn.get(int(ident), 0) >= 3:
                out[k] = 0.
            else:
                out[k] = max(REACT_PRIOR, float(objects[k, 6]) if objects.shape[1] > 6 else 0.)
        return out

    # ----- the game -----
    @staticmethod
    def _normalise(log):
        top = np.max(log, axis=-1, keepdims=True)
        top = np.where(np.isfinite(top), top, 0.)
        w = np.exp(log-top)
        total = np.sum(w, axis=-1, keepdims=True)
        return np.where(total > 0, w/np.maximum(total, 1e-300), 1./log.shape[-1])

    def _equilibrium(self, robot_log, people_log, cost, react):
        """Best-response rounds to the mixed equilibrium: robot weights (n,), people (m, s)."""
        robot = self._normalise(robot_log)
        people = self._normalise(people_log)
        history = []
        for _ in range(ROUNDS):
            expected = np.einsum('mrs,ms->r', cost, people)          # robot vs every person
            robot = self._normalise(robot_log-BETA*expected)
            facing = np.einsum('mrs,r->ms', cost, robot)             # each person vs the robot
            people = self._normalise(people_log-BETA*react[:, None]*facing)
            history.append(float(robot @ expected))
        return robot, people, history

    def plan(self, pose, velocity, desired, tracks, now, goal=None):
        c = self.config
        prepared = self._prepare(pose, velocity, desired, tracks, now, goal)
        if isinstance(prepared, Result):
            return prepared
        pose, velocity, desired, objects, geometry, _ = prepared
        self.urgent = False
        if len(objects):
            far = np.linalg.norm(objects[:, :2]-pose[:2], axis=1) > ENGAGE
            order = np.argsort(np.linalg.norm(objects[:, :2]-pose[:2], axis=1))
            players = np.zeros(len(objects), bool)
            players[order[:PLAYERS]] = True
            players &= ~far
        else:
            players = np.zeros(0, bool)
        seeds = self._robot_seeds(desired)
        paths, firsts = self._rollout(seeds, pose, velocity)
        robot_log = self._robot_prior(seeds, paths, desired)
        if not np.isfinite(robot_log).any():
            robot_log = np.zeros(len(seeds))   # all through walls: let safety pick the least bad
        playing = objects[players]
        clouds, people_log = self._people(playing)
        react = self._reactivity(playing)
        if len(playing):
            cost = self._collisions(paths, clouds, playing)
        else:
            cost = np.zeros((0, len(seeds), SAMPLES))
        # Non-playing objects (far or many): their straight-line forecast as a fixed cost.
        rest = objects[~players] if len(objects) else objects
        if len(rest):
            fixed = self._collisions(paths, rest[:, None, None, :2]+rest[:, None, None, 2:4]
                                     * self.times[None, None, :, None], rest)[:, :, 0]
            robot_log = robot_log-BETA*fixed.sum(axis=0)
        robot, people, history = self._equilibrium(robot_log, people_log, cost, react)
        # The equilibrium's most likely manoeuvre, averaged with the requests near it (a mode,
        # not the mean of two passing sides).
        safety = self._safe(paths, objects)
        # Plan B: a trajectory with about the same first request that stays clear of the
        # straight-line forecasts for CONTINGENCY_TIME, keeping the comfort distance if it can,
        # else the body. Tiers: most likely request with a comfort plan B; with a body plan B;
        # safe for SAFE_TIME; else the one whose plan B comes closest to clear.
        same = np.linalg.norm(seeds[:, None, 0]-seeds[None, :, 0], axis=-1) < MODE_RADIUS
        comfort_b = self._safe(paths, objects, CONTINGENCY_TIME, COMFORT_GAP)
        body_b = self._safe(paths, objects, CONTINGENCY_TIME)
        best_b = np.max(np.where(same, body_b[None, :], -np.inf), axis=1)
        order = np.argsort(-robot)
        chosen, solver = None, 'game'
        for covered, name in ((np.any(same & (comfort_b[None, :] >= 0.), axis=1), 'game'),
                              (best_b >= 0., 'game_body_plan_b'),
                              (np.ones(len(seeds), bool), 'game_no_plan_b')):
            chosen = next((int(k) for k in order if safety[k] >= 0. and covered[k]), None)
            if chosen is not None:
                solver = name
                break
        if chosen is None:
            chosen = int(np.argmax(np.minimum(best_b, safety+1.)))
            solver = 'game_least_intrusive'
        near = np.linalg.norm(seeds[:, 0]-seeds[chosen, 0], axis=-1) < MODE_RADIUS
        near &= (safety >= min(0., safety[chosen])) & (robot >= .25*robot[chosen])
        near[chosen] = True
        mean = np.sum(robot[near, None, None]*seeds[near], axis=0)/max(float(robot[near].sum()), 1e-12)
        mean_paths, mean_first = self._rollout(mean[None], pose, velocity)
        mean_safe = self._safe(mean_paths, objects)[0]
        if mean_safe >= min(0., safety[chosen]):
            request, command, clearance = mean, mean_first[0], mean_safe
        else:
            request, command, clearance = seeds[chosen], firsts[chosen], safety[chosen]
        expected = float(history[-1]) if history else 0.
        self.equilibrium = dict(players=int(len(playing)), rounds=len(history), expected=expected,
                                react=[round(float(r), 2) for r in react])
        command = tuple(float(v) for v in command)
        self.previous[:] = command
        alert = clearance < 0. or expected > .5
        deviation = float(np.linalg.norm(np.asarray(command)[:2]-desired[:2]))
        if np.linalg.norm(command[:2]) < 1e-3 and np.linalg.norm(desired[:2]) > .06:
            mode = 'blocked'
        elif deviation < .05:
            mode = 'follow'
        elif (float(np.asarray(command)[:2] @ desired[:2]) >= 0 and
              abs(command[0]*desired[1]-command[1]*desired[0]) <= .04*max(np.linalg.norm(desired[:2]), .01)):
            mode = 'yield'
        else:
            mode = 'evade'
        return Result(command, mode, alert, float(clearance), solver)

    def _safe(self, paths, objects, horizon=SAFE_TIME, gap=0.):
        """Body clearance (m, negative inside) of each path from every object's straight-line
        forecast over horizon (s), less gap (m), and from walls over the whole plan."""
        c = self.config
        out = np.full(len(paths), np.inf)
        steps = int(np.searchsorted(self.times, horizon))+1
        if len(objects):
            future = objects[None, :, :2]+self.times[:steps, None, None]*objects[None, :, 2:4]
            d = np.linalg.norm(paths[:, :steps, None, :]-future[None], axis=-1)-objects[None, None, :, 4]
            out = np.minimum(out, np.min(d, axis=(1, 2))-c.robot_radius-gap)
        if self.field is not None:
            out = np.minimum(out, np.min(self._field_at(paths, 0), axis=1)-c.robot_radius)
        return out
