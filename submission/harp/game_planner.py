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
import time

import numpy as np

from harp.predictive_mpc import Planner, Result

SAMPLES = 32             # trajectories per person
PLAYERS = 8              # nearest moving objects that play; the rest are fixed forecasts
ENGAGE = 5.0             # m: farther objects are fixed forecasts
ROUNDS = 6               # rounds of best responses (converged in tests by about 4)
BETA = 6.                # weight of an expected collision against the prior (nats per unit)
REACT_PRIOR = .6         # how much someone not yet seen either way is expected to play
RISK_ALPHA = .3          # the robot weighs the worst RISK_ALPHA share of each person's likely
                         # motions (CVaR): 1 is the plain average, small is near the worst case.
                         # Benchmark (90 worlds, aware): passed every check 17 / 19 / 26 / 21 /
                         # 21 at 1 / 0.5 / 0.3 / 0.2 / 0.15 (the MPC: 27, at 46% more time)
COMFORT_GAP = .40        # m past the robot's 0.40 m disc: 0.445 m from its arms (0.355 m), so a
                         # pass keeps the 0.30 m rule after tracking and response error. Planned
                         # at the rule's edge (0.26), many real passes fell just under (benchmark,
                         # 90 map-less worlds, aware: passed every check 15 / 17 / 21 at 0.26 /
                         # 0.33 / 0.40, the MPC 21; walkers who never react unchanged)
HARD = 4.                # collision cost of body overlap per metre, beyond the comfort part
RULE_GAP = .255          # m past the 0.40 m disc: 0.30 m from the arms, the clearance rule
WALL_RULE = 3.3          # the same per metre for walls and fixed objects (the robot's prior): 90
                         # map-less worlds, passed every check with walls 9 / 17 / 20 / 21 and
                         # reached 85 / 83 / 83 / 78 at 0 / 1 / 3.3 / 10 (the MPC: 6, 80); doors under
                         # 1.3 m leave no way to keep 0.30 m from both sides, and 10 made it hesitate
RULE = 20.               # cost per metre inside RULE_GAP (linear): the quadratic comfort part
                         # made a pass just under the rule cheaper than a slow G1 sidestep, and
                         # in 30 of 90 runs a walker passed under 0.20 m while the robot kept on.
                         # 20, not 10, with the speed-scaled prior: benchmark at 0.3 m/s, 63
                         # worlds, passed every check 2 / 4 / 2 and reached 57 / 60 / 57 at 10 /
                         # 20 / 30; at 0.5 m/s, 93 worlds, passed 20 either way, reached 85 / 87
RISK_TIME = 2.           # s: later predicted overlaps count less (prediction grows unsure)
SAFE_TIME = 1.2          # s nobody is assumed to react in (the MPC's YIELD_DELAY). The robot's stop
                         # from 0.5 m/s takes about 1.45 s; the benchmark (73 worlds) favoured 1.45
                         # a little (reached 70 against 67; 1.7: 65), HIM didn't confirm it (aware,
                         # 0.5 m/s: 3 of 8 goals at 1.45 s; 1.2 s 6 of 8, then 2 of 8 the next round)
CONTINGENCY_TIME = 2.5   # s a plan B must stay clear for if people don't react at all
STILL_SPEED = .15        # m/s: slower tracks count as standing in the hard check (velocity noise)
TIE = .02                # m: violations this close count as equal when nothing is safe
PERSON_SPREAD = (.35, .25)   # m/s velocity change and m/s^2 turn spread of a person's cloud
REACTION = .5            # s before a person's change of pace or direction can start: without it
                         # the equilibrium had a close walker sidestep at once, so the robot
                         # kept walking in (benchmark: it closed 0.18 m of the last 2 s before
                         # its closest passes, the MPC 0.07 m) and passes came closer
PRIOR_TEMPERATURE = .08  # request cost (m/s)^2 per nat of robot prior: 2σ², σ = 0.2 m/s at
                         # REFERENCE_SPEED, scaled with the speed cap squared (σ 0.12 m/s at 0.3):
                         # unscaled, at 0.3 m/s σ spanned most of the robot's speeds, and the
                         # benchmark's robot stood 17 s a run and reached 48 of 63 goals (scaled:
                         # 10 s, 57; the MPC: 12 s, 52)
REFERENCE_SPEED = .5     # m/s: the speed cap the weights were set at
STUBBORN_PLAYERS = False # players' own ids for the stubborn rule (HARP_PREDICTIVE_STUBBORN_PLAYERS=1).
                         # Off: with anyone not playing the ids don't line up, so stubborn walkers
                         # keep rho >= .6, as everything was tuned and tested. On: benchmark, 0.5 m/s,
                         # 73 worlds, walkers who give way passed every check 16 -> 20, kept 0.30 m
                         # 21 -> 29; walkers who never react reached 40 -> 35; HIM (aware, 8 runs
                         # each) reached 1 goal against the default's 2: no gain shows there.
BUDGET = .08             # s: plans slower than this on average step down a LEVEL (the filter
                         # stops the robot at 0.15 s; PC2's cores are about 2.5-3x slower than a
                         # Mac's, where plans take 36 ms typically, 49 ms at the 95th percentile)
LEVELS = ((32, 6), (16, 4), (8, 3))   # samples per person and rounds, full to cheapest: 95th
                         # percentile 62 / 39 ms on a Mac. Benchmark at 0.3 m/s, 63 worlds, plans
                         # counted 2.75x slower: reached 58 with the steps down, 49 without (the
                         # filter stood the robot on 16,163 late plans, 54 s a run)
LITE = LEVELS[1]
JUDGE = 3                # plans at a level before its average decides a step
SIDEWAYS = 3.            # a sideways request costs this much more than forward (slow sidestep)
TURNING = .3             # turn-rate deviation weight (a round footprint doesn't turn)
SWITCHING = 1.           # cost of changing the request from the last one, per (m/s)^2
MODE_RADIUS = .12        # m/s: requests this close to the chosen one are averaged with it
KEEP_RIGHT = 1.5         # nats against passing someone met head-on on the wrong side: people
                         # keep right (HIM's walkers step right too), so the robot passing on its
                         # left met them on the same side, and passes came close
HEAD_ON = math.radians(135)   # walking at least this far from the robot's way counts as head-on
STRIDE = 2               # collision costs every 2nd rollout step (0.2 s), for planning time


class GamePlanner(Planner):
    """plan(pose, velocity, desired, tracks, now, goal): as Planner.plan, a game-theoretic core."""

    def __init__(self, config=None, seed=0):
        super().__init__(config)
        self.rng = np.random.default_rng(seed)
        self.stubborn = {}       # track id -> scans seen keeping course into a conflict
        self.equilibrium = {}    # last plan's diagnostics (players, rounds, expected collision)
        self.samples, self.rounds = SAMPLES, ROUNDS   # a cheaper LEVEL while plans run slow
        self.level = self.since = 0
        self.solve = 0.           # recent plan time, s (an average)
        self.time_scale = 1.      # sim tests: plan times count this much slower (PC2's cores)
        self.rule, self.speed_prior = RULE, True   # (10, False): the tuning before, for A/B runs
        self.stubborn_players = STUBBORN_PLAYERS

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
        scale = (self.config.speed/REFERENCE_SPEED)**2 if self.speed_prior else 1.
        log = -(track+switch)/(PRIOR_TEMPERATURE*scale)
        if self.field is not None:
            c = self.config
            wall = self._field_at(paths[:, 1:], 1, self.times[1:])-c.robot_radius
            body = self._field_at(paths, 0)
            log = log-BETA*np.max(np.maximum(0., c.margin-wall)/c.margin, axis=1)
            # Already too close (a standing person or a remembered object beside it): only
            # getting closer is ruled out, or every path is and the robot never moves again.
            limit = np.minimum(c.robot_radius, body[:, 0]-.01)
            log = np.where(np.min(body[:, 1:], axis=1) < limit, -np.inf, log)
            # The clearance rule counts walls too: with only the 7 cm margin the arms passed
            # walls at about 0.12 m (benchmark: 53 of 90 runs under 0.30 m from a wall).
            inside = np.max(np.maximum(0., c.robot_radius+RULE_GAP-body[:, 1:]), axis=1)
            log = log-BETA*WALL_RULE*inside
        return log

    def _keep_right(self, paths, objects, pose):
        """Log-prior penalty (n,) of robot paths that would pass someone met head-on with them
        on its right: at closest approach to their straight walk, they should be on its left."""
        penalty = np.zeros(len(paths))
        heading = np.array([math.cos(pose[2]), math.sin(pose[2])])
        for o in objects:
            v = o[2:4]
            speed = float(np.linalg.norm(v))
            offset = o[:2]-pose[:2]
            if (speed < .3 or float(np.linalg.norm(offset)) > ENGAGE or float(offset @ heading) <= 0
                    or float(v @ heading) > math.cos(HEAD_ON)*speed):
                continue
            walk = o[None, :2]+self.times[:, None]*v[None]                  # (steps, 2)
            gap = walk[None]-paths                                         # (n, steps, 2)
            k = np.argmin(np.einsum('nsk,nsk->ns', gap, gap), axis=1)
            rel = gap[np.arange(len(paths)), k]
            moving = np.diff(paths, axis=1, append=paths[:, -1:])[np.arange(len(paths)), k]
            moving = np.where(np.linalg.norm(moving, axis=1, keepdims=True) > 1e-6, moving, heading[None])
            right = moving[:, 0]*rel[:, 1]-moving[:, 1]*rel[:, 0] < 0       # person on the robot's right
            penalty += KEEP_RIGHT*right
        return penalty

    def _people(self, objects):
        """Each moving object's cloud (m, SAMPLES, steps, 2): its tracked walk, with one
        unchanged sample and the rest changing pace and direction smoothly after REACTION;
        log prior weights (m, SAMPLES): off walls."""
        t = self.times[None, None, :, None]
        late = np.maximum(0., t-REACTION)
        dv = self.rng.normal(0., PERSON_SPREAD[0], (len(objects), self.samples, 1, 2))
        da = self.rng.normal(0., PERSON_SPREAD[1], (len(objects), self.samples, 1, 2))
        dv[:, 0], da[:, 0] = 0., 0.
        base = objects[:, None, None, :2]+objects[:, None, None, 2:4]*t
        clouds = base+dv*late+.5*da*late*late
        log = np.zeros((len(objects), self.samples))
        if self.field is not None and len(objects):
            inside = self._field_at(clouds.reshape(-1, len(self.times), 2), 0)
            inside = inside.reshape(len(objects), self.samples, -1)-objects[:, None, None, 4]
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
        soft = soft+self.rule*np.maximum(0., (body+RULE_GAP)[:, None, None, None]-d)
        hard = HARD*np.maximum(0., body[:, None, None, None]-d)
        return np.max((soft+hard)*weight, axis=-1)

    def _reactivity(self, objects, ids=None):
        """Each object's reactivity; ids: their track ids (default: _prepare's, for all of them)."""
        if ids is None:
            ids = self.object_ids
        if len(ids) != len(objects):
            ids = np.zeros(len(objects), int)
        out = np.empty(len(objects))
        for k, ident in enumerate(ids):
            if self.stubborn.get(int(ident), 0) >= 3:
                out[k] = 0.
            else:
                out[k] = max(REACT_PRIOR, float(objects[k, 6]) if objects.shape[1] > 6 else 0.)
        return out

    @staticmethod
    def _mode_mean(robot, seeds, near, chosen):
        """Weighted mean of the candidates near the chosen one. The weights are scaled to their
        largest first: a chosen plan B far below the equilibrium's favourite has weights under
        1e-12, and dividing by that floor had shrunk the mean toward standing."""
        weights = robot[near]
        top = float(np.max(weights))
        if not top > 0.:
            return seeds[chosen]
        weights = weights/top
        return np.sum(weights[:, None, None]*seeds[near], axis=0)/float(weights.sum())

    # ----- the game -----
    @staticmethod
    def _normalise(log):
        top = np.max(log, axis=-1, keepdims=True)
        top = np.where(np.isfinite(top), top, 0.)
        w = np.exp(log-top)
        total = np.sum(w, axis=-1, keepdims=True)
        return np.where(total > 0, w/np.maximum(total, 1e-300), 1./log.shape[-1])

    @staticmethod
    def _cvar(cost, weights, alpha):
        """Per robot sample (n,), summed over people: the mean cost over the worst alpha share
        of each person's weighted samples (cost (m, n, s), weights (m, s)); alpha 1: the mean."""
        if alpha >= 1. or not cost.size:
            return np.einsum('mrs,ms->r', cost, weights)
        order = np.argsort(-cost, axis=2)                            # worst first
        c = np.take_along_axis(cost, order, axis=2)
        w = np.take_along_axis(np.broadcast_to(weights[:, None, :], cost.shape), order, axis=2)
        before = np.cumsum(w, axis=2)-w
        share = np.clip(alpha-before, 0., w)                         # mass of each in the tail
        return np.sum(np.sum(c*share, axis=2), axis=0)/alpha

    def _equilibrium(self, robot_log, people_log, cost, react):
        """Best-response rounds to the mixed equilibrium: robot weights (n,), people (m, s).
        The robot's best response weighs each person's CVaR (RISK_ALPHA), people theirs by
        expectation: a risk-sensitive robot in a game with risk-neutral people."""
        robot = self._normalise(robot_log)
        people = self._normalise(people_log)
        history = []
        for _ in range(self.rounds):
            expected = self._cvar(cost, people, RISK_ALPHA)          # robot vs every person
            robot = self._normalise(robot_log-BETA*expected)
            facing = np.einsum('mrs,r->ms', cost, robot)             # each person vs the robot
            people = self._normalise(people_log-BETA*react[:, None]*facing)
            history.append(float(robot @ expected))
        return robot, people, history

    def plan(self, pose, velocity, desired, tracks, now, goal=None):
        started = time.perf_counter()
        try:
            return self._play(pose, velocity, desired, tracks, now, goal)
        finally:
            self._budget((time.perf_counter()-started)*self.time_scale)

    def _budget(self, seconds):
        """One LEVEL cheaper while plans average over BUDGET, one dearer once they average under
        half of it; each level is judged on JUDGE plans of its own first."""
        self.solve = seconds if not self.since else .7*self.solve+.3*seconds
        self.since += 1
        if self.since < JUDGE:
            return
        step = 1 if self.solve > BUDGET else -1 if self.solve < .5*BUDGET else 0
        level = min(max(self.level+step, 0), len(LEVELS)-1)
        if level != self.level:
            self.level, self.since = level, 0
            self.samples, self.rounds = (SAMPLES, ROUNDS) if level == 0 else LEVELS[level]

    def _play(self, pose, velocity, desired, tracks, now, goal=None):
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
        if len(objects):
            robot_log = robot_log-self._keep_right(paths, objects, pose)
        playing = objects[players]
        clouds, people_log = self._people(playing)
        # object_ids lists every moving object; with any not playing (far, or past PLAYERS) only
        # the players' own ids recognise stubborn walkers (STUBBORN_PLAYERS).
        own = self.stubborn_players and len(self.object_ids) == len(objects)
        react = self._reactivity(playing, self.object_ids[players] if own else None)
        if len(playing):
            cost = self._collisions(paths, clouds, playing)
        else:
            cost = np.zeros((0, len(seeds), self.samples))
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
            # Nothing is safe: least violation first, then the equilibrium's choice among the
            # (near) ties. Taking the first of tied candidates had frozen the robot.
            worst = np.minimum(best_b, safety+1.)
            ties = np.flatnonzero(worst >= np.max(worst)-TIE)
            chosen = int(ties[np.argmax(robot[ties])])
            solver = 'game_least_intrusive'
        near = np.linalg.norm(seeds[:, 0]-seeds[chosen, 0], axis=-1) < MODE_RADIUS
        near &= (safety >= min(0., safety[chosen])) & (robot >= .25*robot[chosen])
        near[chosen] = True
        mean = self._mode_mean(robot, seeds, near, chosen)
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
        # Plain Python types: the filter writes these into JSON events (a numpy bool there
        # crashed the filter in HIM the first time an alert came, rounds 28-29).
        clearance = float(clearance)
        alert = bool(clearance < 0. or expected > .5)
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

    def _safe(self, paths, objects, horizon=None, gap=0.):
        """Body clearance (m, negative inside) of each path from every object's straight-line
        forecast over horizon (s, SAFE_TIME by default, read here so the benchmark's --set
        reaches it), less gap (m), and from walls over the whole plan."""
        horizon = SAFE_TIME if horizon is None else horizon
        c = self.config
        out = np.full(len(paths), np.inf)
        steps = int(np.searchsorted(self.times, horizon))+1
        # Only what this plan can change counts: every candidate carries on with the last
        # request through the response delay and one period, so judging that shared stretch
        # made a slight drift toward something near rule out every candidate alike.
        first = int(np.searchsorted(self.times, c.response_delay+c.period))
        # From inside the clearance already (HIM: a few cm, from a standing person or a
        # remembered object), only getting closer counts as unsafe: required at most the
        # distance it starts at. Requiring the full clearance froze the robot for good.
        if len(objects):
            velocity = objects[:, 2:4]*(np.linalg.norm(objects[:, 2:4], axis=1) >= STILL_SPEED)[:, None]
            future = objects[None, :, :2]+self.times[:steps, None, None]*velocity[None]
            d = np.linalg.norm(paths[:, :steps, None, :]-future[None], axis=-1)-objects[None, None, :, 4]
            need = np.minimum(c.robot_radius+gap, d[:, first, :]-.01)       # (paths, objects)
            out = np.minimum(out, np.min(np.min(d[:, first:], axis=1)-need, axis=1))
        if self.field is not None:
            d = self._field_at(paths, 0)
            out = np.minimum(out, np.min(d[:, first:], axis=1)-np.minimum(c.robot_radius, d[:, first]-.01))
        return out
