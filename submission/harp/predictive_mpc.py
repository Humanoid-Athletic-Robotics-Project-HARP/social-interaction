"""Bounded nonlinear MPC for generic tracked obstacles (no human-response model).

Velocity blocks and nonlinear body-frame rollout follow the design documented in
SFM_NMPC_REFERENCE.md. The implementation is independent of its Nav2 plugin.
All collision checks use a circular whole-body envelope, response delay and a
braking tail. Feasibility is conditional on these configured motion assumptions.
"""
from dataclasses import dataclass
import math
import time

import numpy as np
from scipy.optimize import minimize

CONTROL_PERIOD = 0.2


@dataclass(frozen=True)
class Config:
    period: float = CONTROL_PERIOD
    dt: float = 0.1
    horizon: float = 4.0
    response_delay: float = 0.2
    speed: float = 0.3
    yaw_speed: float = 0.6
    acceleration: float = 0.4
    yaw_acceleration: float = 1.0
    robot_radius: float = 0.4
    margin: float = 0.07
    uncertainty_rate: float = 0.025
    max_tracks: int = 64
    solve_seconds: float = 0.07
    solver_iterations: int = 8
    tracking_gain: float = 1.0
    lateral_tracking_gain: float = 1.0
    max_collision_uncertainty: float = .15


@dataclass(frozen=True)
class Result:
    command: tuple
    mode: str
    alert: bool
    clearance: float
    solver: str = "not_needed"


class Planner:
    """plan accepts body velocities, an odom pose, and tracks predicted to now."""
    def __init__(self, config=None):
        self.config = config or Config()
        c = self.config
        vals = [c.period, c.dt, c.horizon, c.speed, c.yaw_speed, c.acceleration,
                c.yaw_acceleration, c.robot_radius, c.margin, c.solve_seconds,c.max_collision_uncertainty]
        if not all(math.isfinite(v) and v > 0 for v in vals):
            raise ValueError("Planner limits must be finite and positive")
        if not (0 <= c.response_delay <= 1 and 0 <= c.uncertainty_rate <= 1):
            raise ValueError("Invalid delay/uncertainty growth")
        if not .5 <= c.tracking_gain <= 1.:
            raise ValueError('Invalid velocity tracking gain')
        if not .3 <= c.lateral_tracking_gain <= 1.:
            raise ValueError('Invalid lateral velocity tracking gain')
        self.limits = np.array([c.speed, min(c.speed,.4), c.yaw_speed])
        self.accel = np.array([c.acceleration, c.acceleration, c.yaw_acceleration])
        self.steps = int(math.ceil(c.horizon / c.dt))
        self.delay_steps = int(math.ceil(c.response_delay / c.dt))
        # A full stop plus one standing sample, for even a maximum diagonal request.
        self.tail_steps = int(math.ceil(max(c.speed / c.acceleration,
                                          c.yaw_speed / c.yaw_acceleration) / c.dt)) + 2
        self.times = np.arange(1 + self.delay_steps + self.steps + self.tail_steps) * c.dt
        self.previous = np.zeros(3)
        self.escape_direction = None
        self.initialized = False
        self.goal = None
        self.urgent = False

    def _blocked(self, clearance=-math.inf, reason="invalid_input"):
        self.previous[:] = 0
        return Result((0.0, 0.0, 0.0), "blocked", True, float(clearance), reason)

    @staticmethod
    def _linear_limit(values, limit):
        values = np.array(values, dtype=float, copy=True)
        norms = np.linalg.norm(values[..., :2], axis=-1, keepdims=True)
        values[..., :2] *= np.minimum(1.0, limit / np.maximum(norms, 1e-12))
        return values

    def _rollout(self, controls, pose, velocity, brake_after=None):
        """Vectorized acceleration-limited SE(2) dynamics, then a stopping tail."""
        c = self.config
        controls = self._linear_limit(np.asarray(controls, dtype=float).reshape(-1, 3, 3), c.speed)
        controls = np.clip(controls,-self.limits,self.limits)
        n = len(controls)
        poses = np.tile(pose, (n, 1))
        speeds = np.tile(velocity, (n, 1))
        result = np.empty((n, len(self.times), 2))
        result[:, 0] = poses[:, :2]
        # Ramp the request from the preceding request. Using measured velocity
        # here traps a walker at its tiny launch request while its gait starts.
        change = np.clip(controls[:, 0] - self.previous, -self.accel*c.period, self.accel*c.period)
        change = self._linear_limit(change, c.acceleration*c.period)
        first = self._linear_limit(np.clip(change + self.previous, -self.limits, self.limits), c.speed)
        if self.urgent:
            # GR00T receives a velocity target, not an instantaneous physical
            # velocity. For an incoming collision, do not add a second slow
            # request ramp on top of the delayed acceleration-limited plant.
            first = controls[:,0].copy()
        request = first.copy()
        next_request_time = c.period
        for step in range(1, len(self.times)):
            moving_step = step - self.delay_steps - 1
            if step <= self.delay_steps:
                target = velocity
            elif moving_step < self.steps:
                # Only the first request executes before the next replan.
                block = min(2, moving_step * 3 // self.steps)
                if moving_step*c.dt+1e-9 >= next_request_time:
                    if self.urgent:
                        request = controls[:,block].copy()
                    else:
                        change = np.clip(controls[:,block]-request,
                                         -self.accel*c.period,self.accel*c.period)
                        request += self._linear_limit(change,c.acceleration*c.period)
                    next_request_time += c.period
                target = request
            else:
                target = np.zeros(3)
            if brake_after is not None and moving_step*c.dt >= brake_after:
                target = np.zeros(3)
            target = np.broadcast_to(target,(n,3)).copy()
            if self.goal is not None and step > self.delay_steps:
                offset = self.goal-poses[:,:2]
                distance = np.linalg.norm(offset,axis=-1)
                co,si = np.cos(poses[:,2]),np.sin(poses[:,2])
                toward = ((co*target[:,0]-si*target[:,1])*offset[:,0]
                          +(si*target[:,0]+co*target[:,1])*offset[:,1]) > 0
                norm = np.linalg.norm(target[:,:2],axis=-1)
                scale = np.minimum(1.,.5*np.maximum(0.,distance-.1)/np.maximum(norm,1e-9))
                target[toward,:2] *= scale[toward,None]
            if step > self.delay_steps:
                target[..., 0] *= c.tracking_gain
                target[..., 1] *= c.lateral_tracking_gain
            change = np.clip(target-speeds, -self.accel*c.dt, self.accel*c.dt)
            speeds += self._linear_limit(change, c.acceleration*c.dt)
            mid_yaw = poses[:, 2] + speeds[:, 2]*c.dt/2
            co, si = np.cos(mid_yaw), np.sin(mid_yaw)
            poses[:, 0] += (co*speeds[:, 0] - si*speeds[:, 1])*c.dt
            poses[:, 1] += (si*speeds[:, 0] + co*speeds[:, 1])*c.dt
            poses[:, 2] += speeds[:, 2]*c.dt
            result[:, step] = poses[:, :2]
        return result, first

    @staticmethod
    def _approach(paths, objects, times):
        """Closest approach during each interval, not only at time samples."""
        future = objects[None, :, :2] + times[:, None, None]*objects[None, :, 2:4]
        relative = paths[:, :, None, :] - future[None, :, :, :]
        a, delta = relative[:, :-1], np.diff(relative, axis=1)
        fraction = np.clip(-np.sum(a*delta, axis=-1) /
                           np.maximum(np.sum(delta*delta, axis=-1), 1e-12), 0, 1)
        return np.linalg.norm(a + fraction[..., None]*delta, axis=-1), relative

    def _clearances(self, paths, objects, time_limit=None, recover_margin=False):
        if not len(objects):
            return np.full(len(paths), math.inf)
        c = self.config
        times = self.times
        if time_limit is not None:
            count = max(2,int(time_limit/c.dt)+1)
            paths, times = paths[:,:count], times[:count]
        distance, relative = self._approach(paths, objects, times)
        # Bound curvature/acceleration interpolation error between samples.
        interpolation = (c.acceleration + math.sqrt(2)*c.speed*c.yaw_speed)*c.dt*c.dt
        required = (c.robot_radius + c.margin + interpolation + objects[None, :, 4]
                    + np.minimum(objects[None, :, 5],c.max_collision_uncertainty)
                    + c.uncertainty_rate*times[1:, None])
        required = required[None,:,:]
        if recover_margin:
            # A noisy track can already intrude into the extra forecast margin.
            # Requiring that margin at t=0 makes every escape impossible. Permit
            # only this pre-existing debt to decay after the response interval;
            # the body/observed-shape envelope remains hard throughout.
            physical = c.robot_radius+interpolation+objects[:,4]
            initial_distance = np.linalg.norm(relative[:,0],axis=-1)
            comfort = physical+c.margin+np.minimum(objects[:,5],c.max_collision_uncertainty)
            debt = np.maximum(0.,comfort[None,:]-initial_distance)
            debt[initial_distance < physical[None,:]+.02] = 0.
            decay = np.maximum(0.,1.-np.maximum(0.,times[1:]-c.response_delay-c.period)/1.2)
            allowance = np.where(debt[:,None,:] > 0,
                (debt[:,None,:]+c.uncertainty_rate*times[None,1:,None])*decay[None,:,None],0.)
            required = np.maximum(physical[None,None,:],required-allowance)
        clearance = np.min(distance - required, axis=(1, 2))
        if time_limit is not None:
            return clearance
        # Replan while travelling: do not reserve the stopped pose against
        # speculative pedestrian motion beyond the checked horizon/braking tail.
        return clearance

    def _costs(self, controls, desired, paths=None):
        controls = np.asarray(controls).reshape(-1, 3, 3)
        # Prefer progress and the requested route. Stopping is a candidate, not
        # a privileged action; sideways passing competes with slowing down.
        error = np.mean(np.sum((controls-desired)**2, axis=-1), axis=-1)
        continuity = np.sum((controls[:, 0]-self.previous)**2, axis=-1)
        backwards = np.maximum(0, -controls[:, 0, :2] @ desired[:2])
        direction_cost = np.zeros(len(controls))
        if self.escape_direction is not None:
            direction = controls[:, 0, :2]
            direction = direction / np.maximum(np.linalg.norm(direction, axis=-1, keepdims=True), 1e-9)
            direction_cost = .012 * (1-direction @ self.escape_direction)
        requested = float(np.linalg.norm(desired[:2]))
        stall = (.15*(1-np.minimum(1.,np.linalg.norm(controls[:,0,:2],axis=-1)/requested))
                 if requested > .06 else 0.)
        progress = 0.
        if paths is not None and self.goal is not None:
            # Velocity agreement alone can reward a curve that ends up farther
            # from the goal. Score actual SE(2) rollout progress as well.
            distances = np.linalg.norm(paths-self.goal[None,None,:],axis=-1)
            progress = .25*(distances[:,-1]-distances[:,0])
        return error + .12*continuity + 2*backwards + direction_cost + stall + progress

    def _urgent_risk(self, paths, objects):
        """An imminent collision outranks a speculative deeper overlap far away."""
        if not len(objects):
            return np.zeros(len(paths))
        c = self.config
        future = objects[None,:,:2]+self.times[:,None,None]*objects[None,:,2:4]
        distance = np.linalg.norm(paths[:,:,None,:]-future[None],axis=-1)
        envelope = c.robot_radius+c.margin+objects[:,4]+np.minimum(objects[:,5],c.max_collision_uncertainty)+.2
        overlap = np.maximum(0.,1.-distance/envelope[None,None,:])
        return 10.*np.max(overlap**2*np.exp(-self.times/1.2)[None,:,None],axis=(1,2))

    def _intrusion(self, paths, objects):
        """Time-discounted envelope intrusion of each trajectory, integrated over time.

        Used only when nothing is certifiably clear. Standing is scored like any
        other trajectory: freezing in front of an incoming object is predicted
        to be hit, so it is not a safe default. Entering the body/observed-shape
        envelope dominates entering the comfort margins, and imminent intrusion
        dominates a later constant-velocity overlap that replanning can revisit.
        Integrating (not the worst instant) means an unavoidable present overlap
        cannot hide the difference between leaving it and staying or deepening it.
        """
        c = self.config
        distance, _ = self._approach(paths, objects, self.times)
        times = self.times[1:, None]
        physical = c.robot_radius + objects[:, 4]
        comfort = (physical + c.margin + np.minimum(objects[:, 5], c.max_collision_uncertainty)
                   + c.uncertainty_rate*times)
        intrusion = ((10.*np.maximum(0., physical-distance) + np.maximum(0., comfort-distance))
                     * np.exp(-times/1.5)[None])
        return np.sum(intrusion, axis=(1, 2))*c.dt

    def _evaluate(self, controls, pose, velocity, objects, recover_margin=False):
        paths, commands = self._rollout(controls, pose, velocity)
        return self._clearances(paths, objects,recover_margin=recover_margin), commands

    def plan(self, pose, velocity, desired, tracks, now, goal=None):
        c = self.config
        try:
            self.goal = None if goal is None else np.asarray(goal,dtype=float)
            if self.goal is not None and (self.goal.shape != (2,) or not np.isfinite(self.goal).all()):
                return self._blocked(reason='invalid_goal')
            pose, velocity, desired = (np.asarray(v, dtype=float) for v in (pose, velocity, desired))
            if any(v.shape != (3,) or not np.isfinite(v).all() for v in (pose, velocity, desired)):
                return self._blocked()
            if not math.isfinite(now) or np.any(np.abs(velocity) > self.limits + .15):
                return self._blocked(reason="invalid_velocity")
            tracks = list(tracks)
            if len(tracks) > c.max_tracks:
                return self._blocked(reason="track_capacity")
            objects = np.asarray([[t.x, t.y, t.vx, t.vy, t.radius, t.uncertainty]
                                  for t in tracks], dtype=float).reshape(-1, 6)
            if (not np.isfinite(objects).all() or np.any(objects[:, 4:] < 0)
                    or any(not math.isfinite(t.last_seen) or t.last_seen > now+.05 for t in tracks)):
                return self._blocked()
        except (TypeError, ValueError, AttributeError):
            return self._blocked()
        desired = self._linear_limit(np.clip(desired, -self.limits, self.limits), c.speed)
        if not self.initialized:
            self.previous[:] = self._linear_limit(velocity,c.speed)
            self.initialized = True
        self.urgent = False
        if len(objects):
            stopped,_ = self._rollout(np.zeros((3,3)),pose,velocity,brake_after=0.)
            self.urgent = bool(self._clearances(stopped,objects,time_limit=2.5)[0] < 0.)
        nominal = np.tile(desired, (3, 1))
        clear, commands = self._evaluate(nominal, pose, velocity, objects)
        if clear[0] >= .12 or not len(objects):
            command = tuple(float(v) for v in commands[0])
            self.previous[:] = command
            self.escape_direction = None
            return Result(command, "follow", False, float(clear[0]))

        angles = np.arange(24)*2*math.pi/24
        seeds = [np.zeros((3, 3))]
        for speed in (c.speed, .7*c.speed, .4*c.speed):
            for angle in angles:
                candidate = np.array([speed*math.cos(angle),speed*math.sin(angle),0.])
                seeds.append(np.tile(candidate,(3,1)))
                seeds.append(np.array([candidate,candidate,desired]))
                seeds.append(np.array([candidate,desired,desired]))
            for yaw in (-c.yaw_speed,-.5*c.yaw_speed,.5*c.yaw_speed,c.yaw_speed):
                curve = np.array([speed,0.,yaw])
                seeds.append(np.array([curve,curve,desired]))
        seeds = np.asarray(seeds)
        seeds = self._linear_limit(np.clip(seeds,-self.limits,self.limits),c.speed)
        clearance, _ = self._evaluate(seeds, pose, velocity, objects)
        valid = np.flatnonzero(clearance >= 0)
        recover_margin = False
        if not len(valid):
            relaxed,_ = self._evaluate(seeds,pose,velocity,objects,recover_margin=True)
            valid = np.flatnonzero(relaxed >= 0)
            if len(valid):
                clearance, recover_margin = relaxed, True
        if not len(valid):
            # A distant forecast can reject every constant-velocity continuation.
            # Evaluate a short action followed by braking, rather than assume
            # standing is safest. It still needs a clear swept stopping path.
            braking = seeds.copy()
            braking[:,1:] = 0.
            paths, commands = self._rollout(braking,pose,velocity,brake_after=c.period)
            stopping_time = (c.response_delay+c.period+
                max(max(c.speed,float(np.linalg.norm(velocity[:2])))/c.acceleration,
                    max(c.yaw_speed,abs(float(velocity[2])))/c.yaw_acceleration)+2*c.dt)
            near = self._clearances(paths,objects,time_limit=stopping_time,recover_margin=True)
            safe = np.flatnonzero(near >= 0)
            continuations,firsts = self._rollout(seeds,pose,velocity)
            if not len(safe):
                # Seed 0 is standing. Returning it unconditionally here froze the
                # robot while non-yielding people walked into it. Choose the least
                # intrusive trajectory instead; it is standing only if that wins.
                intrusion = self._intrusion(continuations,objects)
                costs = intrusion + .1*self._costs(seeds,desired,continuations)
                chosen = int(np.argmin(costs))
                command = firsts[chosen]
                # Move only for a material reduction, so forecast noise does not
                # dither between standing and stepping when every option is bad.
                if (chosen == 0 or np.linalg.norm(command) < 1e-6
                        or intrusion[chosen] > .9*intrusion[0]):
                    return self._blocked(float(np.max(clearance)), 'no_feasible_escape')
                self.previous[:] = command
                speed = float(np.linalg.norm(seeds[chosen,0,:2]))
                if speed > .06:
                    self.escape_direction = seeds[chosen,0,:2]/speed
                return Result(tuple(float(v) for v in command),'evade',True,
                              float(clearance[chosen]),'least_intrusion')
            # Do not ignore the forecast that rejected the long continuation:
            # otherwise a greedy short step walks toward an incoming actor until
            # escape becomes impossible. Prefer the least long-horizon risk.
            costs = self._costs(seeds,desired,continuations)-.08*np.minimum(near,1.)+self._urgent_risk(continuations,objects)
            chosen = safe[np.argmin(costs[safe])]
            command = commands[chosen]
            self.previous[:] = command
            return Result(tuple(float(v) for v in command),'evade',True,
                          float(near[chosen]),'braking_recovery')
        # Prefer usable clearance over a mathematically feasible trajectory that
        # disappears after a centimetre of sensor or gait error.
        paths,_ = self._rollout(seeds,pose,velocity)
        costs = self._costs(seeds, desired,paths) + .05/(.1+np.maximum(clearance,0))
        index = valid[np.argmin(costs[valid])]
        best, best_cost = seeds[index].copy(), costs[index]
        solver = 'feasible_seed'
        deadline = time.monotonic()+c.solve_seconds
        def check_time():
            if time.monotonic() > deadline:
                raise TimeoutError('MPC refinement budget')
        def objective(flat):
            check_time()
            paths,_ = self._rollout(flat.reshape(3,3),pose,velocity)
            near = self._clearances(paths,objects,recover_margin=recover_margin)[0]
            return float(self._costs(flat.reshape(3,3), desired,paths)[0] + .05/(.1+max(near,0.)))
        def constraint(flat):
            check_time()
            return float(self._evaluate(flat.reshape(3,3), pose, velocity, objects,recover_margin)[0][0])
        try:
            solution = minimize(objective, best.ravel(), method='SLSQP',
                                bounds=[(-v,v) for _ in range(3) for v in self.limits],
                                constraints={'type':'ineq', 'fun':constraint},
                                options={'maxiter':c.solver_iterations,'ftol':1e-4})
            candidate = self._linear_limit(solution.x.reshape(3,3),c.speed)
            candidate_clear, _ = self._evaluate(candidate, pose, velocity, objects,recover_margin)
            if (solution.success and np.isfinite(candidate).all()
                    and candidate_clear[0] >= 0 and objective(solution.x) <= best_cost):
                best, solver = candidate, 'slsqp'
        except (TimeoutError, ValueError, FloatingPointError):
            solver = 'feasible_seed_budget'
        final_clear, command = self._evaluate(best, pose, velocity, objects,recover_margin)
        self.previous[:] = command[0]
        direction = best[0,:2]
        speed = float(np.linalg.norm(direction))
        if speed > .06:
            self.escape_direction = direction/speed
        # A large change of direction is passing; a near-route slower command yields.
        cross = abs(direction[0]*desired[1]-direction[1]*desired[0])
        evade = cross > .04*max(np.linalg.norm(desired[:2]), .01)
        mode = 'evade' if evade else 'yield'
        return Result(tuple(float(v) for v in command[0]), mode, recover_margin,
                      float(final_clear[0]), ('margin_recovery:' if recover_margin else '')+solver)
