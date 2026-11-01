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
    margin: float = 0.10
    uncertainty_rate: float = 0.025
    max_tracks: int = 64
    solve_seconds: float = 0.07
    solver_iterations: int = 8


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
                c.yaw_acceleration, c.robot_radius, c.margin, c.solve_seconds]
        if not all(math.isfinite(v) and v > 0 for v in vals):
            raise ValueError("Planner limits must be finite and positive")
        if not (0 <= c.response_delay <= 1 and 0 <= c.uncertainty_rate <= 1):
            raise ValueError("Invalid delay/uncertainty growth")
        self.limits = np.array([c.speed, c.speed, c.yaw_speed])
        self.accel = np.array([c.acceleration, c.acceleration, c.yaw_acceleration])
        self.steps = int(math.ceil(c.horizon / c.dt))
        self.delay_steps = int(math.ceil(c.response_delay / c.dt))
        # A full stop plus one standing sample, for even a maximum diagonal request.
        self.tail_steps = int(math.ceil(max(c.speed / c.acceleration,
                                          c.yaw_speed / c.yaw_acceleration) / c.dt)) + 2
        self.times = np.arange(1 + self.delay_steps + self.steps + self.tail_steps) * c.dt
        self.previous = np.zeros(3)
        self.escape_direction = None

    def _blocked(self, clearance=-math.inf, reason="invalid_input"):
        self.previous[:] = 0
        return Result((0.0, 0.0, 0.0), "blocked", True, float(clearance), reason)

    @staticmethod
    def _linear_limit(values, limit):
        values = np.array(values, dtype=float, copy=True)
        norms = np.linalg.norm(values[..., :2], axis=-1, keepdims=True)
        values[..., :2] *= np.minimum(1.0, limit / np.maximum(norms, 1e-12))
        return values

    def _rollout(self, controls, pose, velocity):
        """Vectorized acceleration-limited SE(2) dynamics, then a stopping tail."""
        c = self.config
        controls = self._linear_limit(np.asarray(controls, dtype=float).reshape(-1, 3, 3), c.speed)
        n = len(controls)
        poses = np.tile(pose, (n, 1))
        speeds = np.tile(velocity, (n, 1))
        result = np.empty((n, len(self.times), 2))
        result[:, 0] = poses[:, :2]
        change = np.clip(controls[:, 0] - velocity, -self.accel*c.period, self.accel*c.period)
        change = self._linear_limit(change, c.acceleration*c.period)
        first = self._linear_limit(np.clip(change + velocity, -self.limits, self.limits), c.speed)
        for step in range(1, len(self.times)):
            moving_step = step - self.delay_steps - 1
            if step <= self.delay_steps:
                target = velocity
            elif moving_step < self.steps:
                # Only the first request executes before the next replan.
                block = min(2, moving_step * 3 // self.steps)
                target = first if moving_step * c.dt < c.period else controls[:, block]
            else:
                target = np.zeros(3)
            change = np.clip(target-speeds, -self.accel*c.dt, self.accel*c.dt)
            speeds += self._linear_limit(change, c.acceleration*c.dt)
            mid_yaw = poses[:, 2] + speeds[:, 2]*c.dt/2
            co, si = np.cos(mid_yaw), np.sin(mid_yaw)
            poses[:, 0] += (co*speeds[:, 0] - si*speeds[:, 1])*c.dt
            poses[:, 1] += (si*speeds[:, 0] + co*speeds[:, 1])*c.dt
            poses[:, 2] += speeds[:, 2]*c.dt
            result[:, step] = poses[:, :2]
        return result, first

    def _clearances(self, paths, objects):
        if not len(objects):
            return np.full(len(paths), math.inf)
        c = self.config
        # Closest approach during each interval, not only at time samples.
        future = objects[None, :, :2] + self.times[:, None, None]*objects[None, :, 2:4]
        relative = paths[:, :, None, :] - future[None, :, :, :]
        a, delta = relative[:, :-1], np.diff(relative, axis=1)
        fraction = np.clip(-np.sum(a*delta, axis=-1) /
                           np.maximum(np.sum(delta*delta, axis=-1), 1e-12), 0, 1)
        distance = np.linalg.norm(a + fraction[..., None]*delta, axis=-1)
        # Bound curvature/acceleration interpolation error between samples.
        interpolation = (c.acceleration + math.sqrt(2)*c.speed*c.yaw_speed)*c.dt*c.dt
        required = (c.robot_radius + c.margin + interpolation + objects[None, :, 4]
                    + objects[None, :, 5] + c.uncertainty_rate*self.times[1:, None])
        clearance = np.min(distance - required[None, :, :], axis=(1, 2))
        # The stopped terminal pose must also clear the continuing CV ray. Without
        # this check, backpedalling can keep postponing a head-on impact just past
        # the horizon until sideways escape becomes impossible.
        end_relative = relative[:, -1]
        v = objects[:, 2:4]
        arrival = np.maximum(0, np.sum(end_relative*v[None, :, :], axis=-1) /
                             np.maximum(np.sum(v*v, axis=-1), 1e-12))
        terminal_distance = np.linalg.norm(end_relative-arrival[..., None]*v, axis=-1)
        terminal_clearance = np.min(terminal_distance-required[-1], axis=-1)
        return np.minimum(clearance, terminal_clearance)

    def _costs(self, controls, desired):
        # Energy first for escape; weak route preference and switch penalty break ties.
        controls = np.asarray(controls).reshape(-1, 3, 3)
        energy = np.mean(np.sum(controls[:, :, :2]**2, axis=-1), axis=-1)
        route = np.mean(np.sum((controls-desired)**2, axis=-1), axis=-1)
        switches = np.mean(np.sum(np.diff(controls, axis=1)**2, axis=-1), axis=-1)
        turn = np.mean(controls[:, :, 2]**2, axis=-1)
        continuity = np.sum((controls[:, 0]-self.previous)**2, axis=-1)
        direction_cost = 0.0
        if self.escape_direction is not None:
            direction = controls[:, 0, :2]
            direction = direction / np.maximum(np.linalg.norm(direction, axis=-1, keepdims=True), 1e-9)
            direction_cost = .03 * (1 - direction @ self.escape_direction)
        return energy + .05*route + .05*switches + .05*turn + .04*continuity + direction_cost

    def _evaluate(self, controls, pose, velocity, objects):
        paths, commands = self._rollout(controls, pose, velocity)
        return self._clearances(paths, objects), commands

    def plan(self, pose, velocity, desired, tracks, now):
        c = self.config
        try:
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
        nominal = np.tile(desired, (3, 1))
        clear, commands = self._evaluate(nominal, pose, velocity, objects)
        if clear[0] >= 0:
            command = tuple(float(v) for v in commands[0])
            self.previous[:] = command
            self.escape_direction = None
            return Result(command, "follow", False, float(clear[0]))

        # If braking clears the crossing, yielding avoids an unnecessary sideways step.
        stop = np.zeros((3, 3))
        stop_clear, _ = self._evaluate(stop, pose, velocity, objects)
        if stop_clear[0] >= 0:
            # Test slow forward requests too, highest feasible keeps the route moving.
            slows = np.array([nominal*s for s in (.75, .5, .25)])
            slow_clear, slow_commands = self._evaluate(slows, pose, velocity, objects)
            for i in range(len(slows)):
                if slow_clear[i] >= 0 and np.linalg.norm(slow_commands[i, :2]) >= .06:
                    command = tuple(float(v) for v in slow_commands[i])
                    self.previous[:] = command
                    return Result(command, "yield", False, float(slow_clear[i]), "slow")
            self.previous[:] = 0
            return Result((0.0, 0.0, 0.0), "yield", False, float(stop_clear[0]), "stop")

        # Every direction can be considered; observed geometry determines feasibility.
        angles = np.arange(32)*2*math.pi/32
        seeds = []
        for speed in (.08, .12, .18, .24, .30):
            speed = min(speed, c.speed)
            for angle in angles:
                seeds.append(np.tile([speed*math.cos(angle), speed*math.sin(angle), 0.0], (3, 1)))
        seeds = np.array(seeds)
        clearance, _ = self._evaluate(seeds, pose, velocity, objects)
        valid = np.flatnonzero(clearance >= 0)
        if not len(valid):
            return self._blocked(stop_clear[0], "no_feasible_escape")
        costs = self._costs(seeds, desired)
        best_index = valid[np.argmin(costs[valid])]
        best = seeds[best_index].copy()
        best_cost = costs[best_index]
        solver = "feasible_seed"
        deadline = time.monotonic() + c.solve_seconds
        # Refine the nonlinear velocity blocks; rejected/failed solves never bypass checks.
        def check_time():
            if time.monotonic() > deadline:
                raise TimeoutError("MPC refinement budget")

        def objective(flat):
            check_time()
            return float(self._costs(np.tile(flat, (3, 1)), desired)[0])

        def constraint(flat):
            check_time()
            return float(self._evaluate(np.tile(flat, (3, 1)), pose, velocity, objects)[0][0])

        try:
            # Hold one velocity block across the horizon: optimizing later escape
            # separately lets a receding controller defer it at every new cycle.
            solution = minimize(objective, best[0], method="SLSQP",
                                bounds=[(-v, v) for v in self.limits],
                                constraints={"type": "ineq", "fun": constraint},
                                options={"maxiter": c.solver_iterations, "ftol": 1e-4})
            candidate = np.tile(self._linear_limit(solution.x, c.speed), (3, 1))
            candidate_clear, _ = self._evaluate(candidate, pose, velocity, objects)
            if (solution.success and np.isfinite(candidate).all()
                    and np.all(np.abs(candidate) <= self.limits+1e-9)
                    and candidate_clear[0] >= 0 and objective(solution.x) <= best_cost):
                best, solver = candidate, "slsqp"
        except (TimeoutError, ValueError, FloatingPointError):
            solver = "feasible_seed_budget"
        final_clear, command = self._evaluate(best, pose, velocity, objects)
        self.previous[:] = command[0]
        direction = best[0, :2]
        self.escape_direction = direction / max(np.linalg.norm(direction), 1e-9)
        return Result(tuple(float(v) for v in command[0]), "evade", False,
                      float(final_clear[0]), solver)
