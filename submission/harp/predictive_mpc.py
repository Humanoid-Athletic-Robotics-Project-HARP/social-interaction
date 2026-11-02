"""Bounded nonlinear MPC for generic tracked obstacles, with a cooperative contingency
forecast: objects seen to give way to the robot are expected to keep doing so, but only
after the robot could still stop if they don't (YIELD_DELAY; Responsiveness).

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

from harp.obstacle_tracker import STRUCTURE_ID

CONTROL_PERIOD = 0.2
STATIC_SPEED = .08       # m/s: slower tracked objects join the field (with forecast growth)
FIELD_RESOLUTION = .05   # m between distance-field samples
FIELD_ERROR = .01        # m: bilinear error at that resolution away from object centres
BODY_WEIGHT = 20.        # cost per metre of predicted body penetration
COMFORT_WEIGHT = 2.      # cost per metre-second inside the comfort margins
RISK_TIME = 2.           # s: discount time constant of predicted intrusion (sweep-chosen)
UNCERTIFIED_COST = .6    # preference for a certifiably clear path when one exists
RECOVERY_TIME = 2.       # s to repay an existing comfort intrusion: a ramped walker's launch
FINALISTS = 24           # nominally best candidates re-ranked under the robust model
AFTER_PLAN = 4.          # s after a plan's stop checked for objects walking into it
# Cooperative contingency forecast. A straight-line forecast has every person walk into
# the robot; in a crowd that gives way, nearly every forward plan then looked unsafe and
# the robot slowed, sidestepped and switched modes (froze). An object judged responsive
# (credit 1, Responsiveness) is expected to add separation once it can react: from
# YIELD_DELAY on, at YIELD_ACCEL, by YIELD_MAX at most; credit 0 keeps the straight line.
# Nothing is credited before YIELD_DELAY, about the robot's stop from full speed, so if
# it does not give way the next plans still have a worst-case-safe reaction.
YIELD_DELAY = 1.2        # s
YIELD_ACCEL = 0.6        # m/s^2
YIELD_MAX = 0.5          # m
# Decision chattering: many candidates score nearly alike, so each 0.2 s plan picked a
# different manoeuvre (right, ahead, left, back) from small track changes; urgent requests
# skip the ramp (a head-on escape must not wait), so those jumps reached the robot.
SWITCH_WEIGHT = 2.       # cost per (m/s)^2 of change from the last request: a reversal at
                         # 0.5 m/s costs about 1.5 cm of predicted body overlap
DWELL = 1.0              # s a chosen manoeuvre is kept (minimum dwell, as in switching
                         # control) unless it stops being certifiably clear or turns out
DWELL_RISK = .3          # this much riskier than the best: chatter is a switch every cycle
# Humanoid motion primitives: a biped (or a person) gets round people mostly by curving
# its walk and changing pace; the G1's sidestep is slow and looks unsteady. With smoothing,
# candidates include curved walks at eight turn rates and turning on the spot, and a
# sideways request costs extra, less when urgent so an emergency sidestep stays available.
# (A sideways cost without curved walks to choose instead made the robot back away.)
LATERAL_WEIGHT = 1.      # cost per (m/s)^2 of sideways request
LATERAL_URGENT = .25     # the same, when a collision is near (Planner.urgent)
URGENT_ACCEL = 1.5       # m/s^2 an urgent request may change at (smoothing): about three times
                         # the walker's own acceleration, so an escape isn't delayed, but its
                         # target no longer jumps between opposite sidesteps from cycle to cycle
SUSTAINED = 3            # scans of widening a pass before it counts as giving way (a route
                         # corner turns a scripted walker once, which looked like yielding)
DEFAULT_CREDIT = 0.       # an object not yet seen giving way keeps the straight line: a
                         # credited head-on walker that then does not yield leaves too
                         # little time for the G1's slow sidestep (about 3 s to clear)
# Responsiveness evidence (per scan) while an object within ENGAGE closes on the robot:
# its own velocity change (beyond tracking noise) widening the pass by YIELD_EVIDENCE gives
# way; an unchanged course passing nearer than CONFLICT within HOLD_TIME does not.
ENGAGE = 4.0             # m
CONFLICT = 1.3           # m between centres (body, margins and forecast spread)
YIELD_EVIDENCE = 0.15    # m
NOISE = 0.1              # m/s of velocity change that tracking noise can explain
HOLD_TIME = 3.0          # s: early enough to leave a slow sidestep its time
CREDIT_STEP = 0.1        # credit per scan of giving way; not giving way counts double


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
    # Extra forecast spread per m/s of an object's own speed: a walker can turn
    # or stop, a standing one hardly drifts. Comfort-margin (soft) only.
    speed_spread: float = 0.1
    max_tracks: int = 160  # moving objects after pruning (fixed: 4x); input is capped at 8x
    solve_seconds: float = 0.07
    solver_iterations: int = 8
    tracking_gain: float = 1.0
    lateral_tracking_gain: float = 1.0
    # Walker sidesteps vary with gait phase (native G1: 0.10 m/s rms error on a
    # 0.24 m/s response). Safety is also checked with this share of the lateral gain.
    lateral_pessimism: float = .6
    max_collision_uncertainty: float = .15
    # Research switches (HARP_PREDICTIVE_COOPERATIVE / _SMOOTHING in the image): both off
    # is the planner before the cooperative contingency forecast, for A/B runs in HIM.
    cooperative: bool = True   # credit for objects seen giving way
    smoothing: bool = True     # follow hysteresis, switching cost, minimum dwell, bounded urgent
                               # requests, humanoid primitives (curved walks, sideways cost)
    # HARP_PREDICTIVE_HEADING=1 (with smoothing; off by default): an avoidance step holds its
    # direction on the floor while the robot turns as Nav2 asks. A round footprint doesn't turn,
    # so the heading is free; with body-frame steps a turn swung the step round, so avoiding
    # withheld Nav2's turn or turned against it (HIM building, Nov 2). Faster to the goal in
    # HIM, but every dodge becomes a sidestep that Nav2 turns back from: more heading reversals.
    heading: bool = False


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
        if not (0 <= c.response_delay <= 1 and 0 <= c.uncertainty_rate <= 1
                and 0 <= c.speed_spread <= 1):
            raise ValueError("Invalid delay/uncertainty growth")
        if not .5 <= c.tracking_gain <= 1.:
            raise ValueError('Invalid velocity tracking gain')
        if not .3 <= c.lateral_tracking_gain <= 1.:
            raise ValueError('Invalid lateral velocity tracking gain')
        if not .3 <= c.lateral_pessimism <= 1.:
            raise ValueError('Invalid lateral pessimism')
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
        self.field = None
        self.credits = {}        # track id -> forecast credit in [0, 1] (Responsiveness)
        self.following = False   # the last plan followed the request unchanged
        self.switched_at = -math.inf   # when the current manoeuvre was chosen
        self.chosen = None             # that manoeuvre's request blocks (3, 3), while avoiding
        self.chosen_body = np.ones(3, dtype=bool)   # its blocks that turn with the robot
        self.planned_yaw = None        # heading at the last plan (held steps are turned back)

    def _blocked(self, clearance=-math.inf, reason="invalid_input"):
        self.previous[:] = 0
        self.escape_direction = None
        self.chosen = None
        return Result((0.0, 0.0, 0.0), "blocked", True, float(clearance), reason)

    @staticmethod
    def _linear_limit(values, limit):
        values = np.array(values, dtype=float, copy=True)
        norms = np.linalg.norm(values[..., :2], axis=-1, keepdims=True)
        values[..., :2] *= np.minimum(1.0, limit / np.maximum(norms, 1e-12))
        return values

    @staticmethod
    def _held(requests, body, turned):
        """Requests (n, 3) with the steps not in body (n,) turned back by turned (n,) rad: a
        held step keeps its direction on the floor while the robot turns."""
        out = np.array(requests, dtype=float, copy=True)
        held = ~np.asarray(body, dtype=bool)
        co, si = np.cos(turned)[held], np.sin(turned)[held]
        x, y = out[held, 0].copy(), out[held, 1].copy()
        out[held, 0], out[held, 1] = co*x + si*y, co*y - si*x
        return out

    def _rollout(self, controls, pose, velocity, brake_after=None, lateral_gain=None, body=None):
        """Vectorized acceleration-limited SE(2) dynamics, then a stopping tail. body (n, 3):
        which request blocks turn with the robot (Nav2's own); None is all of them."""
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
            first = self._urgent_step(controls[:, 0], self.previous)
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
                    wanted = controls[:, block]
                    if body is not None:
                        wanted = np.clip(self._held(wanted, body[:, block], poses[:, 2]-pose[2]),
                                         -self.limits, self.limits)
                    if self.urgent:
                        request = self._urgent_step(wanted, request)
                    else:
                        change = np.clip(wanted-request,
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
                target[..., 1] *= c.lateral_tracking_gain if lateral_gain is None else lateral_gain
            change = np.clip(target-speeds, -self.accel*c.dt, self.accel*c.dt)
            speeds += self._linear_limit(change, c.acceleration*c.dt)
            mid_yaw = poses[:, 2] + speeds[:, 2]*c.dt/2
            co, si = np.cos(mid_yaw), np.sin(mid_yaw)
            poses[:, 0] += (co*speeds[:, 0] - si*speeds[:, 1])*c.dt
            poses[:, 1] += (si*speeds[:, 0] + co*speeds[:, 1])*c.dt
            poses[:, 2] += speeds[:, 2]*c.dt
            result[:, step] = poses[:, :2]
        return result, first

    def _urgent_step(self, target, request):
        """An urgent request one period on from request: the target itself, or with
        smoothing, at most URGENT_ACCEL away."""
        if not self.config.smoothing:
            return np.array(target, dtype=float, copy=True)
        change = target-request
        change[..., 2] = np.clip(change[..., 2], -self.accel[2]*self.config.period, self.accel[2]*self.config.period)
        step = self._linear_limit(change, URGENT_ACCEL*self.config.period)
        return self._linear_limit(np.clip(request+step, -self.limits, self.limits), self.config.speed)

    def _growth(self, objects):
        """Forecast spread rate (m/s) of each object."""
        return self.config.uncertainty_rate+self.config.speed_spread*np.linalg.norm(objects[:, 2:4], axis=1)

    @staticmethod
    def _approach(paths, objects, times):
        """Closest approach during each interval, not only at time samples."""
        future = objects[None, :, :2] + times[:, None, None]*objects[None, :, 2:4]
        relative = paths[:, :, None, :] - future[None, :, :, :]
        a, delta = relative[:, :-1], np.diff(relative, axis=1)
        fraction = np.clip(-np.sum(a*delta, axis=-1) /
                           np.maximum(np.sum(delta*delta, axis=-1), 1e-12), 0, 1)
        return np.linalg.norm(a + fraction[..., None]*delta, axis=-1), relative

    def _build_field(self, static, pose, half, drifting=None):
        """Distance from each cell to the nearest fixed object's envelope, once per cycle.

        Walls arrive as many tiles; checking every sampled path against every
        tile dominated planning time in a building. A costmap-style field makes
        each sample one lookup. Moving objects keep exact swept checks. Group 0
        is fixed geometry (wall tiles, remembered surfaces); group 1 holds slow
        tracked objects (`drifting`), whose comfort clearance shrinks with time
        like a slow object's forecast.
        """
        c = self.config
        count = int(math.ceil(2*half/FIELD_RESOLUTION))+1
        origin = np.asarray(pose[:2], dtype=float)-half
        gx = origin[0]+np.arange(count)*FIELD_RESOLUTION
        gy = origin[1]+np.arange(count)*FIELD_RESOLUTION
        group = np.zeros(len(static), int) if drifting is None else np.asarray(drifting, int)
        # Layer 0: observed body only. Layer 1: body plus tracking uncertainty.
        values = np.full((2, 2, count, count), 1e3)   # finite: interpolation weights may be 0
        spread = np.minimum(static[:, 5], c.max_collision_uncertainty)
        for g in (0, 1):
            members = np.flatnonzero(group == g)
            for chunk in np.array_split(members, max(1, len(members)//16)):
                if not len(chunk):
                    continue
                dx = gx[None, :, None]-static[chunk, 0][:, None, None]
                dy = gy[None, None, :]-static[chunk, 1][:, None, None]
                body = np.sqrt(dx*dx+dy*dy)-static[chunk, 4][:, None, None]
                values[0, g] = np.minimum(values[0, g], body.min(axis=0))
                values[1, g] = np.minimum(values[1, g], (body-spread[chunk][:, None, None]).min(axis=0))
        self.field = (origin, values, static, group)

    def _drift(self, layer):
        """Comfort clearance lost per second near a slow tracked object: the
        forecast growth an exact check gives a stationary one. Below STATIC_SPEED a
        track's velocity is mostly filter noise, so holding its position is as
        good as extrapolating it; the body envelope does not grow."""
        return (layer == 1)*self.config.uncertainty_rate

    def _field_at(self, points, layer=1, times=None):
        """Distance to the nearest fixed envelope (layer 0: body only, 1: with
        uncertainty) at the given times, conservative by FIELD_ERROR; exact
        outside the sampled square."""
        c = self.config
        origin, values, static, group = self.field
        values = values[layer]
        scaled = (points-origin)/FIELD_RESOLUTION
        low = np.floor(scaled).astype(int)
        fx, fy = (scaled-low)[..., 0], (scaled-low)[..., 1]
        n = values.shape[-1]
        inside = np.all((low >= 0) & (low < n-1), axis=-1)
        i, j = np.clip(low[..., 0], 0, n-2), np.clip(low[..., 1], 0, n-2)
        result = (values[:, i, j]*(1-fx)*(1-fy)+values[:, i+1, j]*fx*(1-fy)
                  + values[:, i, j+1]*(1-fx)*fy+values[:, i+1, j+1]*fx*fy)
        late = 0. if times is None else np.broadcast_to(times, points.shape[:-1])
        result = np.minimum(result[0], result[1]-self._drift(layer)*late)
        if not inside.all():
            outside = points[~inside]
            exact = (np.linalg.norm(outside[:, None, :]-static[None, :, :2], axis=-1)
                     - static[:, 4]-layer*np.minimum(static[:, 5], c.max_collision_uncertainty))
            if times is not None:
                exact = exact-self._drift(layer)*late[~inside][:, None]*(group[None, :] == 1)
            result[~inside] = exact.min(axis=1)+FIELD_ERROR   # undo the margin added below
        return result-FIELD_ERROR

    def _static_clearance(self, paths, times, interpolation, recover_margin):
        c = self.config
        physical = c.robot_radius+interpolation
        comfort = self._field_at(paths[:, 1:], 1, times[1:])-physical-c.margin
        if not recover_margin:
            return np.min(comfort, axis=-1)
        # As for tracks: only an existing comfort-margin intrusion may decay,
        # while the observed body (no uncertainty) stays a hard limit.
        debt = np.maximum(0., -(self._field_at(paths[:, :1])[:, 0]-physical-c.margin))
        debt[self._field_at(paths[:, :1], 0)[:, 0] < physical+.02] = 0.
        decay = np.maximum(0., 1.-np.maximum(0., times[1:]-c.response_delay-c.period)/RECOVERY_TIME)
        body = self._field_at(paths[:, 1:], 0, times[1:])-physical
        # As for tracks, the allowance includes the forecast growth since t=0.
        allowance = (debt[:, None]+self._drift(1)*times[None, 1:])*decay[None, :]
        return np.min(np.minimum(body, comfort+allowance), axis=-1)

    def _clearances(self, paths, objects, time_limit=None, recover_margin=False, approach=None):
        """approach: this full-horizon path set's _approach result, to reuse."""
        c = self.config
        times = self.times
        if time_limit is not None:
            count = max(2,int(time_limit/c.dt)+1)
            paths, times = paths[:,:count], times[:count]
            approach = None
        # Bound curvature/acceleration interpolation error between samples.
        interpolation = (c.acceleration + math.sqrt(2)*c.speed*c.yaw_speed)*c.dt*c.dt
        clearance = np.full(len(paths), math.inf)
        if self.field is not None:
            clearance = self._static_clearance(paths, times, interpolation, recover_margin)
        if not len(objects):
            return clearance
        distance, relative = approach or self._approach(paths, objects, times)
        required = (c.robot_radius + c.margin + interpolation + objects[None, :, 4]
                    + np.minimum(objects[None, :, 5],c.max_collision_uncertainty)
                    + self._growth(objects)[None, :]*times[1:, None])
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
            decay = np.maximum(0.,1.-np.maximum(0.,times[1:]-c.response_delay-c.period)/RECOVERY_TIME)
            allowance = np.where(debt[:,None,:] > 0,
                (debt[:,None,:]+self._growth(objects)[None,None,:]*times[None,1:,None])*decay[None,:,None],0.)
            required = np.maximum(physical[None,None,:],required-allowance)
        required = required - self._credit(objects, times[1:])[None]
        # Replan while travelling: do not reserve the stopped pose against
        # speculative pedestrian motion beyond the checked horizon/braking tail.
        return np.minimum(clearance, np.min(distance - required, axis=(1, 2)))

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
            # Commit to a passing side: alternating sides each cycle leaves a
            # delayed walker standing still while an actor closes in.
            direction_cost = .06 * (1-direction @ self.escape_direction)
        requested = float(np.linalg.norm(desired[:2]))
        stall = (.15*(1-np.minimum(1.,np.linalg.norm(controls[:,0,:2],axis=-1)/requested))
                 if requested > .06 else 0.)
        progress = 0.
        if paths is not None and self.goal is not None:
            # Velocity agreement alone can reward a curve that ends up farther
            # from the goal. Score actual SE(2) rollout progress as well.
            distances = np.linalg.norm(paths-self.goal[None,None,:],axis=-1)
            progress = .25*(distances[:,-1]-distances[:,0])
        lateral = ((LATERAL_URGENT if self.urgent else LATERAL_WEIGHT)*np.mean(controls[:, :, 1]**2, axis=-1)
                   if self.config.smoothing else 0.)
        return error + .12*continuity + 2*backwards + direction_cost + stall + progress + lateral

    def _risk(self, paths, objects, distance=None):
        """Soft collision cost of each trajectory, standing included.

        Entering an observed body is weighted by its deepest discounted
        penetration, not its duration: walking toward an actor so the encounter
        is shorter is not safer. A small integral still rewards leaving an
        existing overlap. Comfort-margin intrusion is a moderate, tradeable cost,
        so a crowd that never certifies full margins still leaves room to move.
        A 2 s discount beat 3.5 s in a crowd sweep: weighting speculative
        far-future overlaps made the robot dither instead of acting now.
        """
        c = self.config
        times = self.times[1:]
        body = np.zeros((len(paths), len(times)))
        comfort = np.zeros((len(paths), len(times)))
        if self.field is not None:
            body = np.maximum(body, c.robot_radius-self._field_at(paths[:, 1:], 0, times))
            comfort = np.maximum(comfort, c.robot_radius+c.margin-self._field_at(paths[:, 1:], 1, times))
        if len(objects):
            if distance is None:
                distance, _ = self._approach(paths, objects, self.times)
            physical = c.robot_radius+objects[:, 4]
            spread = (physical+c.margin+np.minimum(objects[:, 5], c.max_collision_uncertainty)
                      + self._growth(objects)[None, :]*times[:, None])
            credit = self._credit(objects, times)
            body = np.maximum(body, np.max(physical[None, :]-credit-distance, axis=2))
            comfort = np.maximum(comfort, np.max(spread-credit-distance, axis=2))
        weight = np.exp(-times/RISK_TIME)[None]
        body, comfort = np.maximum(body, 0.)*weight, np.maximum(comfort, 0.)*weight
        risk = (BODY_WEIGHT*(np.max(body, axis=1)+.3*np.sum(body, axis=1)*c.dt)
                + COMFORT_WEIGHT*np.sum(comfort, axis=1)*c.dt)
        if len(objects) and AFTER_PLAN > 0:
            # Every plan ends stopped. An object that will then walk into the stopped
            # robot makes that collision inevitable, not avoided: charge it as if it
            # happened when the plan ends. Otherwise backing away from a faster
            # walker only pushes the hit past the horizon and looks safest.
            # Closest approach of each straight-walking object to the stopped pose.
            end = self.times[-1]
            start = objects[None, :, :2]+end*objects[None, :, 2:4]-paths[:, -1][:, None]
            velocity = objects[None, :, 2:4]
            later = np.clip(-np.sum(start*velocity, axis=-1)
                            / np.maximum(np.sum(velocity*velocity, axis=-1), 1e-12), 0., AFTER_PLAN)
            # A responsive object has had long enough to give way around the stopped robot.
            gap = np.linalg.norm(start+later[..., None]*velocity, axis=-1)+YIELD_MAX*credit_of(objects)[None]
            risk = risk+BODY_WEIGHT*math.exp(-end/RISK_TIME)*np.max(
                np.maximum(0., (c.robot_radius+objects[:, 4])[None]-gap), axis=1)
        return risk

    @staticmethod
    def _credit(objects, times):
        """(times, objects): separation each object is credited with adding by then. Objects
        without a credit column (x, y, vx, vy, radius, uncertainty only) get none."""
        return credit_of(objects)[None, :]*yield_allowance(times)[:, None]

    def _assess(self, controls, pose, velocity, objects, robust=True, body=None):
        """Paths and first requests of the nominal walker model, with clearances
        (strict, margin recovery) and risk taken as the worse of the nominal and
        a slow-sidestep model: robust to how late a sidestep can come."""
        c = self.config
        results = []
        gains = [None]+([c.lateral_tracking_gain*c.lateral_pessimism]
                        if robust and c.lateral_pessimism < 1 else [])
        for gain in gains:
            paths, firsts = self._rollout(controls, pose, velocity, lateral_gain=gain, body=body)
            approach = self._approach(paths, objects, self.times) if len(objects) else None
            results.append((paths, firsts, self._clearances(paths, objects, approach=approach),
                            self._clearances(paths, objects, recover_margin=True, approach=approach),
                            self._risk(paths, objects, None if approach is None else approach[0])))
        paths, firsts = results[0][:2]
        return (paths, firsts, np.min([r[2] for r in results], axis=0),
                np.min([r[3] for r in results], axis=0), np.max([r[4] for r in results], axis=0))

    def plan(self, pose, velocity, desired, tracks, now, goal=None):
        c = self.config
        started = time.monotonic()
        self.field = None
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
            if len(tracks) > 8*c.max_tracks:
                return self._blocked(reason="track_capacity")
            # Wall tiles and remembered lost surfaces are fixed geometry; every
            # tracked object, standing people included, keeps forecast growth.
            fixed = np.array([getattr(t, 'id', 0) >= STRUCTURE_ID or getattr(t, 'id', 0) < 0
                              for t in tracks], dtype=bool)
            credit = [0. if still or not c.cooperative else
                      min(1., max(0., float(self.credits.get(getattr(t, 'id', None), DEFAULT_CREDIT))))
                      for t, still in zip(tracks, fixed)]
            objects = np.asarray([[t.x, t.y, t.vx, t.vy, t.radius, t.uncertainty, k]
                                  for t, k in zip(tracks, credit)], dtype=float).reshape(-1, 7)
            if (not np.isfinite(objects).all() or np.any(objects[:, 4:] < 0)
                    or any(not math.isfinite(t.last_seen) or t.last_seen > now+.05 for t in tracks)):
                return self._blocked()
        except (TypeError, ValueError, AttributeError):
            return self._blocked()
        steer = c.smoothing and c.heading
        if steer and self.planned_yaw is not None:
            # The committed manoeuvre and passing side hold on the floor: turn them back by
            # the heading change since the last plan.
            turned = float(pose[2]-self.planned_yaw)
            if self.chosen is not None:
                self.chosen = self._held(self.chosen, self.chosen_body, np.full(3, turned))
            if self.escape_direction is not None:
                self.escape_direction = self._held(
                    np.r_[self.escape_direction, 0.][None], np.zeros(1, bool), np.full(1, turned))[0, :2]
        self.planned_yaw = float(pose[2])
        # Walls arrive as many surface tiles. Drop objects that cannot come
        # within any checked envelope before the forecast ends, even with the
        # robot and object closing at full speed: no feasibility check changes.
        # Acceleration-limited robot speed never exceeds max(measured, limit).
        if len(objects):
            span = self.times[-1]
            reach = ((max(float(np.linalg.norm(velocity[:2])), c.speed)
                      + np.linalg.norm(objects[:, 2:4], axis=1))*span
                     + c.robot_radius + c.margin + objects[:, 4]
                     + np.minimum(objects[:, 5], c.max_collision_uncertainty)
                     + self._growth(objects)*span + c.acceleration*c.dt*c.dt + .2)
            near = np.linalg.norm(objects[:, :2]-pose[:2], axis=1) <= reach
            objects, fixed = objects[near], fixed[near]
        # Wall tiles and remembered surfaces are fixed; slow tracked objects (a
        # column, a standing person) join the field with forecast growth, so only
        # moving objects need exact swept checks under both walker models.
        slow = ~fixed & (np.linalg.norm(objects[:, 2:4], axis=1) < STATIC_SPEED)
        # Only moving objects need a swept check each; fixed geometry (wall tiles,
        # remembered surfaces) folds into the field, whose cost grows slowly with it.
        # Counting walls against the moving limit stopped the robot beside them.
        if (~(fixed | slow)).sum() > c.max_tracks or (fixed | slow).sum() > 4*c.max_tracks:
            return self._blocked(reason="track_capacity")
        if (fixed | slow).any():
            half = max(float(np.linalg.norm(velocity[:2])), c.speed)*self.times[-1]+.3
            self._build_field(objects[fixed | slow], pose, half, slow[fixed | slow])
        objects = objects[~(fixed | slow)]
        geometry = len(objects) or self.field is not None
        desired = self._linear_limit(np.clip(desired, -self.limits, self.limits), c.speed)
        if not self.initialized:
            self.previous[:] = self._linear_limit(velocity,c.speed)
            self.initialized = True
        self.urgent = False
        if len(objects):
            # An approaching object makes an escape urgent. Standing beside a wall
            # must not drop the request ramp for every command.
            field, self.field = self.field, None
            stopped,_ = self._rollout(np.zeros((3,3)),pose,velocity,brake_after=0.)
            self.urgent = bool(self._clearances(stopped,objects,time_limit=2.5)[0] < 0.)
            self.field = field
        if self.field is not None:
            # So does a body already overlapping fixed geometry (not mere comfort).
            self.urgent |= bool(self._field_at(np.asarray(pose[:2])[None], 0)[0] < c.robot_radius)
        nominal = np.tile(desired, (3, 1))
        _, commands, clear, _, _ = self._assess(nominal, pose, velocity, objects)
        # Hysteresis: entering follow takes room to spare, staying needs only a clear path;
        # one threshold had the robot switch between following and avoiding every cycle.
        if clear[0] >= (0. if self.following and c.smoothing else .12) or not geometry:
            command = tuple(float(v) for v in commands[0])
            self.previous[:] = command
            self.escape_direction = None
            self.following = True
            self.chosen = None
            return Result(command, "follow", False, float(clear[0]))
        entering, self.following = self.following, False

        # Candidates: every direction at three speeds, held, merging back into the
        # route, or stepping and then stopping; turning curves; the previous
        # request (warm start); and standing, scored like any other trajectory.
        # With steer, every step turns as Nav2 asks and the planner's own steps hold on the
        # floor (body: only Nav2's request turns with the robot), so curved walks and turning
        # either way have nothing left to add.
        seeds, body = [np.zeros((3, 3))], [np.zeros(3, dtype=bool) | (not steer)]
        def add(blocks, turns_with_robot=(False, False, False)):
            seeds.append(np.asarray(blocks, dtype=float))
            body.append(np.asarray(turns_with_robot, dtype=bool) | (not steer))
        turn = float(desired[2]) if steer else 0.
        if c.smoothing:
            # Turning on the spot (as Nav2 asks, or either way): the standing footprint, so
            # as safe as standing. Without it, avoiding meant never turning: Nav2 kept asking
            # to realign with its path for seconds while the robot shuffled about.
            for yaw in ({turn} if steer else {float(desired[2]), .5*c.yaw_speed, -.5*c.yaw_speed}) - {0.}:
                add(np.tile([0., 0., yaw], (3, 1)))
        stop = np.array([0., 0., turn])
        for speed in (c.speed, .65*c.speed, .3*c.speed):
            for angle in np.arange(16)*2*math.pi/16:
                candidate = np.array([speed*math.cos(angle),speed*math.sin(angle),turn])
                add(np.tile(candidate,(3,1)))
                add([candidate,candidate,desired], (False, False, True))
                add([candidate,desired,desired], (False, True, True))
                add([candidate,stop,stop])
            turns = (() if steer else (-1., -.75, -.5, -.25, .25, .5, .75, 1.) if c.smoothing
                     else (-1., -.5, .5, 1.))
            for share in turns:
                curve = np.array([speed,0.,share*c.yaw_speed])
                add([curve,curve,desired], (True, True, True))
        add(np.tile(np.r_[self.previous[:2], turn] if steer else self.previous, (3,1)))
        # The manoeuvre chosen last cycle, which its ramped request is still on the way to:
        # the one consistency and the dwell keep (the ramped request alone usually scored
        # worse than the manoeuvre, so the plan switched anyway).
        committed = c.smoothing and self.chosen is not None
        if committed:
            held = self.chosen.copy()
            if steer and np.any(held):   # standing stays standing
                held[~self.chosen_body, 2] = turn
                held[self.chosen_body] = desired
            add(held, self.chosen_body)
        body = np.asarray(body) if steer else None
        seeds = np.asarray(seeds)
        seeds = self._linear_limit(np.clip(seeds,-self.limits,self.limits),c.speed)
        # One soft-constrained ranking replaces the old cascade (certified, margin
        # recovery, brake-then-stop, least intrusion). In a crowd full margins are
        # rarely certifiable, so the fallback had become the normal case.
        # Usable clearance is preferred over a trajectory that vanishes after a
        # centimetre of sensor or gait error.
        # Rank all candidates with the nominal walker model, then re-rank the best
        # few (always standing and the previous request) robustly: rolling out
        # every candidate twice doubled planning time and missed deadlines.
        paths, firsts, strict, relaxed, risk = self._assess(seeds, pose, velocity, objects, robust=False, body=body)
        # Changing a manoeuvre under way costs while it is still certifiably clear (the
        # last seed holds it); once it is not, or from standing, changing is free.
        switch = (SWITCH_WEIGHT*np.sum((seeds[:, 0]-self.previous)**2, axis=-1)
                  if c.smoothing and np.linalg.norm(self.previous[:2]) > .06 and relaxed[-1] >= 0 else 0.)
        def rank(paths, strict, relaxed, risk):
            return (self._costs(seeds, desired, paths)+risk+.05/(.1+np.maximum(relaxed,0.))
                    + UNCERTIFIED_COST*(relaxed < 0) + switch)
        costs = rank(paths, strict, relaxed, risk)
        finalists = np.unique(np.r_[np.argsort(costs)[:FINALISTS], 0, len(seeds)-2, len(seeds)-1])
        _, _, strict_f, relaxed_f, risk_f = self._assess(
            seeds[finalists], pose, velocity, objects, body=None if body is None else body[finalists])
        strict[finalists], relaxed[finalists], risk[finalists] = strict_f, relaxed_f, risk_f
        costs = rank(paths, strict, relaxed, risk)
        others = np.setdiff1d(np.arange(len(seeds)), finalists)
        costs[others] = np.inf   # not robustly checked
        index = int(np.argmin(costs))
        # Plan consistency: keep the previous request (the last seed) unless
        # another is clearly better. Switching sides every cycle leaves a delayed
        # walker standing still while an actor closes in.
        # Only a tie-break between equally safe options: when risk dominates, a
        # cost slack would otherwise keep a materially riskier request.
        keep = len(seeds)-1   # the chosen manoeuvre if committed, else the previous request
        if (np.linalg.norm(self.previous[:2]) > .06
                and costs[keep] <= costs[index]+.1+.05*abs(costs[index])
                and risk[keep] <= risk[index]+.05
                and (relaxed[keep] >= 0 or relaxed[index] < 0)):
            index = keep
        # Also when nothing is certifiably clear, the usual case in a crowd: the kept one
        # need only be about as clear as the best.
        if (c.smoothing and index != keep and not entering and np.linalg.norm(self.previous[:2]) > .06
                and now-self.switched_at < DWELL and np.isfinite(costs[keep])
                and (relaxed[keep] >= 0 or relaxed[keep] >= relaxed[index]-.05)
                and risk[keep] <= risk[index]+DWELL_RISK):
            index = keep
        if index != keep or entering:
            self.switched_at = now
        best, solver = seeds[index].copy(), 'least_intrusion'
        mask = None if body is None else body[index][None]
        if c.smoothing and not steer and abs(desired[2]) > 1e-3 and abs(best[0, 2]) < 1e-6:
            # Keep turning as Nav2 asks while avoiding, if that is as clear and no riskier:
            # a round robot's footprint doesn't turn. Avoiding without turning left the
            # robot off its path's heading, and Nav2 asked it to realign for seconds.
            turned = best.copy()
            turned[:, 2] = desired[2]
            _, _, _, turned_relaxed, turned_risk = self._assess(turned, pose, velocity, objects)
            if turned_relaxed[0] >= relaxed[index]-.02 and turned_risk[0] <= risk[index]+.05:
                best = turned
        recover_margin = strict[index] < 0
        # A kept manoeuvre was refined when it was chosen; refining it again (blind to the
        # switching cost and the dwell) moved it off again, and took most of a plan's time.
        kept = committed and index == keep
        if relaxed[index] >= 0 and not kept:
            solver = 'feasible_seed'
            deadline = min(time.monotonic()+c.solve_seconds, started+c.solve_seconds+.01)
            def check_time():
                if time.monotonic() > deadline:
                    raise TimeoutError('MPC refinement budget')
            evaluated = {}
            def measure(flat):
                # SLSQP asks for the cost and the constraint at the same points;
                # one rollout serves both.
                key = flat.tobytes()
                if key not in evaluated:
                    check_time()
                    paths_,_,strict_,relaxed_,risk_ = self._assess(flat.reshape(3,3),pose,velocity,objects,body=mask)
                    evaluated[key] = (paths_, (relaxed_ if recover_margin else strict_)[0],
                                      relaxed_[0], risk_[0])
                return evaluated[key]
            def objective(flat):
                # Same terms as the seed ranking, so acceptance compares like with like.
                paths_, _, relaxed_, risk_ = measure(flat)
                return float(self._costs(flat.reshape(3,3),desired,paths_)[0]
                             + risk_+.05/(.1+max(relaxed_,0.)))
            def constraint(flat):
                return float(measure(flat)[1])
            try:
                solution = minimize(objective, best.ravel(), method='SLSQP',
                                    bounds=[(-v,v) for _ in range(3) for v in self.limits],
                                    constraints={'type':'ineq', 'fun':constraint},
                                    options={'maxiter':c.solver_iterations,'ftol':1e-4})
                candidate = self._linear_limit(solution.x.reshape(3,3),c.speed)
                if (solution.success and np.isfinite(candidate).all()
                        and constraint(candidate.ravel()) >= 0
                        and objective(candidate.ravel()) <= costs[index]):
                    best, solver = candidate, 'slsqp'
            except (TimeoutError, ValueError, FloatingPointError):
                solver = 'feasible_seed_budget'
        _, command, final_strict, final_relaxed, _ = self._assess(best, pose, velocity, objects, body=mask)
        self.chosen = np.array(best, dtype=float).reshape(3, 3) if c.smoothing else None
        self.chosen_body = np.ones(3, dtype=bool) if mask is None else mask[0].copy()
        final_strict = float(final_strict[0])
        final = final_strict if final_strict >= 0 else float(final_relaxed[0])
        command = command[0]
        alert = final_strict < 0
        if np.linalg.norm(best[0, :2]) < 1e-6 and abs(best[0, 2]) < 1e-6:
            # Standing won the ranking: nothing reachable is better.
            self.previous[:] = 0.
            return Result((0.,0.,0.), 'blocked', alert, final,
                          'no_feasible_escape' if final < 0 else 'hold')
        self.previous[:] = command
        direction = best[0,:2]
        speed = float(np.linalg.norm(direction))
        if speed > .06:
            self.escape_direction = direction/speed
        # Yielding is slowing along the route with a clear path; leaving it,
        # retreating, or any escape that is not certifiably clear is evasion.
        cross = abs(direction[0]*desired[1]-direction[1]*desired[0])
        evade = (alert or cross > .04*max(np.linalg.norm(desired[:2]), .01)
                 or float(direction @ desired[:2]) < 0)
        mode = 'evade' if evade else 'yield'
        if alert and final >= 0:
            solver = 'margin_recovery:'+solver
        return Result(tuple(float(v) for v in command), mode, alert, final, solver)


def credit_of(objects):
    """Each object's forecast credit: its seventh column, or none without one."""
    return objects[:, 6] if objects.shape[1] > 6 else np.zeros(len(objects))


def yield_allowance(times):
    """Separation a fully responsive object adds by each time (m): none before YIELD_DELAY."""
    late = np.maximum(0., np.asarray(times, dtype=float)-YIELD_DELAY)
    return np.minimum(YIELD_MAX, .5*YIELD_ACCEL*late*late)


def closest_pass(relative, velocity):
    """(distance, time) of the straight-line closest approach of relative position and velocity."""
    speed = float(velocity @ velocity)
    when = max(0., -float(relative @ velocity)/speed) if speed > 1e-9 else 0.
    return float(np.linalg.norm(relative+when*velocity)), when


class Responsiveness:
    """Forecast credit of each tracked moving object: does it give way to the robot?

    From when an object within ENGAGE starts closing on the robot, its velocity then is
    kept; the pass it is on course for now is compared with the pass at that velocity,
    both with the robot's current velocity, so only the object's own change counts.
    Widening the pass by YIELD_EVIDENCE (with a velocity change beyond NOISE) is evidence
    it gives way; an unchanged course passing nearer than CONFLICT within HOLD_TIME is
    evidence it does not, counted double; widening must last SUSTAINED scans. Credit
    starts at DEFAULT_CREDIT, stays within
    [0, 1], and is forgotten with its track."""

    def __init__(self):
        self.credit, self.start, self.widening = {}, {}, {}

    def update(self, tracks, pose, velocity):
        """tracks: planning tracks (odom); pose (x, y, yaw); velocity: the robot's body-frame
        (vx, vy, wz). Returns {track id: credit}."""
        c, s = math.cos(pose[2]), math.sin(pose[2])
        robot = np.array([c*velocity[0]-s*velocity[1], s*velocity[0]+c*velocity[1]])
        seen = set()
        for t in tracks:
            if not 0 <= t.id < STRUCTURE_ID or math.hypot(t.vx, t.vy) < STATIC_SPEED:
                continue
            seen.add(t.id)
            relative = np.array([t.x-pose[0], t.y-pose[1]])
            own = np.array([t.vx, t.vy])
            credit = self.credit.get(t.id, DEFAULT_CREDIT)
            if np.linalg.norm(relative) < ENGAGE and relative @ (own-robot) < 0:
                start = self.start.setdefault(t.id, own.copy())
                miss, when = closest_pass(relative, own-robot)
                held, _ = closest_pass(relative, start-robot)
                widened = miss-held >= YIELD_EVIDENCE and np.linalg.norm(own-start) > NOISE
                self.widening[t.id] = self.widening.get(t.id, 0)+1 if widened else 0
                if self.widening[t.id] >= SUSTAINED:
                    credit += CREDIT_STEP
                elif not widened and miss < CONFLICT and when < HOLD_TIME and miss-held < .05:
                    credit -= 2*CREDIT_STEP
            else:
                self.start.pop(t.id, None)
                self.widening.pop(t.id, None)
            self.credit[t.id] = min(1., max(0., credit))
        for gone in set(self.credit)-seen:
            self.credit.pop(gone, None)
            self.start.pop(gone, None)
            self.widening.pop(gone, None)
        return dict(self.credit)
