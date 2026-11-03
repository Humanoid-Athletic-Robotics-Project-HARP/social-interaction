"""Seeded, continuous pedestrian traffic for the native simulator only.

People accelerate, change heading/speed, occasionally pause, avoid each other,
and keep circulating. Some do not yield to the robot. This is a kinematic
pedestrian model, not a claim of calibrated human behavior.
"""
import os
import json
import numpy as np
import mujoco
import rclpy
from std_msgs.msg import String
from harp_sim.movers import Movers


class CrowdMotion(Movers):
    def __init__(self, model):
        super().__init__(model)
        self.rng = np.random.default_rng(int(os.environ.get('HARP_CROWD_SEED','1')))
        self.positions = np.asarray(self.start).reshape(-1,3).copy()
        self.velocity = np.zeros_like(self.positions)
        self.targets = np.asarray(self.end).reshape(-1,3).copy()
        self.speeds = self.rng.uniform(.35,.7,len(self.names))
        self.scale = float(os.environ.get('HARP_CROWD_SPEED','1.'))
        self.phase = self.rng.uniform(0,6.28,len(self.names))
        self.last = 0.
        self.publisher = None
        self.next_report = 0.
        self.minimum_separation = float('inf')
        self.adversarial = os.environ.get('HARP_CROWD_MODE') == 'adversarial'
        fixed = [g for g in range(model.ngeom) if model.geom_bodyid[g] == 0
                 and model.geom_type[g] != mujoco.mjtGeom.mjGEOM_PLANE]
        # Building walls and blocks are long boxes: a circumscribed circle would
        # fill the hallway, so people keep clear of their nearest surface instead.
        boxes = [g for g in fixed if model.geom(g).name.split('/')[-1].startswith(('wall','block'))]
        self.walls = [(model.geom_pos[g,:2].copy(),model.geom_size[g,:2].copy()) for g in boxes]
        self.fixed = [(model.geom_pos[g,:2].copy(),float(np.linalg.norm(model.geom_size[g,:2])))
                      for g in fixed if g not in boxes]
        # Optional building routes, numeric <mover>_route = x0 y0 x1 y1 ...: people
        # walk waypoint to waypoint through doors and hallways, turning back at
        # either end, instead of along one straight start/end line.
        self.routes = []
        for b in range(model.nbody):
            if model.body_mocapid[b] >= 0:
                try:
                    route = np.asarray(model.numeric(model.body(b).name+'_route').data,float)
                    self.routes.append(route.reshape(-1,2))
                except KeyError:
                    self.routes.append(None)
        self.leg = [1]*len(self.names)
        self.heading = [1]*len(self.names)
        self.stuck = np.zeros(len(self.names))
        self.detour = [None]*len(self.names)
        self.detour_until = np.zeros(len(self.names))
        for i,route in enumerate(self.routes):
            if route is not None:
                self.targets[i,:2] = route[1]

    def _room(self, i, point):
        """Free distance around a point: nearest wall surface or other person."""
        gaps = [np.linalg.norm(point-(c+np.clip(point-c,-h,h))) for c,h in self.walls]
        gaps += [np.linalg.norm(point-self.positions[j,:2]) for j in range(len(self.positions)) if j != i]
        return min(gaps, default=np.inf)

    def _out_of_walls(self, i):
        for center,half in self.walls:
            offset = self.positions[i,:2]-center
            away = offset-np.clip(offset,-half,half)
            length = np.linalg.norm(away)
            if length < .22:
                if length < 1e-6:  # centre inside: leave by the shallowest side
                    axis = int(np.argmin(half-abs(offset)))
                    away = np.eye(2)[axis]*np.sign(offset[axis] or 1.)
                    length = 1.
                    offset = offset-np.eye(2)[axis]*(offset[axis]-np.sign(offset[axis] or 1.)*half[axis])
                self.positions[i,:2] = center+np.clip(offset,-half,half)+.22*away/length

    def apply(self, data, t):
        dt = min(.02,max(0.,t-self.last))
        self.last = t
        for i,mocap in enumerate(self.mocap):
            began = self.started[i]
            if began is None or t < began:
                data.mocap_pos[mocap] = self.positions[i]
                continue
            offset = self.targets[i,:2]-self.positions[i,:2]
            distance = np.linalg.norm(offset)
            route = self.routes[i]
            if route is not None and distance < .35:
                if not 0 <= self.leg[i]+self.heading[i] < len(route):
                    self.heading[i] = -self.heading[i]
                    self.speeds[i] = self.rng.uniform(.35,.7)
                self.leg[i] += self.heading[i]
                self.targets[i,:2] = route[self.leg[i]]
                offset = self.targets[i,:2]-self.positions[i,:2]
                distance = np.linalg.norm(offset)
            elif route is None and distance < .3:
                self.targets[i] = self.start[i] if np.linalg.norm(self.targets[i]-self.end[i]) < .1 else self.end[i]
                self.speeds[i] = self.rng.uniform(.35,.7)
            direction = offset/max(distance,.01)
            side = np.array([-direction[1],direction[0]])
            speed = self.scale*self.speeds[i]*(1+.22*np.sin(.7*t+self.phase[i]))
            # A subset stops briefly; they then continue rather than disappearing.
            if i % 3 == 1 and (t-began+self.phase[i]) % 18 < 1.5:
                speed = 0.
            desired = speed*(direction+.16*np.sin(.5*t+self.phase[i])*side)
            for j in range(len(self.mocap)):
                if j == i:
                    continue
                away = self.positions[i,:2]-self.positions[j,:2]
                length = np.linalg.norm(away)
                if .01 < length < 1.8:
                    desired += .7*np.exp((.6-length)/.2)*away/length
                    relative = self.velocity[i,:2]-self.velocity[j,:2]
                    when = np.clip(-away@relative/max(relative@relative,1e-9),0,1.2)
                    if np.linalg.norm(away+when*relative) < .55:
                        desired += .45*np.array([-away[1],away[0]])/length
            for center,radius in self.fixed:
                away = self.positions[i,:2]-center
                length = np.linalg.norm(away)
                if .01 < length < radius+.8:
                    desired += 2.*(radius+.8-length)*away/length
            # Normal pedestrians anticipate contact, as they do with each other.
            # Adversarial mode preserves three deliberately non-yielding agents.
            if not self.adversarial or i % 3:
                away = self.positions[i,:2]-data.qpos[:2]
                length = np.linalg.norm(away)
                relative_velocity = self.velocity[i,:2]-data.qvel[:2]
                closest_t = np.clip(-away@relative_velocity/max(relative_velocity@relative_velocity,1e-9),0,1.2)
                closest = np.linalg.norm(away+closest_t*relative_velocity)
                if .01 < length < 1.8:
                    desired += .9*np.exp((.85-length)/.25)*away/length
                    if closest < .85:
                        tangent = np.array([-away[1],away[0]])/length
                        desired += .45*tangent
            # Someone who wants to walk but has stood still (a doorway standoff, a
            # corner) steps aside toward the side with more room, as people do.
            walking = speed > 0.
            self.stuck[i] = self.stuck[i]+dt if walking and np.linalg.norm(self.velocity[i,:2]) < .1 else 0.
            if self.stuck[i] > 1.2 and t >= self.detour_until[i]:
                sides = [side, -side]
                self.detour[i] = max(sides, key=lambda v: self._room(i, self.positions[i,:2]+.6*v))
                self.detour_until[i], self.stuck[i] = t+1.2, 0.
            if t < self.detour_until[i]:
                # Shift only the goal pull sideways; avoidance of others still applies.
                desired += .75*speed*(self.detour[i]-direction)
            for center,half in self.walls:
                # Short range, so a 1.2 m doorway does not push people back out.
                offset = self.positions[i,:2]-center
                away = offset-np.clip(offset,-half,half)
                length = np.linalg.norm(away)
                if .01 < length < .55:
                    normal = away/length
                    desired += 3.*(.55-length)*normal
                    into = -float(desired@normal)
                    if into > 0:
                        # Slide along the surface toward the waypoint (or round the
                        # nearer corner) instead of pressing into it, so a column or
                        # door frame is never a dead end.
                        tangent = np.array([-normal[1],normal[0]])
                        toward = float(tangent@(self.targets[i,:2]-self.positions[i,:2]))
                        if abs(toward) < .2:   # waypoint straight ahead: nearer corner
                            toward = float(tangent@offset)
                        desired += into*normal+into*np.sign(toward or 1.)*tangent
            desired *= min(1.,.85*self.scale/max(np.linalg.norm(desired),1e-9))
            change = desired-self.velocity[i,:2]
            self.velocity[i,:2] += change*min(1.,.9*self.scale*dt/max(np.linalg.norm(change),1e-9))
            self.positions[i,:2] += self.velocity[i,:2]*dt
            data.mocap_pos[mocap] = self.positions[i]
        # Bodies are solid: two people never overlap, and nobody is pushed into a
        # wall. A few projection passes resolve crowding in doors and gaps.
        for _ in range(3):
            for i in range(len(self.positions)):
                for j in range(i+1, len(self.positions)):
                    away = self.positions[i,:2]-self.positions[j,:2]
                    length = np.linalg.norm(away)
                    if length < .42:
                        push = (.42-length)/2*(away/length if length > 1e-6 else np.array([1.,0.]))
                        self.positions[i,:2] += push
                        self.positions[j,:2] -= push
                self._out_of_walls(i)
        for i,mocap in enumerate(self.mocap):
            data.mocap_pos[mocap] = self.positions[i]
        if len(self.positions) > 1:
            distance = np.linalg.norm(self.positions[:,None,:2]-self.positions[None,:,:2],axis=-1)
            np.fill_diagonal(distance,float('inf'))
            self.minimum_separation = min(self.minimum_separation,float(distance.min()))
        if rclpy.ok() and t >= self.next_report:
            if self.publisher is None:
                self.node = rclpy.create_node('crowd_truth')
                self.publisher = self.node.create_publisher(String,'/sim/crowd',10)
            self.publisher.publish(String(data=json.dumps(dict(time=t,
                minimum_separation=(self.minimum_separation if np.isfinite(self.minimum_separation) else None),
                people=[dict(x=float(p[0]),y=float(p[1]),vx=float(v[0]),vy=float(v[1]))
                        for p,v in zip(self.positions,self.velocity)]))))
            self.next_report = t+.1
