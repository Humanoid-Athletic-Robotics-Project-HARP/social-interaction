"""Making way: getting the robot out of a tight spot it is stuck in among others.

In HIM's office (2026-11-03) the robot stood for over 15 s at a corridor corner, turning on
the spot: a plant, a person standing there and a cart that stopped to give way closed the
corner, the cart waited for the robot and the robot for a gap. A person would step back to
somewhere wider and let the others go first. So when the predictive filter has been avoiding
for STALL_TIME without getting STALL_PROGRESS nearer its goal, MakeWay picks a spot within
REACH that is clear of fixed geometry (walls, and anything standing, as the planner's field
has them), as far as it can be from tracked people, and reachable in a straight line; the
filter then plans towards it, with every usual check, for at most WALK_TIME, and hands back
to Nav2. COOLDOWN keeps it from swinging between the two.
"""
from collections import deque
import math

import numpy as np

STALL_TIME = 6.0        # s avoiding without progress before making way
STALL_PROGRESS = 0.3    # m nearer the goal that counts as progress
NEAR_GOAL = 1.0         # m: never this close to the goal (hovering there is something else)
REACH = 2.0             # m: farthest spot (inside the planner's field)
WALK_TIME = 8.0         # s at most walking to the spot
COOLDOWN = 10.0         # s after making way before it can start again
ARRIVED = 0.25          # m from the spot that counts as there
BETTER = 0.25           # how much better the spot must score than standing still
AVOIDING = ("yield", "evade", "blocked")
STRUCTURE_ID = 10**9


class MakeWay:
    def __init__(self, robot_radius=0.4):
        self.robot_radius = robot_radius
        self.history = deque()     # (time, distance to the goal)
        self.spot = None
        self.until = -math.inf
        self.cooldown = -math.inf

    def update(self, now, pose, goal, mode, tracks, clearance):
        """The spot to walk to now (odom xy), or None to follow Nav2. clearance(points N x 2)
        gives the distance to the nearest fixed envelope, or is None without one."""
        if pose is None or goal is None:
            self.history.clear()
            self.spot = None
            return None
        xy, goal = np.asarray(pose[:2], dtype=float), np.asarray(goal, dtype=float)
        left = float(np.linalg.norm(goal - xy))
        self.history.append((now, left))
        while self.history and now - self.history[0][0] > STALL_TIME + 2.0:
            self.history.popleft()
        if self.spot is not None:
            if now > self.until or np.linalg.norm(self.spot - xy) < ARRIVED:
                self.spot, self.cooldown = None, now + COOLDOWN
                self.history.clear()
                return None
            return self.spot
        if now < self.cooldown or left < NEAR_GOAL or mode not in AVOIDING:
            return None
        before = [d for t, d in self.history if now - t >= STALL_TIME]
        if not before or before[-1] - left > STALL_PROGRESS:
            return None
        self.spot = self.choose(xy, tracks, clearance)
        if self.spot is not None:
            self.until = now + WALK_TIME
        return self.spot

    def score(self, points, tracks, clearance):
        """Higher is a better place to wait: far from people, clear of fixed geometry."""
        people = np.array([[t.x, t.y, t.radius] for t in tracks if t.id < STRUCTURE_ID]).reshape(-1, 3)
        gap = (np.min(np.linalg.norm(points[:, None] - people[None, :, :2], axis=2) - people[None, :, 2],
                      axis=1) - self.robot_radius) if len(people) else np.full(len(points), 1.)
        fixed = (clearance(points) - self.robot_radius) if clearance is not None else np.full(len(points), 1.)
        return np.minimum(gap, 1.) + np.minimum(fixed, 1.), fixed   # beyond 1 m, people and walls are far enough

    def choose(self, xy, tracks, clearance):
        angles = np.arange(16) * 2 * math.pi / 16
        radii = np.array([.6, 1.0, 1.4, REACH])
        spots = (xy + np.stack([np.outer(radii, np.cos(angles)), np.outer(radii, np.sin(angles))], -1)).reshape(-1, 2)
        score, fixed = self.score(spots, tracks, clearance)
        score = score - .1 * np.linalg.norm(spots - xy, axis=1)
        here, room = self.score(xy[None], tracks, clearance)
        ok = fixed >= .15
        if clearance is not None:
            # The straight way there clear of fixed geometry too, never tighter than where it
            # stands (it is stuck somewhere tight); the planner still holds the body limit.
            need = min(self.robot_radius + .05, room[0] + self.robot_radius - .05)
            for f in (.25, .5, .75):
                ok &= clearance(xy + f * (spots - xy)) >= need
        if not ok.any():
            return None
        best = int(np.argmax(np.where(ok, score, -np.inf)))
        return spots[best] if score[best] > here[0] + BETTER else None


def toward(pose, spot, speed):
    """A body-frame request (vx, vy, 0) to walk straight to spot, keeping the heading."""
    d = np.asarray(spot, dtype=float) - np.asarray(pose[:2], dtype=float)
    distance = float(np.linalg.norm(d))
    if distance < 1e-6:
        return (0., 0., 0.)
    c, s = math.cos(pose[2]), math.sin(pose[2])
    body = np.array([c * d[0] + s * d[1], -s * d[0] + c * d[1]]) / distance * min(speed, distance)
    return (float(body[0]), float(body[1]), 0.)
