"""Where the robot is on a saved map (harp/occupancy.py): the map-to-odom correction (x, y,
heading) that lays the scan's wall returns onto the map's walls, for harp/mapper.py.

Leg odometry drifts; the map doesn't. Each match is a correlative search (Olson 2009) round the
current estimate, coarse to fine: every pose in a small window is scored by how near its returns
land to mapped walls (a smooth likelihood field, so no local minima inside the window), and the
best wins. The window turns the scan about the robot (pivot), not odom's origin: turned about
an origin metres away, a small heading change would also move the robot by heading x distance,
past the window's reach. A small pull towards the previous estimate (TIE) breaks ties in a
direction the scan can't fix, such as along a corridor, so odometry keeps it there. A match is used only if enough
returns land on walls (MIN_INLIERS, MIN_FRACTION) and it moves the estimate less than MAX_JUMP.
"""
import math

import numpy as np
from scipy.ndimage import distance_transform_edt

CAP = 1.0                              # m: distances beyond this are clipped
SIGMA = .05                            # m: the likelihood field's width
INLIER = .1                            # m: a return this near a wall fits the map
TIE = (.02, .5, math.radians(10))      # score lost at (xy m, heading rad) from the prior
WINDOW = (.15, math.radians(3))        # m, rad: each match's search round the estimate
SEARCH = (.5, math.radians(10))        # m, rad: the start's search round the given pose
LEVELS = ((.03, math.radians(1)), (.01, math.radians(.3)), (.003, math.radians(.1)))
MIN_INLIERS = 80
MIN_FRACTION = .4
MAX_JUMP = (.3, math.radians(8))       # m, rad: a larger change in one match is refused
WALL_Z = (.3, 1.8)                     # m: returns used (walls and furniture, not the floor)
MAX_POINTS = 500


def compose(pose, points):
    """points (N, 2) in odom -> map, for pose (x, y, heading) the map-to-odom transform."""
    c, s = math.cos(pose[2]), math.sin(pose[2])
    return points @ np.array([[c, s], [-s, c]]) + pose[:2]


def around(pose, pivot):
    """(the pivot's map position, heading) of map-to-odom pose."""
    return np.r_[compose(pose, np.asarray(pivot, dtype=float)[None])[0], pose[2]]


def from_pivot(centre, pivot):
    """The map-to-odom pose putting the pivot (odom) at centre (map x, y, heading)."""
    c, s = math.cos(centre[2]), math.sin(centre[2])
    return np.array([centre[0] - (c * pivot[0] - s * pivot[1]), centre[1] - (s * pivot[0] + c * pivot[1]), centre[2]])


def offsets(half, step):
    n = int(round(half / step))
    return np.arange(-n, n + 1) * step


class Localizer:
    def __init__(self, grid, origin, resolution):
        occupied = grid == 100
        if not occupied.any():
            raise ValueError("the map has no walls to localize on")
        self.origin, self.resolution = np.asarray(origin, dtype=float), float(resolution)
        distance = np.minimum(distance_transform_edt(~occupied) * resolution, CAP)
        self.field = distance
        self.likely = np.exp(-distance ** 2 / (2 * SIGMA ** 2))
        self.rng = np.random.default_rng(0)

    def sample(self, xy, grid=None):
        """Bilinear value of grid (default: the distance to the nearest wall) at each map point;
        off the map, CAP for the distance and 0 for the likelihood."""
        grid = self.field if grid is None else grid
        f = (xy - self.origin) / self.resolution - .5
        c0, r0 = np.floor(f[..., 0]).astype(int), np.floor(f[..., 1]).astype(int)
        fx, fy = f[..., 0] - c0, f[..., 1] - r0
        rows, cols = grid.shape
        inside = (c0 >= 0) & (r0 >= 0) & (c0 < cols - 1) & (r0 < rows - 1)
        c0, r0 = np.clip(c0, 0, cols - 2), np.clip(r0, 0, rows - 2)
        v = ((1 - fx) * (1 - fy) * grid[r0, c0] + fx * (1 - fy) * grid[r0, c0 + 1]
             + (1 - fx) * fy * grid[r0 + 1, c0] + fx * fy * grid[r0 + 1, c0 + 1])
        return np.where(inside, v, CAP if grid is self.field else 0.0)

    def thin(self, points):
        """At most MAX_POINTS returns, one per map cell."""
        cells = np.floor(points / self.resolution).astype(int)
        _, first = np.unique(cells[:, 0] * 1000003 + cells[:, 1], return_index=True)
        points = points[first]
        if len(points) > MAX_POINTS:
            points = points[self.rng.choice(len(points), MAX_POINTS, replace=False)]
        return points

    def best(self, local, centre, half, step, prior):
        """The best-scoring (pivot map x, y, heading) on a grid of step round centre (half the
        window's size); local: the points relative to the pivot, in odom."""
        dx = offsets(half[0], step[0])
        shifts = np.stack(np.meshgrid(dx, dx), axis=-1).reshape(-1, 2)
        top, top_score = centre, -np.inf
        for turn in offsets(half[1], step[1]):
            heading = centre[2] + turn
            xy = centre[:2] + shifts
            base = compose(np.array([0.0, 0.0, heading]), local)
            score = self.sample(base[None] + xy[:, None], self.likely).mean(axis=1)
            score = score - TIE[0] * (((xy - prior[:2]) ** 2).sum(axis=1) / TIE[1] ** 2
                                      + math.remainder(heading - prior[2], math.tau) ** 2 / TIE[2] ** 2)
            k = int(np.argmax(score))
            if score[k] > top_score:
                top_score, top = score[k], np.array([xy[k, 0], xy[k, 1], heading])
        return top

    def refine(self, points, pivot, centre, half, prior):
        local = points - pivot
        found = self.best(local, centre, half, LEVELS[0], prior)
        for coarse, fine in zip(LEVELS, LEVELS[1:]):
            found = self.best(local, found, coarse, fine, prior)
        pose = from_pivot(found, pivot)
        d = self.sample(compose(pose, points))
        inliers = int((d < INLIER).sum())
        return pose, inliers, inliers / len(points)

    def prepared(self, points, pivot):
        points = self.thin(np.asarray(points, dtype=float))
        pivot = points.mean(axis=0) if pivot is None and len(points) else pivot
        return points, None if pivot is None else np.asarray(pivot, dtype=float)

    def match(self, points, prior, pivot=None):
        """(pose, inliers, fraction): the map-to-odom pose fitting points (N, 2, odom) best, with
        the pivot (the robot's odom position; default the points' mean) within WINDOW of where
        prior puts it."""
        prior = np.asarray(prior, dtype=float)
        points, pivot = self.prepared(points, pivot)
        if not len(points):
            return prior, 0, 0.0
        centre = around(prior, pivot)
        return self.refine(points, pivot, centre, WINDOW, centre)

    def search(self, points, guess, pivot=None):
        """The start: the same, within SEARCH of guess."""
        guess = np.asarray(guess, dtype=float)
        points, pivot = self.prepared(points, pivot)
        if not len(points):
            return guess, 0, 0.0
        centre = around(guess, pivot)
        found = self.best(points - pivot, centre, SEARCH, (.05, math.radians(1)), centre)
        return self.refine(points, pivot, found, (.05, math.radians(1)), centre)


def accept(pose, current, inliers, fraction, pivot=(0.0, 0.0)):
    """Whether a match may update the estimate (how far it moves the pivot, the robot)."""
    jump = np.hypot(*(around(np.asarray(pose, dtype=float), pivot)[:2] - around(np.asarray(current, dtype=float), pivot)[:2]))
    turn = abs(math.remainder(pose[2] - current[2], math.tau))
    return (inliers >= MIN_INLIERS and fraction >= MIN_FRACTION
            and jump <= MAX_JUMP[0] and turn <= MAX_JUMP[1])
