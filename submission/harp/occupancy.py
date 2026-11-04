"""A 2D map of what doesn't move, from /harp/static_cloud (harp/mapper.py builds it on a mapping
walk), kept in the run's events and given to later walks (course.py --map RUN): Nav2's static
layer plans on it and harp/localizer.py keeps the robot on it.

Cells hold log-odds. A return between MIN_Z and MAX_Z (Nav2's obstacle heights) marks its cell.
A ray clears only the cells it passes below CLEAR_Z: a ray high over a cell says nothing about a
low box there, so mostly floor returns clear, near where they land.
"""
import base64
import math
import zlib

import numpy as np

RESOLUTION = .05          # m, as Nav2's costmaps
SIZE = 32.0               # m: the square mapped, centred on where mapping started
MIN_Z, MAX_Z = .08, 1.8   # m: what counts as an obstacle (nav2_params.yaml's heights)
CLEAR_Z = .4              # m: a ray clears a cell only passing below this
MAX_RANGE = 8.0           # m: returns farther are left out (sparse, noisier)
MAX_RAYS = 3000           # clearing rays per scan, at most
HIT, MISS = .85, -.4      # log-odds per scan
LOW, HIGH = -2.0, 3.5     # log-odds limits, so a moved box is unlearnt in a few scans
OCCUPIED, FREE = .6, -.4  # log-odds above/below which a cell is occupied/free
MARGIN = 1.0              # m kept round the known cells when saving


class OccupancyMap:
    """map.integrate(points, origin) per scan (map-frame points and the sensor's position);
    map.grid() gives Nav2's values: 100 occupied, 0 free, -1 unknown."""

    def __init__(self, size=SIZE, resolution=RESOLUTION):
        self.resolution = resolution
        self.cells = int(round(size / resolution))
        self.origin = np.array([-size / 2, -size / 2])   # map-frame corner of cell (0, 0)
        self.odds = np.zeros((self.cells, self.cells), dtype=np.float32)
        self.rng = np.random.default_rng(0)

    def index(self, xy):
        """Cell (row, column) of each point, and whether it is on the map."""
        ij = np.floor((xy - self.origin) / self.resolution).astype(int)
        inside = ((ij >= 0) & (ij < self.cells)).all(axis=1)
        return ij[:, 1], ij[:, 0], inside

    def integrate(self, points, origin):
        points = np.asarray(points, dtype=float)
        origin = np.asarray(origin, dtype=float)
        if not len(points) or not np.isfinite(origin).all():
            return
        points = points[np.isfinite(points).all(axis=1)]
        reach = np.hypot(*(points[:, :2] - origin[:2]).T)
        points = points[(reach > 1e-3) & (reach <= MAX_RANGE)]
        hit = (points[:, 2] >= MIN_Z) & (points[:, 2] <= MAX_Z)
        rows, cols, inside = self.index(points[hit, :2])
        marked = np.unique(rows[inside] * self.cells + cols[inside])
        cleared = self.clear(points, hit, origin)
        cleared = np.setdiff1d(cleared, marked, assume_unique=True)
        flat = self.odds.reshape(-1)
        flat[marked] = np.minimum(flat[marked] + HIT, HIGH)
        flat[cleared] = np.maximum(flat[cleared] + MISS, LOW)

    def clear(self, points, hit, origin):
        """Flat indices of the cells this scan's rays pass below CLEAR_Z (not a hit's own cell)."""
        low = points[:, 2] < CLEAR_Z
        rays, hits = points[low], hit[low]
        if origin[2] <= CLEAR_Z or not len(rays):
            return np.empty(0, dtype=int)
        if len(rays) > MAX_RAYS:
            keep = self.rng.choice(len(rays), MAX_RAYS, replace=False)
            rays, hits = rays[keep], hits[keep]
        # Along each ray from where it drops below CLEAR_Z to its end (short of a hit's cell).
        start = (origin[2] - CLEAR_Z) / (origin[2] - rays[:, 2])
        length = np.hypot(*(rays[:, :2] - origin[:2]).T)
        end = 1.0 - np.where(hits, 1.5 * self.resolution / np.maximum(length, 1e-6), 0.0)
        span = np.maximum(end - start, 0.0) * length
        steps = int(np.ceil(span.max() / (self.resolution / 2))) + 1 if len(span) else 0
        if steps <= 1:
            return np.empty(0, dtype=int)
        f = np.linspace(0.0, 1.0, steps)
        t = start[:, None] + f[None, :] * (end - start)[:, None]
        valid = (end > start)[:, None] & np.ones_like(t, dtype=bool)
        xy = origin[:2] + t[..., None] * (rays[:, None, :2] - origin[:2])
        rows, cols, inside = self.index(xy[valid])
        return np.unique(rows[inside] * self.cells + cols[inside])

    def grid(self):
        values = np.full(self.odds.shape, -1, dtype=np.int8)
        values[self.odds >= OCCUPIED] = 100
        values[self.odds <= FREE] = 0
        return values


def encode(grid, origin, resolution):
    """A saved map: the known part of grid (and MARGIN round it) as a JSON-able dict."""
    known = np.argwhere(grid >= 0)
    if not len(known):
        return None
    pad = int(math.ceil(MARGIN / resolution))
    r0, c0 = np.maximum(known.min(axis=0) - pad, 0)
    r1, c1 = np.minimum(known.max(axis=0) + pad + 1, grid.shape)
    part = np.ascontiguousarray(grid[r0:r1, c0:c1], dtype=np.int8)
    return {"resolution": float(resolution),
            "origin": [round(float(origin[0] + c0 * resolution), 4), round(float(origin[1] + r0 * resolution), 4)],
            "width": int(part.shape[1]), "height": int(part.shape[0]),
            "cells": base64.b64encode(zlib.compress(part.tobytes(), 9)).decode("ascii")}


def decode(saved):
    """(grid, origin, resolution) of encode's dict; ValueError if it is malformed."""
    try:
        width, height = int(saved["width"]), int(saved["height"])
        resolution = float(saved["resolution"])
        origin = np.array(saved["origin"][:2], dtype=float)
        cells = np.frombuffer(zlib.decompress(base64.b64decode(saved["cells"])), dtype=np.int8)
    except (KeyError, TypeError, IndexError, ValueError, zlib.error) as error:
        raise ValueError(f"bad map: {error}")
    if (width <= 0 or height <= 0 or cells.size != width * height or not resolution > 0
            or not np.isfinite(origin).all() or not np.isin(cells, (-1, 0, 100)).all()):
        raise ValueError("bad map: size, resolution, origin or values")
    return cells.reshape(height, width).copy(), origin, resolution


def pgm(grid):
    """The map as a PGM image (map_server's colours: occupied black, free white, unknown grey),
    top row north."""
    image = np.full(grid.shape, 205, dtype=np.uint8)
    image[grid == 0] = 254
    image[grid == 100] = 0
    image = image[::-1]
    return b"P5\n%d %d\n255\n" % (image.shape[1], image.shape[0]) + image.tobytes()
