"""Which still objects are not part of the building, from a map made in it empty (harp/mapper.py):
a compact track standing on what the map shows as open floor, clear of every mapped wall, is a
person or something left there since.

harp/predictive_avoidance.py leaves such tracks out of its static cloud, as it does moving ones,
so Nav2 doesn't take a person standing by the only doorway for a wall. With the building's walls
known, Nav2 then had no route at all and gave up, again and again (held-out 20261129: a walker
stood by the partition's doorway the whole run); without a map it had invented a way round
through space it hadn't seen. The filter, which plans round every track, still or moving, keeps
the robot clear of them. Pillars, boxes and furniture are in the map and stay in Nav2's.
Off by default (HARP_PREDICTIVE_MAP_FLOOR=1): in HIM it kept Nav2's route, but the robot then stood
off at the doorway and none of 5 such runs got through (lab log, November 4).
"""
import math

import numpy as np
from scipy.ndimage import distance_transform_edt

GAP = .1   # m beyond a track's radius that must be clear of mapped walls


class MapFloor:
    def __init__(self, grid, origin, resolution):
        self.origin, self.resolution = np.asarray(origin, dtype=float), float(resolution)
        self.free = grid == 0
        occupied = grid == 100
        self.clear = (distance_transform_edt(~occupied) * resolution if occupied.any()
                      else np.full(grid.shape, math.inf))

    def on_floor(self, xy, radius):
        """Whether a still object of this radius at map point xy stands on open floor."""
        c, r = np.floor((np.asarray(xy, dtype=float) - self.origin) / self.resolution).astype(int)
        rows, cols = self.free.shape
        if not (0 <= r < rows and 0 <= c < cols):
            return False
        return bool(self.free[r, c]) and self.clear[r, c] >= radius + GAP

    def temporary(self, tracks, stamp, pose, size, structure_id):
        """Ids of compact (radius under size), freshly seen tracks standing on open floor;
        pose: the map-to-odom transform (x, y, heading)."""
        c, s = math.cos(pose[2]), math.sin(pose[2])
        out = set()
        for t in tracks:
            if t.id >= structure_id or t.last_seen != stamp or t.radius >= size:
                continue
            xy = (pose[0] + c * t.x - s * t.y, pose[1] + s * t.x + c * t.y)
            if self.on_floor(xy, t.radius):
                out.add(t.id)
        return out
