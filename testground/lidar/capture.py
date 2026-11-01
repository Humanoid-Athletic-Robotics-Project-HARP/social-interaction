"""Reads a capture folder written by record.py: the clouds as numpy arrays laid
out the way the robot sent them, the lidar IMU and the robot's low state.

Also the lidar's assumed mount (submission/harp/lidar_mount.py, the one Nav2 and the
sim use), so points can be shown in torso_link, whose axes match the robot's
(x forward, y left, z up). The analysis checks that assumption against the floor.
"""

import json
from pathlib import Path
import sys

import numpy as np

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "submission" / "harp"))
import lidar_mount  # noqa: E402  (numpy only)

# sensor_msgs/PointField datatypes
DTYPES = {1: "i1", 2: "u1", 3: "i2", 4: "u2", 5: "i4", 6: "u4", 7: "f4", 8: "f8"}

MOUNT_R = lidar_mount.rotation()       # the lidar's orientation in torso_link
MOUNT_T = lidar_mount.POSITION         # m, its origin in torso_link


def to_torso(xyz):
    """Points in the lidar's frame to torso_link, by the assumed mount."""
    return xyz @ MOUNT_R.T + MOUNT_T


def read_table(path):
    """A CSV with a header row, as a dict of column name to float64 array."""
    if not path.exists() or path.stat().st_size == 0:
        return {}
    names = path.open().readline().strip().split(",")
    rows = np.loadtxt(path, delimiter=",", skiprows=1, ndmin=2)
    if rows.size == 0:
        return {n: np.zeros(0) for n in names}
    return {n: rows[:, i] for i, n in enumerate(names)}


class Capture:
    def __init__(self, folder):
        self.folder = Path(folder)
        self.meta = json.loads((self.folder / "meta.json").read_text())
        self.layout = self.meta.get("layout")
        if self.layout is None:
            raise ValueError(f"{self.folder}: no clouds were recorded")
        # clouds.csv's ns values are exact in int64, not in float64
        index = np.loadtxt(self.folder / "clouds.csv", delimiter=",", skiprows=1,
                           dtype=np.int64, ndmin=2)
        self.arrival_ns, self.stamp_ns = index[:, 0], index[:, 1]
        self.offset, self.nbytes = index[:, 2], index[:, 3]
        self.width, self.height = index[:, 4], index[:, 5]
        t0 = self.stamp_ns[0]
        self.t = (self.stamp_ns - t0) * 1e-9            # s, robot clock, from the first cloud
        self.arrival = (self.arrival_ns - t0) * 1e-9    # s, PC2's clock, same origin
        self.dtype = self.point_dtype()
        self.data = np.memmap(self.folder / "clouds.bin", dtype=np.uint8, mode="r")
        self.imu = read_table(self.folder / "imu.csv")
        self.lowstate = read_table(self.folder / "lowstate.csv")

    def point_dtype(self):
        order = ">" if self.layout["is_bigendian"] else "<"
        names, formats, offsets = [], [], []
        for f in self.layout["fields"]:
            names.append(f["name"])
            kind = order + DTYPES[f["datatype"]]
            formats.append((kind, f["count"]) if f["count"] > 1 else kind)
            offsets.append(f["offset"])
        return np.dtype({"names": names, "formats": formats, "offsets": offsets,
                         "itemsize": self.layout["point_step"]})

    @property
    def fields(self):
        return [f["name"] for f in self.layout["fields"]]

    def __len__(self):
        return len(self.offset)

    def cloud(self, i):
        """Cloud i as a structured array, one record per point, every field."""
        raw = self.data[self.offset[i]:self.offset[i] + self.nbytes[i]]
        return np.frombuffer(raw, dtype=self.dtype, count=len(raw) // self.dtype.itemsize)

    def xyz(self, i, cloud=None):
        """Cloud i's returns in the lidar's frame, (n, 3), and a mask of which of the
        cloud's points they are: finite and not at the origin (no return)."""
        c = self.cloud(i) if cloud is None else cloud
        xyz = np.stack([c["x"], c["y"], c["z"]], axis=1).astype(np.float64)
        keep = np.isfinite(xyz).all(axis=1) & (np.abs(xyz).sum(axis=1) > 0)
        return xyz[keep], keep

    def frames(self, limit=None):
        """Indices of up to `limit` clouds spread evenly over the capture."""
        n = len(self)
        if limit is None or limit >= n:
            return np.arange(n)
        return np.unique(np.linspace(0, n - 1, limit).round().astype(int))
