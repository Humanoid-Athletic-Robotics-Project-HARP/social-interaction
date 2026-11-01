"""Reads a capture folder written by record.py: the MID-360's raw Livox SDK2 packets,
decoded into 100 ms clouds of points, its IMU, its status messages, and the robot's
low state.

Packets (Livox SDK2, as this lidar sends them): a 36-byte header (version, length,
time_interval in 0.1 us, dot_num, udp_cnt, frame_cnt, data_type, time_type, crc32,
timestamp in ns) then the data: 96 points of int32 x, y, z in mm plus uint8
reflectivity and tag (data_type 1), or the IMU's float32 gyro (rad/s) and
accelerometer (g) (data_type 0). Status messages are Livox control frames (cmd
0x0102) holding the lidar's key-value parameters: serial number, firmware, network,
field of view, temperature. A point at (0, 0, 0) is no return.

The stream has no frames of its own: a cloud here is 100 ms of points by the lidar's
clock (one full pass at its 10 Hz). Points are in the lidar's own frame (upside down
in the head); to_torso() applies the mount Nav2 and the sim assume
(submission/harp/lidar_mount.py), which the analysis checks against the floor.
"""

import json
from pathlib import Path
import struct
import sys

import numpy as np

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "submission" / "harp"))
import lidar_mount  # noqa: E402  (numpy only)

PERIOD = 0.1   # s of points in one cloud
RECORD = struct.Struct("<qHH")   # before each packet in udp.bin: arrival ns, port, length
PORTS = {56301: "points", 56401: "imu", 56201: "status"}
HEADER = np.dtype({"names": ["version", "length", "time_interval", "dot_num", "udp_cnt",
                             "frame_cnt", "data_type", "time_type", "crc32", "timestamp"],
                   "formats": ["u1", "<u2", "<u2", "<u2", "<u2", "u1", "u1", "u1", "<u4", "<u8"],
                   "offsets": [0, 1, 3, 5, 7, 9, 10, 11, 24, 28], "itemsize": 36})
POINT = np.dtype([("x", "<i4"), ("y", "<i4"), ("z", "<i4"), ("reflectivity", "u1"), ("tag", "u1")])
IMU = np.dtype([("gx", "<f4"), ("gy", "<f4"), ("gz", "<f4"), ("ax", "<f4"), ("ay", "<f4"),
                ("az", "<f4")])

# Livox SDK2's parameter keys (livox_lidar_def.h), as they appear in status messages
KEYS = {0x0000: "pcl_data_type", 0x0001: "pattern_mode", 0x0002: "dual_emit_en",
        0x0003: "point_send_en", 0x0004: "lidar_ip", 0x0005: "status_host",
        0x0006: "point_host", 0x0007: "imu_host", 0x0008: "ctl_host", 0x0009: "log_host",
        0x0010: "vehicle_speed", 0x0011: "environment_temp", 0x0012: "install_attitude",
        0x0013: "blind_spot", 0x0014: "frame_rate", 0x0015: "fov_cfg0", 0x0016: "fov_cfg1",
        0x0017: "fov_cfg_en", 0x0018: "detect_mode", 0x0019: "func_io_cfg",
        0x001A: "work_mode", 0x001B: "glass_heat", 0x001C: "imu_data_en", 0x001D: "fusa_en",
        0x001E: "force_heat_en", 0x0020: "work_mode_after_boot",
        0x8000: "sn", 0x8001: "product_info", 0x8002: "version_app",
        0x8003: "version_loader", 0x8004: "version_hardware", 0x8005: "mac",
        0x8006: "cur_work_state", 0x8007: "core_temp", 0x8008: "power_up_count",
        0x8009: "local_time_now", 0x800A: "last_sync_time", 0x800B: "time_offset",
        0x800C: "time_sync_type", 0x800D: "status_code", 0x800E: "lidar_diag_status",
        0x800F: "lidar_flash_status", 0x8010: "fw_type", 0x8011: "hms_code",
        0x8012: "cur_glass_heat_state"}

MOUNT_R = lidar_mount.rotation()       # the lidar's orientation in torso_link
MOUNT_T = lidar_mount.POSITION         # m, its origin in torso_link


def to_torso(xyz):
    """Points in the lidar's frame to torso_link, by the assumed mount."""
    return xyz @ MOUNT_R.T + MOUNT_T


def ip_port(v):
    """A Livox host config: IPv4, then uint16 port(s)."""
    ports = struct.unpack_from(f"<{(len(v) - 4) // 2}H", v, 4)
    return f"{'.'.join(str(b) for b in v[:4])} ports {', '.join(str(p) for p in ports if p)}"


def value(key, v):
    """A status value made readable where its layout is known, else hex."""
    name = KEYS.get(key)
    if name in ("sn", "product_info"):
        return v.split(b"\0")[0].decode(errors="replace")
    if name == "lidar_ip":
        return f"{'.'.join(map(str, v[:4]))} mask {'.'.join(map(str, v[4:8]))} gw {'.'.join(map(str, v[8:12]))}"
    if name and name.endswith("_host"):
        return ip_port(v)
    if name and name.startswith("version_"):
        return ".".join(str(b) for b in v)
    if name == "mac":
        return ":".join(f"{b:02x}" for b in v)
    if name in ("fov_cfg0", "fov_cfg1"):
        yaw0, yaw1, pitch0, pitch1 = struct.unpack_from("<4i", v)
        return f"yaw {yaw0} to {yaw1} deg, pitch {pitch0} to {pitch1} deg"
    if name == "install_attitude":
        r, p, y, tx, ty, tz = struct.unpack_from("<3f3i", v)
        return f"roll {r} pitch {p} yaw {y} deg, x {tx} y {ty} z {tz} mm"
    if name == "core_temp":
        return f"{struct.unpack_from('<i', v)[0] / 100:.2f} C"
    if name in ("local_time_now", "last_sync_time", "time_offset"):
        return f"{struct.unpack_from('<q', v)[0] / 1e9:.3f} s"
    if len(v) in (1, 2, 4) and name not in ("hms_code",):
        return str(int.from_bytes(v, "little"))
    return v.hex()


def decode_status(packet):
    """A status message's parameters as {name (or hex key): readable value}."""
    if len(packet) < 28 or packet[0] != 0xAA:
        return {}
    count = struct.unpack_from("<H", packet, 24)[0]
    out, i = {}, 28
    for _ in range(count):
        if i + 4 > len(packet):
            break
        key, length = struct.unpack_from("<HH", packet, i)
        out[KEYS.get(key, f"0x{key:04x}")] = value(key, packet[i + 4:i + 4 + length])
        i += 4 + length
    return out


def read_table(path):
    """A CSV with a header row, as a dict of column name to float64 array."""
    if not path.exists() or path.stat().st_size == 0:
        return {}
    names = path.open().readline().strip().split(",")
    rows = np.loadtxt(path, delimiter=",", skiprows=1, ndmin=2)
    if rows.size == 0:
        return {n: np.zeros(0) for n in names}
    return {n: rows[:, i] for i, n in enumerate(names)}


def packets(raw):
    """udp.bin's records as {port: (arrival ns array, list of packet bytes)}."""
    out = {}
    i = 0
    while i + RECORD.size <= len(raw):
        arrival, port, length = RECORD.unpack_from(raw, i)
        i += RECORD.size
        times, data = out.setdefault(port, ([], []))
        times.append(arrival)
        data.append(raw[i:i + length])
        i += length
    return {p: (np.array(t, dtype=np.int64), d) for p, (t, d) in out.items()}


class Capture:
    def __init__(self, folder):
        self.folder = Path(folder)
        self.meta = json.loads((self.folder / "meta.json").read_text())
        self.headers_only = self.meta.get("headers_only", False)
        by_port = packets((self.folder / "udp.bin").read_bytes())
        self.status = [decode_status(p) for p in by_port.get(56201, ([], []))[1]]
        self.status_arrival_ns = by_port.get(56201, (np.zeros(0, np.int64), []))[0]
        self.read_points(*by_port.get(56301, (np.zeros(0, np.int64), [])))
        self.read_imu(*by_port.get(56401, (np.zeros(0, np.int64), [])))
        self.lowstate = read_table(self.folder / "lowstate.csv")

    def read_points(self, arrival, data):
        if not data:
            raise ValueError(f"{self.folder}: no point packets were recorded")
        self.packet_arrival_ns = arrival
        self.header = np.concatenate([np.frombuffer(d, HEADER, 1) for d in data])
        self.t0 = int(self.header["timestamp"][0])      # ns, the lidar's clock (since power-on)
        stamp = (self.header["timestamp"] - self.t0).astype(np.int64)
        self.packet_t = stamp * 1e-9                     # s from the first packet, lidar clock
        self.packet_arrival = (arrival - arrival[0]) * 1e-9
        frame = (stamp // int(PERIOD * 1e9)).astype(np.int64)
        starts = np.flatnonzero(np.r_[True, np.diff(frame) != 0])
        self.frame_start = starts                        # first packet of each cloud
        self.frame_end = np.r_[starts[1:], len(frame)]
        self.t = frame[starts] * PERIOD                  # s, each cloud's start, lidar clock
        self.arrival = self.packet_arrival[starts]
        if self.headers_only:
            self.points = None
            return
        dots = self.header["dot_num"].astype(np.int64)
        self.points = np.concatenate([np.frombuffer(d, POINT, n, 36)
                                      for d, n in zip(data, dots)])
        self.point_start = np.r_[0, np.cumsum(dots)]
        # per point: its packet's stamp plus its share of the packet's time_interval
        within = np.arange(dots.sum()) - np.repeat(self.point_start[:-1], dots)
        step = np.repeat(self.header["time_interval"] * 1e-7 / np.maximum(dots, 1), dots)
        self.point_t = np.repeat(self.packet_t, dots) + within * step

    def read_imu(self, arrival, data):
        self.imu = {}
        if not data:
            return
        header = np.concatenate([np.frombuffer(d, HEADER, 1) for d in data])
        values = np.concatenate([np.frombuffer(d, IMU, 1, 36) for d in data])
        self.imu = {"arrival_ns": arrival, "stamp_ns": header["timestamp"].astype(np.int64),
                    "udp_cnt": header["udp_cnt"]}
        self.imu.update({n: values[n].astype(np.float64) for n in IMU.names})

    @property
    def fields(self):
        return list(POINT.names) + ["time"]

    def __len__(self):
        return len(self.frame_start)

    def cloud(self, i):
        """Cloud i as a structured array: every point (returns or not), with its time (s)."""
        if self.points is None:
            raise ValueError("a headers-only capture has no points")
        a, b = self.point_start[self.frame_start[i]], self.point_start[self.frame_end[i]]
        out = np.empty(b - a, dtype=POINT.descr + [("time", "<f8")])
        for name in POINT.names:
            out[name] = self.points[name][a:b]
        out["time"] = self.point_t[a:b]
        return out

    def xyz(self, i, cloud=None):
        """Cloud i's returns in the lidar's frame (m), (n, 3), and a mask of which of the
        cloud's points they are (not at the origin: no return)."""
        c = self.cloud(i) if cloud is None else cloud
        xyz = np.stack([c["x"], c["y"], c["z"]], axis=1)
        keep = np.any(xyz != 0, axis=1)
        return xyz[keep] * 1e-3, keep

    def frames(self, limit=None):
        """Indices of up to `limit` clouds spread evenly over the capture."""
        n = len(self)
        if limit is None or limit >= n:
            return np.arange(n)
        return np.unique(np.linspace(0, n - 1, limit).round().astype(int))
