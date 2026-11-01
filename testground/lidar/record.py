"""Record the G1's head lidar, the lidar's IMU and the robot's low state on PC2, to
copy to a laptop and analyse there (report.py, view.py). Read-only: it only subscribes,
through Unitree's SDK on the robot's DDS domain, as submission/harp/lidar_relay.py
does; nothing is ever sent to the robot.

  python3 record.py NAME [--seconds 60] [--note "wall at 2.00 m"]

writes captures/NAME/ beside this file:
  meta.json     what was recorded and how, the cloud's layout, message counts
  clouds.bin    every cloud's point data as received, back to back
  clouds.csv    one row per cloud: arrival time (this computer's clock, ns), the
                robot's stamp (ns), where its bytes are in clouds.bin, its size
  imu.csv       rt/utlidar/imu_livox_mid360, every sample
  lowstate.csv  rt/lowstate (pelvis IMU and joints), thinned to --lowstate-hz

About 5 MB/s with the full cloud. Python 3.8 (PC2's), unitree_sdk2py and
cyclonedds 0.10.2 (what ../pc2/probe.sh checks for).
"""

import argparse
import csv
from dataclasses import dataclass
import datetime
import json
from pathlib import Path
import platform
import shutil
import signal
import socket
import subprocess
import sys
import time

import cyclonedds.idl as idl
import cyclonedds.idl.annotations as annotate
import cyclonedds.idl.types as types
from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.idl.geometry_msgs.msg.dds_ import Quaternion_, Vector3_
from unitree_sdk2py.idl.sensor_msgs.msg.dds_ import PointCloud2_
from unitree_sdk2py.idl.std_msgs.msg.dds_ import Header_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_

CLOUD = "rt/utlidar/cloud_livox_mid360"
IMU = "rt/utlidar/imu_livox_mid360"
LOWSTATE = "rt/lowstate"
RATE = 5e6        # bytes/s the full cloud takes on disk, for the space check
QUIET = 5.0       # s without a cloud before warning
JOINTS = 35       # motor_state slots in unitree_hg's LowState_ (the G1 uses 29)


# The SDK has no sensor_msgs Imu; this is ROS 2's, under the SDK's naming.
@dataclass
@annotate.final
@annotate.autoid("sequential")
class Imu_(idl.IdlStruct, typename="sensor_msgs.msg.dds_.Imu_"):
    header: Header_
    orientation: Quaternion_
    orientation_covariance: types.array[types.float64, 9]
    angular_velocity: Vector3_
    angular_velocity_covariance: types.array[types.float64, 9]
    linear_acceleration: Vector3_
    linear_acceleration_covariance: types.array[types.float64, 9]


def stamp_ns(header):
    return header.stamp.sec * 1_000_000_000 + header.stamp.nanosec


def table(path, columns):
    f = open(path, "w", newline="")
    rows = csv.writer(f)
    rows.writerow(columns)
    return f, rows


class Recorder:
    """Writes each topic from its SDK reader's callback, one file each."""

    def __init__(self, folder, lowstate_hz):
        self.cloud_file = open(folder / "clouds.bin", "wb")
        self.cloud_csv, self.cloud_rows = table(
            folder / "clouds.csv", ["arrival_ns", "stamp_ns", "offset", "nbytes", "width", "height"])
        self.imu_csv, self.imu_rows = table(
            folder / "imu.csv", ["arrival_ns", "stamp_ns", "qx", "qy", "qz", "qw",
                                 "gx", "gy", "gz", "ax", "ay", "az"])
        self.low_csv, self.low_rows = table(
            folder / "lowstate.csv",
            ["arrival_ns", "tick", "mode_machine", "qw", "qx", "qy", "qz", "gx", "gy", "gz",
             "ax", "ay", "az", "roll", "pitch", "yaw"] + [f"q{i}" for i in range(JOINTS)])
        self.low_period = 1e9 / lowstate_hz
        self.low_last = 0
        self.layout = None          # the first cloud's frame and point layout
        self.layout_changes = 0     # later clouds laid out differently
        self.first_offset = None    # s, arrival minus the robot's stamp, first cloud
        self.counts = {"clouds": 0, "points": 0, "imu": 0, "lowstate": 0, "lowstate_kept": 0}
        self.last_cloud = None      # time.monotonic() of the last cloud

    def on_cloud(self, msg):
        arrival = time.time_ns()
        data = bytes(msg.data)
        layout = {"frame_id": msg.header.frame_id, "point_step": msg.point_step,
                  "is_bigendian": msg.is_bigendian, "is_dense": msg.is_dense,
                  "fields": [{"name": f.name, "offset": f.offset, "datatype": f.datatype,
                              "count": f.count} for f in msg.fields]}
        if self.layout is None:
            self.layout = layout
            self.first_offset = (arrival - stamp_ns(msg.header)) * 1e-9
            print(f"First cloud: {msg.width * msg.height} points in {msg.header.frame_id}, fields "
                  f"{','.join(f.name for f in msg.fields)}, {msg.point_step} bytes a point; the "
                  f"robot's stamp is {self.first_offset:+.3f} s from this clock.", flush=True)
        elif layout != self.layout:
            self.layout_changes += 1
        offset = self.cloud_file.tell()
        self.cloud_file.write(data)
        self.cloud_rows.writerow([arrival, stamp_ns(msg.header), offset, len(data),
                                  msg.width, msg.height])
        self.counts["clouds"] += 1
        self.counts["points"] += msg.width * msg.height
        self.last_cloud = time.monotonic()

    def on_imu(self, msg):
        o, w, a = msg.orientation, msg.angular_velocity, msg.linear_acceleration
        self.imu_rows.writerow([time.time_ns(), stamp_ns(msg.header), o.x, o.y, o.z, o.w,
                                w.x, w.y, w.z, a.x, a.y, a.z])
        self.counts["imu"] += 1

    def on_lowstate(self, msg):
        arrival = time.time_ns()
        self.counts["lowstate"] += 1
        if arrival - self.low_last < self.low_period:
            return
        self.low_last = arrival
        imu = msg.imu_state
        self.low_rows.writerow([arrival, msg.tick, msg.mode_machine] + list(imu.quaternion)
                               + list(imu.gyroscope) + list(imu.accelerometer) + list(imu.rpy)
                               + [m.q for m in msg.motor_state])
        self.counts["lowstate_kept"] += 1

    def flush(self):
        for f in (self.cloud_file, self.cloud_csv, self.imu_csv, self.low_csv):
            f.flush()

    def close(self):
        for f in (self.cloud_file, self.cloud_csv, self.imu_csv, self.low_csv):
            f.close()


def versions():
    try:
        from importlib.metadata import version
    except ImportError:
        return {}
    out = {}
    for name in ("unitree_sdk2py", "cyclonedds", "numpy"):
        try:
            out[name] = version(name)
        except Exception:
            out[name] = None
    return out


def commit(folder):
    """The git commit these scripts are at, if they're in a clone (else None)."""
    try:
        out = subprocess.run(["git", "-C", str(folder), "describe", "--always", "--dirty"],
                             capture_output=True, text=True, timeout=5)
    except Exception:
        return None
    return out.stdout.strip() or None


def utc():
    return datetime.datetime.now(datetime.timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")


def stop(signum, frame):
    raise KeyboardInterrupt


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("name", help="the capture's folder name, e.g. static-open")
    parser.add_argument("--seconds", type=float, default=60.0, help="how long to record")
    parser.add_argument("--note", default="", help="what the scene was (distances, objects)")
    parser.add_argument("--interface", default="eth0", help="PC2's interface on the robot network")
    parser.add_argument("--domain", type=int, default=0)
    parser.add_argument("--lowstate-hz", type=float, default=50.0)
    args = parser.parse_args()

    folder = Path(__file__).resolve().parent / "captures" / args.name
    if folder.exists():
        sys.exit(f"{folder} already exists; pick another name.")
    free = shutil.disk_usage(folder.parent.parent).free
    if free < RATE * args.seconds + 1e9:
        sys.exit(f"Only {free / 1e9:.1f} GB free; {args.seconds:.0f} s needs about "
                 f"{RATE * args.seconds / 1e9 + 1:.1f} GB.")
    folder.mkdir(parents=True)

    meta = {"name": args.name, "note": args.note, "started": utc(), "host": socket.gethostname(),
            "python": platform.python_version(), "versions": versions(),
            "commit": commit(folder.parent.parent),
            "interface": args.interface, "domain": args.domain,
            "topics": {"cloud": CLOUD, "imu": IMU, "lowstate": LOWSTATE},
            "seconds_requested": args.seconds, "lowstate_hz": args.lowstate_hz}
    rec = Recorder(folder, args.lowstate_hz)
    ChannelFactoryInitialize(args.domain, args.interface)
    subs = []
    try:
        for topic, kind, handler in ((CLOUD, PointCloud2_, rec.on_cloud), (IMU, Imu_, rec.on_imu),
                                     (LOWSTATE, LowState_, rec.on_lowstate)):
            sub = ChannelSubscriber(topic, kind)
            sub.Init(handler, 0)
            subs.append(sub)
        print(f"Recording {folder} for {args.seconds:.0f} s (Ctrl-C to stop early).", flush=True)
        signal.signal(signal.SIGTERM, stop)
        start = time.monotonic()
        before = dict(rec.counts)
        warned = False
        try:
            while time.monotonic() - start < args.seconds:
                time.sleep(1.0)
                rec.flush()
                c = dict(rec.counts)
                clouds = c["clouds"] - before["clouds"]
                points = (c["points"] - before["points"]) // max(1, clouds)
                print(f"{time.monotonic() - start:4.0f} s  clouds {clouds:2d}/s  {points:6d} points  "
                      f"imu {c['imu'] - before['imu']:3d}/s  "
                      f"lowstate {c['lowstate'] - before['lowstate']:3d}/s  "
                      f"{rec.cloud_file.tell() / 1e6:5.0f} MB", flush=True)
                before = c
                since = rec.last_cloud or start
                if not warned and time.monotonic() - since > QUIET:
                    print(f"No cloud on {CLOUD} for {QUIET:.0f} s.", flush=True)
                    warned = True
        except KeyboardInterrupt:
            print("Stopped early.", flush=True)
    finally:
        # The SDK's readers first: with one still open the interpreter hangs on the way out.
        for sub in subs:
            sub.Close()
        rec.close()
        meta.update({"ended": utc(), "counts": rec.counts, "layout": rec.layout,
                     "layout_changes": rec.layout_changes, "first_offset_s": rec.first_offset})
        (folder / "meta.json").write_text(json.dumps(meta, indent=2) + "\n")
        print(f"Wrote {folder}: {', '.join(f'{k} {v}' for k, v in rec.counts.items())}.",
              flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
