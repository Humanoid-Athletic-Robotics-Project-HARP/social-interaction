"""Look at a capture in Rerun: the cloud cloud by cloud (scrub the timeline), placed on
the torso by the assumed mount, coloured by a field, returns within NEAR m in red,
and the lidar IMU and the robot's low state as plots on the same timeline.

  ../.venv/bin/python view.py captures/NAME [--color intensity] [--accumulate 10]
                                            [--every 2] [--save FILE.rrd]

Entities: torso/ is torso_link (x forward, y left, z up); torso/lidar/ is the lidar's
own frame under it (upside down), holding the raw points as recorded.
"""

import argparse
from collections import deque

import numpy as np
import rerun as rr

from capture import MOUNT_R, MOUNT_T, Capture, to_torso
from report import NEAR

# dark blue → blue → green → yellow → red
STOPS = np.array([[48, 18, 59], [40, 140, 240], [100, 240, 90], [250, 200, 40], [190, 30, 10]],
                 dtype=float)


def colors(values):
    lo, hi = np.percentile(values, [1, 99]) if len(values) else (0, 1)
    t = np.clip((values - lo) / (hi - lo + 1e-9), 0, 1) * (len(STOPS) - 1)
    i = np.minimum(t.astype(int), len(STOPS) - 2)
    f = (t - i)[:, None]
    return (STOPS[i] * (1 - f) + STOPS[i + 1] * f).astype(np.uint8)


def axes(path, length=0.15):
    rr.log(path, rr.Arrows3D(vectors=np.eye(3) * length, colors=[[230, 60, 60], [60, 200, 60],
                                                                 [60, 110, 240]]), static=True)


def plot(path, t, values, names):
    rr.log(path, rr.SeriesLines(names=names), static=True)
    rr.send_columns(path, indexes=[rr.TimeColumn("time", duration=t)],
                    columns=rr.Scalars.columns(scalars=values))


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("capture")
    parser.add_argument("--color", default=None,
                        help="a field (default intensity or reflectivity if present), "
                             "'height' (torso_link z) or 'range'")
    parser.add_argument("--accumulate", type=int, default=1,
                        help="show the last N clouds together (the scan pattern fills in)")
    parser.add_argument("--every", type=int, default=1, help="only every K-th cloud")
    parser.add_argument("--save", help="write an .rrd file instead of opening the viewer")
    args = parser.parse_args()

    cap = Capture(args.capture)
    color = args.color or next((f for f in ("intensity", "reflectivity") if f in cap.fields),
                               "height")
    rr.init(f"g1_lidar_{cap.meta['name']}")
    if args.save:
        rr.save(args.save)
    else:
        rr.spawn()
    rr.log("torso", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)
    axes("torso/axes")
    rr.log("torso/lidar", rr.Transform3D(translation=MOUNT_T, mat3x3=MOUNT_R), static=True)
    axes("torso/lidar/axes", 0.1)
    if cap.meta.get("note"):
        rr.log("note", rr.TextDocument(cap.meta["note"]), static=True)

    recent = deque(maxlen=max(1, args.accumulate))
    for i in range(0, len(cap), args.every):
        cloud = cap.cloud(i)
        xyz, keep = cap.xyz(i, cloud)
        if color == "height":
            value = to_torso(xyz)[:, 2]
        elif color == "range":
            value = np.linalg.norm(xyz, axis=1)
        else:
            value = cloud[color][keep].astype(np.float64)
        recent.append((xyz, value))
        pts = np.concatenate([p for p, _ in recent])
        vals = np.concatenate([v for _, v in recent])
        near = np.linalg.norm(pts, axis=1) < NEAR
        rr.set_time("frame", sequence=i)
        rr.set_time("time", duration=cap.t[i])
        rr.log("torso/lidar/cloud", rr.Points3D(pts[~near], colors=colors(vals[~near]), radii=0.008))
        rr.log("torso/lidar/near", rr.Points3D(pts[near], colors=[255, 40, 40], radii=0.012))
        rr.log("cloud/returns", rr.Scalars(len(xyz)))

    imu = cap.imu
    if imu and len(imu["stamp_ns"]):
        t = (imu["stamp_ns"] - cap.t0) * 1e-9   # the lidar clock, as the clouds
        plot("imu/accel", t, np.stack([imu["ax"], imu["ay"], imu["az"]], axis=1), ["x", "y", "z"])
        plot("imu/gyro", t, np.stack([imu["gx"], imu["gy"], imu["gz"]], axis=1), ["x", "y", "z"])
    low = cap.lowstate
    if low and len(low["arrival_ns"]):
        t = (low["arrival_ns"] - cap.packet_arrival_ns[0]) * 1e-9   # no stamp: PC2's arrival
        plot("lowstate/pelvis_rpy", t, np.degrees(np.stack([low["roll"], low["pitch"], low["yaw"]],
                                                           axis=1)), ["roll", "pitch", "yaw"])
        plot("lowstate/waist", t, np.degrees(np.stack([low[f"q{j}"] for j in (12, 13, 14)], axis=1)),
             ["yaw", "roll", "pitch"])
    if args.save:
        print(f"Wrote {args.save}; open it with: ../.venv/bin/rerun {args.save}")


if __name__ == "__main__":
    main()
