"""What one capture says about the lidar, as Markdown: printed, and saved as
report.md in the capture's folder.

  ../.venv/bin/python report.py captures/NAME [--box X0 X1 Y0 Y1 Z0 Z1 [--truth M]]

Sections: the lidar's own status (model, serial, firmware, network, configured field
of view, temperature); the stream (rates, lost packets, points per cloud, reflectivity
and tag values); timing (packet spacing, the lidar's clock against PC2's); range and field of view, and how the scan pattern fills
in over time; the floor (the lidar's height and tilt, checked against the mount
Nav2 and the sim assume, and how near the floor first shows up); points near the
lidar (the robot's own body); the lidar's IMU; the robot's low state.

--box: a flat target (wall, board) inside this box, in torso_link metres
(x forward, y left, z up, by the assumed mount): its noise and distance, frame by
frame; --truth: the tape-measured distance from the lidar to it.
"""

import argparse
from pathlib import Path

import numpy as np

from capture import MOUNT_R, PERIOD, Capture, to_torso

NEAR = 0.7          # m: returns nearer than this are checked as the robot's own body
FLOOR_TOL = 0.03    # m: RANSAC inlier distance for the floor
SAMPLE = 100        # clouds used for the geometry sections


def fmt(x, digits=3):
    return "–" if x is None or not np.isfinite(x) else f"{x:.{digits}f}"


def table(header, rows):
    out = ["| " + " | ".join(header) + " |", "|" + "---|" * len(header)]
    out += ["| " + " | ".join(str(c) for c in row) + " |" for row in rows]
    return out


def stats(values, digits=3):
    v = np.asarray(values, dtype=float)
    if v.size == 0:
        return "none"
    return (f"median {fmt(np.median(v), digits)}, min {fmt(v.min(), digits)}, "
            f"max {fmt(v.max(), digits)}")


def lidar_rpy(up):
    """Roll and pitch (deg) of a frame whose gravity-up direction, in that frame, is `up`."""
    up = up / np.linalg.norm(up)
    return (np.degrees(np.arctan2(up[1], up[2])),
            np.degrees(np.arctan2(-up[0], np.hypot(up[1], up[2]))))


def angle(a, b):
    return np.degrees(np.arccos(np.clip(abs(a @ b) / np.linalg.norm(a) / np.linalg.norm(b), 0, 1)))


def fit_plane(points):
    """Unit normal n and offset h of the best-fit plane n.p = h, and residual RMS."""
    centre = points.mean(axis=0)
    _, _, vt = np.linalg.svd(points - centre, full_matrices=False)
    n = vt[2]
    h = n @ centre
    return n, h, np.sqrt(np.mean((points @ n - h) ** 2))


def ransac_floor(points, up, rng, tries=400):
    """The plane with the most points within FLOOR_TOL whose normal is within 30 deg of
    `up`; normal oriented so the lidar (the origin) is on its positive side."""
    best, best_count = None, 0
    for _ in range(tries):
        a, b, c = points[rng.choice(len(points), 3, replace=False)]
        n = np.cross(b - a, c - a)
        if np.linalg.norm(n) < 1e-9:
            continue
        n /= np.linalg.norm(n)
        if abs(n @ up) < np.cos(np.radians(30)):
            continue
        count = np.count_nonzero(np.abs(points @ n - n @ a) < FLOOR_TOL)
        if count > best_count:
            best, best_count = (n, n @ a), count
    if best is None:
        return None
    n, h = best
    inliers = points[np.abs(points @ n - h) < FLOOR_TOL]
    n, h, rms = fit_plane(inliers)
    if h > 0:                       # make the origin's side positive: n.p = -height
        n, h = -n, -h
    return n, -h, rms, inliers


class Report:
    def __init__(self, cap, args):
        self.c, self.args = cap, args
        self.lines = []
        self.rng = np.random.default_rng(0)
        self.sample = cap.frames(SAMPLE)
        self.clouds = [] if cap.headers_only else [cap.xyz(i)[0] for i in self.sample]
        self.floor_up = None        # the floor's up direction in the lidar frame, once found

    def out(self, *lines):
        self.lines.extend(lines)

    def section(self, title):
        self.out("", f"## {title}", "")

    def run(self):
        m = self.c.meta
        self.out(f"# Lidar capture `{m['name']}`", "")
        if m.get("note"):
            self.out(f"> {m['note']}", "")
        self.out(f"Recorded {m['started']} to {m.get('ended', '?')} on {m['host']}"
                 f" (scripts at {m.get('commit') or 'no git commit'}); "
                 f"{len(self.c)} clouds of {PERIOD * 1e3:.0f} ms over {self.c.packet_t[-1]:.1f} s"
                 f"{' (headers only: no points)' if self.c.headers_only else ''}.")
        parts = [self.lidar, self.stream, self.timing]
        if not self.c.headers_only:
            parts += [self.view, self.floor, self.near]
        for part in parts + [self.imu, self.lowstate, self.plane]:
            part()
        return "\n".join(self.lines) + "\n"

    def lidar(self):
        self.section("The lidar (its status messages)")
        status = [s for s in self.c.status if s]
        if not status:
            self.out("No status messages.")
            return
        first, last = status[0], status[-1]
        rows = [[k, v if k not in ("core_temp", "local_time_now") or v == last.get(k)
                 else f"{v} → {last.get(k)}"] for k, v in first.items()]
        self.out(f"{len(status)} messages; the first (and where it changed, the last):", "",
                 *table(["key", "value"], rows))

    def stream(self):
        c, h, m = self.c, self.c.header, self.c.meta
        self.section("The stream")
        seconds = c.packet_t[-1]
        lost = int(np.sum((np.diff(h["udp_cnt"].astype(np.int64)) - 1) % 65536))
        dots = h["dot_num"]
        self.out(f"- Point packets: {len(h)} ({len(h) / seconds:.0f}/s), "
                 f"{stats(h['length'], 0)} bytes, {stats(dots, 0)} points each "
                 f"({dots.sum() / seconds:,.0f} points/s); data_type "
                 f"{', '.join(str(v) for v in np.unique(h['data_type']))}, time_type "
                 f"{', '.join(str(v) for v in np.unique(h['time_type']))}, frame_cnt "
                 f"{', '.join(str(v) for v in np.unique(h['frame_cnt'])[:8])}.",
                 f"- Lost on the way (gaps in udp_cnt): {lost} point packets; dropped by PC2's "
                 f"kernel while recording: {m.get('kernel_drops', {})}.")
        if c.imu:
            imu_lost = int(np.sum((np.diff(c.imu["udp_cnt"].astype(np.int64)) - 1) % 65536))
            self.out(f"- IMU packets: {len(c.imu['stamp_ns'])}, {imu_lost} lost (udp_cnt gaps).")
        if not c.headers_only:
            valid = [len(p) for p in self.clouds]
            per = (c.point_start[c.frame_end] - c.point_start[c.frame_start])[self.sample]
            self.out(f"- Per {PERIOD * 1e3:.0f} ms cloud: {stats(per, 0)} points, of which returns "
                     f"(not at the origin) {stats(valid, 0)}.")
            sample = np.concatenate([c.cloud(i) for i in self.sample[:20]])
            for name in ("reflectivity", "tag"):
                v = sample[name]
                u, n = np.unique(v, return_counts=True)
                top = ", ".join(f"{a} ({100 * b / len(v):.0f}%)" for a, b in
                                sorted(zip(u, n), key=lambda x: -x[1])[:6])
                self.out(f"- `{name}`: {len(u)} values, {v.min()} to {v.max()}; most common {top}.")

    def timing(self):
        c = self.c
        self.section("Timing")
        dt = np.diff(c.packet_t)
        da = np.diff(c.packet_arrival)
        ti = c.header["time_interval"] * 1e-7
        offset = (c.packet_arrival - c.packet_t) * 1e3   # ms, PC2 minus lidar clock, from the start
        fit = np.polyfit(c.packet_t, offset, 1)
        drift = fit[0] * 1e3                              # ppm
        late = offset - np.polyval(fit, c.packet_t)       # the drift taken out
        late -= late.min()
        self.out(f"- Between packets, by the lidar's stamps: {stats(dt * 1e6, 0)} us; "
                 f"each packet's time_interval {stats(ti * 1e6, 0)} us (the span of its points).",
                 f"- Between arrivals on PC2: {stats(da * 1e6, 0)} us.",
                 f"- The lidar's clock: {c.t0 / 1e9:.1f} s at the start (time_type 0: since power-on, "
                 "not synced to anything).",
                 f"- PC2's clock against it drifts {drift:+.1f} ppm "
                 f"({'meaningful' if c.packet_t[-1] > 120 else 'too short to trust'} over "
                 f"{c.packet_t[-1]:.0f} s).",
                 f"- Arrival delay beyond the smallest (drift taken out): median {np.median(late):.2f}, 99% "
                 f"{np.percentile(late, 99):.2f}, max {late.max():.2f} ms.")

    def view(self):
        self.section("Range and field of view (lidar frame)")
        pts = np.concatenate(self.clouds)
        r = np.linalg.norm(pts, axis=1)
        el = np.degrees(np.arctan2(pts[:, 2], np.hypot(pts[:, 0], pts[:, 1])))
        az = np.degrees(np.arctan2(pts[:, 1], pts[:, 0]))
        p = np.percentile(r, [0, 0.1, 1, 50, 99, 99.9, 100])
        self.out(f"- Range: min {p[0]:.3f}, 0.1% {p[1]:.3f}, 1% {p[2]:.3f}, median {p[3]:.2f}, "
                 f"99% {p[4]:.2f}, 99.9% {p[5]:.2f}, max {p[6]:.2f} m "
                 f"(spec: 0.1 m blind zone, 40 m at 10% reflectivity).",
                 f"- Elevation: {el.min():.1f} to {el.max():.1f} deg (0.5–99.5%: "
                 f"{np.percentile(el, 0.5):.1f} to {np.percentile(el, 99.5):.1f}; spec −7 to +52).",
                 f"- Azimuth: {np.count_nonzero(np.histogram(az, 360, (-180, 180))[0])} of 360 "
                 f"one-degree sectors have returns.")
        # How the non-repetitive pattern fills the view: 1 x 1 deg cells hit by n clouds in a row
        lo, hi = np.floor(np.percentile(el, 0.5)), np.ceil(np.percentile(el, 99.5))
        cells = 360 * (hi - lo)
        rows, run = [], []
        consecutive = [self.c.xyz(i)[0] for i in range(min(len(self.c), 50))]
        for n in (1, 2, 5, 10, 20, 50):
            if n > len(consecutive):
                break
            q = np.concatenate(consecutive[:n])
            qa = np.degrees(np.arctan2(q[:, 1], q[:, 0]))
            qe = np.degrees(np.arctan2(q[:, 2], np.hypot(q[:, 0], q[:, 1])))
            hit = np.unique(np.floor(qa).astype(int) * 1000 + np.floor(qe).astype(int))
            rows.append([n, f"{n * np.median(np.diff(self.c.t)) if len(self.c) > 1 else 0:.1f} s",
                         f"{100 * len(hit) / cells:.0f}%"])
        self.out("", f"Share of the 1° × 1° cells in {lo:.0f} to {hi:.0f} deg elevation hit by "
                     "consecutive clouds:", "", *table(["clouds", "time", "cells hit"], rows))
        # Where it sees: returns by direction around the robot (torso_link, by the mount)
        t = to_torso(pts)
        taz = np.degrees(np.arctan2(t[:, 1], t[:, 0]))
        rows = []
        for a in range(-180, 180, 30):
            sel = (taz >= a) & (taz < a + 30)
            ground = np.hypot(t[sel, 0], t[sel, 1])
            rows.append([f"{a:+d} to {a + 30:+d}°", f"{np.count_nonzero(sel) / len(self.clouds):.0f}",
                         f"{100 * np.count_nonzero(sel) / len(pts):.1f}%",
                         fmt(np.median(ground), 2) if sel.any() else "–",
                         fmt(np.percentile(ground, 1), 2) if sel.any() else "–"])
        self.out("", "Returns by direction around the robot (torso_link azimuth by the assumed "
                     "mount: 0° = +x, + to the left; the front if the mount is right):", "",
                 *table(["direction", "returns a cloud", "share", "median distance m",
                         "nearest (1%) m"], rows))

    def floor(self):
        self.section("Floor and mount")
        pts = np.concatenate(self.clouds)
        pts = pts[self.rng.choice(len(pts), min(len(pts), 200_000), replace=False)]
        expected = MOUNT_R.T @ np.array([0.0, 0.0, 1.0])   # torso up, in the lidar frame
        found = ransac_floor(pts, expected, self.rng)
        if found is None:
            self.out("No horizontal-ish plane found.")
            return
        n, height, rms, inliers = found
        self.floor_up = n
        er, ep = lidar_rpy(expected)
        mr, mp = lidar_rpy(n)
        below = n @ expected > 0
        self.out(f"Largest plane within 30° of horizontal: {len(inliers)} of {len(pts)} sampled "
                 f"returns, RMS {rms * 1e3:.1f} mm, **{height:.3f} m** from the lidar.", "",
                 f"- Per the assumed mount (upside down, `lidar_mount.py`) it is "
                 f"{'below the lidar: the floor' if below else 'ABOVE the lidar: a ceiling, or the mount is wrong'}.",
                 f"- Its up direction in the lidar frame: ({n[0]:+.3f}, {n[1]:+.3f}, {n[2]:+.3f}) → "
                 f"lidar roll {mr:.1f}°, pitch {mp:.1f}° from level.",
                 f"- The mount predicts ({expected[0]:+.3f}, {expected[1]:+.3f}, {expected[2]:+.3f}) "
                 f"→ roll {er:.1f}°, pitch {ep:.1f}° with the torso upright; "
                 f"{angle(n, expected):.1f}° apart (includes the torso's own tilt: see low state).")
        if below:
            horizontal = np.linalg.norm(inliers - np.outer(inliers @ n, n), axis=1)
            q = np.percentile(horizontal, [0, 1, 5, 50])
            rings = np.histogram(horizontal, bins=np.arange(0, 5.01, 0.25))[0]
            first = np.argmax(rings > 0.01 * rings.max()) * 0.25
            self.out(f"- Floor seen from {q[0]:.2f} m out (1%: {q[1]:.2f}, 5%: {q[2]:.2f}, "
                     f"median {q[3]:.2f} m), measured along the floor from below the lidar; "
                     f"the first 0.25 m ring with real coverage starts at {first:.2f} m "
                     "(the blind ring is inside that).")

    def near(self):
        self.section(f"Returns within {NEAR} m (the robot's own body?)")
        counts, voxels = [], {}
        for pts in self.clouds:
            close = pts[np.linalg.norm(pts, axis=1) < NEAR]
            counts.append(len(close))
            for key in set(map(tuple, np.floor(to_torso(close) / 0.05).astype(int))):
                voxels[key] = voxels.get(key, 0) + 1
        counts = np.array(counts)
        self.out(f"- In {np.count_nonzero(counts)} of {len(counts)} clouds; per cloud {stats(counts, 0)}.")
        steady = np.array([k for k, v in voxels.items() if v >= 0.3 * len(self.clouds)])
        if len(steady) == 0:
            self.out("- No 5 cm cell is hit in 30% of clouds: nothing steady near the lidar.")
            return
        lo, hi = steady.min(axis=0) * 0.05, (steady.max(axis=0) + 1) * 0.05
        self.out(f"- {len(steady)} 5 cm cells hit in at least 30% of clouds (something fixed, "
                 f"likely the body), in torso_link x {lo[0]:+.2f} to {hi[0]:+.2f}, "
                 f"y {lo[1]:+.2f} to {hi[1]:+.2f}, z {lo[2]:+.2f} to {hi[2]:+.2f} m.")
        centres = (steady + 0.5) * 0.05
        az = np.degrees(np.arctan2(centres[:, 1], centres[:, 0]))
        hit = np.histogram(az, bins=24, range=(-180, 180))[0] > 0
        runs, start = [], None      # neighbouring 15 deg sectors with cells, merged
        for i in range(25):
            if i < 24 and hit[i] and start is None:
                start = i
            elif (i == 24 or not hit[i]) and start is not None:
                runs.append(f"{-180 + 15 * start:+d} to {-180 + 15 * i:+d}°")
                start = None
        self.out(f"- Directions (torso_link azimuth, 0° ahead, + to the left): {', '.join(runs)}.")

    def imu(self):
        imu = self.c.imu
        self.section("Lidar IMU")
        if not imu or len(imu["arrival_ns"]) < 2:
            self.out("No samples (check the topic and type in robot/record.py).")
            return
        t = (imu["stamp_ns"] - imu["stamp_ns"][0]) * 1e-9
        ta = (imu["arrival_ns"] - imu["arrival_ns"][0]) * 1e-9
        acc = np.stack([imu["ax"], imu["ay"], imu["az"]], axis=1)
        gyro = np.stack([imu["gx"], imu["gy"], imu["gz"]], axis=1)
        mag = np.linalg.norm(acc, axis=1).mean()
        unit = "g" if 0.5 < mag < 1.5 else "m/s²" if 5 < mag < 15 else "?"
        up = acc.mean(axis=0)
        self.out(f"- {len(t)} samples: {(len(t) - 1) / t[-1] if t[-1] > 0 else np.nan:.1f} Hz by "
                 f"stamp, {(len(ta) - 1) / ta[-1]:.1f} Hz by arrival.",
                 f"- Accelerometer mean ({up[0]:+.3f}, {up[1]:+.3f}, {up[2]:+.3f}), magnitude "
                 f"{mag:.3f} (units: {unit}); std per axis "
                 f"{', '.join(fmt(s, 4) for s in acc.std(axis=0))}.",
                 f"- Gyro mean (bias) {', '.join(fmt(np.degrees(s), 3) for s in gyro.mean(axis=0))} deg/s, "
                 f"std {', '.join(fmt(np.degrees(s), 3) for s in gyro.std(axis=0))} deg/s.")
        r, p = lidar_rpy(up)
        line = f"- Gravity from the accelerometer → roll {r:.1f}°, pitch {p:.1f}°"
        if self.floor_up is not None:
            line += (f"; {angle(up, self.floor_up):.1f}° from the floor's up direction "
                     "(if the IMU's axes are the lidar's, as Unitree's docs say)")
        self.out(line + ".")

    def lowstate(self):
        low = self.c.lowstate
        self.section("Robot low state")
        if not low or len(low["arrival_ns"]) < 2:
            self.out("No samples.")
            return
        ta = (low["arrival_ns"] - low["arrival_ns"][0]) * 1e-9
        joints = np.stack([low[f"q{i}"] for i in range(29)], axis=1)
        moved = joints.max(axis=0) - joints.min(axis=0)
        self.out(f"- {len(ta)} samples kept ({(len(ta) - 1) / ta[-1]:.1f} Hz; "
                 f"{(self.c.meta.get('lowstate') or {}).get('received', '?')} received); mode_machine "
                 f"{', '.join(str(int(v)) for v in np.unique(low['mode_machine']))}.",
                 f"- Pelvis roll {np.degrees(np.mean(low['roll'])):.1f}°, pitch "
                 f"{np.degrees(np.mean(low['pitch'])):.1f}° (if rad); the lidar sits on the "
                 "torso, which the waist joints (12–14) turn relative to the pelvis.",
                 f"- Pelvis gyro mean {', '.join(fmt(np.degrees(np.mean(low[k])), 3) for k in ('gx', 'gy', 'gz'))} "
                 "deg/s (compare the lidar IMU's bias).",
                 f"- Waist joints 12–14:{', '.join(fmt(np.degrees(joints[:, i].mean()), 1) for i in (12, 13, 14))} deg.",
                 f"- Joints moved at most {np.degrees(moved.max()):.2f}° (joint {int(moved.argmax())}) "
                 "over the capture.")

    def plane(self):
        box = self.args.box
        if box is None:
            return
        self.section("Flat target in the box")
        lo, hi = np.array(box[0::2]), np.array(box[1::2])
        rows, dists, noise, everything = [], [], [], []
        for i in self.c.frames():
            pts = self.c.xyz(i)[0]
            t = to_torso(pts)
            inside = pts[np.all((t >= lo) & (t <= hi), axis=1)]
            if len(inside) < 10:
                continue
            n, h, rms = fit_plane(inside)
            dists.append(abs(h))
            noise.append(rms)
            everything.append(inside)
        if not dists:
            self.out("Fewer than 10 returns in the box in every cloud.")
            return
        allpts = np.concatenate(everything)
        n, h, rms = fit_plane(allpts)
        d = np.array(dists)
        self.out(f"- {len(d)} clouds with the target, {len(allpts) / len(d):.0f} returns a cloud.",
                 f"- Noise about each cloud's plane: {stats(np.array(noise) * 1e3, 1)} mm RMS; "
                 f"about one plane through every cloud: {rms * 1e3:.1f} mm RMS.",
                 f"- Distance from the lidar to the plane: mean {d.mean():.4f} m, std across "
                 f"clouds {d.std() * 1e3:.1f} mm.")
        if self.args.truth is not None:
            self.out(f"- Against the tape ({self.args.truth:.3f} m): "
                     f"{(d.mean() - self.args.truth) * 1e3:+.1f} mm.")


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("capture", type=Path)
    parser.add_argument("--box", type=float, nargs=6, metavar=("X0", "X1", "Y0", "Y1", "Z0", "Z1"))
    parser.add_argument("--truth", type=float, help="tape-measured lidar-to-target distance, m")
    args = parser.parse_args()
    text = Report(Capture(args.capture), args).run()
    (args.capture / "report.md").write_text(text)
    print(text)


if __name__ == "__main__":
    main()
