"""Check the lidar self-filter (submission/harp/self_filter.py) on a real capture: the
robot's own body returns before and after it, with the capture's joint angles, and how
far each body return is from where the simulator's math puts the body.

  docker run --rm -v "$PWD/../..:/repo" -w /repo/testground/lidar --entrypoint python3 \\
      harp-submission:local self_filter_check.py captures/NAME

(from testground/lidar; the submission image has MuJoCo and HIM's G1 model, /harp/model.)
The capture needs points and lowstate.csv (record.py saves both). Saves self_filter.md in
the capture's folder.

For each cloud:
- Joints: the lowstate rows within the relay's window before the cloud's arrival
  (self_filter.scan_poses, as lidar_relay.py picks /joint_states), so this checks what
  the robot would run.
- Returns within NEAR of the lidar (the body, and anything else that close) before, and
  after the filter at its MARGIN and at smaller margins: how much room the margin has.
- Rays: the simulator's ray along each return within NEAR (harp/lidar.py's cast: every
  robot geom but the head's) at the newest joints. "Real minus sim" is the measured range
  minus where that ray meets the model, so 0 means the model's surface is where the real
  body is. A ray that meets no part of the model is a return the model can't explain
  (something that isn't the robot, or a part of it the model is missing).
- Origin: the same rays within BODY from origins lowered along torso_link's z. The
  lidar measures from its optical centre, which the head mesh puts 21 mm below the URDF
  frame lidar_mount.POSITION uses (context/g1-face-visor.md); the drop that best matches
  says where the real origin is, apart from arm-joint errors when the capture holds
  several arm poses.
"""

import argparse
import sys
from pathlib import Path

import mujoco
import numpy as np

from capture import REPO, Capture

sys.path.insert(0, str(REPO / "submission"))
from harp import lidar_mount  # noqa: E402
from harp.self_filter import MARGIN, SelfFilter, scan_poses  # noqa: E402

MODELS = (Path("/harp/model/g1_robot.xml"),
          REPO / "submission" / ".build" / "him" / "environment" / "g1_robot.xml")
NEAR = 0.7          # m from the lidar: the body is within about 0.6 m (report.py's NEAR)
BODY = 0.5          # m: every return this near was the body in the 2026-11-02 capture
MARGINS = (0.0, 0.01, 0.02, 0.03, 0.04, MARGIN)
DROPS = (0.0, 0.01, 0.021, 0.03, 0.04)   # m the rays' origin is lowered in torso_link


def model_path():
    for path in MODELS:
        if path.exists():
            return path
    sys.exit("no G1 model: run in the submission image, or run submission/build.sh first")


def sim_ranges(m, d, q, directions, drop=0.0):
    """Where the simulator's rays along unit directions (lidar frame) meet the robot posed
    with joints q: range (m) each, NaN where a ray meets nothing. drop lowers the rays'
    origin along torso_link's z."""
    d.qpos[:] = 0.0
    d.qpos[3] = 1.0
    d.qpos[7:36] = q
    mujoco.mj_kinematics(m, d)
    torso_id = m.body(lidar_mount.PARENT).id
    torso = d.xmat[torso_id].reshape(3, 3)
    origin = d.xpos[torso_id] + torso @ (lidar_mount.POSITION - np.array([0.0, 0.0, drop]))
    n = len(directions)
    geomid, dist = np.zeros(n, np.int32), np.zeros(n)
    mujoco.mj_multiRay(m, d, origin, (directions @ (torso @ lidar_mount.rotation()).T).ravel(),
                       None, True, torso_id, geomid, dist, n, NEAR + 0.5)
    return np.where(geomid >= 0, dist, np.nan)


def percentiles(values):
    p = np.percentile(values, [5, 50, 95]) * 100
    return f"{p[0]:+.1f} / {p[1]:+.1f} / {p[2]:+.1f} cm"


def check(cap, path):
    filters = {margin: SelfFilter(path, margin) for margin in MARGINS}
    m = mujoco.MjModel.from_xml_path(str(path))
    d = mujoco.MjData(m)
    ls = cap.lowstate
    times = ls["arrival_ns"] * 1e-9
    joints = np.stack([ls[f"q{i}"] for i in range(29)], axis=1)
    near = 0
    body = 0
    left_near = {margin: 0 for margin in MARGINS}
    left_body = {margin: 0 for margin in MARGINS}
    far, far_dropped = 0, 0
    residuals, missed = [], []
    by_drop = {drop: [] for drop in DROPS}   # real minus sim within BODY, NaN for a miss
    clouds = 0
    for i in range(len(cap)):
        arrival = cap.packet_arrival_ns[cap.frame_end[i] - 1] * 1e-9
        poses = scan_poses(times, joints, arrival)
        if not poses:
            continue
        points, _ = cap.xyz(i)
        r = np.linalg.norm(points, axis=1)
        clouds += 1
        near += (r < NEAR).sum()
        body += (r < BODY).sum()
        for margin, f in filters.items():
            keep = f.keep(points, poses)
            left_near[margin] += (keep & (r < NEAR)).sum()
            left_body[margin] += (keep & (r < BODY)).sum()
            if margin == MARGIN:
                far += (r >= NEAR).sum()
                far_dropped += (~keep & (r >= NEAR)).sum()
        close = r < NEAR
        if close.any():
            sim = sim_ranges(m, d, poses[0], points[close] / r[close, None])
            hit = np.isfinite(sim)
            residuals.append(r[close][hit] - sim[hit])
            missed.append(r[close][~hit])
        inner = r < BODY
        if inner.any() and i % 4 == 0:     # every 4th cloud is plenty
            for drop in DROPS:
                by_drop[drop].append(r[inner] - sim_ranges(m, d, poses[0], points[inner] / r[inner, None], drop))
    if clouds == 0:
        sys.exit("no cloud has joint angles within the relay's window (lowstate.csv empty?)")
    residuals, missed = np.concatenate(residuals), np.concatenate(missed)
    per = lambda n: f"{n / clouds:.2f}"
    q = np.median(joints, axis=0)
    lines = [
        f"# Self-filter on {cap.meta['name']}", "",
        f"{clouds} clouds with joint angles; HIM's G1 model at {path}; margin {MARGIN * 100:.0f} cm.",
        f"Median arm joints (rad), left: {np.round(q[15:22], 2).tolist()}, right: {np.round(q[22:29], 2).tolist()}.", "",
        "## Returns near the lidar, per cloud", "",
        "| margin | within {:.1f} m | within {:.1f} m |".format(BODY, NEAR), "|---|---|---|",
        f"| no filter | {per(body)} | {per(near)} |",
    ]
    lines += [f"| {margin * 100:.0f} cm | {per(left_body[margin])} | {per(left_near[margin])} |"
              for margin in MARGINS]
    lines += [
        "", f"Beyond {NEAR} m, the filter dropped {far_dropped} of {far} returns at {MARGIN * 100:.0f} cm.", "",
        f"## Real minus sim, along the rays within {NEAR} m", "",
        f"{len(residuals)} returns meet the model: 5th / median / 95th percentile {percentiles(residuals)}"
        if len(residuals) else "No return within that range meets the model.",
        f"(within 3 cm: {np.mean(np.abs(residuals) <= 0.03) * 100:.0f}%)." if len(residuals) else "",
        f"{len(missed)} meet nothing in the model ({per(len(missed))} a cloud)"
        + (f", at {np.round(np.percentile(missed, [5, 50, 95]), 2).tolist()} m (5th / median / 95th)."
           if len(missed) else "."),
    ]
    lines += ["", f"## Origin: real minus sim within {BODY} m, rays from a lower origin", "",
              "| lowered | meet the model | 5th / median / 95th | within 2 cm |", "|---|---|---|---|"]
    for drop in DROPS:
        res = np.concatenate(by_drop[drop]) if by_drop[drop] else np.zeros(0)
        hit = res[np.isfinite(res)]
        if len(hit):
            lines.append(f"| {drop * 1000:.0f} mm | {len(hit) / len(res) * 100:.0f}% | {percentiles(hit)} | "
                         f"{np.mean(np.abs(hit) <= 0.02) * 100:.0f}% |")
    return "\n".join(lines) + "\n"


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("capture", type=Path)
    args = parser.parse_args()
    cap = Capture(args.capture)
    if cap.headers_only:
        sys.exit("a headers-only capture has no points")
    if not cap.lowstate or len(cap.lowstate["arrival_ns"]) == 0:
        sys.exit("the capture has no lowstate.csv: no joint angles to pose the body")
    text = check(cap, model_path())
    (args.capture / "self_filter.md").write_text(text)
    print(text)


if __name__ == "__main__":
    main()
