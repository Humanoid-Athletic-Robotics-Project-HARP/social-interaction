"""Map where the head lidar gets returns around the robot's front, from one capture:
the blind wedge ahead. What makes it (the face visor): visor_model.py.

  ../.venv/bin/python blind_map.py captures/NAME

Directions are from the lidar in torso_link (azimuth 0 deg ahead, + to the left;
elevation + up), by the mounts in submission/harp. A cell's visibility is its returns
beyond NEAR m over the median of its elevation row (the scan pattern's density varies
with elevation), so 1 is as often as anywhere else and 0 is never. Returns ahead are
also checked for being stray (reflectivity 0-1, unlike real surfaces at that range).
Saves blind_map.svg and blind_map.md in the capture's folder.
"""

import argparse
import sys
from pathlib import Path

import numpy as np

from capture import MOUNT_R, Capture
from report import NEAR

STEP = 2.0                                 # deg, cell size
AZ = np.arange(-180, 180 + STEP, STEP)
EL = np.arange(-60, 14 + STEP, STEP)
SHOWN = 120                                # deg either side of ahead in the picture
AHEAD = 30                                 # deg: "straight ahead" for the summary


def directions(xyz):
    """Torso-frame azimuth and elevation (deg) and range (m) of lidar-frame points."""
    d = xyz @ MOUNT_R.T
    r = np.linalg.norm(d, axis=1)
    return np.degrees(np.arctan2(d[:, 1], d[:, 0])), np.degrees(np.arcsin(d[:, 2] / r)), r


def measure(cap):
    """Returns beyond and within NEAR per cell, and the reflectivity of the returns
    ahead and of those elsewhere at the same ranges."""
    far = np.zeros((len(AZ) - 1, len(EL) - 1))
    near = np.zeros_like(far)
    ahead_ref, ahead_r, other = [], [], []
    for i in cap.frames():
        c = cap.cloud(i)
        xyz, keep = cap.xyz(i, c)
        az, el, r = directions(xyz)
        ref = c["reflectivity"][keep]
        far += np.histogram2d(az[r >= NEAR], el[r >= NEAR], (AZ, EL))[0]
        near += np.histogram2d(az[r < NEAR], el[r < NEAR], (AZ, EL))[0]
        ahead = np.abs(az) < AHEAD
        ahead_ref.append(ref[ahead])
        ahead_r.append(r[ahead])
        other.append(np.stack([r[np.abs(az) > 2 * AHEAD], ref[np.abs(az) > 2 * AHEAD]], 1))
    return far, near, np.concatenate(ahead_ref), np.concatenate(ahead_r), np.concatenate(other)


def visibility(far, near):
    """Per cell: returns beyond and within NEAR over the median of the elevation row, and
    which rows the lidar covers."""
    ref = np.median(far + near, axis=0)
    rows = ref > 0.05 * ref.max()
    scale = np.where(ref > 0, ref, 1)
    return np.where(rows, far / scale, np.nan), np.where(rows, near / scale, 0), rows


def summary(name, vis, rows, ahead_ref, ahead_r, other):
    out = [f"# Where the lidar sees around the front: `{name}`", "",
           f"Torso_link directions from the lidar, {STEP:.0f}° cells; visibility is returns beyond "
           f"{NEAR} m relative to the same elevation elsewhere. Picture: `blind_map.svg`.", ""]
    # The blind wedge: from ahead outwards, the first azimuth on each side that sees
    centre = int(np.searchsorted(AZ, 0))
    sees = np.where(rows[None, :], vis >= 0.5, False).mean(axis=1) > 0.5
    left = next((AZ[i] for i in range(centre, len(AZ) - 1) if sees[i]), None)
    right = next((AZ[i + 1] for i in range(centre - 1, -1, -1) if sees[i]), None)
    out.append(f"- Blind wedge ahead (most elevations see under half as often as elsewhere): "
               f"{right:+.0f}° to {left:+.0f}°.")
    out += ["", f"Visible share within ±{AHEAD}° of ahead, by elevation:", "",
            "| elevation | visible |", "|---|---|"]
    ahead = np.abs(AZ[:-1] + STEP / 2) < AHEAD
    for j in range(len(EL) - 2, -1, -2):
        if rows[j]:
            out.append(f"| {EL[j - 1]:+.0f}° to {EL[j + 1]:+.0f}° | {np.nanmean(vis[ahead, j - 1:j + 1]):.1%} |")
    lo, hi = (np.percentile(ahead_r, [5, 95]) if len(ahead_r) else (0, 0))
    same = other[(other[:, 0] >= lo) & (other[:, 0] <= hi), 1]
    out += ["",
            f"- Returns within ±{AHEAD}° of ahead: {len(ahead_ref)}, range 5–95% {lo:.2f}–{hi:.2f} m, "
            f"reflectivity 0–1: {np.mean(ahead_ref <= 1):.0%}"
            + (f" (elsewhere at those ranges: {np.mean(same <= 1):.0%}, median {np.median(same):.0f})" if len(same) else "")
            + ". Mostly 0–1 means stray returns, not a surface."]
    return "\n".join(out) + "\n"


def svg(name, vis, body, rows):
    """The map around the front as an SVG: grey by visibility, own body blue; seen from
    behind the robot (its left on the left)."""
    C, L, T = 9, 70, 56
    ai = [i for i in range(len(AZ) - 1) if -SHOWN <= AZ[i] < SHOWN][::-1]
    ej = [j for j in range(len(EL) - 1) if rows[j]][::-1]
    W, H = L + len(ai) * C + 30, T + len(ej) * C + 100
    X = {i: L + k * C for k, i in enumerate(ai)}
    Y = {j: T + k * C for k, j in enumerate(ej)}
    o = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H}" viewBox="0 0 {W} {H}" '
         'font-family="Helvetica,Arial,sans-serif" font-size="12">',
         f'<rect width="{W}" height="{H}" fill="#fff"/>',
         f'<text x="{L}" y="22" font-size="15" font-weight="bold">Head lidar around the front: where it gets returns ({name})</text>',
         f'<text x="{L}" y="41" fill="#444">Direction from the lidar in torso_link. Shade = returns beyond {NEAR} m, '
         'relative to the same elevation elsewhere: white always, black never.</text>']
    for i in ai:
        for j in ej:
            v = min(np.nan_to_num(vis[i, j]), 1)
            fill = "#3b6fd8" if body[i, j] > 0.5 and body[i, j] > v else "rgb({0},{0},{0})".format(int(255 * v))
            o.append(f'<rect x="{X[i]}" y="{Y[j]}" width="{C}" height="{C}" fill="{fill}"/>')
    bottom = T + len(ej) * C
    for a in range(-SHOWN, SHOWN + 1, 30):
        x = L + (SHOWN - a) / STEP * C
        o.append(f'<line x1="{x}" y1="{T}" x2="{x}" y2="{bottom + 4}" stroke="{"#16a34a" if a == 0 else "#aaa"}" '
                 f'stroke-width="{2 if a == 0 else 0.7}"/><text x="{x}" y="{bottom + 18}" text-anchor="middle">{a:+d}°</text>')
    o.append(f'<text x="{L + len(ai) * C / 2}" y="{bottom + 35}" text-anchor="middle">'
             "azimuth (0° straight ahead, + to the robot's left; seen from behind the robot)</text>")
    top = EL[ej[0] + 1]
    for e in range(0, int(EL[ej[-1]]) - 1, -10):
        y = T + (top - e) / STEP * C
        o.append(f'<line x1="{L - 3}" y1="{y}" x2="{L + len(ai) * C}" y2="{y}" stroke="#aaa" stroke-width="0.5"/>'
                 f'<text x="{L - 7}" y="{y + 4}" text-anchor="end">{e:+d}°</text>')
    mid = T + len(ej) * C / 2
    o.append(f'<text x="18" y="{mid}" transform="rotate(-90 18 {mid})" text-anchor="middle">elevation</text>')
    o.append(f'<rect x="{L}" y="{bottom + 52}" width="13" height="13" fill="#3b6fd8"/>'
             f'<text x="{L + 20}" y="{bottom + 63}">own body within {NEAR} m</text>')
    return "\n".join(o + ["</svg>"]) + "\n"


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("capture", type=Path)
    args = parser.parse_args()
    cap = Capture(args.capture)
    if cap.headers_only:
        sys.exit("a headers-only capture has no points")
    far, near, ahead_ref, ahead_r, other = measure(cap)
    vis, body, rows = visibility(far, near)
    name = cap.meta["name"]
    (args.capture / "blind_map.svg").write_text(svg(name, vis, body, rows))
    text = summary(name, vis, rows, ahead_ref, ahead_r, other)
    (args.capture / "blind_map.md").write_text(text)
    print(text)


if __name__ == "__main__":
    main()
