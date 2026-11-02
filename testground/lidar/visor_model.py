"""Model the G1's face visor on Unitree's head mesh and cast the head lidar's rays in
MuJoCo with and without it, against what one capture measured: what taking the visor
off would let the lidar see.

  docker run --rm -v "$PWD/../..:/repo" -w /repo/testground/lidar --entrypoint python3 \\
      harp-submission:local visor_model.py captures/NAME

(from testground/lidar; the submission image has MuJoCo, and submission/build.sh stages
Unitree's G1 model in submission/.build.) Saves visor_model.md and visor_model.svg in
the capture's folder.

The model, in torso_link:
- Unitree's head_link.STL has no visor: its face is an open oval frame, the camera in
  a band at its top, the lidar's window dome below the cap.
- The lidar's rays start at its optical centre: Livox's Mid-360 drawing puts it 47 mm
  from the mounting face and the window from 39.5 to 60 mm, so upside down it is 13 mm
  above the dome's bottom. The dome itself is left out (it is the window).
- The visor covers the frame's front at each height, across its full width (Unitree's
  side drawing in the 中邮证券 G1 teardown shows it in front of both sensors), opaque.
- The D435i is Intel's 90 x 25 x 25 mm body, its URDF frame the depth origin (the left
  imager, 17.5 mm left of the body's centre and 4.2 mm behind the front glass).
- The robot is posed with the capture's median joint angles, so the arms are where they were.
"""

import argparse
import re
import struct
import sys
import tempfile
from pathlib import Path

import mujoco
import numpy as np

from blind_map import AZ, EL, STEP, measure, visibility
from capture import REPO, Capture
from report import NEAR

sys.path.insert(0, str(REPO / "submission"))
from harp import camera_mount, lidar_mount  # noqa: E402  (numpy only)

MODEL = REPO / "submission" / ".build" / "him" / "environment" / "third_party" / "unitree_g1"
ABOVE_DOME = 0.013                                   # m: optical centre over the window's bottom
CAMERA_HALF = np.array([0.0125, 0.045, 0.0125])      # m: D435i half depth, width, height
CAMERA_CENTRE = np.array([0.0042 - 0.0125, -0.0175, 0.0])   # m: body centre from its URDF frame
VISOR_GROUP = 4                                      # geom group of the visor, so rays can leave it out
LEAN = np.radians(6.0)                               # torso lean standing, and the lidar's height
HEIGHT = 1.21                                        # above the floor (g1-lidar-measured)


def read_stl(path):
    data = Path(path).read_bytes()
    n = np.frombuffer(data, "<u4", 1, 80)[0]
    rec = np.frombuffer(data, np.dtype([("n", "<3f4"), ("v", "<9f4"), ("a", "<u2")]), n, 84)
    return rec["v"].reshape(-1, 3, 3).astype(np.float64)


def write_stl(path, tris):
    n = np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0])
    n /= np.linalg.norm(n, axis=1, keepdims=True) + 1e-15
    out = bytearray(80) + struct.pack("<I", len(tris))
    for k in range(len(tris)):
        out += struct.pack("<12fH", *n[k], *tris[k].ravel(), 0)
    Path(path).write_bytes(out)


def head_parts(xml):
    """The head mesh without the lidar's dome (head_link frame), the optical centre and
    the visor (torso_link), from Unitree's head mesh and where g1.xml puts it."""
    pos = re.search(r'<geom pos="([^"]+)"[^>]*mesh="head_link"', xml).group(1)
    offset = np.array([float(v) for v in pos.split()])
    raw = read_stl(MODEL / "assets" / "head_link.STL")
    head = raw + offset
    c = head.mean(axis=1)
    dome = (c[:, 2] < 0.4185) & (c[:, 2] > 0.39) & (np.hypot(c[:, 0], c[:, 1]) < 0.03)
    optical = np.array([lidar_mount.POSITION[0], 0.0, head[dome][..., 2].min() + ABOVE_DOME])
    # The face frame's front-most x and half-width at each height
    frame = (c[:, 0] > 0.02) & (np.abs(c[:, 1]) > 0.035) & (np.abs(c[:, 1]) < 0.07)
    profile = []
    for z in np.arange(0.350, 0.462, 0.002):
        s = frame & (np.abs(c[:, 2] - z) < 0.002)
        if s.sum() >= 3:
            profile.append((z, c[s, 0].max(), np.abs(c[s, 1]).max()))
    ys = np.linspace(-1, 1, 41)
    visor = []
    for (z0, x0, w0), (z1, x1, w1) in zip(profile[:-1], profile[1:]):
        for a, b in zip(ys[:-1], ys[1:]):
            p = [(x0, a * w0, z0), (x0, b * w0, z0), (x1, b * w1, z1), (x1, a * w1, z1)]
            visor += [[p[0], p[1], p[2]], [p[0], p[2], p[3]]]
    return raw[~dome], optical, np.array(visor)


def camera_box():
    """The D435i's body as 12 triangles in torso_link."""
    R = lidar_mount.rpy_matrix(*camera_mount.RPY)
    centre = camera_mount.POSITION + R @ CAMERA_CENTRE
    tris = []
    for axis in range(3):
        for sign in (-1, 1):
            quad = []
            for a, b in ((-1, -1), (1, -1), (1, 1), (-1, 1)):
                s = [0, 0, 0]
                s[axis], s[(axis + 1) % 3], s[(axis + 2) % 3] = sign, a, b
                quad.append(centre + R @ (CAMERA_HALF * np.array(s)))
            tris += [[quad[0], quad[1], quad[2]], [quad[0], quad[2], quad[3]]]
    return np.array(tris)


def build(xml, folder, shell, visor):
    """g1.xml with the head's dome removed and the visor and camera added on torso_link."""
    for name, tris in (("head_shell", shell), ("visor", visor), ("camera_box", camera_box())):
        write_stl(folder / f"{name}.stl", tris)
    xml = xml.replace('meshdir="assets"', f'meshdir="{MODEL / "assets"}"')
    xml = xml.replace('<mesh file="head_link.STL"/>', '<mesh file="head_link.STL"/>' + "".join(
        f'<mesh name="{n}" file="{folder / (n + ".stl")}"/>' for n in ("head_shell", "visor", "camera_box")))
    xml, n = re.subn(r'(<geom [^>]*class="collision"[^>]*) mesh="head_link"/>',
                     rf'\1 mesh="head_shell"/><geom type="mesh" mesh="visor" group="{VISOR_GROUP}" '
                     r'contype="0" conaffinity="0"/><geom type="mesh" mesh="camera_box" group="1" '
                     r'contype="0" conaffinity="0"/>', xml)
    assert n == 1, "g1.xml's head collision geom not found"
    return mujoco.MjModel.from_xml_string(xml)


def cast(m, d, optical, visor):
    """Cells whose ray from the optical centre hits the robot within NEAR (collision
    geoms, the camera, and the visor if asked)."""
    torso = m.body("torso_link").id
    R, p = d.xmat[torso].reshape(3, 3), d.xpos[torso]
    a, e = np.meshgrid(np.radians(AZ[:-1] + STEP / 2), np.radians(EL[:-1] + STEP / 2), indexing="ij")
    dirs = np.stack([np.cos(e) * np.cos(a), np.cos(e) * np.sin(a), np.sin(e)], -1).reshape(-1, 3)
    groups = np.array([0, 1, 0, 1, int(visor), 0], np.uint8)
    geom, dist = np.zeros(len(dirs), np.int32), np.zeros(len(dirs))
    mujoco.mj_multiRay(m, d, p + R @ optical, (dirs @ R.T).ravel(), groups, 1, -1, geom, dist, len(dirs), 10.0)
    return ((dist >= 0) & (dist < NEAR)).reshape(a.shape)


def span(open_col, centre):
    """The run of open cells around straight ahead, as (from, to) degrees, or None."""
    if not open_col[centre]:
        return None
    lo = hi = centre
    while lo > 0 and open_col[lo - 1]:
        lo -= 1
    while hi < len(open_col) - 1 and open_col[hi + 1]:
        hi += 1
    return AZ[lo], AZ[hi + 1]


def summary(name, optical, vis, rows, on, off):
    ac = AZ[:-1] + STEP / 2
    front = (np.abs(ac) < 90)[:, None] & rows[None, :]
    blind = (vis < 0.1) & rows[None, :]
    out = [f"# The face visor and the head lidar: `{name}`", "",
           f"Optical centre {optical[2]:.4f} m up torso_link, {(lidar_mount.POSITION[2] - optical[2]) * 1000:.0f} mm "
           "below the URDF lidar origin. Within ±90° of ahead, over the lidar's elevations "
           f"(blind: hits the robot within {NEAR} m; measured: under 10% visible):", "",
           "| | blind | cells agreeing with the measurement |", "|---|---|---|",
           f"| measured | {blind[front].mean():.1%} | |"]
    for label, m in (("modelled, visor on", on), ("modelled, visor off", off)):
        out.append(f"| {label} | {m[front].mean():.1%} | {(m == blind)[front].mean():.1%} |")
    centre = int(np.argmin(np.abs(ac)))
    out += ["", "Open around straight ahead, by elevation (torso_link):", "",
            "| elevation | measured | visor on | visor off |", "|---|---|---|---|"]
    for j in range(len(EL) - 2, -1, -2):
        if rows[j]:
            cells = [span(vis[:, j] >= 0.5, centre), span(~on[:, j], centre), span(~off[:, j], centre)]
            out.append(f"| {EL[j] + STEP / 2:+.0f}° | " + " | ".join(
                "none" if s is None else f"{s[0]:+.0f}° to {s[1]:+.0f}°" for s in cells) + " |")
    return "\n".join(out) + "\n"


def figure(name, rows, today, opened):
    """Three views of what taking the visor off opens: the directions around the front,
    straight ahead from the side with a person 2 m away, and the floor from above."""
    ac, ec = AZ[:-1] + STEP / 2, EL[:-1] + STEP / 2
    grey, green, blue, ink = "#1f1f1f", "#16a34a", "#2563eb", "#333"
    o = []

    def text(x, y, s, **k):
        attrs = " ".join(f'{a.replace("_", "-")}="{v}"' for a, v in k.items())
        o.append(f'<text x="{x:.1f}" y="{y:.1f}" {attrs}>{s}</text>')

    def colour(i, j):
        return blue if today[i, j] else green if opened[i, j] else None

    def world(azd, eld):
        a, e = np.radians(azd), np.radians(eld)
        d = np.array([np.cos(e) * np.cos(a), np.cos(e) * np.sin(a), np.sin(e)])
        c, s = np.cos(LEAN), np.sin(LEAN)
        return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]]) @ d

    text(30, 30, f"Head lidar with and without the face visor ({name})", font_size=16, font_weight="bold")
    text(30, 50, "Green = opened by taking the visor off (MuJoCo model).  Blue = seen today (measured).", fill="#444")
    text(30, 66, "Black = blocked either way. Today's few stray returns ahead count as blocked.", fill="#444")
    # A: every direction around the front
    C, L, T = 6, 70, 100
    ai = [i for i in range(len(ac)) if -120 <= ac[i] < 120][::-1]
    ej = [j for j in range(len(ec)) if rows[j]][::-1]
    text(L, T - 8, "A. Every direction around the front, as the lidar sees it", font_weight="bold")
    for x, i in enumerate(ai):
        for y, j in enumerate(ej):
            o.append(f'<rect x="{L + x * C}" y="{T + y * C}" width="{C}" height="{C}" fill="{colour(i, j) or grey}"/>')
    ph = len(ej) * C
    for a in range(-120, 121, 30):
        x = L + (120 - a) / STEP * C
        o.append(f'<line x1="{x}" y1="{T + ph}" x2="{x}" y2="{T + ph + 4}" stroke="{ink}"/>')
        text(x, T + ph + 16, f"{a:+d}°" if a else "ahead", text_anchor="middle", font_size=11)
    text(L + len(ai) * C / 2, T + ph + 32, "azimuth (robot's left on the left)", text_anchor="middle", font_size=11, fill="#444")
    for e in (0, -20, -40):
        text(L - 6, T + (ec[ej[0]] + STEP / 2 - e) / STEP * C + 4, f"{e:+d}°", text_anchor="end", font_size=11)
    # B: straight ahead from the side, a 1.7 m person 2 m away
    T2, S = T + ph + 70, 150
    gx, gy = (lambda x: L + x * S), (lambda z: T2 + (1.8 - z) * S)
    text(L, T2 - 8, "B. Straight ahead, from the side: a 1.7 m person 2 m away", font_weight="bold")
    o.append(f'<line x1="{gx(-0.2)}" y1="{gy(0)}" x2="{gx(4.2)}" y2="{gy(0)}" stroke="{ink}" stroke-width="1.5"/>')
    for x in range(5):
        text(gx(x), gy(0) + 15, f"{x} m", text_anchor="middle", font_size=11)
    o.append(f'<rect x="{gx(1.92)}" y="{gy(1.7)}" width="{0.16 * S}" height="{1.7 * S}" rx="6" fill="#e5e7eb" stroke="#9ca3af"/>')
    o.append(f'<circle cx="{gx(0)}" cy="{gy(HEIGHT)}" r="6" fill="{ink}"/>')
    text(gx(0) - 8, gy(HEIGHT) - 10, "lidar", text_anchor="end", font_size=11)
    i0 = int(np.argmin(np.abs(ac)))
    for j in (j for j in range(len(ec)) if rows[j]):
        d, col = world(0.0, ec[j]), colour(i0, j)
        if col is None:
            o.append(f'<line x1="{gx(0)}" y1="{gy(HEIGHT)}" x2="{gx(d[0] * 0.12)}" y2="{gy(HEIGHT + d[2] * 0.12)}" stroke="#bbb"/>')
            continue
        floor = HEIGHT / -d[2] if d[2] < 0 else np.inf
        person = 2.0 / d[0]
        t = person if person < floor and HEIGHT + person * d[2] <= 1.7 else min(floor, 4.2 / d[0])
        o.append(f'<line x1="{gx(0)}" y1="{gy(HEIGHT)}" x2="{gx(t * d[0])}" y2="{gy(HEIGHT + t * d[2])}" stroke="{col}" stroke-width="1.6"/>')
        if t == person:
            o.append(f'<circle cx="{gx(t * d[0])}" cy="{gy(HEIGHT + t * d[2])}" r="3" fill="{col}"/>')
    text(gx(2.4), gy(1.2), "grey stubs: rays the head stops", font_size=11, fill="#444")
    # C: the floor from above, out to 5 m
    T3, S3 = gy(0) + 60, 70
    cx, cy = L + 0.5 * S3, T3 + 3.2 * S3
    text(L, T3 - 8, "C. From above: where its rays reach the floor, out to 5 m (robot at the left, facing right)", font_weight="bold")
    o.append(f'<defs><clipPath id="c"><rect x="{L}" y="{T3}" width="{6 * S3}" height="{6.4 * S3}"/></clipPath></defs>'
             f'<rect x="{L}" y="{T3}" width="{6 * S3}" height="{6.4 * S3}" fill="#fafafa" stroke="#ddd"/><g clip-path="url(#c)">')
    for r in range(1, 6):
        o.append(f'<circle cx="{cx}" cy="{cy}" r="{r * S3}" fill="none" stroke="#e5e5e5"/>')
        text(cx + r * S3 + 3, cy - 4, f"{r} m", font_size=10, fill="#999")
    for i in range(len(ac)):
        for j in range(len(ec)):
            col = colour(i, j) if rows[j] and abs(ac[i]) <= 120 else None
            d = world(ac[i], ec[j])
            if col and d[2] < 0 and np.hypot(*(HEIGHT / -d[2] * d[:2])) <= 5:
                x, y = HEIGHT / -d[2] * d[:2]
                o.append(f'<circle cx="{cx + x * S3:.1f}" cy="{cy - y * S3:.1f}" r="2.2" fill="{col}"/>')
    o.append(f'</g><polygon points="{cx - 8},{cy - 9} {cx + 12},{cy} {cx - 8},{cy + 9}" fill="{ink}"/>')
    H = T3 + 6.4 * S3 + 40
    text(30, H - 14, f"Visor off is modelled. Lidar {HEIGHT} m up, torso leaning {np.degrees(LEAN):.0f}° forward; arms as recorded.",
         font_size=10, fill="#666")
    return (f'<svg xmlns="http://www.w3.org/2000/svg" width="900" height="{H:.0f}" viewBox="0 0 900 {H:.0f}" '
            'font-family="Helvetica,Arial,sans-serif" font-size="12"><rect width="900" height="100%" fill="#fff"/>'
            + "\n".join(o) + "</svg>\n")


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("capture", type=Path)
    args = parser.parse_args()
    cap = Capture(args.capture)
    if cap.headers_only:
        sys.exit("a headers-only capture has no points")
    xml = (MODEL / "g1.xml").read_text()
    shell, optical, visor = head_parts(xml)
    with tempfile.TemporaryDirectory() as folder:
        m = build(xml, Path(folder), shell, visor)
    d = mujoco.MjData(m)
    d.qpos[:7] = [0, 0, 0.79, 1, 0, 0, 0]
    d.qpos[7:36] = [np.median(cap.lowstate[f"q{i}"]) for i in range(29)]
    mujoco.mj_forward(m, d)
    on, off = cast(m, d, optical, True), cast(m, d, optical, False)
    vis, _, rows = visibility(*measure(cap)[:2])
    ac = AZ[:-1] + STEP / 2
    today = (np.nan_to_num(vis) >= 0.5) & (np.abs(ac) >= 60)[:, None]   # the returns ahead are stray
    opened = on & ~off & ~today
    name = cap.meta["name"]
    (args.capture / "visor_model.svg").write_text(figure(name, rows, today, opened))
    text = summary(name, optical, vis, rows, on, off)
    (args.capture / "visor_model.md").write_text(text)
    print(text)


if __name__ == "__main__":
    main()
