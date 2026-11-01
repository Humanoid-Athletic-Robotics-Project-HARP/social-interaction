#!/usr/bin/env python3
"""Browser-based live viewer for the G1's head camera (Intel RealSense D435i).

    python3 camera_viewer.py            # then open http://<robot-wlan-ip>:8081

Streams
  depth   /dev/video0, Z16 in mm, read straight through V4L2 (no librealsense). It is
          a different node from the colour one, so Unitree's videohub keeps working.
  colour  Unitree's videohub service over DDS (GetImageSample, a JPEG), because
          videohub owns the colour node (/dev/video4). This sends it image requests
          and nothing else; --no-colour skips it.

The page also draws a top-down map of what the depth camera sees ahead. Depth pixels
become points with nominal intrinsics (Intel's 87 x 58 deg field of view), levelled
on a floor plane fitted every second, so the page also shows the camera's real
height and pitch (the URDF says 47.6 deg down from the torso).

Python 3.8 and what PC2 has: numpy, OpenCV, unitree_sdk2py. Runs at low priority
beside HARP's harness.
"""
import argparse
import fcntl
import json
import math
import mmap
import os
import select
import struct
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

import cv2
import numpy as np

# V4L2 (linux/videodev2.h, 64-bit layouts)
S_FMT, S_PARM, REQBUFS = 0xC0D05605, 0xC0CC5616, 0xC0145608
QUERYBUF, QBUF, DQBUF = 0xC0585609, 0xC058560F, 0xC0585611
STREAMON, STREAMOFF = 0x40045612, 0x40045613
CAPTURE, MMAP = 1, 1

HFOV, VFOV = 87.0, 58.0          # deg, the D435i depth camera (Intel's nominal values)
URDF_PITCH = math.degrees(0.8307767239493009)   # d435_joint, down from torso_link
DEPTH_RANGE = (0.2, 4.0)         # m, the colour scale of the depth image
MIN_DEPTH = 0.2                  # m, nearer is noise (the D435i's minimum is about 0.2 m)
STEP = 4                         # depth pixels per point, each way, for the map and floor fit
MAP_W, MAP_H, MAP_AHEAD = 640, 480, 3.0   # px, and m ahead, of the top-down map
OBSTACLE = 0.08                  # m above the floor: higher is something sticking up
SELF_RADIUS = 0.3                # m around the camera's foot that is the robot itself


def fourcc(code):
    return struct.unpack("<I", code.encode())[0]


class V4L2Stream(threading.Thread):
    """The newest frame from one V4L2 capture node (memory-mapped streaming)."""

    def __init__(self, device, code, width, height, fps, dtype):
        super().__init__(daemon=True)
        self.device, self.code, self.dtype = device, code, np.dtype(dtype)
        self.size, self.fps = (width, height), fps
        self.lock = threading.Lock()
        self.frame, self.count, self.status = None, 0, "starting"
        self.running = True

    def open(self):
        self.fd = os.open(self.device, os.O_RDWR | os.O_NONBLOCK)
        fmt = bytearray(208)
        struct.pack_into("<I", fmt, 0, CAPTURE)
        struct.pack_into("<III", fmt, 8, self.size[0], self.size[1], fourcc(self.code))
        fcntl.ioctl(self.fd, S_FMT, fmt)
        w, h, _, _, self.stride = struct.unpack_from("<5I", fmt, 8)
        self.size = (w, h)
        parm = bytearray(204)
        struct.pack_into("<I", parm, 0, CAPTURE)
        struct.pack_into("<II", parm, 12, 1, self.fps)
        fcntl.ioctl(self.fd, S_PARM, parm)
        req = bytearray(struct.pack("<III", 4, CAPTURE, MMAP) + bytes(8))
        fcntl.ioctl(self.fd, REQBUFS, req)
        self.maps = []
        for i in range(struct.unpack_from("<I", req, 0)[0]):
            buf = self.buffer(i)
            fcntl.ioctl(self.fd, QUERYBUF, buf)
            offset, length = struct.unpack_from("<I", buf, 64)[0], struct.unpack_from("<I", buf, 72)[0]
            self.maps.append(mmap.mmap(self.fd, length, mmap.MAP_SHARED, mmap.PROT_READ,
                                       offset=offset))
            fcntl.ioctl(self.fd, QBUF, buf)
        fcntl.ioctl(self.fd, STREAMON, struct.pack("<I", CAPTURE))

    @staticmethod
    def buffer(index=0):
        buf = bytearray(88)
        struct.pack_into("<II", buf, 0, index, CAPTURE)
        struct.pack_into("<I", buf, 60, MMAP)
        return buf

    def run(self):
        try:
            self.open()
        except OSError as e:
            self.status = f"can't open {self.device}: {e}"
            return
        w, h = self.size
        self.status = f"{w}x{h} {self.code} at {self.fps} fps"
        while self.running:
            if not select.select([self.fd], [], [], 1.0)[0]:
                self.status = f"no frames from {self.device}"
                continue
            buf = self.buffer()
            try:
                fcntl.ioctl(self.fd, DQBUF, buf)
            except BlockingIOError:
                continue
            index, used = struct.unpack_from("<I", buf, 0)[0], struct.unpack_from("<I", buf, 8)[0]
            raw = np.frombuffer(self.maps[index][:used], np.uint8)
            fcntl.ioctl(self.fd, QBUF, buf)
            if raw.size < self.stride * h:
                continue
            row = w * self.dtype.itemsize
            frame = raw[:self.stride * h].reshape(h, self.stride)[:, :row].copy().view(self.dtype)
            with self.lock:
                self.frame, self.count = frame, self.count + 1
            self.status = f"{w}x{h} {self.code} at {self.fps} fps"

    def latest(self):
        with self.lock:
            return self.frame

    def close(self):
        self.running = False
        self.join(timeout=2)
        try:
            fcntl.ioctl(self.fd, STREAMOFF, struct.pack("<I", CAPTURE))
            for m in self.maps:
                m.close()
            os.close(self.fd)
        except (AttributeError, OSError):
            pass


class Colour(threading.Thread):
    """JPEGs from Unitree's videohub service (as ~/Workspace/test/g1_camera_mosaic.py)."""

    def __init__(self, interface, period):
        super().__init__(daemon=True)
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize
        from unitree_sdk2py.go2.video.video_api import VIDEO_API_ID_GETIMAGESAMPLE, VIDEO_API_VERSION
        from unitree_sdk2py.rpc.client import Client

        class Hub(Client):
            def __init__(self):
                super().__init__("videohub", False)

            def Init(self):
                self._SetApiVerson(VIDEO_API_VERSION)
                self._RegistApi(VIDEO_API_ID_GETIMAGESAMPLE, 0)

            def image(self):
                return self._CallBinary(VIDEO_API_ID_GETIMAGESAMPLE, [])

        ChannelFactoryInitialize(0, interface)
        self.hub = Hub()
        self.hub.SetTimeout(2.0)
        self.hub.Init()
        self.period = period
        self.jpeg, self.count, self.status = None, 0, "starting"
        self.running = True

    def run(self):
        while self.running:
            try:
                code, data = self.hub.image()
            except Exception as e:
                code, data = f"error {e}", None
            if code != 0 or not data:
                self.status = f"videohub answered {code} (3102: no camera)"
                time.sleep(1.0)
                continue
            self.jpeg, self.count = bytes(data), self.count + 1
            self.status = "videohub JPEG"
            time.sleep(self.period)


class Depth:
    """Turns depth frames into the page's images and numbers, and fits the floor."""

    def __init__(self, stream, lean):
        self.stream = stream
        w, h = stream.size
        self.fx = (w / 2) / math.tan(math.radians(HFOV / 2))
        self.fy = (h / 2) / math.tan(math.radians(VFOV / 2))
        self.cx, self.cy = (w - 1) / 2, (h - 1) / 2
        pitch = math.radians(URDF_PITCH + lean)      # nominal, until the floor is found
        self.up = np.array([0.0, -math.cos(pitch), -math.sin(pitch)])   # world up, camera frame
        self.height, self.floor_ok, self.inliers = 1.22, False, 0
        self.rng = np.random.default_rng()
        self.lock = threading.Lock()
        self.depth_jpeg = self.map_jpeg = None
        self.stats = {}
        self.wanted = 0.0           # time.monotonic() a page last asked for an image

    def points(self, depth, step=STEP):
        """Camera-frame points (x right, y down, z forward), m, from every step-th pixel."""
        z = depth[::step, ::step].astype(np.float32) * 1e-3
        v, u = np.mgrid[0:depth.shape[0]:step, 0:depth.shape[1]:step]
        keep = z > MIN_DEPTH
        z, u, v = z[keep], u[keep], v[keep]
        return np.stack([(u - self.cx) * z / self.fx, (v - self.cy) * z / self.fy, z], axis=1)

    def fit_floor(self, pts):
        if len(pts) < 500:
            return
        sample = pts[self.rng.choice(len(pts), min(len(pts), 6000), replace=False)]
        best, best_count = None, 0
        for _ in range(150):
            a, b, c = sample[self.rng.choice(len(sample), 3, replace=False)]
            n = np.cross(b - a, c - a)
            norm = np.linalg.norm(n)
            if norm < 1e-9:
                continue
            n = n / norm
            if n @ self.up < 0:
                n = -n
            if n @ self.up < math.cos(math.radians(25)):
                continue
            d = -(n @ a)                 # n.p + d = 0, d is the camera's height above it
            if not 0.5 < d < 2.0:
                continue
            count = int(np.count_nonzero(np.abs(sample @ n + d) < 0.03))
            if count > best_count:
                best, best_count = (n, d), count
        if best is None or best_count < 300:
            return
        n, d = best
        inliers = sample[np.abs(sample @ n + d) < 0.03]
        centre = inliers.mean(axis=0)
        n = np.linalg.svd(inliers - centre, full_matrices=False)[2][2]
        n = n if n @ self.up > 0 else -n
        d = -(n @ centre)
        if self.floor_ok:                # smooth out frame-to-frame jitter
            n = 0.7 * self.up + 0.3 * n
            n /= np.linalg.norm(n)
            d = 0.7 * self.height + 0.3 * d
        self.up, self.height, self.floor_ok, self.inliers = n, float(d), True, len(inliers)

    def level(self, pts):
        """Camera-frame points to (ahead, left, height above the floor), m."""
        ahead = np.array([0.0, 0.0, 1.0]) - self.up[2] * self.up   # optical axis, levelled
        ahead /= np.linalg.norm(ahead)
        left = np.cross(self.up, ahead)
        return np.stack([pts @ ahead, pts @ left, pts @ self.up + self.height], axis=1)

    def pitch(self):
        """The optical axis's angle below horizontal, deg."""
        return math.degrees(math.asin(max(-1.0, min(1.0, -self.up[2]))))

    def draw_depth(self, depth):
        m = depth.astype(np.float32) * 1e-3
        near, far = DEPTH_RANGE
        scaled = np.clip((far - m) / (far - near) * 255, 0, 255).astype(np.uint8)   # near = red
        img = cv2.applyColorMap(scaled, cv2.COLORMAP_TURBO)
        img[depth == 0] = (20, 20, 20)
        return cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 80])[1].tobytes()

    def draw_map(self, ground):
        """Top-down, the robot at the bottom facing up: what the depth camera sees ahead."""
        w, h = MAP_W, MAP_H
        px = (h - 70) / MAP_AHEAD                  # px per m
        ox, oy = w // 2, h - 45                    # the robot's centre
        font, aa = cv2.FONT_HERSHEY_SIMPLEX, cv2.LINE_AA
        img = np.full((h, w, 3), (18, 14, 11), np.uint8)
        for r in range(1, int(MAP_AHEAD) + 1):     # rings every metre
            cv2.circle(img, (ox, oy), int(r * px), (90, 80, 70), 1, aa)
            cv2.putText(img, f"{r} m", (ox + 6, oy - int(r * px) - 6), font, 0.5, (190, 175, 160), 1, aa)
        ahead, left, height = ground[:, 0], ground[:, 1], ground[:, 2]
        u = np.round(ox - left * px).astype(int)
        v = np.round(oy - ahead * px).astype(int)
        inside = (u >= 1) & (u < w - 1) & (v >= 1) & (v < h - 1)
        floor = inside & (height < OBSTACLE)
        img[v[floor], u[floor]] = (120, 120, 120)
        thing = inside & (height >= OBSTACLE) & (height < 2.0)
        if thing.any():
            order = np.argsort(height[thing])      # tall drawn last, on top
            t = 0.3 + 0.7 * np.clip(height[thing][order] / 1.5, 0, 1)
            colours = cv2.applyColorMap((t * 255).astype(np.uint8).reshape(-1, 1),
                                        cv2.COLORMAP_VIRIDIS).reshape(-1, 3)
            for du in (-1, 0, 1):                  # 3x3 dots
                for dv in (-1, 0, 1):
                    img[v[thing][order] + dv, u[thing][order] + du] = colours
        # the robot, seen from above, and its front
        cv2.ellipse(img, (ox, oy), (int(0.22 * px), int(0.12 * px)), 0, 0, 360, (225, 225, 225), -1, aa)
        cv2.arrowedLine(img, (ox, oy - int(0.12 * px)), (ox, oy - int(0.45 * px)), (40, 140, 255), 2,
                        aa, tipLength=0.35)
        cv2.putText(img, "FRONT", (ox + 10, oy - int(0.36 * px)), font, 0.45, (40, 140, 255), 1, aa)
        cv2.putText(img, "robot", (ox - 22, oy + int(0.12 * px) + 18), font, 0.45, (225, 225, 225), 1, aa)
        return cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 85])[1].tobytes()

    def run(self):
        last_fit, last_count, last_t = 0.0, 0, time.monotonic()
        while True:
            time.sleep(0.1)
            depth = self.stream.latest()
            if depth is None:
                continue
            now = time.monotonic()
            pts = self.points(depth)
            if now - last_fit > 1.0:
                self.fit_floor(pts)
                last_fit = now
            if now - self.wanted > 3.0:          # nobody looking: keep only the floor fit
                continue
            jpeg = self.draw_depth(depth)
            ground = self.level(pts)
            map_jpeg = self.draw_map(ground)
            centre = depth[depth.shape[0] // 3:2 * depth.shape[0] // 3,
                           depth.shape[1] // 3:2 * depth.shape[1] // 3]
            valid = centre[centre > 0]
            things = ground[(ground[:, 2] > OBSTACLE) & (ground[:, 2] < 2.0)]
            things = things[np.hypot(things[:, 0], things[:, 1]) > SELF_RADIUS]   # not the robot's own arms
            with self.lock:
                self.depth_jpeg, self.map_jpeg = jpeg, map_jpeg
                if now - last_t >= 1.0:
                    self.stats["depth_fps"] = round((self.stream.count - last_count) / (now - last_t), 1)
                    last_count, last_t = self.stream.count, now
                self.stats.update(
                    valid=round(100 * float(np.count_nonzero(depth)) / depth.size),
                    centre=round(float(np.median(valid)) * 1e-3, 2) if valid.size else None,
                    nearest_thing=round(float(np.min(np.hypot(things[:, 0], things[:, 1]))), 2)
                    if len(things) else None,
                    floor_ok=self.floor_ok, height=round(self.height, 3),
                    pitch=round(self.pitch(), 1), inliers=self.inliers)

    def measure(self, x, y):
        """The point under a click (x, y in 0..1 of the depth image)."""
        depth = self.stream.latest()
        if depth is None:
            return {"ok": False}
        h, w = depth.shape
        u, v = int(x * (w - 1)), int(y * (h - 1))
        patch = depth[max(0, v - 2):v + 3, max(0, u - 2):u + 3]
        patch = patch[patch > 0]
        if patch.size == 0:
            return {"ok": False, "why": "no depth there"}
        z = float(np.median(patch)) * 1e-3
        p = np.array([[(u - self.cx) * z / self.fx, (v - self.cy) * z / self.fy, z]])
        ahead, left, height = self.level(p)[0]
        return {"ok": True, "range": round(float(np.linalg.norm(p)), 3), "ahead": round(float(ahead), 2),
                "left": round(float(left), 2), "height": round(float(height), 2)}


PAGE = r"""<!doctype html>
<html><head><meta charset="utf-8"><title>G1 Camera View</title>
<meta name="viewport" content="width=device-width, initial-scale=1">
<style>
  :root{--bg:#0b0e12;--panel:#11151b;--text:#d7dee6;--muted:#8a96a3;--accent:#ff8c1a;--line:#2a323c}
  html,body{margin:0;background:var(--bg);color:var(--text);font:13px/1.45 system-ui,sans-serif}
  header{padding:10px 16px;border-bottom:1px solid var(--line);display:flex;flex-wrap:wrap;gap:6px 18px;align-items:baseline}
  h1{font-size:15px;margin:0}
  .muted{color:var(--muted)} .warn{color:#ffb066}
  main{display:grid;grid-template-columns:repeat(auto-fit,minmax(320px,1fr));gap:12px;padding:12px 16px}
  figure{margin:0;background:var(--panel);border:1px solid var(--line);border-radius:8px;overflow:hidden}
  figcaption{padding:6px 10px;display:flex;justify-content:space-between;gap:8px;flex-wrap:wrap}
  img{display:block;width:100%;height:auto;background:#000}
  #depth{cursor:crosshair}
  .legend{height:8px;margin:0 10px 8px;border-radius:3px;background:linear-gradient(90deg,#7a0403,#f05b12,#e6c42f,#3be08a,#2b8ad8,#30123b)}
  .lab{display:flex;justify-content:space-between;margin:0 10px 8px;font-size:11px;color:var(--muted)}
  #pick{color:var(--accent)}
  .keys{display:grid;gap:4px;padding:8px 10px 2px;font-size:12px}
  .keys i{display:inline-block;width:12px;height:12px;border-radius:2px;vertical-align:-2px;margin-right:7px}
  .keys i.hgrad{width:30px;background:linear-gradient(90deg,#31688e,#21918c,#35b779,#90d743,#fde725)}
  .note{padding:2px 10px 8px;font-size:12px}
  label{cursor:pointer}
</style></head>
<body>
<header><h1>G1 head camera</h1><span id="stats" class="muted">connecting…</span>
  <label><input type="checkbox" id="pause"> Pause</label></header>
<main>
  <figure id="colourbox"><figcaption><b>Colour</b><span id="colourtxt" class="muted"></span></figcaption>
    <img id="colour" alt="colour image"></figure>
  <figure><figcaption><b>Depth</b><span id="pick" class="muted">click to measure</span></figcaption>
    <img id="depth" alt="depth image"><div class="legend"></div>
    <div class="lab"><span>0.2 m</span><span>2 m</span><span>4 m+</span></div></figure>
  <figure><figcaption><b>In front of the robot, seen from above</b><span class="muted">from the depth camera</span></figcaption>
    <img id="map" alt="top-down map of what the camera sees in front of the robot">
    <div class="keys">
      <span><i style="background:#787878"></i>floor the camera sees</span>
      <span><i class="hgrad"></i>something sticking up; its colour is its height (0.1 m → 1.5 m)</span>
      <span><i style="background:#0b0e12;outline:1px solid #2a323c"></i>dark: out of the camera's view</span>
    </div>
    <div id="floortxt" class="muted note"></div></figure>
</main>
<script>
const $ = id => document.getElementById(id);
function poll(id, url, period) {
  const img = $(id);
  const next = () => setTimeout(load, period);
  function load() {
    if ($('pause').checked) return next();
    const fresh = new Image();
    fresh.onload = () => { img.src = fresh.src; next(); };
    fresh.onerror = next;
    fresh.src = url + '?t=' + Date.now();
  }
  load();
}
async function stats() {
  try {
    const s = await (await fetch('stats')).json();
    const parts = [`depth ${s.depth_fps ?? '–'} fps`, `${s.valid ?? '–'}% pixels with depth`];
    if (s.centre != null) parts.push(`centre ${s.centre} m`);
    if (s.nearest_thing != null) parts.push(`nearest thing in view ${s.nearest_thing} m away`);
    $('stats').textContent = parts.join(' · ');
    $('floortxt').innerHTML = s.floor_ok
      ? `Camera measured from the floor: ${s.height.toFixed(2)} m up, looking ${s.pitch.toFixed(1)}&deg; down.`
      : '<span class="warn">floor not found yet: nominal pose</span>';
    $('colourtxt').textContent = s.colour || '';
    $('colourbox').hidden = !s.colour;
  } catch (e) { $('stats').textContent = 'lost connection to the viewer'; }
  setTimeout(stats, 1000);
}
$('depth').addEventListener('click', async e => {
  const r = e.target.getBoundingClientRect();
  const q = `x=${(e.clientX - r.left) / r.width}&y=${(e.clientY - r.top) / r.height}`;
  const m = await (await fetch('measure?' + q)).json();
  $('pick').textContent = m.ok
    ? `${m.range.toFixed(2)} m away · ${m.ahead.toFixed(2)} m ahead, ${Math.abs(m.left).toFixed(2)} m ${m.left >= 0 ? 'left' : 'right'}, ${m.height.toFixed(2)} m above the floor`
    : (m.why || 'nothing there');
});
poll('depth', 'depth.jpg', 100); poll('map', 'map.jpg', 200);
poll('colour', 'colour.jpg', 150); stats();
</script></body></html>
"""


def make_handler(depth, colour):
    class Handler(BaseHTTPRequestHandler):
        def send(self, body, ctype):
            if body is None:
                self.send_error(503, "not ready")
                return
            try:
                self.send_response(200)
                self.send_header("Content-Type", ctype)
                self.send_header("Content-Length", str(len(body)))
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                self.wfile.write(body)
            except (BrokenPipeError, ConnectionResetError):
                pass   # the browser gave up on this request

        def do_GET(self):
            url = urlparse(self.path)
            if url.path in ("/", "/index.html"):
                self.send(PAGE.encode(), "text/html; charset=utf-8")
            elif url.path == "/depth.jpg":
                depth.wanted = time.monotonic()
                self.send(depth.depth_jpeg, "image/jpeg")
            elif url.path == "/map.jpg":
                self.send(depth.map_jpeg, "image/jpeg")
            elif url.path == "/colour.jpg":
                self.send(colour.jpeg if colour else None, "image/jpeg")
            elif url.path == "/stats":
                with depth.lock:
                    s = dict(depth.stats)
                s["depth_status"] = depth.stream.status
                s["colour"] = colour.status if colour else None
                self.send(json.dumps(s).encode(), "application/json")
            elif url.path == "/measure":
                q = parse_qs(url.query)
                m = depth.measure(float(q["x"][0]), float(q["y"][0]))
                self.send(json.dumps(m).encode(), "application/json")
            else:
                self.send_error(404)

        def log_message(self, *args):
            pass

    return Handler


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", type=int, default=8081, help="HTTP port")
    ap.add_argument("--size", default="848x480", help="depth resolution, e.g. 640x480 or 1280x720")
    ap.add_argument("--fps", type=int, default=15, help="depth frame rate: 6, 15, 30, 60 or 90")
    ap.add_argument("--no-colour", action="store_true", help="don't ask videohub for colour")
    ap.add_argument("--lean", type=float, default=6.0,
                    help="deg the torso leans forward, for the pose until the floor is found")
    ap.add_argument("--interface", default="eth0", help="PC2's port on the robot network")
    args = ap.parse_args()
    os.nice(10)   # below HARP's harness

    w, h = (int(v) for v in args.size.split("x"))
    stream = V4L2Stream("/dev/video0", "Z16 ", w, h, args.fps, "<u2")
    stream.start()
    colour = None
    if not args.no_colour:
        try:
            colour = Colour(args.interface, 0.1)
            colour.start()
        except Exception as e:
            print(f"No colour ({type(e).__name__}: {e}); depth only.")
    time.sleep(0.5)
    depth = Depth(stream, args.lean)
    threading.Thread(target=depth.run, daemon=True).start()
    server = ThreadingHTTPServer(("0.0.0.0", args.port), make_handler(depth, colour))
    print(f"Serving on http://0.0.0.0:{args.port}  (Ctrl+C to stop)")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
        stream.close()
        if colour:
            colour.running = False


if __name__ == "__main__":
    main()
