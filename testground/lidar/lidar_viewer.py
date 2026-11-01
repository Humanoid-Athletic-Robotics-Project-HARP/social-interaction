#!/usr/bin/env python3
"""Browser-based live viewer for the G1's Livox MID-360.

Reads point packets straight off UDP (the lidar already pushes to this host,
see ~/Workspace/test/mid360_config.json) and serves a three.js page that shows
the cloud around a mock G1, levelled onto the floor.

    python3 lidar_viewer.py            # then open http://<robot-wlan-ip>:8080

Frames
  The MID-360 sits upside down in the G1 head, so points are rotated 180 deg
  about x (x forward, y left, z up) before they are served. The floor plane is
  fitted continuously and the page uses it to level the cloud and to place the
  robot model, so the view stays right whether the robot stands or hangs.

It also shows where the lidar can't see: the directions the robot's own head
blocks (counted from the live returns, so it is what this lidar really sees) and
the floor around the feet below its field of view.

Only one process can receive the lidar stream, so stop livox_ros_driver2 or
any other SDK2 program first.
"""
import argparse
import json
import socket
import threading
import time
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import numpy as np

HEADER_SIZE = 36  # SDK2 point packet header
# data_type 1: Cartesian, high precision (mm)
POINT_DTYPE = np.dtype([("x", "<i4"), ("y", "<i4"), ("z", "<i4"), ("refl", "u1"), ("tag", "u1")])
# Sensor is mounted upside down: rotate 180 deg about x
FLIP = np.array([1.0, -1.0, -1.0], np.float32)
# URDF: pelvis 0.793 above feet + waist 0.054 + mid360 0.406 above torso
LIDAR_ABOVE_FEET = 1.253
# Where the lidar can't see: returns per 5 deg of azimuth, past the robot's own body
SECTORS = 72
SELF_RANGE = 0.6      # m: nearer returns hit the robot, they're no view past it
BLOCKED, SPARSE = 0.15, 0.4   # a sector's returns against the median sector's


class PointBuffer:
    """Keeps the most recent `window` seconds of points."""

    def __init__(self, window):
        self.window = window
        self.chunks = deque()
        self.lock = threading.Lock()
        self.received = 0

    def add(self, pts):
        now = time.monotonic()
        with self.lock:
            self.chunks.append((now, pts))
            self.received += len(pts)
            while self.chunks and now - self.chunks[0][0] > self.window:
                self.chunks.popleft()

    def snapshot(self):
        """Nx4 float32: x, y, z (m, robot-aligned, z up) and reflectivity."""
        with self.lock:
            if not self.chunks:
                return np.zeros((0, 4), np.float32)
            pts = np.concatenate([c for _, c in self.chunks])
        # Drop zero returns (no echo)
        pts = pts[(pts["x"] != 0) | (pts["y"] != 0) | (pts["z"] != 0)]
        out = np.empty((len(pts), 4), np.float32)
        out[:, 0] = pts["x"]
        out[:, 1] = pts["y"]
        out[:, 2] = pts["z"]
        out[:, :3] *= FLIP / 1000.0
        out[:, 3] = pts["refl"]
        return out


def tidy(flags, fill, shortest):
    """Fill gaps of up to `fill` sectors, then drop runs shorter than `shortest` (circular),
    so a doorway or a far-off gap doesn't show up as blocked."""
    flags = flags.copy()
    n = len(flags)
    for value, longest in ((False, fill), (True, shortest - 1)):
        if flags.all() or not flags.any():
            break
        start = int(np.argmax(flags != flags[0]))   # begin at a boundary
        k = 0
        while k < n:
            first = flags[(start + k) % n]
            j = k
            while j < n and flags[(start + j) % n] == first:
                j += 1
            if first == value and j - k <= longest:
                for m in range(k, j):
                    flags[(start + m) % n] = not value
            k = j
    return flags


def runs(flags):
    """Runs of True sectors as [start, end] deg, joined across +-180."""
    n, width = len(flags), 360.0 / len(flags)
    if flags.all():
        return [[-180.0, 180.0]]
    first = int(np.argmin(flags))          # start the walk on a False sector
    out, start = [], None
    for k in range(n + 1):
        i = (first + k) % n
        if flags[i] and start is None:
            start = k
        elif not flags[i] and start is not None:
            a0 = -180.0 + ((first + start) % n) * width
            out.append([a0, a0 + (k - start) * width])
            start = None
    return out


class FloorEstimator:
    """Fits the largest near-horizontal plane below the sensor (RANSAC), and counts
    where the lidar sees past the robot."""

    def __init__(self, buf):
        self.buf = buf
        self.lock = threading.Lock()
        self.state = {"normal": [0.0, 0.0, 1.0], "height": LIDAR_ABOVE_FEET, "ok": False,
                      "inliers": 0, "pps": 0, "blocked": [], "sparse": [], "floor_from": None}
        self.rng = np.random.default_rng()
        self.sector_counts = None

    def fit(self, pts):
        if len(pts) < 500:
            return None
        if len(pts) > 8000:
            pts = pts[self.rng.choice(len(pts), 8000, replace=False)]
        best_n, best_d, best_cnt = None, None, 0
        for _ in range(200):
            p = pts[self.rng.choice(len(pts), 3, replace=False)]
            n = np.cross(p[1] - p[0], p[2] - p[0])
            norm = np.linalg.norm(n)
            if norm < 1e-9:
                continue
            n /= norm
            if n[2] < 0:
                n = -n
            if n[2] < 0.85:  # more than ~30 deg from level: a wall, not the floor
                continue
            d = -(n @ p[0])  # plane n.p + d = 0, d = sensor height above it
            if d < 0.3:
                continue
            cnt = int((np.abs(pts @ n + d) < 0.03).sum())
            if cnt > best_cnt:
                best_n, best_d, best_cnt = n, d, cnt
        if best_cnt < 300:
            return None
        inl = pts[np.abs(pts @ best_n + best_d) < 0.03]
        c = inl.mean(0)
        n = np.linalg.svd(inl - c, full_matrices=False)[2][2]
        if n[2] < 0:
            n = -n
        # how near the robot the floor first shows up (along the floor, from below the lidar)
        along = np.linalg.norm(inl - np.outer(inl @ n, n), axis=1)
        return n, float(-(n @ c)), len(inl), float(np.percentile(along, 2))

    def coverage(self, pts):
        """Blocked and sparse azimuth sectors, from returns past the robot's body."""
        far = pts[np.linalg.norm(pts, axis=1) > SELF_RANGE]
        counts = np.histogram(np.degrees(np.arctan2(far[:, 1], far[:, 0])), SECTORS,
                              (-180, 180))[0].astype(float)
        if self.sector_counts is None:
            self.sector_counts = counts
        else:                      # about ten seconds' memory, so a passer-by doesn't flicker it
            self.sector_counts = 0.9 * self.sector_counts + 0.1 * counts
        typical = np.median(self.sector_counts)
        if typical <= 0:
            return [], []
        share = self.sector_counts / typical
        blocked = tidy(share < BLOCKED, 2, 3)            # 10 deg gaps filled, 15 deg at least
        sparse = tidy((share < SPARSE) & ~blocked, 1, 2) & ~blocked
        return runs(blocked), runs(sparse)

    def run(self):
        last_count, last_t = self.buf.received, time.monotonic()
        while True:
            time.sleep(1.0)
            now = time.monotonic()
            pps = (self.buf.received - last_count) / (now - last_t)
            last_count, last_t = self.buf.received, now
            pts = self.buf.snapshot()[:, :3].astype(np.float64)
            res = self.fit(pts)
            blocked, sparse = self.coverage(pts) if len(pts) else ([], [])
            with self.lock:
                s = self.state
                s.update(pps=int(pps), blocked=blocked, sparse=sparse)
                if res is None:
                    continue
                n, d, cnt, near = res
                if s["ok"]:  # smooth out frame-to-frame jitter
                    n = 0.7 * np.array(s["normal"]) + 0.3 * n
                    n /= np.linalg.norm(n)
                    d = 0.7 * s["height"] + 0.3 * d
                    near = 0.7 * (s["floor_from"] or near) + 0.3 * near
                s.update(normal=[float(v) for v in n], height=float(d), ok=True, inliers=cnt,
                         floor_from=near)

    def json(self):
        with self.lock:
            return json.dumps(self.state).encode()


def receive(buf, port):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 << 20)
    sock.bind(("0.0.0.0", port))
    while True:
        data, _ = sock.recvfrom(65535)
        if len(data) < HEADER_SIZE or data[10] != 1:
            continue
        dot_num = int.from_bytes(data[5:7], "little")
        buf.add(np.frombuffer(data, POINT_DTYPE, count=dot_num, offset=HEADER_SIZE).copy())


PAGE = r"""<!doctype html>
<html><head><meta charset="utf-8"><title>G1 LiDAR View</title>
<meta name="viewport" content="width=device-width, initial-scale=1">
<style>
  :root{--panel:#11151bd9;--text:#d7dee6;--muted:#8a96a3;--accent:#ff8c1a;--line:#2a323c}
  html,body{margin:0;height:100%;background:#0b0e12;color:var(--text);font:13px/1.45 system-ui,sans-serif;overflow:hidden}
  #hud{position:fixed;top:10px;left:10px;width:290px;max-width:calc(100vw - 32px);max-height:calc(100vh - 20px);overflow:auto;
       background:var(--panel);backdrop-filter:blur(6px);border:1px solid var(--line);border-radius:8px;padding:10px 12px;box-sizing:border-box}
  h1{font-size:14px;margin:0 0 4px}
  .row{display:flex;align-items:center;gap:6px;flex-wrap:wrap;margin:5px 0}
  .muted{color:var(--muted)}
  .warn{color:#ffb066}
  .sec{border-top:1px solid var(--line);margin-top:8px;padding-top:6px}
  button{font:inherit;color:var(--text);background:#1d242d;border:1px solid var(--line);border-radius:5px;padding:3px 8px;cursor:pointer}
  button:hover{border-color:var(--accent)}
  select,input{font:inherit}
  label.row{cursor:pointer;margin:3px 0}
  #legend canvas{width:100%;height:10px;border-radius:3px;display:block}
  #legend .lab{display:flex;justify-content:space-between;font-size:11px;color:var(--muted)}
  #pick{position:fixed;bottom:14px;left:50%;transform:translateX(-50%);max-width:calc(100vw - 32px);background:var(--panel);
        border:1px solid var(--accent);border-radius:8px;padding:6px 12px;display:none;text-align:center}
  @media (max-width:600px){#hud{width:auto;right:10px;max-height:45vh}}
</style>
<script type="importmap">
{"imports":{"three":"https://cdn.jsdelivr.net/npm/three@0.160.0/build/three.module.js",
"three/addons/":"https://cdn.jsdelivr.net/npm/three@0.160.0/examples/jsm/"}}
</script></head>
<body>
<div id="hud">
  <h1>G1 LiDAR view</h1>
  <div id="stats" class="muted">connecting...</div>
  <div id="calibtxt"></div>
  <div class="sec row"><span class="muted">View</span>
    <button data-view="behind">Behind</button><button data-view="top">Top</button>
    <button data-view="front">Front</button><button data-view="side">Side</button>
    <button data-view="pov">Robot's eyes</button></div>
  <div class="sec">
    <div class="row"><span class="muted">Color by</span>
      <select id="mode"><option value="h">height above floor</option><option value="d">distance from robot</option>
      <option value="r">reflectivity</option></select></div>
    <div id="legend"><canvas width="256" height="1"></canvas>
      <div class="lab"><span id="l0"></span><span id="l1"></span><span id="l2"></span></div></div>
    <div class="row"><span class="muted">Point size</span><input id="size" type="range" min="1" max="6" step="0.5"></div>
  </div>
  <div class="sec">
    <label class="row"><input type="checkbox" id="hidefloor"> Hide floor points (people &amp; objects stand out)</label>
    <label class="row"><input type="checkbox" id="hideself"> Hide hits on the robot itself</label>
    <label class="row"><input type="checkbox" id="showrobot"> Show robot model</label>
    <label class="row"><input type="checkbox" id="showguides"> Show distance rings &amp; labels</label>
    <label class="row"><input type="checkbox" id="showblind"> Show where the lidar can't see</label>
    <label class="row"><input type="checkbox" id="flip"> Swap front/back</label>
    <label class="row"><input type="checkbox" id="pause"> Pause</label>
  </div>
  <div class="sec muted" id="blindkey"><span style="color:#ff6b6b">Red</span>: directions the robot's own head blocks,
    measured from what the lidar returns right now. <span style="color:#ffa64d">Orange</span>: it sees only a little there.
    <span style="color:#ffd060">Yellow disk</span>: floor too close to the feet for the lidar to see.</div>
  <div id="blindtxt"></div>
  <div class="sec muted">Orange arrow = the robot's front. To check it, stand in front of the robot:
    you should show up near the arrow. If you show up behind the robot, tick "Swap front/back".<br>
    Click a point to measure it. Drag to orbit, right-drag to pan, scroll to zoom.</div>
</div>
<div id="pick"></div>
<script type="module">
import * as THREE from 'three';
import {OrbitControls} from 'three/addons/controls/OrbitControls.js';

const LIDAR_ABOVE_FEET = 1.253;   // URDF: pelvis 0.793 + waist 0.054 + mid360 0.406
const MAX_PTS = 400000;
const $ = id => document.getElementById(id);
const V = (x, y, z) => new THREE.Vector3(x, y, z);
const store = {
  get(k, d) { try { const v = localStorage.getItem('g1lidar.' + k); return v === null ? d : JSON.parse(v); } catch (e) { return d; } },
  set(k, v) { try { localStorage.setItem('g1lidar.' + k, JSON.stringify(v)); } catch (e) {} },
};

// ---------- scene ----------
const renderer = new THREE.WebGLRenderer({antialias: true});
renderer.setPixelRatio(devicePixelRatio);
document.body.appendChild(renderer.domElement);
const scene = new THREE.Scene();
scene.background = new THREE.Color(0x0b0e12);
const camera = new THREE.PerspectiveCamera(60, 1, 0.03, 200);
camera.up.set(0, 0, 1);
const controls = new OrbitControls(camera, renderer.domElement);
controls.enableDamping = true;
scene.add(new THREE.HemisphereLight(0xdde6ff, 0x202020, 1.6));
const sun = new THREE.DirectionalLight(0xffffff, 1.6); sun.position.set(3, 2, 5); scene.add(sun);
function resize() { renderer.setSize(innerWidth, innerHeight); camera.aspect = innerWidth / innerHeight; camera.updateProjectionMatrix(); }
addEventListener('resize', resize); resize();

function label(text, {h = 0.3, color = '#ffffff', bg = 'rgba(10,14,18,0.7)'} = {}) {
  const c = document.createElement('canvas'), ctx = c.getContext('2d'), fs = 56, font = `600 ${fs}px system-ui,sans-serif`;
  ctx.font = font;
  c.width = Math.ceil(ctx.measureText(text).width) + 36; c.height = fs + 26;
  ctx.font = font;
  if (bg) { ctx.fillStyle = bg; ctx.beginPath(); ctx.roundRect ? ctx.roundRect(0, 0, c.width, c.height, 16) : ctx.rect(0, 0, c.width, c.height); ctx.fill(); }
  ctx.fillStyle = color; ctx.textBaseline = 'middle'; ctx.fillText(text, 18, c.height / 2 + 2);
  const tex = new THREE.CanvasTexture(c); tex.colorSpace = THREE.SRGBColorSpace;
  const s = new THREE.Sprite(new THREE.SpriteMaterial({map: tex, depthTest: false, transparent: true}));
  s.scale.set(h * c.width / c.height, h, 1); s.renderOrder = 10;
  return s;
}
function setLabel(sprite, text, opts) {   // swap a label's texture in place
  const fresh = label(text, opts);
  sprite.material.map.dispose(); sprite.material.dispose();
  sprite.material = fresh.material; sprite.scale.copy(fresh.scale);
}
function line(points, color, dashed = false) {
  const g = new THREE.BufferGeometry().setFromPoints(points);
  const m = dashed ? new THREE.LineDashedMaterial({color, dashSize: 0.05, gapSize: 0.04, depthTest: false})
                   : new THREE.LineBasicMaterial({color, transparent: true, opacity: 0.75});
  const l = new THREE.Line(g, m); if (dashed) { l.computeLineDistances(); l.renderOrder = 9; }
  return l;
}

// ---------- floor & guides (world frame: floor at z=0, x = robot front) ----------
const floor = new THREE.Mesh(new THREE.CircleGeometry(15, 72),
  new THREE.MeshBasicMaterial({color: 0x131920, transparent: true, opacity: 0.7, depthWrite: false}));
scene.add(floor);
const grid = new THREE.GridHelper(30, 30, 0x2c3644, 0x1b222a); grid.rotation.x = Math.PI / 2; grid.position.z = 0.002; scene.add(grid);
const guides = new THREE.Group(); scene.add(guides);
for (const r of [1, 2, 3, 5, 8]) {
  const pts = []; for (let i = 0; i <= 128; i++) { const a = i / 128 * Math.PI * 2; pts.push(V(r * Math.cos(a), r * Math.sin(a), 0.004)); }
  guides.add(line(pts, 0x3f7aa0));
  const l = label(`${r} m`, {h: 0.2, color: '#8cc3e6'}); l.position.set(r * Math.cos(-0.6), r * Math.sin(-0.6), 0.1); guides.add(l);
}
for (const [t, x, y, col] of [['FRONT', 6.5, 0, '#ffb066'], ['BACK', -6.5, 0, '#c9d2dc'], ['LEFT', 0, 6.5, '#c9d2dc'], ['RIGHT', 0, -6.5, '#c9d2dc']]) {
  const l = label(t, {h: 0.45, color: col}); l.position.set(x, y, 0.3); guides.add(l);
}
guides.add(new THREE.ArrowHelper(V(1, 0, 0), V(0.3, 0, 0.01), 1.3, 0xff8c1a, 0.28, 0.16));

// ---------- robot (rig = lidar pose in world; robot body frame, lidar at origin) ----------
const rig = new THREE.Group(); scene.add(rig);
const robot = new THREE.Group(); rig.add(robot);
(function buildRobot() {
  const g = new THREE.Group(); robot.add(g);
  const shell = new THREE.MeshStandardMaterial({color: 0xc9ced6, roughness: 0.6, metalness: 0.1});
  const dark = new THREE.MeshStandardMaterial({color: 0x2b3038, roughness: 0.7});
  const visor = new THREE.MeshStandardMaterial({color: 0x0a0c10, emissive: 0x2a6bff, emissiveIntensity: 0.5, roughness: 0.3});
  const orange = new THREE.MeshStandardMaterial({color: 0xff8c1a, emissive: 0xff8c1a, emissiveIntensity: 0.6});
  const add = (geo, m, p) => { const o = new THREE.Mesh(geo, m); o.position.copy(p); g.add(o); return o; };
  const box = (sx, sy, sz, p, m = shell) => add(new THREE.BoxGeometry(sx, sy, sz), m, p);
  const ball = (r, p, m = dark) => add(new THREE.SphereGeometry(r, 20, 14), m, p);
  const limb = (a, b, r, m = shell) => {
    const d = b.clone().sub(a), o = add(new THREE.CylinderGeometry(r, r, d.length(), 16), m, a.clone().addScaledVector(d, 0.5));
    o.quaternion.setFromUnitVectors(V(0, 1, 0), d.normalize()); return o;
  };
  // Standing coordinates in metres, feet at z=0, roughly G1-sized
  for (const s of [1, -1]) {
    const y = 0.12 * s, ya = 0.21 * s;
    box(0.20, 0.075, 0.04, V(0.035, y, 0.02), dark);           // foot
    ball(0.035, V(0, y, 0.07));                                 // ankle
    limb(V(0, y, 0.07), V(0.01, y, 0.43), 0.04);                // shin
    ball(0.05, V(0.01, y, 0.43));                               // knee
    limb(V(0.01, y, 0.43), V(0, y, 0.72), 0.055);               // thigh
    ball(0.055, V(0, y, 0.73));                                 // hip
    ball(0.05, V(0, 0.175 * s, 1.10));                          // shoulder
    limb(V(0, 0.19 * s, 1.10), V(0.02, ya, 0.87), 0.038);       // upper arm
    ball(0.04, V(0.02, ya, 0.87));                              // elbow
    limb(V(0.02, ya, 0.87), V(0.07, ya, 0.67), 0.033);          // forearm
    ball(0.035, V(0.075, ya, 0.64), shell);                     // hand
  }
  box(0.14, 0.24, 0.10, V(0, 0, 0.78));                         // pelvis
  limb(V(0, 0, 0.82), V(0, 0, 0.87), 0.08, dark);               // waist
  box(0.16, 0.28, 0.28, V(0, 0, 1.0));                          // torso
  box(0.012, 0.10, 0.10, V(0.083, 0, 1.03), orange);            // chest stripe marks the front
  limb(V(0, 0, 1.13), V(0, 0, 1.16), 0.035, dark);              // neck
  ball(1, V(0, 0, 1.185), shell).scale.set(0.08, 0.075, 0.075); // head
  box(0.02, 0.11, 0.05, V(0.075, 0, 1.185), visor);             // face / camera
  limb(V(0, 0, 1.228), V(0, 0, 1.278), 0.034, orange);          // MID-360
  g.position.z = -LIDAR_ABOVE_FEET;
  const l = label('LiDAR', {h: 0.07, color: '#ffb066'}); l.position.set(0, 0, 0.09); robot.add(l);
})();
const gapLine = line([V(0, 0, 0), V(0, 0, 1)], 0xffb066, true); scene.add(gapLine);
const gapLabel = label('-', {h: 0.09, color: '#ffb066'}); scene.add(gapLabel);
let gapText = '';

// ---------- where the lidar can't see (measured by the server, /calib) ----------
const blind = new THREE.Group(); scene.add(blind);
let blindKey = '';
function clearGroup(g) {
  for (const o of [...g.children]) {
    g.remove(o);
    if (o.geometry) o.geometry.dispose();
    if (o.material) { if (o.material.map) o.material.map.dispose(); o.material.dispose(); }
  }
}
function fan(a0, a1, r0, r1, color, opacity) {   // a floor sector, degrees from the front, + to the left
  const g = new THREE.RingGeometry(r0, r1, Math.max(4, Math.ceil((a1 - a0) / 3)), 1,
                                   a0 * Math.PI / 180, (a1 - a0) * Math.PI / 180);
  return new THREE.Mesh(g, new THREE.MeshBasicMaterial({color, transparent: true, opacity, depthWrite: false,
                                                        side: THREE.DoubleSide}));
}
function where(mid) {                            // a direction in words
  const a = ((mid + 540) % 360) - 180, b = Math.abs(a);
  if (b < 25) return 'ahead';
  if (b > 155) return 'behind';
  return `${b < 90 ? 'ahead and ' : b > 90 ? 'behind and ' : ''}to the ${a > 0 ? 'left' : 'right'}`;
}
function drawBlind() {
  const c = rig.localToWorld(V(0, 0, 0));
  blind.position.set(c.x, c.y, 0.006);
  const show = $('showblind').checked;
  $('blindkey').hidden = !show;
  const deg = yaw * 180 / Math.PI;
  const blocked = (calib.blocked || []).map(([a0, a1]) => [a0 + deg, a1 + deg]);
  const key = JSON.stringify([show, blocked, calib.sparse, deg, calib.floor_from && calib.floor_from.toFixed(2)]);
  if (key === blindKey) return;
  blindKey = key; clearGroup(blind);
  $('blindtxt').innerHTML = !show ? '' : blocked.length
    ? 'Can\'t see: ' + blocked.map(([a0, a1]) => `<b>${Math.round(a1 - a0)}&deg; ${where((a0 + a1) / 2)}</b>`).join(', ') + '.'
    : 'No blocked directions right now.';
  if (!show) return;
  for (const [a0, a1] of calib.sparse || []) blind.add(fan(a0 + deg, a1 + deg, 0.3, 8, 0xff9a3c, 0.13));
  for (const [a0, a1] of blocked) {
    blind.add(fan(a0, a1, 0.3, 8, 0xff4040, 0.3));
    if (a1 - a0 < 30) continue;                   // label only the big ones
    const mid = (a0 + a1) / 2 * Math.PI / 180;
    const l = label("lidar can't see here", {h: 0.22, color: '#ff8080'});
    l.position.set(3.4 * Math.cos(mid), 3.4 * Math.sin(mid), 0.2); blind.add(l);
  }
  if (calib.floor_from) {
    const r = calib.floor_from;
    blind.add(new THREE.Mesh(new THREE.CircleGeometry(r, 64), new THREE.MeshBasicMaterial(
      {color: 0xffd060, transparent: true, opacity: 0.14, depthWrite: false})));
    const pts = []; for (let i = 0; i <= 96; i++) { const a = i / 96 * Math.PI * 2; pts.push(V(r * Math.cos(a), r * Math.sin(a), 0.002)); }
    blind.add(line(pts, 0xffd060));
    const l = label(`floor unseen within ${r.toFixed(1)} m`, {h: 0.12, color: '#ffd060'});
    l.position.set(r * Math.cos(-2.3), r * Math.sin(-2.3), 0.12); blind.add(l);
  }
}

// ---------- point cloud ----------
const posArr = new Float32Array(MAX_PTS * 3), colArr = new Float32Array(MAX_PTS * 3);
const geom = new THREE.BufferGeometry();
geom.setAttribute('position', new THREE.BufferAttribute(posArr, 3).setUsage(THREE.DynamicDrawUsage));
geom.setAttribute('color', new THREE.BufferAttribute(colArr, 3).setUsage(THREE.DynamicDrawUsage));
geom.setDrawRange(0, 0);
geom.boundingSphere = new THREE.Sphere(V(0, 0, 0), 200);
const mat = new THREE.PointsMaterial({size: 2, sizeAttenuation: false, vertexColors: true});
const cloud = new THREE.Points(geom, mat); cloud.frustumCulled = false; scene.add(cloud);

function turbo(t) {   // Google Turbo colormap (polynomial fit), sRGB
  const r = 0.13572138 + t * (4.61539260 + t * (-42.66032258 + t * (132.13108234 + t * (-152.94239396 + t * 59.28637943))));
  const g = 0.09140261 + t * (2.19418839 + t * (4.84296658 + t * (-14.18503333 + t * (4.27729857 + t * 2.82956604))));
  const b = 0.10667330 + t * (12.64194608 + t * (-60.58204836 + t * (110.36276771 + t * (-89.90310912 + t * 27.34824973))));
  return [r, g, b].map(v => Math.min(1, Math.max(0, v)));
}
const LUT = new Float32Array(256 * 3);   // linear-space colors for the renderer
for (let i = 0; i < 256; i++) { const c = turbo(0.08 + 0.84 * i / 255); for (let j = 0; j < 3; j++) LUT[3 * i + j] = Math.pow(c[j], 2.2); }
const LEGEND = {h: ['0 m (floor)', '1 m', '2 m+'], d: ['0 m', '5 m', '10 m+'], r: ['0', '75', '150+']};
const SCALE = {h: 2.0, d: 10.0, r: 150};
(function drawLegend() {
  const c = document.querySelector('#legend canvas'), ctx = c.getContext('2d');
  for (let x = 0; x < 256; x++) { const [r, g, b] = turbo(0.08 + 0.84 * x / 255); ctx.fillStyle = `rgb(${r * 255},${g * 255},${b * 255})`; ctx.fillRect(x, 0, 1, 1); }
})();

let calib = {normal: [0, 0, 1], height: LIDAR_ABOVE_FEET, ok: false, pps: 0};
let yaw = 0, last = null;

function applyCalib() {
  yaw = $('flip').checked ? Math.PI : 0;
  const n = V(...calib.normal).applyAxisAngle(V(0, 0, 1), yaw).normalize();
  rig.quaternion.setFromUnitVectors(n, V(0, 0, 1));
  rig.position.set(0, 0, calib.height);
  rig.updateMatrixWorld(true);
  const feet = rig.localToWorld(V(0.035, 0, -LIDAR_ABOVE_FEET));
  const gap = feet.z, tilt = Math.acos(Math.min(1, n.z)) * 180 / Math.PI;
  gapLine.geometry.setFromPoints([feet, V(feet.x, feet.y, 0)]); gapLine.computeLineDistances();
  gapLine.visible = gapLabel.visible = $('showrobot').checked && gap > 0.02;
  const t = `feet ~${Math.round(gap * 100)} cm off the floor`;
  if (t !== gapText) { setLabel(gapLabel, t, {h: 0.09, color: '#ffb066'}); gapText = t; }
  gapLabel.position.set(feet.x, feet.y + 0.35, Math.max(0.08, gap / 2));
  let status;
  if (!calib.ok) status = '<span class="warn">Floor not found yet, so the height is a guess.</span>';
  else if (gap > 0.02) status = `Robot's feet are <b>~${Math.round(gap * 100)} cm above the floor</b> (hanging).`;
  else if (gap < -0.04) status = `<span class="warn">Model feet sit ${Math.round(-gap * 100)} cm below the floor: the robot is crouched or the floor fit is off.</span>`;
  else status = "Robot's feet are on the floor.";
  $('calibtxt').innerHTML = `LiDAR is ${calib.height.toFixed(2)} m above the floor, tilted ${tilt.toFixed(1)}&deg;.<br>${status}`;
  drawBlind();
}

function updateCloud() {
  if (!last) return;
  const a = last, n = Math.min(a.length / 4, MAX_PTS), e = rig.matrixWorld.elements;
  const mode = $('mode').value, scale = SCALE[mode], hideFloor = $('hidefloor').checked, hideSelf = $('hideself').checked;
  const c = Math.cos(yaw), s = Math.sin(yaw);
  let k = 0;
  for (let i = 0; i < n; i++) {
    const xs = a[4 * i], ys = a[4 * i + 1], z = a[4 * i + 2];
    const x = c * xs - s * ys, y = s * xs + c * ys;            // robot body frame, lidar at origin
    if (hideSelf && x * x + y * y < 0.1 && z > -1.35 && z < 0.12) continue;
    const wx = e[0] * x + e[4] * y + e[8] * z + e[12];
    const wy = e[1] * x + e[5] * y + e[9] * z + e[13];
    const wz = e[2] * x + e[6] * y + e[10] * z + e[14];
    if (hideFloor && wz < 0.06) continue;
    const v = mode === 'h' ? wz : mode === 'd' ? Math.hypot(wx, wy) : a[4 * i + 3];
    const li = Math.max(0, Math.min(255, (v / scale * 255) | 0)) * 3;
    posArr[3 * k] = wx; posArr[3 * k + 1] = wy; posArr[3 * k + 2] = wz;
    colArr[3 * k] = LUT[li]; colArr[3 * k + 1] = LUT[li + 1]; colArr[3 * k + 2] = LUT[li + 2];
    k++;
  }
  geom.setDrawRange(0, k);
  geom.attributes.position.needsUpdate = true; geom.attributes.color.needsUpdate = true;
  $('stats').textContent = `${k.toLocaleString()} points shown · ${(calib.pps / 1000).toFixed(0)}k points/s from the lidar`;
}

// ---------- controls ----------
const settings = {mode: 'h', size: 2, hidefloor: false, hideself: true, showrobot: true, showguides: true,
                  showblind: true, flip: false};
for (const [k, d] of Object.entries(settings)) {
  const el = $(k), v = store.get(k, d);
  if (el.type === 'checkbox') el.checked = v; else el.value = v;
  el.addEventListener(el.type === 'range' ? 'input' : 'change', () => { store.set(k, el.type === 'checkbox' ? el.checked : el.value); refresh(); });
}
function refresh() {
  mat.size = +$('size').value;
  robot.visible = $('showrobot').checked;
  guides.visible = $('showguides').checked;
  const lg = LEGEND[$('mode').value]; $('l0').textContent = lg[0]; $('l1').textContent = lg[1]; $('l2').textContent = lg[2];
  applyCalib(); updateCloud();
}

const VIEWS = {
  behind: () => [V(-3.2, 0, 2.4), V(1.5, 0, 0.7)],
  top:    () => [V(-0.01, 0, 13), V(0, 0, 0)],
  front:  () => [V(3.0, -0.9, 1.5), V(0, 0, 0.8)],
  side:   () => [V(0, -4, 1.7), V(0, 0, 0.7)],
  pov:    () => { const eye = rig.localToWorld(V(0.13, 0, -0.07)); return [eye, rig.localToWorld(V(3, 0, -0.9))]; },
};
let anim = null;
function goTo(name, instant = false) {
  const [pos, tgt] = VIEWS[name]();
  if (instant) { camera.position.copy(pos); controls.target.copy(tgt); return; }
  anim = {p0: camera.position.clone(), t0: controls.target.clone(), p1: pos, t1: tgt, start: performance.now()};
  store.set('view', name);
}
document.querySelectorAll('[data-view]').forEach(b => b.onclick = () => goTo(b.dataset.view));

// ---------- click to measure ----------
const ray = new THREE.Raycaster(); ray.params.Points.threshold = 0.06;
const marker = new THREE.Mesh(new THREE.SphereGeometry(0.05, 16, 12), new THREE.MeshBasicMaterial({color: 0xffffff, depthTest: false}));
marker.renderOrder = 11; marker.visible = false; scene.add(marker);
const pickLine = line([V(0, 0, 0), V(1, 0, 0)], 0xffffff, true); pickLine.visible = false; scene.add(pickLine);
let down = null;
renderer.domElement.addEventListener('pointerdown', e => down = [e.clientX, e.clientY]);
renderer.domElement.addEventListener('pointerup', e => {
  if (!down) return;
  const moved = Math.hypot(e.clientX - down[0], e.clientY - down[1]); down = null;
  if (moved > 4) return;
  ray.setFromCamera(new THREE.Vector2(e.clientX / innerWidth * 2 - 1, -(e.clientY / innerHeight) * 2 + 1), camera);
  const hit = ray.intersectObject(cloud)[0];
  if (!hit) { marker.visible = pickLine.visible = false; $('pick').style.display = 'none'; return; }
  const i = hit.index, p = V(posArr[3 * i], posArr[3 * i + 1], posArr[3 * i + 2]);
  const b = rig.worldToLocal(p.clone());                       // robot body frame
  const dist = Math.hypot(b.x, b.y), bear = Math.atan2(b.y, b.x) * 180 / Math.PI, ab = Math.abs(bear);
  const dir = ab < 12 ? 'straight ahead' : ab > 168 ? 'directly behind'
            : `${ab.toFixed(0)}&deg; to the ${bear > 0 ? 'left' : 'right'}${ab > 90 ? ' (behind)' : ''}`;
  marker.position.copy(p); marker.visible = true;
  const base = rig.localToWorld(V(0, 0, 0)); base.z = 0;
  pickLine.geometry.setFromPoints([base, V(p.x, p.y, 0), p]); pickLine.computeLineDistances(); pickLine.visible = true;
  $('pick').innerHTML = `<b>${dist.toFixed(2)} m</b> from the robot &middot; ${dir} &middot; ${p.z.toFixed(2)} m above the floor`;
  $('pick').style.display = 'block';
});

// ---------- data ----------
async function pollPoints() {
  if (!$('pause').checked) {
    try { last = new Float32Array(await (await fetch('points')).arrayBuffer()); updateCloud(); }
    catch (e) { $('stats').textContent = 'lost connection to the viewer: ' + e; }
  }
  setTimeout(pollPoints, 100);
}
async function pollCalib() {
  try { if (!$('pause').checked) { calib = await (await fetch('calib')).json(); applyCalib(); } } catch (e) {}
  setTimeout(pollCalib, 1000);
}

refresh();
goTo(store.get('view', 'behind'), true);
pollCalib(); pollPoints();
(function loop() {
  requestAnimationFrame(loop);
  if (anim) {
    const t = Math.min(1, (performance.now() - anim.start) / 600), k = t * t * (3 - 2 * t);
    camera.position.lerpVectors(anim.p0, anim.p1, k); controls.target.lerpVectors(anim.t0, anim.t1, k);
    if (t === 1) anim = null;
  }
  controls.update(); renderer.render(scene, camera);
})();
</script></body></html>
"""


def make_handler(buf, floor):
    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            if self.path == "/points":
                body = buf.snapshot().tobytes()
                ctype = "application/octet-stream"
            elif self.path == "/calib":
                body = floor.json()
                ctype = "application/json"
            elif self.path in ("/", "/index.html"):
                body = PAGE.encode()
                ctype = "text/html; charset=utf-8"
            else:
                self.send_error(404)
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

        def log_message(self, *args):
            pass

    return Handler


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", type=int, default=8080, help="HTTP port")
    ap.add_argument("--lidar-port", type=int, default=56301, help="UDP point data port on this host")
    ap.add_argument("--window", type=float, default=0.3,
                    help="seconds of points to accumulate (MID-360 is non-repetitive; longer = denser)")
    args = ap.parse_args()

    buf = PointBuffer(args.window)
    floor = FloorEstimator(buf)
    threading.Thread(target=receive, args=(buf, args.lidar_port), daemon=True).start()
    threading.Thread(target=floor.run, daemon=True).start()
    server = ThreadingHTTPServer(("0.0.0.0", args.port), make_handler(buf, floor))
    print(f"Serving on http://0.0.0.0:{args.port}  (Ctrl+C to stop)")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
