#!/usr/bin/env python3
"""Browser-based live viewer of the G1's head lidar and head camera depth together.

    python3 hybrid_viewer.py            # then open http://<robot-wlan-ip>:8082

Tabs along the top switch between this combined view and the lidar's and the
camera's own viewers (lidar_viewer.py, camera_viewer.py), all served from this one
program, since only one program can read each sensor.

The MID-360 can't see a cone of about 100 deg in front of the robot (its own head
is in the way) nor the floor within about 1 m; the RealSense looks down at exactly
that. This puts both in one frame, so the gaps the lidar leaves are filled where the
camera sees:

  frame   x forward, y left, z up; the origin on the floor below the lidar.
  lidar   UDP points as lidar_viewer.py reads them, levelled on its own floor fit.
  camera  depth as camera_viewer.py reads it, levelled on its own floor fit, then
          moved by the camera's offset from the lidar in Unitree's URDF
          (g1_29dof_rev_1_0: 5.7 cm ahead, 1.8 cm left, same height).

Levelling each sensor on the floor it sees takes out the torso's lean and any mount
error in tilt and height; heading and the sideways offset come from the URDF
(--camera-yaw corrects the heading if the two disagree where they overlap).

Uses ../lidar/lidar_viewer.py and ../camera/camera_viewer.py. Only one program can
read the lidar's port or stream the depth camera: if another has one, or --no-lidar /
--no-depth leaves it to someone else, that part says "Not connected." and the rest
keeps running (colour, from videohub, can always be shared).
"""
import argparse
import json
import math
import os
from pathlib import Path
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path[:0] = [str(HERE.parent / "lidar"), str(HERE.parent / "camera")]
import camera_viewer  # noqa: E402
import lidar_viewer  # noqa: E402

# d435_joint minus mid360_joint on torso_link, g1_29dof_rev_1_0.urdf (m)
CAMERA_OFFSET = (0.0576235 - 0.0002835, 0.01753 - 0.00003)
LIDAR_SELF = 0.6        # m: lidar returns nearer than this hit the robot
CAMERA_SELF = 0.3       # m around the camera's foot that is the robot's own arms
OBSTACLE = 0.08         # m above the floor: higher is something sticking up
LIDAR_MAX = 40000       # points sent from the lidar per update (it has about 60k)


def level_rotation(n):
    """The rotation that turns floor normal n (unit) into +z, by the shortest way."""
    z = np.array([0.0, 0.0, 1.0])
    axis = np.cross(n, z)
    s, c = np.linalg.norm(axis), float(n @ z)
    if s < 1e-9:
        return np.eye(3)
    k = axis / s
    kx = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + s * kx + (1 - c) * kx @ kx


class Hybrid:
    """Both sensors in the robot's floor frame, and how much of the lidar's gap the
    camera fills."""

    def __init__(self, lidar_buf, lidar_floor, depth, camera_yaw):
        self.buf, self.floor, self.depth = lidar_buf, lidar_floor, depth
        y = math.radians(camera_yaw)
        self.yaw = np.array([[math.cos(y), -math.sin(y)], [math.sin(y), math.cos(y)]])
        self.rng = np.random.default_rng()
        self.lock = threading.Lock()
        self.packed = b""
        self.stats = {}
        self.colour, self.colour_error = None, None
        self.colour_lock = threading.Lock()
        self.interface = "eth0"

    def start_colour(self):
        """Unitree's videohub colour, started the first time the camera tab is opened."""
        with self.colour_lock:
            if self.colour is None and self.colour_error is None:
                try:
                    self.colour = camera_viewer.Colour(self.interface, 0.1)
                    self.colour.start()
                except Exception as e:
                    self.colour_error = f"{type(e).__name__}: {e}"
                    print(f"No colour ({self.colour_error}).")
        return self.colour

    def lidar(self):
        pts = self.buf.snapshot()
        with self.floor.lock:
            s = dict(self.floor.state)
        if len(pts) > LIDAR_MAX:
            pts = pts[self.rng.choice(len(pts), LIDAR_MAX, replace=False)]
        xyz = pts[:, :3].astype(np.float64)
        self_hit = np.linalg.norm(xyz, axis=1) < LIDAR_SELF
        out = xyz @ level_rotation(np.array(s["normal"])).T
        out[:, 2] += s["height"]
        return out, pts[:, 3], self_hit, s

    def camera(self):
        frame = self.depth.stream.latest()
        if frame is None:
            return np.zeros((0, 3)), np.zeros(0, bool)
        g = self.depth.level(self.depth.points(frame))      # ahead, left, height
        self_hit = np.hypot(g[:, 0], g[:, 1]) < CAMERA_SELF
        g[:, :2] = g[:, :2] @ self.yaw.T + CAMERA_OFFSET
        return g, self_hit

    def fill(self, cam, floor_state):
        """How much of the lidar's blind area out to 2.5 m the camera sees (10 cm cells)."""
        sectors = floor_state.get("blocked") or []
        if not sectors:
            return None, None
        cell = 0.1
        gx, gy = np.meshgrid(np.arange(-2.5, 2.5, cell) + cell / 2, np.arange(-2.5, 2.5, cell) + cell / 2)
        r, az = np.hypot(gx, gy), np.degrees(np.arctan2(gy, gx))
        blind = np.zeros_like(r, bool)
        for a0, a1 in sectors:
            blind |= ((az - a0) % 360) < (a1 - a0)
        blind &= (r > 0.3) & (r <= 2.5)
        if not blind.any() or len(cam) == 0:
            return 0.0, int(blind.sum())
        i = np.floor((cam[:, 1] + 2.5) / cell).astype(int)
        j = np.floor((cam[:, 0] + 2.5) / cell).astype(int)
        ok = (i >= 0) & (i < r.shape[0]) & (j >= 0) & (j < r.shape[1])
        seen = np.zeros_like(blind)
        seen[i[ok], j[ok]] = True
        return round(100.0 * float((seen & blind).sum()) / float(blind.sum())), int(blind.sum())

    def run(self):
        last_stats = 0.0
        while True:
            time.sleep(0.2)
            lid, refl, lid_self, floor_state = self.lidar()
            cam, cam_self = self.camera()
            n_l, n_c = len(lid), len(cam)
            # x, y, z in mm (int16), source (0 lidar, 1 camera), flags (1 robot itself), value
            rec = np.zeros(n_l + n_c, dtype=[("x", "<i2"), ("y", "<i2"), ("z", "<i2"),
                                              ("src", "u1"), ("flags", "u1")])
            both = np.concatenate([lid, cam]) if n_c else lid
            mm = np.clip(np.round(both * 1000), -32000, 32000)
            rec["x"], rec["y"], rec["z"] = mm[:, 0], mm[:, 1], mm[:, 2]
            rec["src"][n_l:] = 1
            rec["flags"][:n_l][lid_self] = 1
            if n_c:
                rec["flags"][n_l:][cam_self] = 1
            now = time.monotonic()
            with self.lock:
                self.packed = rec.tobytes()
                if now - last_stats > 1.0:
                    filled, cells = self.fill(cam[~cam_self] if n_c else cam, floor_state)
                    d = self.depth
                    lidar_state = json.loads(self.floor.json())
                    camera_ok = d.stream.connected()
                    self.stats = {
                        "lidar": {"ok": floor_state["ok"], "height": floor_state["height"],
                                  "pps": floor_state["pps"], "blocked": floor_state.get("blocked", []),
                                  "floor_from": floor_state.get("floor_from"),
                                  "connected": lidar_state["connected"], "why": lidar_state["why"]},
                        "camera": {"ok": d.floor_ok, "height": round(d.height, 3),
                                   "pitch": round(d.pitch(), 1), "status": d.stream.status,
                                   "connected": camera_ok,
                                   "why": None if camera_ok else (d.stream.why or "no depth frames for 2 s")},
                        "points": {"lidar": n_l, "camera": n_c}, "filled": filled, "blind_cells": cells,
                        "camera_offset": CAMERA_OFFSET}
                    last_stats = now


PAGE = r"""<!doctype html>
<html><head><meta charset="utf-8"><title>G1 Hybrid View</title>
<meta name="viewport" content="width=device-width, initial-scale=1">
<style>
  :root{--panel:#11151bd9;--text:#d7dee6;--muted:#8a96a3;--accent:#ff8c1a;--line:#2a323c}
  html,body{margin:0;height:100%;background:#0b0e12;color:var(--text);font:13px/1.45 system-ui,sans-serif;overflow:hidden}
  #hud{position:fixed;top:10px;left:10px;width:300px;max-width:calc(100vw - 32px);max-height:calc(100vh - 20px);overflow:auto;
       background:var(--panel);backdrop-filter:blur(6px);border:1px solid var(--line);border-radius:8px;padding:10px 12px;box-sizing:border-box}
  h1{font-size:14px;margin:0 0 4px}
  .muted{color:var(--muted)} .warn{color:#ffb066}
  .sec{border-top:1px solid var(--line);margin-top:8px;padding-top:6px}
  .row{display:flex;align-items:center;gap:6px;flex-wrap:wrap;margin:4px 0}
  button{font:inherit;color:var(--text);background:#1d242d;border:1px solid var(--line);border-radius:5px;padding:3px 8px;cursor:pointer}
  button:hover{border-color:var(--accent)}
  label.row{cursor:pointer;margin:3px 0}
  .sw{display:inline-block;width:11px;height:11px;border-radius:2px;vertical-align:-1px;margin-right:4px}
  #fill{font-size:15px;margin:6px 0 2px}
  #nc{position:fixed;top:50%;left:50%;transform:translate(-50%,-50%);background:var(--panel);border:1px solid var(--line);
       border-radius:8px;padding:14px 20px;text-align:center;font-size:20px;max-width:calc(100vw - 48px)}
  #nc .muted{font-size:13px;margin-top:4px}
  @media (max-width:600px){#hud{width:auto;right:10px;max-height:45vh}}
</style>
<script type="importmap">
{"imports":{"three":"https://cdn.jsdelivr.net/npm/three@0.160.0/build/three.module.js",
"three/addons/":"https://cdn.jsdelivr.net/npm/three@0.160.0/examples/jsm/"}}
</script></head>
<body>
<div id="nc" hidden>Not connected.<div class="muted">neither the lidar nor the depth camera</div></div>
<div id="hud">
  <h1>G1 hybrid view: lidar + camera</h1>
  <div id="fill">…</div>
  <div id="stats" class="muted">connecting…</div>
  <div class="sec">
    <div><span class="sw" style="background:#4fb8ff"></span>lidar &nbsp; <span class="sw" style="background:#ffa040"></span>camera depth
      &nbsp; <span class="muted">(dim: floor, bright: things sticking up)</span></div>
  </div>
  <div class="sec row"><span class="muted">View</span>
    <button data-view="top">Top</button><button data-view="behind">Behind</button><button data-view="front">Front</button></div>
  <div class="sec">
    <label class="row"><input type="checkbox" id="showlidar"> Show lidar</label>
    <label class="row"><input type="checkbox" id="showcamera"> Show camera depth</label>
    <label class="row"><input type="checkbox" id="hidefloor"> Hide floor points</label>
    <label class="row"><input type="checkbox" id="hideself"> Hide hits on the robot itself</label>
    <label class="row"><input type="checkbox" id="pause"> Pause</label>
  </div>
  <div class="sec muted">Each sensor is levelled on the floor it sees; the camera sits 5.7 cm ahead of the
    lidar (Unitree's URDF). The orange in front is where the camera covers what the lidar can't see. Drag to orbit,
    right-drag to pan, scroll to zoom.</div>
</div>
<script type="module">
import * as THREE from 'three';
import {OrbitControls} from 'three/addons/controls/OrbitControls.js';
const $ = id => document.getElementById(id);
const V = (x, y, z) => new THREE.Vector3(x, y, z);
const store = {
  get(k, d) { try { const v = localStorage.getItem('g1hybrid.' + k); return v === null ? d : JSON.parse(v); } catch (e) { return d; } },
  set(k, v) { try { localStorage.setItem('g1hybrid.' + k, JSON.stringify(v)); } catch (e) {} },
};
const renderer = new THREE.WebGLRenderer({antialias: true});
renderer.setPixelRatio(devicePixelRatio);
document.body.appendChild(renderer.domElement);
const scene = new THREE.Scene(); scene.background = new THREE.Color(0x0b0e12);
const camera = new THREE.PerspectiveCamera(55, 1, 0.03, 200); camera.up.set(0, 0, 1);
const controls = new OrbitControls(camera, renderer.domElement); controls.enableDamping = true;
function resize() { renderer.setSize(innerWidth, innerHeight); camera.aspect = innerWidth / innerHeight; camera.updateProjectionMatrix(); }
addEventListener('resize', resize); resize();

function label(text, h, color) {
  const c = document.createElement('canvas'), ctx = c.getContext('2d'), font = '600 56px system-ui,sans-serif';
  ctx.font = font; c.width = Math.ceil(ctx.measureText(text).width) + 36; c.height = 82; ctx.font = font;
  ctx.fillStyle = 'rgba(10,14,18,0.7)'; ctx.fillRect(0, 0, c.width, c.height);
  ctx.fillStyle = color; ctx.textBaseline = 'middle'; ctx.fillText(text, 18, 43);
  const t = new THREE.CanvasTexture(c); t.colorSpace = THREE.SRGBColorSpace;
  const s = new THREE.Sprite(new THREE.SpriteMaterial({map: t, depthTest: false, transparent: true}));
  s.scale.set(h * c.width / c.height, h, 1); s.renderOrder = 10; return s;
}
function ring(r, color) {
  const p = []; for (let i = 0; i <= 128; i++) { const a = i / 128 * Math.PI * 2; p.push(V(r * Math.cos(a), r * Math.sin(a), 0.003)); }
  return new THREE.Line(new THREE.BufferGeometry().setFromPoints(p), new THREE.LineBasicMaterial({color, transparent: true, opacity: 0.7}));
}
const grid = new THREE.GridHelper(20, 20, 0x2c3644, 0x1b222a); grid.rotation.x = Math.PI / 2; scene.add(grid);
for (const r of [1, 2, 3, 5]) {
  scene.add(ring(r, 0x3f7aa0));
  const l = label(`${r} m`, 0.18, '#8cc3e6'); l.position.set(r * Math.cos(-0.5), r * Math.sin(-0.5), 0.1); scene.add(l);
}
for (const [t, x, y, col] of [['FRONT', 4.2, 0, '#ffb066'], ['BACK', -4.2, 0, '#c9d2dc'], ['LEFT', 0, 4.2, '#c9d2dc'], ['RIGHT', 0, -4.2, '#c9d2dc']]) {
  const l = label(t, 0.35, col); l.position.set(x, y, 0.25); scene.add(l);
}
scene.add(new THREE.ArrowHelper(V(1, 0, 0), V(0.2, 0, 0.01), 0.9, 0xff8c1a, 0.22, 0.13));
const body = new THREE.Mesh(new THREE.CylinderGeometry(0.16, 0.16, 1.25, 24),
  new THREE.MeshBasicMaterial({color: 0xc9ced6, transparent: true, opacity: 0.25}));
body.rotation.x = Math.PI / 2; body.position.set(0, 0, 0.625); scene.add(body);

// points
const MAX = 120000, pos = new Float32Array(MAX * 3), col = new Float32Array(MAX * 3);
const geom = new THREE.BufferGeometry();
geom.setAttribute('position', new THREE.BufferAttribute(pos, 3).setUsage(THREE.DynamicDrawUsage));
geom.setAttribute('color', new THREE.BufferAttribute(col, 3).setUsage(THREE.DynamicDrawUsage));
geom.setDrawRange(0, 0); geom.boundingSphere = new THREE.Sphere(V(0, 0, 0), 200);
const cloud = new THREE.Points(geom, new THREE.PointsMaterial({size: 2.2, sizeAttenuation: false, vertexColors: true}));
cloud.frustumCulled = false; scene.add(cloud);
const lin = c => Math.pow(c / 255, 2.2);
const COLOURS = [[lin(79), lin(184), lin(255)], [lin(255), lin(160), lin(64)]];   // lidar, camera
let last = null, stats = null;
function updateCloud() {
  if (!last) return;
  const v = new DataView(last), n = Math.min(last.byteLength / 8, MAX);
  const showL = $('showlidar').checked, showC = $('showcamera').checked, hideFloor = $('hidefloor').checked, hideSelf = $('hideself').checked;
  let k = 0, nl = 0, nc = 0;
  for (let i = 0; i < n; i++) {
    const o = i * 8, src = v.getUint8(o + 6), self = v.getUint8(o + 7);
    if ((src === 0 && !showL) || (src === 1 && !showC) || (self && hideSelf)) continue;
    const z = v.getInt16(o + 4, true) / 1000, floorPt = z < 0.08;
    if (floorPt && hideFloor) continue;
    pos[3 * k] = v.getInt16(o, true) / 1000; pos[3 * k + 1] = v.getInt16(o + 2, true) / 1000; pos[3 * k + 2] = z;
    const c = COLOURS[src], f = floorPt ? 0.35 : 1.0;
    col[3 * k] = c[0] * f; col[3 * k + 1] = c[1] * f; col[3 * k + 2] = c[2] * f;
    k++; if (src === 0) nl++; else nc++;
  }
  geom.setDrawRange(0, k); geom.attributes.position.needsUpdate = true; geom.attributes.color.needsUpdate = true;
}
function showStats() {
  if (!stats || !stats.lidar) return;
  const L = stats.lidar, C = stats.camera;
  $('fill').innerHTML = stats.filled == null ? '<span class="muted">Waiting for the lidar\'s blind area…</span>'
    : `The camera sees <b>${stats.filled}%</b> of the lidar's blind area out to 2.5 m.`;
  const nc = why => `<span class="warn">Not connected.</span> ${why || ''}`;
  const parts = [`lidar: ${L.connected === false ? nc(L.why) : (L.pps / 1000).toFixed(0) + 'k points/s, ' + (L.ok ? L.height.toFixed(2) + ' m above the floor' : 'floor not found yet')}`,
                 `camera: ${C.connected === false ? nc(C.why) : (C.ok ? C.height.toFixed(2) + ' m up, ' + C.pitch.toFixed(0) + '&deg; down' : 'floor not found yet') + ' (' + C.status + ')'}`];
  $('stats').innerHTML = parts.join('<br>');
  $('nc').hidden = !(L.connected === false && C.connected === false);
}
const VIEWS = {top: [V(-0.01, 0, 9), V(0.8, 0, 0)], behind: [V(-3, 0, 2.6), V(1.5, 0, 0.4)], front: [V(3.2, -1.0, 1.6), V(0, 0, 0.6)]};
let anim = null;
function goTo(name, instant) {
  const [p, t] = VIEWS[name]; store.set('view', name);
  if (instant) { camera.position.copy(p); controls.target.copy(t); return; }
  anim = {p0: camera.position.clone(), t0: controls.target.clone(), p1: p, t1: t, start: performance.now()};
}
document.querySelectorAll('[data-view]').forEach(b => b.onclick = () => goTo(b.dataset.view));
const settings = {showlidar: true, showcamera: true, hidefloor: false, hideself: true};
for (const [k, d] of Object.entries(settings)) {
  const el = $(k); el.checked = store.get(k, d);
  el.addEventListener('change', () => { store.set(k, el.checked); updateCloud(); });
}
async function pollPoints() {
  if (!$('pause').checked) {
    try { last = await (await fetch('points')).arrayBuffer(); updateCloud(); }
    catch (e) { $('stats').textContent = 'lost connection to the viewer'; }
  }
  setTimeout(pollPoints, 200);
}
async function pollStats() {
  try { stats = await (await fetch('stats')).json(); showStats(); } catch (e) {}
  setTimeout(pollStats, 1000);
}
goTo(store.get('view', 'top'), true); pollPoints(); pollStats();
(function loop() {
  requestAnimationFrame(loop);
  if (anim) {
    const t = Math.min(1, (performance.now() - anim.start) / 600), e = t * t * (3 - 2 * t);
    camera.position.lerpVectors(anim.p0, anim.p1, e); controls.target.lerpVectors(anim.t0, anim.t1, e);
    if (t === 1) anim = null;
  }
  controls.update(); renderer.render(scene, camera);
})();
</script></body></html>
"""


TABS = r"""<!doctype html>
<html><head><meta charset="utf-8"><title>G1 Sensors</title>
<meta name="viewport" content="width=device-width, initial-scale=1">
<style>
  :root{--bg:#0b0e12;--text:#d7dee6;--muted:#8a96a3;--accent:#ff8c1a;--line:#2a323c}
  html,body{margin:0;height:100%;background:var(--bg);color:var(--text);font:13px/1.45 system-ui,sans-serif}
  body{display:flex;flex-direction:column}
  nav{display:flex;gap:6px;align-items:center;flex-wrap:wrap;padding:6px 12px;border-bottom:1px solid var(--line)}
  nav b{margin-right:8px}
  nav button{font:inherit;color:var(--text);background:#1d242d;border:1px solid var(--line);border-radius:6px;padding:4px 12px;cursor:pointer}
  nav button:hover{border-color:var(--accent)}
  nav button[aria-selected="true"]{border-color:var(--accent);background:#2a2116;color:#fff}
  iframe{flex:1;width:100%;border:0;display:block}
</style></head>
<body>
<nav role="tablist"><b>G1 sensors</b>
  <button role="tab" data-tab="hybrid">Lidar + camera</button>
  <button role="tab" data-tab="lidar">Lidar</button>
  <button role="tab" data-tab="camera">Camera</button></nav>
<iframe id="view" title="sensor view"></iframe>
<script>
const NAMES = ['hybrid', 'lidar', 'camera'], tabs = document.querySelectorAll('[data-tab]'), frame = document.getElementById('view');
function show(name) {
  tabs.forEach(b => b.setAttribute('aria-selected', String(b.dataset.tab === name)));
  frame.src = name + '/';
  history.replaceState(null, '', '#' + name);
  try { localStorage.setItem('g1sensors.tab', name); } catch (e) {}
}
tabs.forEach(b => b.onclick = () => show(b.dataset.tab));
let first = location.hash.slice(1);
if (!NAMES.includes(first)) { try { first = localStorage.getItem('g1sensors.tab'); } catch (e) {} }
show(NAMES.includes(first) ? first : 'hybrid');
</script></body></html>
"""


def make_handler(hybrid):
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
            path, depth = url.path, hybrid.depth
            if path in ("/hybrid", "/lidar", "/camera"):      # pages need the trailing slash
                self.send_response(301)
                self.send_header("Location", path + "/")
                self.end_headers()
            elif path in ("/", "/index.html"):
                self.send(TABS.encode(), "text/html; charset=utf-8")
            elif path == "/hybrid/":
                self.send(PAGE.encode(), "text/html; charset=utf-8")
            elif path == "/hybrid/points":
                with hybrid.lock:
                    body = hybrid.packed
                self.send(body, "application/octet-stream")
            elif path == "/hybrid/stats":
                with hybrid.lock:
                    body = json.dumps(hybrid.stats).encode()
                self.send(body, "application/json")
            elif path == "/lidar/":
                self.send(lidar_viewer.PAGE.encode(), "text/html; charset=utf-8")
            elif path == "/lidar/points":
                self.send(hybrid.buf.snapshot().tobytes(), "application/octet-stream")
            elif path == "/lidar/calib":
                self.send(hybrid.floor.json(), "application/json")
            elif path == "/camera/":
                hybrid.start_colour()
                self.send(camera_viewer.PAGE.encode(), "text/html; charset=utf-8")
            elif path == "/camera/depth.jpg":
                depth.wanted = time.monotonic()
                self.send(depth.depth_jpeg, "image/jpeg")
            elif path == "/camera/map.jpg":
                self.send(depth.map_jpeg, "image/jpeg")
            elif path == "/camera/colour.jpg":
                colour = hybrid.start_colour()
                self.send(colour.jpeg if colour else None, "image/jpeg")
            elif path == "/camera/stats":
                s = camera_viewer.camera_stats(depth, hybrid.colour,
                                               hybrid.colour_error or "starting")
                self.send(json.dumps(s).encode(), "application/json")
            elif path == "/camera/measure":
                q = parse_qs(url.query)
                self.send(json.dumps(depth.measure(float(q["x"][0]), float(q["y"][0]))).encode(),
                          "application/json")
            else:
                self.send_error(404)

        def log_message(self, *args):
            pass

    return Handler


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", type=int, default=8082, help="HTTP port")
    ap.add_argument("--lidar-port", type=int, default=56301, help="UDP point data port on this host")
    ap.add_argument("--window", type=float, default=0.3, help="seconds of lidar points shown")
    ap.add_argument("--size", default="848x480", help="depth resolution")
    ap.add_argument("--fps", type=int, default=15, help="depth frame rate")
    ap.add_argument("--no-lidar", action="store_true",
                    help="leave the lidar's port to another program: it says Not connected")
    ap.add_argument("--no-depth", action="store_true",
                    help="leave the depth camera to another program: it says Not connected")
    ap.add_argument("--camera-yaw", type=float, default=0.0,
                    help="deg to turn the camera's points, if they don't line up with the lidar's")
    ap.add_argument("--interface", default="eth0", help="PC2's port on the robot network (camera colour)")
    ap.add_argument("--lean", type=float, default=6.0,
                    help="deg the torso leans forward, for the camera until it finds the floor")
    args = ap.parse_args()
    os.nice(10)   # below HARP's harness

    buf = lidar_viewer.PointBuffer(args.window)
    floor = lidar_viewer.FloorEstimator(buf)
    if args.no_lidar:
        buf.why = "turned off (--no-lidar)"
    else:
        threading.Thread(target=lidar_viewer.receive, args=(buf, args.lidar_port), daemon=True).start()
    threading.Thread(target=floor.run, daemon=True).start()
    w, h = (int(v) for v in args.size.split("x"))
    stream = camera_viewer.V4L2Stream("/dev/video0", "Z16 ", w, h, args.fps, "<u2")
    if args.no_depth:
        stream.why = stream.status = "turned off (--no-depth)"
    else:
        stream.start()
    time.sleep(0.5)
    depth = camera_viewer.Depth(stream, args.lean)
    threading.Thread(target=depth.run, daemon=True).start()   # floor fit, and the camera tab's images
    hybrid = Hybrid(buf, floor, depth, args.camera_yaw)
    hybrid.interface = args.interface
    threading.Thread(target=hybrid.run, daemon=True).start()
    server = ThreadingHTTPServer(("0.0.0.0", args.port), make_handler(hybrid))
    print(f"Serving on http://0.0.0.0:{args.port}  (Ctrl+C to stop)")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
        stream.close()
        if hybrid.colour:
            hybrid.colour.running = False


if __name__ == "__main__":
    main()
