"""See a sim run as the robot does, in Rerun: its lidar, where it thinks it is and where it
is, the people, Nav2's local costmap and plan, our events and HIM's state timing.
  python3 submission/lidar_view.py            follow each sim run live, until Ctrl-C
  python3 submission/lidar_view.py --run RUN  a finished run folder
It prints a link to Rerun's web viewer (--native for its own window instead).

Live, it reads our container's output (docker logs -f g1-desktop-sim-submission-1): the
frames harp/view_feed.py prints there a few times a second, and our events. A finished
run's are in its view.jsonl and events.jsonl (course.py), or still in its policy.log.
Each run is its own recording, on the sim_time timeline. In the 3D view, /world/truth
is the sim's truth (the pelvis, the people, the fixed obstacles) and /world/odom what
Nav2 sees (leg odometry, the cloud, the costmap, the plan, the goal), drawn as if odom
were the world: the gap between the two robots is the drift.

Needs Rerun (rerun-sdk) in .venv-view beside this checkout or the main checkout; it
re-executes itself there:
  python3 -m venv .venv-view && .venv-view/bin/pip install rerun-sdk==0.39.0
"""

import argparse
import base64
import json
import os
import signal
import subprocess
import sys
import time
import zlib
from pathlib import Path

import checkout  # beside this file

CONTAINER = "g1-desktop-sim-submission-1"   # HIM's Compose project and service
GRPC_PORT, WEB_PORT = 9876, 9090
MEMORY = "500MB"      # the viewer drops the oldest data past this
VENVS = [checkout.TOP / ".venv-view", checkout.MAIN / ".venv-view"]

try:
    import numpy as np
    import rerun as rr
    import rerun.blueprint as rrb
except ImportError:
    python = next((v / "bin/python" for v in VENVS if (v / "bin/python").is_file()), None)
    if python is None or Path(sys.prefix).resolve() == python.parents[1].resolve():
        raise SystemExit("Needs Rerun in .venv-view: python3 -m venv .venv-view && "
                         ".venv-view/bin/pip install rerun-sdk==0.39.0")
    os.execv(str(python), [str(python), __file__, *sys.argv[1:]])


def our_record(line):
    """Our JSON record in a line of output (after compose's "submission-1  | "), or None."""
    text = line.split("| ", 1)[-1].strip()
    if not text.startswith("{"):
        return None
    try:
        record = json.loads(text)
    except ValueError:
        return None
    return record if isinstance(record, dict) and "event" in record else None


def unpack(text, dtype):
    return np.frombuffer(zlib.decompress(base64.b64decode(text)), dtype=dtype)


def rotation(w, x, y, z):
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def heights(z):
    """Colors for points by height: blue on the floor to red at 2 m."""
    s = np.clip(z / 2.0, 0.0, 1.0)[:, None]
    return (np.hstack([s, 1 - np.abs(2 * s - 1), 1 - s]) * 255).astype(np.uint8)


def costs(values):
    """Colors for the costmap's OccupancyGrid values: lethal red, inflation yellow to blue."""
    v = values.astype(np.float32)
    colors = np.stack([v / 100 * 255, 200 - v, 255 - v * 2.55, np.full_like(v, 160)], axis=1)
    colors[values >= 99] = (230, 40, 40, 220)
    return np.clip(colors, 0, 255).astype(np.uint8)


def blueprint():
    return rrb.Blueprint(rrb.Horizontal(
        rrb.Spatial3DView(origin="/world", name="world"),
        rrb.Vertical(rrb.TextLogView(origin="/events", name="events"),
                     rrb.TimeSeriesView(origin="/timing", name="HIM state timing (s)")),
        column_shares=[3, 2]))


class Recording:
    """One run in Rerun: draws our records as they come."""

    def __init__(self, name, url):
        self.rec = rr.RecordingStream("harp_sim", recording_id=name)
        self.rec.connect_grpc(url)
        self.rec.send_recording_name(name)
        self.rec.send_blueprint(blueprint())
        self.clock = None        # (wall, sim time) of the latest record with both
        self.truth_path, self.odom_path = [], []
        self.rec.log("/world", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)

    def at(self, record):
        """Set the time to the record's sim time (estimated from wall time if it has none)."""
        if "sim_time" in record:
            if "wall" in record:
                self.clock = (record["wall"], record["sim_time"])
            t = record["sim_time"]
        elif "wall" in record and self.clock:
            t = self.clock[1] + record["wall"] - self.clock[0]
        else:
            return False
        self.rec.set_time("sim_time", duration=t)
        return True

    def add(self, record):
        if not self.at(record) and record.get("event") != "world":
            return
        kind = record["event"]
        if record.get("source") == "view":
            {"world": self.world, "frame": self.frame}.get(kind, lambda r: None)(record)
        elif kind == "timing":
            for key in ("age", "reply", "gap"):
                self.rec.log(f"/timing/{key}", rr.Scalars(record[key]))
        elif kind == "odom":
            pass                                  # the frames have both poses
        else:
            fields = " ".join(f"{k}={json.dumps(v)}" for k, v in record.items()
                              if k not in ("source", "event", "wall", "sim_time"))
            self.rec.log("/events", rr.TextLog(f"{record.get('source', '-')} {kind} {fields}"))
            if kind == "goal_sent":
                self.rec.log("/world/odom/goal", rr.Points3D([[record["x"], record["y"], 0.02]],
                                                             radii=0.12, colors=(40, 200, 80)))

    def world(self, record):
        boxes, cylinders, spheres = [], [], []
        for g in record["geoms"]:
            color = [int(c * 255) for c in g["rgba"]]
            if g["type"] == "box":
                boxes.append((g["pos"], g["size"], g["quat"], color))
            elif g["type"] in ("cylinder", "capsule"):
                cylinders.append((g["pos"], g["size"], g["quat"], color))
            elif g["type"] == "sphere":
                spheres.append((g["pos"], g["size"][0], color))
        xyzw = lambda q: [q[1], q[2], q[3], q[0]]
        if boxes:
            self.rec.log("/world/truth/obstacles/boxes", rr.Boxes3D(
                centers=[b[0] for b in boxes], half_sizes=[b[1] for b in boxes],
                quaternions=[xyzw(b[2]) for b in boxes], colors=[b[3] for b in boxes],
                fill_mode="solid"), static=True)
        if cylinders:
            self.rec.log("/world/truth/obstacles/cylinders", rr.Cylinders3D(
                centers=[c[0] for c in cylinders], radii=[c[1][0] for c in cylinders],
                lengths=[2 * c[1][1] for c in cylinders], quaternions=[xyzw(c[2]) for c in cylinders],
                colors=[c[3] for c in cylinders], fill_mode="solid"), static=True)
        if spheres:
            self.rec.log("/world/truth/obstacles/spheres", rr.Points3D(
                [s[0] for s in spheres], radii=[s[1] for s in spheres],
                colors=[s[2] for s in spheres]), static=True)

    def frame(self, f):
        log = self.rec.log
        if "truth" in f:
            x, y, z, *q = f["truth"]
            log("/world/truth/robot", rr.Transform3D(translation=[x, y, z],
                                                     quaternion=[q[1], q[2], q[3], q[0]]))
            log("/world/truth/robot/body", rr.Boxes3D(half_sizes=[0.1, 0.17, 0.55],
                                                      centers=[0, 0, -0.15], colors=(150, 150, 150, 120)))
            self.truth_path.append([x, y, 0.01])
            log("/world/truth/path", rr.LineStrips3D([self.truth_path], colors=(150, 150, 150)))
        if "odom" in f:
            x, y, yaw = f["odom"]
            pose = np.array([[np.cos(yaw), -np.sin(yaw), 0, x], [np.sin(yaw), np.cos(yaw), 0, y],
                             [0, 0, 1, 0]])
            log("/world/odom/robot", rr.Arrows3D(origins=[[x, y, 0.05]],
                                                 vectors=[[0.4 * np.cos(yaw), 0.4 * np.sin(yaw), 0]],
                                                 colors=(255, 140, 0)))
            self.odom_path.append([x, y, 0.02])
            log("/world/odom/path", rr.LineStrips3D([self.odom_path], colors=(255, 140, 0)))
            if "lidar" in f and "points" in f:
                lx, ly, lz, *lq = f["lidar"]
                points = unpack(f["points"], np.int16).reshape(-1, 3) / 100.0
                in_base = points @ rotation(*lq).T + [lx, ly, lz]
                in_odom = in_base @ pose[:, :3].T + pose[:, 3]
                log("/world/odom/lidar", rr.Points3D(in_odom, radii=0.025, colors=heights(in_odom[:, 2])))
        if "people" in f:
            names = list(f["people"])
            log("/world/truth/people", rr.Cylinders3D(
                centers=[[*f["people"][n], 0.86] for n in names], radii=0.25, lengths=1.7,
                colors=(80, 100, 220, 200), labels=names, fill_mode="solid"))
        if "plan" in f:
            log("/world/odom/plan", rr.LineStrips3D([[[x, y, 0.03] for x, y in f["plan"]]],
                                                    colors=(40, 200, 80), radii=0.02))
        if "costmap" in f:
            c = f["costmap"]
            cells = unpack(c["cells"], np.uint8).view(np.int8).reshape(c["height"], c["width"])
            rows, cols = np.nonzero(cells > 0)
            centers = np.stack([c["origin"][0] + (cols + 0.5) * c["resolution"],
                                c["origin"][1] + (rows + 0.5) * c["resolution"],
                                np.zeros(len(rows))], axis=1)
            log("/world/odom/costmap", rr.Boxes3D(
                centers=centers, half_sizes=[c["resolution"] / 2, c["resolution"] / 2, 0.005],
                colors=costs(cells[rows, cols]), fill_mode="solid"))


def viewer(native):
    """Start Rerun's viewer (a window, or its web viewer and the link to it); its process."""
    exe = Path(sys.executable).parent / "rerun"
    if native:
        args = [str(exe), "--port", str(GRPC_PORT), "--memory-limit", MEMORY]
    else:
        args = [str(exe), "--serve-web", "--port", str(GRPC_PORT), "--web-viewer-port",
                str(WEB_PORT), "--server-memory-limit", MEMORY]
    # Its own process group: the rerun script starts the viewer as its child.
    process = subprocess.Popen(args, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                               start_new_session=True)
    time.sleep(2.0)
    if not native:
        print(f"Rerun at http://127.0.0.1:{WEB_PORT}/?url=rerun%2Bhttp://127.0.0.1:{GRPC_PORT}/proxy",
              flush=True)
    return process, f"rerun+http://127.0.0.1:{GRPC_PORT}/proxy"


def container():
    """(id, image, started) of our running sim container, or None."""
    result = subprocess.run(["docker", "inspect", "-f",
                             "{{.Id}} {{.State.Running}} {{.Config.Image}} {{.State.StartedAt}}",
                             CONTAINER], capture_output=True, text=True)
    parts = result.stdout.split()
    return (parts[0], parts[2], parts[3]) if result.returncode == 0 and parts[1:2] == ["true"] else None


def follow(url):
    """Every run as it comes: a recording per container, until Ctrl-C."""
    seen = None
    print(f"Waiting for {CONTAINER} (a sim run in a world: course.py, sim.sh --world)", flush=True)
    while True:
        found = container()
        if not found or found[0] == seen:
            time.sleep(1.0)
            continue
        seen, image, started = found
        name = f"{image.split(':')[-1]} {started[:19].replace('T', ' ')}Z"
        print(f"Showing {name}", flush=True)
        recording = Recording(name, url)
        logs = subprocess.Popen(["docker", "logs", "-f", seen], stdout=subprocess.PIPE,
                                stderr=subprocess.STDOUT, text=True, errors="replace")
        for line in logs.stdout:
            record = our_record(line)
            if record:
                recording.add(record)
        logs.wait()
        recording.rec.flush()
        print(f"{name}: run over", flush=True)


def show_run(run, url):
    run = Path(run)
    recording = Recording(run.name, url)
    files = [run / "view.jsonl", run / "events.jsonl"]
    if not files[1].is_file():
        files = [run / "policy.log"]
    for path in files:
        if path.is_file():
            for line in path.read_text(errors="replace").splitlines():
                record = our_record(line)
                if record:
                    recording.add(record)
    recording.rec.flush()
    print(f"{run.name}: sent; Ctrl-C closes the viewer", flush=True)
    while True:
        time.sleep(3600)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--run", metavar="RUN", help="show this finished run folder instead")
    parser.add_argument("--native", action="store_true", help="Rerun's own window, not the web viewer")
    args = parser.parse_args()
    process, url = viewer(args.native)
    signal.signal(signal.SIGTERM, signal.default_int_handler)   # closes the viewer too
    try:
        show_run(args.run, url) if args.run else follow(url)
    except KeyboardInterrupt:
        pass
    finally:
        os.killpg(process.pid, signal.SIGTERM)
        process.wait()
    return 0


if __name__ == "__main__":
    sys.exit(main())
