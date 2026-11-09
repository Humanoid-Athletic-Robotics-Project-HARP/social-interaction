"""Collect our simulation logs and render them through the existing Rerun presentation."""
import collections
import heapq
import json
import os
from pathlib import Path
import queue
import shutil
import signal
import subprocess
import time

from feeds import Source

ROOT = Path(__file__).resolve().parents[1]


def record(line):
    text = line.split("| ", 1)[-1].strip()
    if not text.startswith("{"):
        return None
    try:
        value = json.loads(text)
    except ValueError:
        return None
    return value if isinstance(value, dict) and "event" in value else None


class Simulation(Source):
    def __init__(self, config):
        super().__init__(config)
        self.worker = None
        self.logs = None
        self.render_queue = queue.Queue(maxsize=128)
        self.ports = None
        self.worker_error = None
        self.render_dropped = 0
        self.last_frame = None
        self.last_event = None
        self.finished = False
        self.seen = None
        self.recent = collections.deque(maxlen=250)
        self.world = None
        self.pieces = {}   # avoidance, detector, behavior: the names their start events give

    def note_piece(self, value):
        """Which optional pieces this run has, from their started events (lidar_view.py
        shows the details)."""
        source, kind = value.get("source"), value.get("event")
        if kind == "started" and source in ("predictive", "approach"):
            self.pieces["avoidance"] = "predictive avoidance" if source == "predictive" else "approach stop"
        elif kind == "started" and source == "people":
            self.pieces["detector"] = str(value.get("detector", "person detector"))
        elif kind == "loop_start" and source == "behavior":
            self.pieces["behavior"] = "behavior loop"

    def start(self):
        self.start_thread(self.render)
        self.start_thread(self.replay if self.kind == "recording" else self.follow)

    def close(self):
        self.stop.set()
        if self.logs and self.logs.poll() is None:
            self.logs.terminate()
        worker = self.worker
        if worker and worker.poll() is None:
            try:
                os.killpg(worker.pid, signal.SIGTERM)
                worker.wait(4)
            except (OSError, subprocess.TimeoutExpired):
                try:
                    os.killpg(worker.pid, signal.SIGKILL)
                except OSError:
                    pass
        super().close()
        if worker:
            for stream in (worker.stdin, worker.stdout):
                if stream:
                    stream.close()

    def offer(self, event):
        try:
            self.render_queue.put_nowait(event)
        except queue.Full:
            self.render_dropped += 1

    def new_session(self, session):
        with self.lock:
            self.session = session
            self.last_frame = self.last_event = None
            self.finished = False
            self.recent.clear()
            self.world = None
            self.pieces = {}
            self.render_dropped = 0
        while True:
            try:
                self.render_queue.get_nowait()
            except queue.Empty:
                break
        self.offer({"_session": session, "_name": self.config.get("label", self.id)})

    def ingest(self, value):
        with self.lock:
            self.last_event = time.monotonic()
            if value.get("source") == "view" and value.get("event") == "frame":
                self.last_frame = self.last_event
            elif value.get("source") == "view" and value.get("event") == "world":
                self.world = value
            else:
                self.recent.append(value)
                self.note_piece(value)
        self.offer(value)

    def status(self):
        with self.lock:
            frame_age = None if self.last_frame is None else time.monotonic() - self.last_frame
            event_age = None if self.last_event is None else time.monotonic() - self.last_event
            live = self.error is None and not self.finished and frame_age is not None and frame_age < 4
            self.feeds = {"diagnostics": {"online": live, "reason": "run ended" if self.finished else
                          "waiting for our view frames" if frame_age is None else "no view frames for %.1f s" % frame_age if not live else "receiving our view frames"}}
            if self.kind == "recording":
                self.feeds = {"diagnostics": {"online": self.error is None, "reason": "saved recording"}}
            self.detail = {"viewer": "/source/%s/diagnostics/" % self.id,
                           "renderer": "ready" if self.ports else (self.worker_error or "waiting for data"),
                           "frame_age": frame_age, "event_age": event_age,
                           "finished": self.finished, "dropped": self.render_dropped,
                           "rrd": self.kind == "recording" and self.config.get("path", "").lower().endswith(".rrd"),
                           "events": list(self.recent)[-30:],
                           "pieces": [self.pieces[k] for k in ("avoidance", "detector", "behavior")
                                      if k in self.pieces]}
            # Replay remains available after EOF; live freshness is based on frames, not process existence.
            self.updated = time.monotonic() if self.error is None else self.updated
            value = super().status()
            if self.kind == "recording":
                value["mode"] = "replay"
            return value

    def follow(self):
        configured = self.config.get("docker")
        docker = configured or shutil.which("docker")
        mac = Path("/Applications/Docker.app/Contents/Resources/bin/docker")
        if not docker and mac.is_file():
            docker = str(mac)
        container = self.config.get("container", "g1-desktop-sim-submission-1")
        while not self.stop.is_set():
            try:
                if not docker:
                    raise OSError("Docker CLI not found; configure this source's docker path")
                result = subprocess.run([docker, "inspect", container], capture_output=True, text=True, timeout=5)
                if result.returncode:
                    raise OSError("waiting for simulation container " + container)
                info = json.loads(result.stdout)[0]
                if not info["State"]["Running"]:
                    raise OSError("simulation container is stopped")
                session = info["Id"] + ":" + info["State"]["StartedAt"]
                if session == self.seen:
                    self.stop.wait(1)
                    continue
                self.seen = session
                self.new_session(session)
                self.error = None
                self.logs = subprocess.Popen([docker, "logs", "--follow", "--since", info["State"]["StartedAt"], info["Id"]],
                                             stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, errors="replace")
                for line in self.logs.stdout:
                    if self.stop.is_set():
                        break
                    if len(line) > 512 * 1024:
                        continue
                    value = record(line)
                    if value:
                        self.ingest(value)
                self.logs.wait()
                self.logs.stdout.close()
                with self.lock:
                    self.finished = True
                self.offer({"_flush": True})
                # A lost Docker log connection can return before the container ends.
                # Reinspect on the next pass rather than permanently abandoning it.
                self.seen = None
            except (OSError, ValueError, KeyError, IndexError, subprocess.TimeoutExpired) as error:
                self.failed(error)
            self.stop.wait(1)

    def replay(self):
        try:
            path = Path(self.config["path"]).expanduser().resolve()
            if path.suffix.lower() == ".rrd":
                if not path.is_file():
                    raise ValueError("recording file does not exist")
                self.new_session(str(path))
            else:
                paths = [path / "view.jsonl", path / "events.jsonl"]
                if not paths[0].is_file():
                    paths = [path / "policy.log"]
                if not any(p.is_file() for p in paths):
                    raise ValueError("run has no view.jsonl or policy.log")
                self.new_session(str(path))
                files = [file.open(errors="replace") for file in paths if file.is_file()]
                try:
                    # Frames and events interleaved by wall time, as they happened, so the
                    # status that events set follows the frames.
                    merged = heapq.merge(*((v for v in map(record, f) if v) for f in files),
                                         key=lambda v: v.get("wall", 0))
                    for value in merged:
                        if self.stop.is_set():
                            return
                        # Replay is lossless; backpressure here cannot delay a robot.
                        with self.lock:
                            if value.get("source") != "view":
                                self.recent.append(value)
                                self.note_piece(value)
                        while not self.stop.is_set():
                            try:
                                self.render_queue.put(value, timeout=.2)
                                break
                            except queue.Full:
                                if self.worker_error:
                                    raise ValueError(self.worker_error)
                finally:
                    for f in files:
                        f.close()
                self.offer({"_flush": True})
            with self.lock:
                self.error = None
                self.finished = True
        except (OSError, ValueError, KeyError) as error:
            self.failed(error)

    def render(self):
        # Start Rerun only once actual simulation data or an explicit recording exists.
        while not self.stop.is_set():
            try:
                first = self.render_queue.get(timeout=.2)
                break
            except queue.Empty:
                continue
        else:
            return
        python = Path(self.config.get("python", str(ROOT / "testground/.venv/bin/python"))).expanduser()
        if not python.is_file():
            self.worker_error = "Install testground/requirements.txt in testground/.venv (see viewer/README.md)"
            return
        try:
            self.worker = subprocess.Popen([str(python), str(Path(__file__).with_name("rerun_worker.py"))],
                                           stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                                           text=True, bufsize=1, start_new_session=True)
            def output():
                for line in self.worker.stdout:
                    try:
                        value = json.loads(line)
                        with self.lock:
                            if value.get("ready"):
                                self.ports = value
                            elif value.get("error"):
                                self.worker_error = value["error"]
                    except ValueError:
                        pass
                if not self.stop.is_set():
                    self.worker_error = self.worker_error or "Rerun renderer stopped"
                    self.ports = None
            self.start_thread(output)
            value = first
            while not self.stop.is_set():
                if value is not None:
                    self.worker.stdin.write(json.dumps(value) + "\n")
                    self.worker.stdin.flush()
                try:
                    value = self.render_queue.get(timeout=.2)
                except queue.Empty:
                    value = None
        except (OSError, ValueError) as error:
            self.worker_error = str(error)
