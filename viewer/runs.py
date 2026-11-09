"""Saved sim runs for the dashboard's Runs page: course.py's run folders, read as they are.
Standard library only.

Each folder (context/course-runs.md) holds run.json (world, tag, start, commit), and once
course.py has scored it summary.json (outcome, pass, HIM's stop reason, measurements);
events.jsonl has our events, whose started events name the run's pieces. A run with
view.jsonl or policy.log can be replayed. Summaries are cached until a file changes.
timing() reads one run's state timing (harp_node.py's timing and late_state lines) beside
the host's memory each second (run.json host_memory) for the Runs page's timing chart.
"""
import json
import os
from pathlib import Path
import re
import threading
import time

from configuration import main_checkout
from simulation import PIECES, piece_of

NAME = re.compile(r"\d{4}-\d{2}-\d{2}_\d{2}-\d{2}-\d{2}Z_[A-Za-z0-9_.-]{1,120}")
FILES = ("run.json", "summary.json", "events.jsonl")
FRESH = 5.0   # s a listing is reused before folders are looked at again
DEADLINE_S = 0.2   # HIM stops a run on a command answering a state this old
LATE_S = 0.04      # harp_node.py's LATE: a state this old gets its own late_state line
PAGED_IN_MB = 50   # course.py's PAGED_IN: a second the host paged in this much is worth a look
MEASURES = ("outcome", "passed", "failed_checks", "him_reason", "seconds", "closest_m",
            "contacts", "end_error_m", "path_m")


def runs_folder(root):
    """Where course.py keeps runs: HARP_RUNS_DIR, or the main checkout's runs/."""
    return Path(os.environ.get("HARP_RUNS_DIR") or main_checkout(root) / "runs")


def read_json(path):
    try:
        value = json.loads(path.read_text())
        return value if isinstance(value, dict) else {}
    except (OSError, ValueError):
        return {}


def scan(events):
    """The run's avoidance filter, detector and loop, from its started events, and whether
    it has timing lines."""
    found, timed = {}, False
    try:
        with events.open(errors="replace") as lines:
            for line in lines:
                if not timed and '"timing"' in line:
                    timed = True
                if '"started"' in line or '"loop_start"' in line:
                    try:
                        piece = piece_of(json.loads(line))
                    except ValueError:
                        continue
                    if piece:
                        found[piece[0]] = piece[1]
    except OSError:
        pass
    return [found[k] for k in PIECES if k in found], timed


def ms(value):
    return round(value * 1000, 1) if isinstance(value, (int, float)) else None


def timing(run):
    """A run's state timing in seconds from our policy's first line: each second's worst
    age, reply, extra and gap (ms), each late state, and the host's memory each second."""
    meta, summary = read_json(run / "run.json"), read_json(run / "summary.json")
    seconds, late, start, end = [], [], None, None
    try:
        with (run / "events.jsonl").open(errors="replace") as lines:
            for line in lines:
                if '"policy"' not in line:
                    continue
                try:
                    event = json.loads(line)
                except ValueError:
                    continue
                wall = event.get("wall")
                if event.get("source") != "policy" or not isinstance(wall, (int, float)):
                    continue
                start = wall if start is None else start
                end = wall
                at = {"t": round(wall - start, 2), "sim_time": event.get("sim_time")}
                if event.get("event") == "timing":
                    seconds.append(dict(at, n=event.get("n"), **{k: ms(event.get(k)) for k in ("age", "reply", "extra", "gap")}))
                elif event.get("event") == "late_state":
                    late.append(dict(at, age=ms(event.get("age")), gap=ms(event.get("gap")), missed=event.get("missed")))
    except OSError:
        pass
    host = meta.get("host_memory") if isinstance(meta.get("host_memory"), dict) else {}
    memory = []
    if start is not None:
        for sample in host.get("seconds") or []:
            if isinstance(sample, list) and len(sample) == 3 and all(isinstance(v, (int, float)) for v in sample):
                t = sample[0] - start
                if -2 <= t <= end - start + 2:
                    memory.append({"t": round(t, 2), "free_mb": sample[1], "paged_mb": sample[2]})
    return {"name": run.name, "world": meta.get("world") or summary.get("world"),
            "tag": meta.get("tag") or summary.get("tag"), "outcome": summary.get("outcome"),
            "passed": summary.get("passed"), "him_status": summary.get("him_status"),
            "him_reason": summary.get("him_reason"), "replay": (run / "view.jsonl").is_file() or (run / "policy.log").is_file(),
            "deadline_ms": ms(DEADLINE_S), "late_ms": ms(LATE_S), "paged_in_mb": PAGED_IN_MB,
            "end": round(end - start, 2) if start is not None else None,
            "seconds": seconds, "late": late, "memory": memory,
            "host": {k: host.get(k) for k in ("min_free_mb", "max_compressed_mb", "decompressed_mb",
                                               "most_decompressed_mb_in_1s")}}


def describe(run):
    meta, summary = read_json(run / "run.json"), read_json(run / "summary.json")
    item = {"name": run.name, "world": meta.get("world") or summary.get("world"),
            "tag": meta.get("tag") or summary.get("tag"), "started": meta.get("started") or run.name[:20],
            "commit": (meta.get("commit") or "")[:9], "dirty": bool(meta.get("dirty")),
            "scored": bool(summary),
            "replay": (run / "view.jsonl").is_file() or (run / "policy.log").is_file()}
    item["pieces"], item["timing"] = scan(run / "events.jsonl")
    item.update({k: summary.get(k) for k in MEASURES})
    return item


class Runs:
    def __init__(self, folder):
        self.folder = Path(folder)
        self.lock = threading.Lock()
        self.cache = {}          # name -> (file stamps, description)
        self.listed = (0.0, [])  # (monotonic s, newest first)

    def path(self, name):
        """The folder of a listed run, or None: names come from browsers, never paths."""
        if not isinstance(name, str) or not NAME.fullmatch(name):
            return None
        run = self.folder / name
        return run if run.is_dir() else None

    def timing(self, name):
        run = self.path(name)
        if run is None:
            raise KeyError("no such run")
        return timing(run)

    def list(self):
        with self.lock:
            if time.monotonic() - self.listed[0] < FRESH:
                return self.listed[1]
            found = []
            try:
                names = sorted((p.name for p in self.folder.iterdir()
                                if p.is_dir() and NAME.fullmatch(p.name)), reverse=True)
            except OSError:
                names = []
            for name in names:
                run = self.folder / name
                stamps = tuple((run / f).stat().st_mtime if (run / f).is_file() else None for f in FILES)
                cached = self.cache.get(name)
                if not cached or cached[0] != stamps:
                    cached = self.cache[name] = (stamps, describe(run))
                found.append(cached[1])
            self.listed = (time.monotonic(), found)
            return found
