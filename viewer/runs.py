"""Saved sim runs for the dashboard's Runs page: course.py's run folders, read as they are.
Standard library only.

Each folder (context/course-runs.md) holds run.json (world, tag, start, commit), and once
course.py has scored it summary.json (outcome, pass, HIM's stop reason, measurements);
events.jsonl has our events, whose started events name the run's pieces. A run with
view.jsonl or policy.log can be replayed. Summaries are cached until a file changes.
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


def pieces(events):
    """The run's avoidance filter, detector and loop, from its started events."""
    found = {}
    try:
        with events.open(errors="replace") as lines:
            for line in lines:
                if '"started"' in line or '"loop_start"' in line:
                    try:
                        piece = piece_of(json.loads(line))
                    except ValueError:
                        continue
                    if piece:
                        found[piece[0]] = piece[1]
    except OSError:
        pass
    return [found[k] for k in PIECES if k in found]


def describe(run):
    meta, summary = read_json(run / "run.json"), read_json(run / "summary.json")
    item = {"name": run.name, "world": meta.get("world") or summary.get("world"),
            "tag": meta.get("tag") or summary.get("tag"), "started": meta.get("started") or run.name[:20],
            "commit": (meta.get("commit") or "")[:9], "dirty": bool(meta.get("dirty")),
            "scored": bool(summary), "pieces": pieces(run / "events.jsonl"),
            "replay": (run / "view.jsonl").is_file() or (run / "policy.log").is_file()}
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
