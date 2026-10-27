"""Run logging for our programs: events and samples that end up in the run
folder (runs/<run>/), next to what run_recorder (harp_eval) records from ROS.

  runlog.event("walker", "policy_on")
  runlog.event("move_distance", "move_done", target=1.0, estimate=0.98)
  runlog.sample("odom", t=..., x=..., y=..., yaw=...)

A program with a ROS node calls attach() with a function that publishes the
event on /harp/events, and run_recorder writes it. A program without ROS
(move_distance, walk) writes straight to the active run folder instead:
HARP_RUN_DIR if it is set, else the run named in runs/ACTIVE, which
run_recorder writes while it records. With neither, nothing is written, so
the programs work the same without a recorder.

No ROS here: harp_control stays usable without it.
"""

import csv
import json
import os
import threading
import time
from pathlib import Path

ACTIVE = "ACTIVE"         # file in the runs folder: the run being recorded, and its recorder's pid
EVENT_FIELDS = ["t", "source", "event", "details"]

_publish = None
_lock = threading.Lock()
_writers = {}             # path -> (file, csv writer)


def runs_dir():
    """The runs/ folder: $HARP_RUNS_DIR (the ROS 2 environment sets it), else the repo's runs/."""
    if os.environ.get("HARP_RUNS_DIR"):
        return Path(os.environ["HARP_RUNS_DIR"]).resolve()
    return Path(__file__).resolve().parents[4] / "runs"


def active_run():
    """The run folder to write to, or None when nothing is being recorded."""
    if os.environ.get("HARP_RUN_DIR"):
        return Path(os.environ["HARP_RUN_DIR"])
    marker = runs_dir() / ACTIVE
    try:
        name, pid = marker.read_text().split()
        os.kill(int(pid), 0)          # the recorder is still running
    except (OSError, ValueError):
        return None
    return runs_dir() / name


def attach(publish):
    """Send events through publish(json_text) (a ROS publisher) instead of a file."""
    global _publish
    _publish = publish


def event(source, name, **details):
    """Something happened: source is the program, name what happened."""
    record = {"t": time.time(), "source": source, "event": name, "details": details}
    if _publish is not None:
        _publish(json.dumps(record))
        return
    run = active_run()
    if run is not None:
        _write(run / f"events-{source}.csv", EVENT_FIELDS,
               {**record, "details": json.dumps(details)})


def sample(stream, **fields):
    """One row of a time series, for programs without ROS (rate-limit it yourself)."""
    run = active_run()
    if run is not None:
        _write(run / f"{stream}.csv", list(fields), fields)


def _write(path, fields, row):
    with _lock:
        if path not in _writers:
            new = not path.exists()
            f = open(path, "a", newline="")
            writer = csv.DictWriter(f, fieldnames=fields)
            if new:
                writer.writeheader()
            _writers[path] = (f, writer)
        f, writer = _writers[path]
        writer.writerow(row)
        f.flush()
