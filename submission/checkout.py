"""Where this checkout's shared pieces are, so any git worktree of this repo can build and
run the sim (context/worktrees.md):
  HIM    HIM's checkout, beside the main checkout (HIM_REPO to change it)
  MODELS models/ beside it, with HIM's recovery policy
  IMAGE  our image's repository: harp-submission in the main checkout, and
         harp-submission-NAME in a worktree whose folder is NAME
  STAGE  where sim.sh stages the joint profile and scenes, in HIM's ignored .cache (its
         Compose mounts only its checkout): .cache/harp, or .cache/harp/worktrees/NAME
  RUNS   the main checkout's runs/, shared by every worktree (HARP_RUNS_DIR to change it)
  LOCK   the machine-wide lock: one build or sim at a time, since HIM's launcher starts
         by stopping whatever sim is running, and two runs would share the host's memory
build.sh and sim.sh read these as shell variables (eval "$(python3 checkout.py)");
course.py imports it. Standard library only: build.sh and sim.sh run it with the host's
python3.
  python3 checkout.py lock FD WHAT   take the lock on the caller's open file FD (of LOCK),
                                     waiting for whoever holds it; the caller holds it
                                     until it closes FD or exits
A process that holds the lock sets HARP_SIM_LOCK_HELD=1 for the scripts it starts.
"""

import datetime
import fcntl
import os
import re
import shlex
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent


def git(*args):
    return subprocess.run(["git", "-C", str(HERE), *args], capture_output=True,
                          text=True).stdout.strip()


TOP = Path(git("rev-parse", "--show-toplevel") or HERE.parent)   # this checkout or worktree
common = git("rev-parse", "--path-format=absolute", "--git-common-dir")
MAIN = Path(common).parent if common else TOP                    # the main checkout
NAME = "" if TOP == MAIN else re.sub(r"[^a-z0-9]+", "-", TOP.name.lower()).strip("-") or "worktree"
HIM = Path(os.environ.get("HIM_REPO") or MAIN.parent / "him-sim2real-management")
MODELS = MAIN.parent / "models"
IMAGE = "harp-submission" + (f"-{NAME}" if NAME else "")
STAGE = HIM / ".cache/harp" / (f"worktrees/{NAME}" if NAME else "")
RUNS = Path(os.environ.get("HARP_RUNS_DIR") or MAIN / "runs")
LOCK = HIM / ".cache/harp/sim.lock"


def branch():
    return git("rev-parse", "--abbrev-ref", "HEAD") or None


def lock(fd, what):
    """Take the lock on open file fd (of LOCK), waiting for its holder, and say who holds it."""
    try:
        fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        holder = LOCK.read_text(errors="replace").strip() or "another run"
        print(f"Waiting for the sim lock, held by {holder}", file=sys.stderr, flush=True)
        fcntl.flock(fd, fcntl.LOCK_EX)
    os.ftruncate(fd, 0)
    since = datetime.datetime.now().astimezone().strftime("%Y-%m-%d %H:%M:%S %Z")
    os.write(fd, f"{what} in {TOP} (branch {branch()}), since {since}\n".encode())


class Hold:
    """The lock for a with block (no wait if a parent holds it), and HARP_SIM_LOCK_HELD for
    the scripts started inside it."""

    def __init__(self, what):
        self.what, self.file = what, None

    def __enter__(self):
        if not os.environ.get("HARP_SIM_LOCK_HELD"):
            LOCK.parent.mkdir(parents=True, exist_ok=True)
            self.file = open(LOCK, "a")
            lock(self.file.fileno(), self.what)
            os.environ["HARP_SIM_LOCK_HELD"] = "1"
        return self

    def __exit__(self, *exc):
        if self.file:
            del os.environ["HARP_SIM_LOCK_HELD"]
            self.file.close()


if __name__ == "__main__":
    if sys.argv[1:2] == ["lock"]:
        lock(int(sys.argv[2]), " ".join(sys.argv[3:]) or "a script")
    else:
        for key, value in [("HIM", HIM), ("MODELS", MODELS), ("IMAGE_REPO", IMAGE),
                           ("STAGE", STAGE), ("RUNS", RUNS), ("LOCK", LOCK)]:
            print(f"HARP_{key}={shlex.quote(str(value))}")
