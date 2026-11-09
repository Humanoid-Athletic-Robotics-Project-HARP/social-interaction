"""Whether this computer can run a sim without HIM stopping it on a deadline. Standard
library only: course.py records and warns with it, and the dashboard (viewer/) shows it.

HIM stops a run when a state reaches our node 0.2 s late. On a Mac with Docker Desktop
that is almost always the Mac short of memory (context/sim-deadline-stops.md): macOS
compresses the Docker VM's memory and the sim pauses while pages come back. Before a run
that guide asks for about FREE_PERCENT free (memory_pressure's figure) and at most
COMPRESSED_MB compressed, with no HIM live viewer left open (about 700 MB each).
"""

import re
import subprocess
import sys
from pathlib import Path

FREE_PERCENT = 73     # macOS: kern.memorystatus_level, what memory_pressure prints
COMPRESSED_MB = 2500  # macOS: vm_stat's pages occupied by the compressor
VIEWER = "containers/sim/live.py"   # HIM's live viewer process


def run(args):
    return subprocess.run(args, capture_output=True, text=True, timeout=5).stdout


def host_memory():
    """The host's memory now in MB, or None: free, compressed and decompressed so far on
    macOS (vm_stat), available and swapped in so far on Linux (/proc)."""
    try:
        if sys.platform == "darwin":
            text = run(["vm_stat"])
            mb = int(re.search(r"page size of (\d+)", text).group(1)) / 2**20
            pages = {k.strip('"'): int(v.strip(" .")) for k, v in
                     (line.split(":") for line in text.splitlines()[1:] if ":" in line)}
            return {"free_mb": round((pages["Pages free"] + pages["Pages speculative"]) * mb),
                    "compressed_mb": round(pages["Pages occupied by compressor"] * mb),
                    "decompressed_mb": round(pages["Decompressions"] * mb)}
        if sys.platform.startswith("linux"):
            import os
            info = dict(line.split(":") for line in Path("/proc/meminfo").read_text().splitlines())
            vmstat = dict(line.split() for line in Path("/proc/vmstat").read_text().splitlines())
            return {"free_mb": round(int(info["MemAvailable"].split()[0]) / 1024),
                    "swapped_in_mb": round(int(vmstat["pswpin"]) * os.sysconf("SC_PAGE_SIZE") / 2**20)}
    except (OSError, ValueError, KeyError, AttributeError, subprocess.SubprocessError):
        pass
    return None


def viewers():
    """Process ids of HIM live viewers still up."""
    try:
        return run(["pgrep", "-f", VIEWER]).split()
    except (OSError, subprocess.SubprocessError):
        return []


def macos_pressure():
    """(free percent, pressure level: normal, warning or critical), or (None, None)."""
    try:
        free = int(run(["sysctl", "-n", "kern.memorystatus_level"]).strip())
        level = run(["sysctl", "-n", "kern.memorystatus_vm_pressure_level"]).strip()
        return free, {"1": "normal", "2": "warning", "4": "critical"}.get(level, "normal")
    except (OSError, ValueError, subprocess.SubprocessError):
        return None, None


def memory_warnings():
    """What to warn about before a run: HIM viewers still up, macOS short of memory."""
    notes = []
    up = viewers()
    if up:
        notes.append(f"{len(up)} HIM viewer(s) still up (pid {', '.join(up)}), about "
                     "700 MB each: Ctrl-C them where they run")
    if sys.platform == "darwin":
        _, level = macos_pressure()
        if level in ("warning", "critical"):
            notes.append(f"macOS reports {level} memory pressure: close apps first")
    return notes


def health():
    """This computer now, for the dashboard: its memory, and whether a sim run is likely to
    keep its deadlines (ready True or False with reasons; None where unmeasured)."""
    memory, up = host_memory() or {}, viewers()
    result = {"platform": sys.platform, "viewers": len(up), "ready": None, "reasons": [], **memory}
    if up:
        result["reasons"].append(f"{len(up)} HIM viewer{'s' if len(up) > 1 else ''} still open "
                                 "(about 700 MB each)")
    if sys.platform == "darwin":
        free, level = macos_pressure()
        result.update(free_percent=free, pressure=level)
        if free is not None and free < FREE_PERCENT:
            result["reasons"].append(f"{free}% memory free (aim for {FREE_PERCENT}% or more)")
        if memory.get("compressed_mb", 0) > COMPRESSED_MB:
            result["reasons"].append(f"{memory['compressed_mb'] / 1000:.1f} GB compressed (aim for "
                                     f"{COMPRESSED_MB / 1000:.1f} GB or less)")
        if level in ("warning", "critical"):
            result["reasons"].append(f"macOS reports {level} memory pressure")
        result["ready"] = free is not None and not result["reasons"]
    elif up:
        result["ready"] = False
    return result


if __name__ == "__main__":
    import json
    print(json.dumps(health(), indent=1))
