---
name: sim-deadline-stops
description: Read when HIM stops a sim run on a deadline (observation_timeout, checked_command_expired, stale policy state), or before changing how runs are watched or timed
metadata:
  type: project
  updated: 2026-10-29T23:00:34Z
  covers: [submission/harp_node.py, submission/course.py, submission/sim.sh]
---

HIM stops a run when the command it accepted answers a state now 0.2 s old (`observation_timeout`, `checked_command_expired`). On this MacBook Air (16 GB, Docker Desktop), course runs with HIM's live viewer were stopped about 1 in 4 times (8 of 35), mostly 2.5–6 s in; without it, 3 of 30 (all after the goal: `action_timeout` about 93 s into slalom and obstacle_course, `observation_timeout` 52 s into one_box). So `course.py` never uses the live viewer: `--view` replays each run after it is scored ([[submission-image]]). Plain `sim.sh` moves keep the live viewer: they are short and haven't been stopped.

**Where the time goes:** `harp_node.py` prints a `timing` line each second: wall time, the worst state age (from HIM's `received_at_secs`, on the VM's shared monotonic clock), our reply, republishing the state, and the gap between states. Our reply takes about 2 ms. The late part is the state reaching us: a second subscriber in its own process saw the same ages at the same moments, so the delay is in HIM's sim process (stepping, checking and publishing in one single-threaded executor; with the viewer also a per-step write to the Mac-mounted output) or the Docker VM.

**Cause: the Mac running out of memory** (2026-10-29, five runs, `runs/2026-10-29_*_probe-*`). The 40–240 ms spikes come and go with the Mac's free memory, not with what runs in the sim. With the Mac about 80% free (`memory_pressure`), a 120 s move run, the same with the live viewer, and `obstacle_course` with the live viewer all stayed under 40 ms, but for one 77 ms state as Nav2 started. With about 60 MB free and 4.1 GB compressed, a plain 120 s move without the viewer had three, up to 100 ms. Once 1.4 GB was freed, `obstacle_course` with the live viewer stayed under 35 ms. macOS had compressed 1.2 GB of the Docker VM's own memory: a process in the VM that touches such a page waits while macOS restores it. A sleep-loop probe in another container and one on the Mac saw none of the spikes, so the VM is never paused as a whole.
- **HIM's live viewer is a big user:** its `live.py` runs on the Mac and keeps every frame (each visible geom's pose, as Python lists): about 520 MB once the page loads, about 700 MB after a 100 s run, and it keeps that until Ctrl-C. Two left up after their runs held 1.4 GB. Other apps on this Mac (Edge about 2 GB) count too. Close old viewers before a run.
- **Nav2's start-up** (5 s after our node arms, `entrypoint.sh`) is the other late moment, when about ten processes join DDS; it gave the 77 ms state with memory to spare.
- **Ruled out:** Python's garbage collector in HIM's sim (its runner, stepped 150 s with the trace on, added about one tracked object per step and never ran a full collection); a VM-wide pause or CPU (the probes).

**Other ruled out:** HIM's `--lockstep` stamps states with sim time, so once the sim is 0.2 s behind, HIM's client (and so `harp_node.py`) calls every state stale and exits; the container isn't told it is in lockstep.
