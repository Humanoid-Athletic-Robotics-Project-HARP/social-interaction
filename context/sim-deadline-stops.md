---
name: sim-deadline-stops
description: Read when HIM stops a sim run on a deadline (observation_timeout, checked_command_expired, stale policy state), or before changing how runs are watched or timed
metadata:
  type: project
  updated: 2026-10-28T23:03:38Z
  covers: [submission/harp_node.py, submission/course.py, submission/sim.sh]
---

HIM stops a run when the command it accepted answers a state now 0.2 s old (`observation_timeout`, `checked_command_expired`). On this MacBook Air (16 GB, Docker Desktop), course runs with HIM's live viewer were stopped about 1 in 4 times (8 of 35), mostly 2.5–6 s in; without it, 3 of 30 (all after the goal: `action_timeout` about 93 s into slalom and obstacle_course, `observation_timeout` 52 s into one_box). So `course.py` never uses the live viewer: `--view` replays each run after it is scored ([[submission-image]]). Plain `sim.sh` moves keep the live viewer: they are short and haven't been stopped.

**Where the time goes:** `harp_node.py` prints a `timing` line each second: wall time, the worst state age (from HIM's `received_at_secs`, on the VM's shared monotonic clock), our reply, republishing the state, and the gap between states. Our reply takes about 2 ms. The late part is the state reaching us: a second subscriber in its own process saw the same ages at the same moments, so the delay is in HIM's sim process (stepping, checking and publishing in one single-threaded executor; with the viewer also a per-step write to the Mac-mounted output) or the Docker VM.

**Open:** runs have 40–240 ms spikes at random moments, with or without the viewer, and with only our walker node too (up to 172 ms over a 120 s move run, `runs/2026-10-28_22-46-55Z_sim_default_task_walker-120`): nothing we add causes them. Long runs see more of them, which is likely why HIM's users (10 s runs by default) don't hit stops. Not CPU: both containers used about 1 of the VM's 8 CPUs. Leading suspect, unconfirmed: macOS memory pressure (about 4 GB compressed, 60 MB free, 2026-10-28).

**Ruled out:** HIM's `--lockstep` stamps states with sim time, so once the sim is 0.2 s behind, HIM's client (and so `harp_node.py`) calls every state stale and exits; the container isn't told it is in lockstep.
