---
name: sim-deadline-stops
description: Read when HIM stops a sim run on a deadline or a run ends inconclusive — why it happens (the computer short of memory), what to check and do, and before changing how runs are watched or timed
metadata:
  type: feedback
  updated: 2026-11-04T22:40:35Z
  covers: [submission/harp_node.py, submission/course.py, submission/sim.sh]
---

HIM stops a run when the command it accepted answers a state 0.2 s old (`observation_timeout`, `action_timeout`, `checked_command_expired`). On a Mac with Docker Desktop that is almost always **the Mac short of memory, not our code**: don't change code for it, and don't count the run in results.

**Why** (measured on a 16 GB MacBook Air, 2026-10-29/30, `runs/2026-10-29_*_probe-*`, `runs/*_sim-late-probe`):
- Our reply takes about 2 ms (`harp_node.py`'s `timing` lines). The late part is the state reaching us: HIM's whole sim process pauses 40–240 ms, then catches up (`late_state` lines, `missed` 0).
- The pauses follow the Mac's free memory, not what runs in the sim: macOS compresses the Docker VM's memory, and a process touching such a page waits while it is restored. Each run allocates GBs in the VM that macOS keeps as the VM's afterwards (3–3.6 GB after a few runs), so later runs get worse.
- HIM's live viewer (`live.py`, on the Mac) keeps every frame: about 700 MB after a 100 s run, until Ctrl-C. Browsers held about 2.5 GB.
- At a run's end HIM writes `trace.npz` within a 10 s deadline; short of memory it overran (`plant_error`, a cut-off trace renamed `trace.truncated.npz`, which can't be scored).
- Nav2's start-up was the riskiest moment, so `entrypoint.sh` starts Nav2, the lidar and the task first and our node waits for Nav2 (`harp/nav2_ready.py`, at most 15 s): HIM times nothing before our first command.
- Ruled out: Python's garbage collector in HIM's sim, a VM-wide pause or CPU (probes in another container and on the Mac saw no spikes), and HIM's `--lockstep` (its client then calls every state stale and exits). Still unmeasured: whether HIM's paused thread is busy, waiting for a CPU or blocked.

**Check first:** `summary.json` `outcome` `inconclusive` (HIM's report has `host_stall`) or `violation` with a deadline `him_reason`; `course.py --events RUN` shows our events, `late_state` lines and each second the Mac decompressed 50 MB or more (`PAGED_IN`, from `run.json` `host_memory`) in order. If nothing of ours acted at the stop, it's the host. Stopped runs ranged widely (`max_compressed_mb` about 2,000–6,500, `decompressed_mb` about 100–6,200): high values make a stop likelier but don't predict one.

**Then, on macOS with Docker Desktop**
1. Stop the batch first: the steps below end any run in progress, another session's too, so check nobody else holds the sim lock ([[worktrees]]).
2. Close HIM viewers still up: `pgrep -fl containers/sim/live.py` (Ctrl-C them where they run). `course.py --view` closes its own.
3. Ask the user to quit apps they don't need (browsers hold the most). Never quit their apps yourself.
4. Restart Docker Desktop, which hands back the VM's compressed memory: `osascript -e 'quit app "Docker Desktop"'`, wait until `pgrep -f com.apple.Virtualization.VirtualMachine` finds nothing, `open -a Docker`, wait for `docker info`. Do it between batches whenever compressed memory is high, not only after a stop.
5. Before rerunning: `memory_pressure | tail -1` about 73% free or more, and `vm_stat` "Pages occupied by compressor" (× 16 KB) about 2.5 GB or less. The VM at 4 GB (Docker Desktop, Resources) is enough: a course run uses about 1.4 GB.
6. Rerun the stopped run with the same tag; long runs (over about 100 s) with `--no-view`, then watch the replay ([[course-runs]]).

**On Linux** (no VM): check `free -m` and swap (`vmstat 1`); steps 2, 3 and 6 apply.

**Why this is a rule:** the user asked for it on 2026-10-30, after runs kept failing from memory pressure; restarting Docker and closing viewers and browsers fixed it each time.
