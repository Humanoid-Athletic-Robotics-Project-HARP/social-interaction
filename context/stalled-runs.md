---
name: stalled-runs
description: Read when a sim run ends inconclusive or HIM stops it on a deadline (observation_timeout, checked_command_expired, postcheck_timeout, action_timeout) — what to check and do before rerunning
metadata:
  type: feedback
  updated: 2026-10-30T21:48:27Z
  covers: []
---

A run HIM stops on a deadline is usually the computer short of memory, not our code ([[sim-deadline-stops]] has the why). Don't change code for it, and don't count the run in results.

**Check first**
- `summary.json`: `outcome` `inconclusive` (HIM's report has `host_stall`) or `violation` with a deadline `him_reason`.
- `run.json` `host_memory` (macOS): the stopped runs had `max_compressed_mb` about 3,700–4,900 and `decompressed_mb` 1,000–2,600 (2026-10-29/30, this repo's MacBook Air, 16 GB, Docker at 4 GB). These make a stop likely but don't predict one: a clean run once decompressed 4,160 MB, and `min_free_mb` is under 100 in most runs, clean ones too.
- Our events before the stop: if nothing of ours acted then, it's the host.

**Then, on macOS with Docker Desktop**
1. Stop the batch first: the steps below end any run in progress, another session's too, so check nobody else holds the sim lock ([[worktrees]]).
2. Close HIM viewers still up from earlier runs: `pgrep -fl containers/sim/live.py` (about 700 MB each; Ctrl-C them where they run). `course.py --view` closes its own.
3. Ask the user to quit apps they don't need (browsers hold the most; Edge held about 2.5 GB). Never quit their apps yourself.
4. Restart Docker Desktop, which hands back the memory macOS has compressed for its VM (3–3.6 GB after a few runs): `osascript -e 'quit app "Docker Desktop"'`, wait until the VM process is gone (`pgrep -f com.apple.Virtualization.VirtualMachine`), `open -a Docker`, and wait for `docker info`. Do this between batches whenever compressed memory is high, not only after a stop.
5. Check before rerunning: `memory_pressure | tail -1` about 73% free or more, and `vm_stat` "Pages occupied by compressor" (× 16 KB) about 2.5 GB or less.
6. Rerun the stopped run with the same tag.

**On Linux** (no VM): check `free -m` and swap use (`vmstat 1`); the same steps 2, 3 and 6 apply.

**Why:** the user asked for this on 2026-10-30, after runs kept failing from memory pressure; restarting Docker and closing viewers and browsers fixed it each time.
