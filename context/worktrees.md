---
name: worktrees
description: Read before making, using or removing a git worktree of this repo, or when two sessions build or run the sim on one computer — how we set them up, what is shared, the sim lock, cleaning up
metadata:
  type: reference
  updated: 2026-10-30T21:49:44Z
  covers: [submission/checkout.py, submission/build.sh, submission/sim.sh, .gitignore]
---

**How we make them** (the maintainer's call, 2026-10-30; ask before changing it): by hand with git, never with the app's new-session button (its branches get a `claude/` prefix). Inside the project, at `.claude/worktrees/NAME` (ignored by git), on a branch of the same plain name:

    git worktree add .claude/worktrees/NAME -b NAME main

If the main checkout has `context/workspace-specific-memory.md`, link it in, since it is ignored and so not in a new worktree (the link is ignored too):

    ln -s ../../../../context/workspace-specific-memory.md .claude/worktrees/NAME/context/

Then run `submission/build.sh` once in it before its first sim run (it builds `harp-submission-NAME:local`). Planned: `sim-improvements` and `end-to-end-loop`.

**Shared and per worktree** (`submission/checkout.py` works it out; `build.sh`, `sim.sh` and `course.py` use it)
- HIM's checkout and `models/` are found beside the **main** checkout (`HIM_REPO`, `HIM_HOLOSOMA_ONNX` override).
- Run folders go to the main checkout's `runs/`, so `course.py --compare` sees all branches; `run.json` keeps each run's `branch` ([[submission-image]]). `--compare` groups by world and tag only, so give a branch's runs tags of their own.
- Each worktree has its own image repository `harp-submission-NAME` (`:local`, `:WORLD` per world layer), `submission/.build/` and `submission.tar`, and staging folder in HIM's `.cache/harp/worktrees/NAME/`. The main checkout keeps `harp-submission` and `.cache/harp/`.

**One build or sim at a time per computer** (the lock, HIM's `.cache/harp/sim.lock`): HIM's launcher starts by stopping whatever sim is running (`docker compose down` on its one Compose project), and two runs would share the host's memory ([[sim-deadline-stops]]). `build.sh` and `sim.sh` hold it for their whole run, `course.py` for each run with its scoring and pruning (`slim` strips traces in the shared `runs/`); the others print who holds it and wait. A plain `sim.sh` run with its viewer holds it until the viewer is closed (Ctrl-C), so close it promptly; `course.py --view` closes its own. Replays don't take it.
- Always run through our scripts, never HIM's `desktop_sim.sh` directly: that skips the lock and stops the other session's run.
- Who holds it: `lsof HIM/.cache/harp/sim.lock` (the file's text names the last holder, so it can be stale). Don't restart Docker Desktop or close a viewer ([[stalled-runs]]) while another session holds it.
- Tested 2026-10-30 (`runs/2026-10-30_21-4*_open_floor_lock-test*`): a worktree's `build.sh` waited about 3 minutes for an `open_floor` run from the main checkout, then the worktree's own run passed on its own image and staging, both runs listed by `--compare`.

**When its branch is merged** (never lose files: [[feedback-untracked-files]])
1. `git -C .claude/worktrees/NAME status --short --ignored`: nothing untracked or modified, and nothing ignored but build output (`submission/.build/`, `submission.tar`, `joint_profile.json`, `__pycache__/`) and the workspace memory's link (removing it leaves the main checkout's file). Move anything else (`.env`, notes, a workspace memory that is a real file rather than the link) out first, or ask.
2. `git worktree remove .claude/worktrees/NAME`, never `--force`: it refuses when untracked or modified files are left.
3. `git branch -d NAME` (lowercase `-d`: refuses an unmerged branch).
4. `docker image rm` its `harp-submission-NAME` images (`docker images harp-submission-NAME`; about 1 GB in Docker's VM), and HIM's `.cache/harp/worktrees/NAME/`. Its runs stay in the main checkout's `runs/`. If it says a container uses the image: HIM's `compose down` leaves the last run's stopped `g1-desktop-sim-submission-1` (a Compose profile service; its output is already in that run's `policy.log`), so with the lock free, `docker rm g1-desktop-sim-submission-1` (no `-f`) first.
