---
name: cloud-sessions
description: Read before setting up or running the sim in a Claude Code cloud session, or building the image for an x86 computer
metadata:
  type: project
  updated: 2026-11-11T01:15:23Z
  covers: [submission/cloud_setup.sh, submission/build.sh]
---

A Claude Code cloud session is an x86_64 Ubuntu 24.04 VM (about 4 vCPUs, 16 GB RAM, 30 GB disk) with Docker but no GPU, mic or display. `submission/cloud_setup.sh` (README "Setup") puts HIM's checkout, its `.venv-policy-safety` and the Holosoma checkpoint beside the repo and starts `dockerd`; each step is skipped once done.

**x86 image:** `HARP_PLATFORM=linux/amd64 submission/build.sh` builds the same Dockerfile natively. Every package we install has x86 builds, and HIM's sim image has no platform pin, so it builds for the host. The x86 image is for the sim only, so `build.sh` doesn't save it as `submission.tar`. Building it on an Apple Silicon Mac runs emulated: fine to check that it builds, useless for runs (deadlines).

**Access:** HIM's repo is private, and cloud sessions reach only the GitHub repos attached to them, so attach `HIMRobotics/him-sim2real-management` or the clone fails. The default network allowlist already covers Docker Hub, ghcr.io, PyPI and raw.githubusercontent.com (the checkpoint). Speech models need Hugging Face allowed, and live speech can't run there.

**Limits:** run courses with `--no-view`. One command runs up to 10 minutes in the foreground, then 30 more in the background, so long batches need splitting. Whether 4 vCPUs keep HIM's 0.2 s deadline with Nav2 is untested as of 2026-11-10 ([[sim-deadline-stops]]); compare against the `rectangle` baseline in [[course-runs]].

A SessionStart hook in `.claude/settings.json` could run the setup script automatically, but it isn't added: that file is shared, so the hook would touch every teammate's sessions. Add it only with the team's agreement.
