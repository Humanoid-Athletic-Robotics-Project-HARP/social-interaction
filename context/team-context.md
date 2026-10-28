---
name: team-context
description: Read before giving OS-specific commands or assuming who you're talking to — several people, mixed OSes, Claude and Codex
metadata:
  type: project
  updated: 2026-10-28T08:59:18Z
  covers: []
---

This is the HARP Human Interaction team's shared repo ([[project-overview]]). Several people work on it from different computers and OSes, some with Claude and some with Codex. The person you're talking to may not be whoever wrote a given memory.

- Don't assume a Mac or the same person as earlier sessions. Check (`uname -a`, or your environment's platform) before OS-specific commands.
- Only Apple Silicon Macs (macOS 15) with Docker Desktop are tested: `submission/sim.sh` uses HIM's `desktop_sim.sh` there. On Linux it uses HIM's `sim.sh` (network namespaces, sudo); Windows would need HIM's Compose path. Both untested as of 2026-10-28. The image is `linux/arm64`, as HIM requires. Add findings for other OSes next to the Mac ones.
- `context/` is shared memory for every agent; Codex reaches it through `AGENTS.md` → `CLAUDE.md`. Keep it plain Markdown.
