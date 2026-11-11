---
name: team-context
description: Read before giving OS-specific commands or assuming who you're talking to — several people, mixed OSes, Claude and Codex
metadata:
  type: project
  updated: 2026-11-11T01:15:23Z
  covers: [submission/sim.sh]
---

This is the HARP social interaction team's shared repo ([[project-overview]]). Several people work on it from different computers and OSes, some with Claude and some with Codex. The person you're talking to may not be whoever wrote a given memory.

- Don't assume a Mac or the same person as earlier sessions. Check (`uname -a`, or your environment's platform) before OS-specific commands.
- Only Apple Silicon Macs (macOS 15) with Docker Desktop are tested. `submission/sim.sh` uses HIM's Compose launcher (`desktop_sim.sh`) on every OS, which on Linux needs no sudo or network namespaces. Linux and Windows are untested as of 2026-11-10. The robot's image is `linux/arm64`, as HIM requires; an x86 computer builds a sim-only one ([[cloud-sessions]]). Add findings for other OSes next to the Mac ones.
- `context/` is shared memory for every agent; Codex reaches it through `AGENTS.md` → `CLAUDE.md`. Keep it plain Markdown.
