---
name: team-context
description: Read before giving OS-specific commands or assuming who you're talking to — several people, mixed OSes, Claude and Codex
metadata:
  type: project
  updated: 2026-10-26T22:40:11Z
  covers: []
---

This is the HARP Human Interaction team's shared repo ([[project-overview]]). Several people work on it from different computers and OSes, some with Claude and some with Codex. The person you're talking to may not be whoever wrote a given memory.

- Don't assume a Mac or the same person as earlier sessions. Check (`uname -a`, or your environment's platform) before OS-specific commands.
- Only Apple Silicon Macs (macOS 14+, Python 3.12) are tested: the pixi environment in `ros2/` lists only that platform. Linux needs its platform added to `ros2/pixi.toml` (and `lo` instead of `lo0`); Windows can't run the keyboard controllers (termios). Both untested as of 2026-10-26. Add findings for other OSes next to the Mac ones.
- `context/` is shared memory for every agent; Codex reaches it through `AGENTS.md` → `CLAUDE.md`. Keep it plain Markdown.
