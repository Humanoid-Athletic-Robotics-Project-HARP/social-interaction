---
name: team-context
description: This repo is shared by a team on different machines and OSes, using both Claude and Codex; don't assume a single user or a Mac
metadata:
  type: project
  updated: 2026-10-26T21:32:46Z
  covers: []
---

This is a team repo (HARP Human Interaction team, see [[project-overview]]). Several people work on it from **different computers and operating systems**, and some use **Claude** while others use **Codex**. Whoever you are talking to may be a different person, on a different OS, than whoever wrote a given memory.

**How to apply:**
- Don't assume the current person is on a Mac, or is the same person as earlier sessions. Ask, or check (`uname -a`, or the platform in your environment), before giving OS-specific commands.
- Setup notes that say "Mac" (e.g. [[mujoco-sim]], [[ros2-mac]]) describe the one machine that was tested: an Apple Silicon Mac with Python 3.12. The pixi environment in `ros2/` only lists Apple Silicon Macs (macOS 14+); Linux would need its platform added, and Windows can't run the keyboard controllers (termios). Neither has been tried (as of 2026-10-25). When someone gets it working on another OS, add those findings next to the Mac ones rather than replacing them.
- When writing a memory, say "a teammate" rather than "the user" for decisions and approvals, and name the OS or machine for anything platform-specific.
- `context/` is the shared memory for every agent. Codex reaches it through `AGENTS.md`, which points to `CLAUDE.md`. Keep memories plain Markdown so both tools can read them.
