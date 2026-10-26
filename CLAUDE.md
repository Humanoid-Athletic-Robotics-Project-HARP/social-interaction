# CLAUDE.md

## Memory

Use the `context/` folder in this repository for **all** memory in this project. Do not use the default per-user memory directory (e.g. `~/.claude/projects/.../memory/`).

- Store each memory as its own Markdown file in `context/`.
- Keep the memory index at `context/MEMORY.md` (one line per memory: `- [Title](file.md) — short hook`). This replaces the `MEMORY.md` index that would normally be kept internally.
- Read `context/MEMORY.md` at the start of a session, and update it whenever you add, change, or remove a memory file.

### Updating memory and the README

Full rules: `context/readme-and-memory-guidelines.md` (read it before editing either).
- Memories are direct facts and instructions. Each fact lives in one memory; others link to it with `[[name]]`. Dates are absolute.
- Each memory's front matter has `metadata.updated` (UTC only, `2026-10-26T21:30:00Z`, never written by hand) and `metadata.covers` (repo paths or globs it describes; `[]` for things outside the repo).
- Before relying on a memory, run `python3 context/check_memory.py` once per session. It prints "All memory current" when nothing needs checking. Act only on listed memories your task relies on: read their covered files instead of trusting them, fix them if the task touches that topic, and leave unrelated ones alone.
- After editing a memory, or confirming a listed one is still right, run `python3 context/check_memory.py --touch FILE`.
- When you change code, update the memories that cover it, and the README if it's affected, in the same commit.

## Team

This is a shared team repo. Several people work on it from different machines and operating systems, using either Claude or Codex. Don't assume the person you're talking to is on a Mac or is the same person as in earlier sessions. See `context/team-context.md`.
