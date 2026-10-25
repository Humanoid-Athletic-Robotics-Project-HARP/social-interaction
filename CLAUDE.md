# CLAUDE.md

## Memory

Use the `context/` folder in this repository for **all** memory in this project. Do not use the default per-user memory directory (e.g. `~/.claude/projects/.../memory/`).

- Store each memory as its own Markdown file in `context/`.
- Keep the memory index at `context/MEMORY.md` (one line per memory: `- [Title](file.md) — short hook`). This replaces the `MEMORY.md` index that would normally be kept internally.
- Read `context/MEMORY.md` at the start of a session, and update it whenever you add, change, or remove a memory file.
