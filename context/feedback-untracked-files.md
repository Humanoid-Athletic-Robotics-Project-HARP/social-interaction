---
name: feedback-untracked-files
description: Read before deleting files, cleaning a checkout, or removing a worktree or branch — never lose untracked or ignored files
metadata:
  type: feedback
  updated: 2026-10-30T21:48:27Z
  covers: []
---

Never delete untracked files by accident, in the main checkout or a worktree. Before anything that removes files (removing a worktree, switching or deleting a branch, resetting), list what isn't committed: `git status --short --ignored`. Keep or move anything that isn't build output, and ask when unsure.
- No `git clean`, `git worktree remove --force`, `git reset --hard`, `git checkout -- .` or `rm -rf` on a checkout unless a teammate asks for that exact command.
- Ignored files count too: `git worktree remove` deletes them without asking, and they can hold things that exist nowhere else (`context/workspace-specific-memory.md`, `.env`, `runs/`).
- Delete only files you made yourself in this session and have checked are copies or build output, and say which.

How to remove a worktree safely: [[worktrees]].

**Why:** the maintainer asked for this on 2026-10-30, when we started using worktrees.
