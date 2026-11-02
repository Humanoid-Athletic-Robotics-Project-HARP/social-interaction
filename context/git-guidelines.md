---
name: git-guidelines
description: Read whenever using git in any way — committing, merging, rebasing, reviewing or opening a pull request
metadata:
  type: feedback
  updated: 2026-11-02T21:37:47Z
  covers: []
---

**Committing:** update memory. Each commit that changes code also updates the memories that cover it (their `covers`) and the README if affected, then `python3 context/check_memory.py --touch FILE` for each memory edited ([[readme-and-memory-guidelines]]).

**Pull requests and merges:** don't stop at the git diff or at textual conflicts. Read the changed code on both sides and look for where their intent conflicts, even where git merges cleanly:
- a function, message or setting whose meaning, units, defaults or arguments changed on one side, still used the old way on the other;
- an assumption one side relies on that the other changed (a sensor setting, a frame, a timing, which process drives `/cmd_vel`);
- the same behavior changed in two places in different ways, or something removed on one side that the other now depends on;
- memories, README and tests that describe one side's version and are now wrong.
After merging, run the tests, and the sim courses the changes touch.

Worktrees and branches: [[worktrees]]. Never lose untracked files: [[feedback-untracked-files]].

**Why:** the maintainer asked for this on 2026-11-02.
