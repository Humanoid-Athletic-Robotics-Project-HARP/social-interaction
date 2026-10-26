---
name: readme-and-memory-guidelines
description: Read before editing README.md or any file in context/ — how to write them, what to leave out, and the freshness check
metadata:
  type: feedback
  updated: 2026-10-26T22:40:10Z
  covers: [context/check_memory.py]
---

## README

`README.md` is the front page for newcomers and the maintainer.
- Say what the robot does and has. Leave out what it doesn't do, what we don't use, and comparisons with what we replaced. Safety steps stay, phrased as what to do ("keep the robot supported").
- Give only the correct way, not what fails: the right command (`pixi shell --as-is`), with the reason and symptom in `context/`.
- No process notes ("ask the team"), no "draft" labels. HARP is a robotics team: never "course" or "course staff".
- Short: commands with one line on what they do, bullets over paragraphs, the stand-up sequence explained once and referred to elsewhere.
- Measured results, limits, edge cases, design reasons and gotchas go in `context/`.
- Update it in the same commit as the code it describes.

## README "Goals and progress"

The only place for goals and their status; nothing else (memory included) keeps a goal or status list.
- A few wide goals, each a `###` heading in plain words, no numbers. New work goes under an existing goal.
- A summary table at the top: Goal | Sim | Robot | Next step. Nothing is done on the robot until it has run there.
- Under each goal, bullets only: "Done when:" first, then one per milestone with ✅ done, 🟡 in progress, ⬜ not started or ⛔ blocked, and "Sim:"/"Robot:" where it matters. Name the command or file, don't explain it.
- Update in the same commit as the work: tick milestones and the table row, mark 🟡 when started, ⛔ with the blocker. No progress log; git history covers that.
- Adding, removing, merging or redefining a goal, or changing a "Done when", is the team's call: propose and ask. Ticking milestones needs no approval.

## Writing memories

A memory tells a future agent what is true now and how to act on it. Write the final state, not the story of how we got there.

**Keep**
- Current facts and instructions: how to run, what each setting is and why.
- Findings that would save someone from repeating work: a measured limit, a bug's cause when the fix constrains future changes (e.g. "clearing uses only floor points, because full clearing erased thin tabletops"), an option tried and rejected with the one-line reason.
- The current baseline numbers, naming the pinned run folder they come from ([[run-logging]]).
- Open questions and next steps, briefly.

**Leave out**
- History: "built on", "since", "changed on X, then on Y", "the old flag is gone", results from before a fix. If something was replaced, describe only what's there now.
- Provenance on every line ("a teammate's call, 2026-10-26"). Note who decided only where it limits what an agent may change (e.g. "the team chose this; ask before changing it").
- Copies of what the code or config already says: full flag lists (`--help` has them), parameter tables, limits that live in a YAML file. Name the file and give the reason behind the value instead.
- Anything the README already says, and one-off mistakes that aren't expected to recur. When someone hits a problem, fix the code or docs if that helps everyone.

**Form**
- `description` is a hook: when to read the memory ("Read when tuning Nav2 or changing a course"), not a table of contents or its backstory. The `MEMORY.md` line says the same in fewer words.
- Short: aim for under about 30 lines; split a memory that covers two topics an agent wouldn't need together.
- One place per fact. Others link with `[[name]]` instead of repeating it; copies drift.
- Dates absolute (2026-10-26), only where the date matters (e.g. "unchecked as of"). Say "a teammate", not "the user", and name the OS or machine for anything platform-specific ([[team-context]]).
- Front matter: `name` (file name without `.md`), `description`, `metadata.type` (user, feedback, project or reference), `metadata.updated`, `metadata.covers`. Every memory has one line in `context/MEMORY.md`.

## Keeping memories current

`context/check_memory.py` (plain Python 3 and git) flags memories whose covered files changed after they were last confirmed.
- `metadata.updated`: UTC only (`2026-10-26T21:30:00Z`), written by the script, never by hand.
- `metadata.covers`: repo paths or globs whose changes can make the memory wrong. `[]` for memories about things outside the repo (web docs, research, team facts); the script skips them.
- Before relying on memory, run `python3 context/check_memory.py` once per session ("All memory current" means nothing is flagged). For flagged memories your task relies on, read the covered files instead of trusting the memory, and fix it if the task touches that topic. Leave unrelated flagged ones alone.
- After editing a memory, or confirming a flagged one, run `python3 context/check_memory.py --touch FILE`. Add new paths to `covers` when a memory starts describing new code.
- When you change code, update the memories that cover it, and the README if affected, in the same commit.
- Optional, not installed: `python3 context/check_memory.py --pre-commit` from `.git/hooks/pre-commit` stamps memories in each commit and warns about covered files committed without them.
- `CLAUDE.md` (which `AGENTS.md` points to) summarizes these rules for every Claude and Codex session.
