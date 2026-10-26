---
name: readme-and-memory-guidelines
description: How to write and update the README (front page, Goals and progress) and the memories in context/ (style, one place per fact, updated/covers front matter, the staleness check); read before editing either
metadata:
  type: feedback
  updated: 2026-10-26T21:43:06Z
  covers: [context/check_memory.py]
---

Every rule for writing and updating `README.md` and the memories in `context/` lives here (merged on 2026-10-26 at a teammate's request from three notes: README style, the goals workflow, memory style).

## README

`README.md` is the front page: newcomers and the project maintainer read it, so write it for them.
- **Say what the robot does and has.** Leave out what it doesn't do, what we don't use, and comparisons with what we replaced (no "not Unitree's sport mode"). Safety steps stay, phrased as what to do ("keep the robot supported").
- **Only the correct way, not what fails.** Give the right command (e.g. `pixi shell --as-is`) without explaining the failure it avoids; the reason and the symptom go in `context/` (here [[ros2-mac]]). A teammate asked for this on 2026-10-26.
- **No process notes:** nothing like "ask the team" or "ask the mentors", and no "draft" labels. HARP is a robotics team, so never "course" or "course staff".
- **Short.** Commands with a line on what they do; bullets over paragraphs; the stand-up sequence explained once and referred to elsewhere.
- **Detail lives in `context/`**: measured results, limits, edge cases, design reasons and gotchas. The README's "More detail" line points there, and Claude or Codex summarize it on request.
- **Update it in the same change as the code** it describes: new commands, flags, topics, or a section that's now wrong.

Why: a teammate trimmed the README from about 500 to 270 lines on 2026-10-26, moving the detail into memory, and asked for it to focus on what the robot does.

## README "Goals and progress"

The project's goals and their status live only there. Don't keep a second goal or status list anywhere, memory included; other notes link to it ([[project-overview]]).
- **Format** (a teammate's rules, 2026-10-26):
  - A few **wide goals**, each a `###` heading in plain words ("Move like a person", "Get around obstacles"). No numbers. New work goes under an existing goal; add a goal only for something none of them covers.
  - A summary table at the top: Goal | Sim | Robot | Next step. Sim and robot are tracked separately, because nothing counts as done on the robot until it has run there.
  - Under each goal, **bullets only**: first a "Done when:" bullet (the observable result), then one bullet per milestone, starting with ✅ done, 🟡 in progress, ⬜ not started or ⛔ blocked, and "Sim:" or "Robot:" where it matters.
  - Short and illustrative: name the command or file (`move_distance`, `walker_node`) rather than explaining it.
- **When to update:** in the same commit as the work. Tick a milestone's bullet and update the table row when it's reached. Mark work 🟡 when it starts, and ⛔ with the blocker when blocked. There's no progress log; git history covers that.
- **What needs the team:** adding, removing, merging or redefining a goal, or changing a "Done when", is the team's call (goals set 2026-10-26): propose and ask, don't just edit. Ticking milestones needs no approval.

## Writing memories

- **Direct instructions and facts:** how to do things, and what is true. No notes about a mistake one teammate made once ("the most common mix-up"), unless it's genuinely common and expected to recur. When someone hits a problem, fix the code or docs if that helps everyone, and keep memory to the plain instruction. (A teammate's request, 2026-10-25.)
- **Say "a teammate"**, not "the user", for decisions and approvals, and name the OS or machine for anything platform-specific ([[team-context]]).
- **Dates:** absolute (2026-10-26), never "today" or "yesterday".
- **One place per fact.** Each fact lives in one memory; others link to it with `[[name]]` instead of repeating it. Copies drift: on 2026-10-26 `pixi shell` (without `--as-is`) survived in four memories after the fix, and a file name in two.
- **Results name their run folder** (`runs/...`, pinned) when they come from a recorded run ([[run-logging]]).
- Front matter: `name` (the file name without `.md`), `description` (one line, used to judge relevance), `metadata.type` (user, feedback, project or reference), `metadata.updated` and `metadata.covers` (below). Add every memory to `context/MEMORY.md`, one line each, and keep that line in step with the file.

## Keeping memories current

`context/check_memory.py` (plain Python 3 and git) finds memories that may be out of date, so nobody rereads every memory.
- **Front matter:**
  - `metadata.updated`: when the memory was last written or confirmed, **UTC only**, `2026-10-26T21:30:00Z`. Written by the script, never by hand; it rejects any other form (a `-07:00` offset, a bare date).
  - `metadata.covers`: the repo paths or globs whose changes can make the memory wrong, e.g. `[ros2/src/harp_ros/config/nav2_params.yaml, ros2/src/harp_sim/worlds/*.xml]`. Only the script reads it. `[]` is deliberate for memories about things outside the repo (web docs, research, team facts, a local branch): no repo change can make those stale, so the script skips them.
- **Reading memory:** before relying on a memory, run `python3 context/check_memory.py` once per session. It prints "All memory current" when no memory is flagged, and otherwise one line per memory whose covered paths changed after its `updated` time.
  - Act only on listed memories your task relies on. Read their covered files instead of trusting them, and fix them if the task touches that topic: you've just read the source, so it's the cheapest moment.
  - If one is still right, confirm it with `--touch`.
  - Leave unrelated listed memories alone. They stay listed until someone works on that topic.
- **Editing memory:** after editing a memory, run `python3 context/check_memory.py --touch FILE` to stamp `updated`. When a memory starts describing new code, add those paths to its `covers`.
- **Changing code:** update the memories that cover it, and the README if it's affected, in the same commit.
- **Optional pre-commit hook** (not installed; a teammate's call): `python3 context/check_memory.py --pre-commit` from `.git/hooks/pre-commit`. It stamps `updated` on memories in each commit and warns (never blocks) about covered files committed without their memory.
- The same rules are summarized in `CLAUDE.md`, which every Claude and Codex session loads (a teammate's request, 2026-10-26).
