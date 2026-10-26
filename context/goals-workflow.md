---
name: goals-workflow
description: How to read and edit the project's goals and progress (README "Goals and progress"): format rules, when to update, what needs team agreement
metadata:
  type: feedback
---

The project's goals and their status live in one place: the README's **"Goals and progress"** section. Don't keep a second goal or status list anywhere else, memory included; other notes link to it ([[project-overview]]).

**Format** (a teammate's rules, 2026-10-26):
- A few **wide goals**, each a `###` heading in plain words ("Move like a person", "Get around obstacles"). **No numbers.** Put new work under an existing goal; add a goal only for something none of them covers.
- A summary table at the top: Goal | Sim | Robot | Next step. Sim and robot are tracked separately, because nothing counts as done on the robot until it has run there.
- Under each goal, **bullets only, no paragraphs**: first a "Done when:" bullet (the observable result), then one bullet per milestone, starting with ✅ done, 🟡 in progress, ⬜ not started or ⛔ blocked, and "Sim:" or "Robot:" where it matters.
- Keep it **short and illustrative for humans**. Name the command or file (`move_distance`, `walker_node`) rather than explaining it. Measurements, how-tos and reasons belong in the code, the other README sections and memory; Claude can summarize them on request.

**When to update:** in the same commit as the work. When a milestone is reached, tick its bullet and update the table row (status and "Next step"). There's no progress log: git history covers that. When work starts, mark it 🟡; when something blocks it, ⛔ with the blocker in a few words.

**What needs the team:** the goals were set on 2026-10-26. Adding, removing, merging or redefining a goal, or changing a "Done when", is the team's call: propose it and ask, don't just edit it. Ticking milestones needs no approval. The README itself carries no "ask the team" or draft notes ([[feedback-readme-style]]).

**Why:** A teammate asked for goals that people can read at a glance, with detail kept in the code and memory (2026-10-26). See [[feedback-memory-style]] for how the rest of memory is written.
