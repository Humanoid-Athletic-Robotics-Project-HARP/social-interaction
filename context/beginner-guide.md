---
name: beginner-guide
description: Read only when someone is new to the repo (just cloned it, "how do I get started") — the order to walk them through the README
metadata:
  type: reference
  updated: 2026-10-28T09:48:21Z
  covers: [README.md]
---

Walk the newcomer through one stage at a time, and ask which they need: the sim (laptop) or the robot. The README has the commands; point to its section for each stage instead of retyping it.

**0. Orientation:** the robot and team ([[project-overview]]), the house rule ([[feedback-minimal-setup]]), and that `context/` is shared agent memory indexed in `context/MEMORY.md`.

**1. Sim (start here).** Ask their OS first ([[team-context]]).
1. README "Setup": HIM's checkout next to this repo, its Python environment, the Holosoma checkpoint, Docker Desktop.
2. README "Run it in the sim": `submission/build.sh`, then a move with `G1_TASK_COMMAND` in `submission/sim.sh`, watched in the browser (press Start). HIM's GR00T balance holds the robot for 5 s first ([[planned-moves]]).
3. A course: `python3 submission/course.py one_box`, then `--compare` ([[submission-image]], [[nav2-sim]]).

**2. Robot (not tested on hardware yet).** README "On the robot" and [[safety]], plus [[g1-official-docs]] for the network and login. Plan the first session with someone on the gantry rope, and don't reflash the Jetson without asking the team ([[unitree-resources]]).

**3. Next:** [[him-sim2real]], [[off-robot-testing]], [[g1-builtin-apis]].
