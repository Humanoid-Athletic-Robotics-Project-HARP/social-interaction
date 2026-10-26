---
name: feedback-readme-style
description: How to write README.md: a short front page for newcomers and the maintainer, describing what the robot does and has; detail goes in context/
metadata:
  type: feedback
---

`README.md` is the front page: newcomers and the project maintainer read it, so write it for them.
- **Say what the robot does and has.** Leave out what it doesn't do, what we don't use, and comparisons with what we replaced (e.g. no "not Unitree's sport mode"). Safety steps stay, phrased as what to do ("keep the robot supported").
- **No process notes:** nothing like "ask the team", "decide with the team" or "ask the mentors", and no "draft" labels. HARP is a robotics team, so never "course" or "course staff".
- **Short.** Commands with a line on what they do; bullets over paragraphs; the stand-up sequence explained once and referred to elsewhere.
- **Detail lives in `context/`**: measured results, limits, edge cases, design reasons and gotchas. The README's "More detail" line points there, and Claude or Codex summarize it on request. Goals follow [[goals-workflow]].

**Why:** A teammate trimmed the README from about 500 to 270 lines on 2026-10-26, moving the detail into memory ([[move-distance]], [[ros2-mac]]), and asked for it to focus on what the robot does.
