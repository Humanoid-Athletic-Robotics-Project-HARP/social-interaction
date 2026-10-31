---
name: feedback-watching-runs
description: Read before running a sim run someone wants to watch (sim.sh, course.py --view) — give them the viewer link, don't drive the browser pane
metadata:
  type: feedback
  updated: 2026-10-30T23:46:10Z
  covers: []
---

Don't open the browser pane or press Start on HIM's viewer for the user. When a watched run's viewer is up, give its link (http://127.0.0.1:8765, or http://localhost:8765, the port `sim.sh` or `course.py --view` prints) so they open it and press Start themselves (it plays the run from its first frame). Runs never wait for that: `sim.sh` starts each run at once ([[submission-image]]). With `course.py --view`, give each run's link as it comes up.

**Why:** the user asked for this on 2026-10-30; they watch runs in their own browser, and are often away, so a run must not wait for them.
