---
name: feedback-watching-runs
description: Read before running a sim run someone wants to watch (sim.sh, course.py --view) — give them the viewer link, don't drive the browser pane
metadata:
  type: feedback
  updated: 2026-10-30T00:07:39Z
  covers: []
---

Don't open the browser pane or press Start on HIM's viewer for the user. When a watched run's viewer is up, give its link (http://127.0.0.1:8765, or http://localhost:8765, the port `sim.sh` or `course.py --view` prints) and say it is waiting for Start, so they open it and press Start themselves. With `course.py --view`, give each run's link as it comes up; a run nobody starts goes ahead unseen after 2 minutes ([[submission-image]]).

**Why:** the user asked for this on 2026-10-30; they watch runs in their own browser.
