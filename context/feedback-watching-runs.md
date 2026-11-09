---
name: feedback-watching-runs
description: Read before showing a watched simulation or replay; share the unified dashboard link and leave playback controls to the user
metadata:
  type: feedback
  updated: 2026-11-09T05:26:28Z
  covers: []
---

Use [[unified-viewer]] for watched simulations and replays. Give its relevant view link so the person opens it in their own browser. Do not open the browser pane or press HIM's Start for them. Keep the required backend viewer running; the dashboard wraps it rather than replacing its process.

Runs must not wait for the person: `sim.sh` starts immediately, and HIM's Start plays from the first frame. For `course.py --view`, share the dashboard link when each run's backend is available. The printed backend URL is for checking the configured port, not the default link to share.

Close backend viewers after use, especially before another run. Going Home or closing an iframe does not free their memory. Flags, replay launch and deadline recovery belong in [[course-runs]] and [[sim-deadline-stops]].
