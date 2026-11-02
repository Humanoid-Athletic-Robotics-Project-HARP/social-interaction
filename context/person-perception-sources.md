---
name: person-perception-sources
description: Read before evaluating or updating the pinned person-detector/tracker forks
metadata:
  type: reference
  updated: 2026-11-01T03:22:39Z
  covers: [.gitmodules, third_party/**]
---

Team forks of `ultralytics/ultralytics` and `NirAharon/BoT-SORT` are pinned as Git submodules under `third_party/`. Checkout, upstream links and licenses are in `third_party/README.md`; Git records their exact revisions. Upstream code and notices stay intact.

These are source references for later evaluation, outside the submission image/build path. No detector, tracker or model weights have been installed or benchmarked on the target. Ultralytics already includes a BoT-SORT integration; the standalone fork is an additional reference, not a requirement to install both runtimes.

Image IDs/boxes do not supply a spatial target: validate camera/depth, timestamps/transforms, target identity/freshness and approach-goal behavior before motion integration. This source preparation does not implement Lam's learner-owned tracking/prediction work.
