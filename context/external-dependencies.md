---
name: external-dependencies
description: Read before adding, borrowing from or updating any outside code, package or model weights: the no-copying rule, which licenses are fine, and where each dependency and its license is recorded
metadata:
  type: feedback
  updated: 2026-11-09T04:02:54Z
  covers: [README.md, speech/requirements.txt, testground/requirements.txt, submission/Dockerfile, third_party/README.md]
---

**Never copy code from other projects** (a teammate's rule, 2026-11-02). Use outside work only as an external dependency with a permissive license:
- a pinned package (`requirements.txt`, the `Dockerfile`)
- a pinned download checked by its hash (models)
- a pinned submodule (`third_party/`)
- HIM's checkout, used as it is ([[him-sim2real]])

Projects with no license are all rights reserved: read their write-ups for ideas and facts, never their code. The team publishes this repo as open source, so everything it ships must be redistributable.

**Licenses**
- **Fine:** MIT, BSD, Apache-2.0, and similar permissive ones.
- **Attribution:** CC-BY needs credit: the author, the license, a link, and what was changed (e.g. converted to ONNX).
- **Avoid in what we run or ship:**
  - copyleft (GPL, AGPL; e.g. the `piper-tts` Python package, GPL-3.0)
  - non-commercial terms
  - licenses that limit where it may be used (no-compete clauses, "only inside product X")
- **Reference only:** AGPL code kept outside the image, like the Ultralytics fork in `third_party/`, is a reference, not a dependency.
- **Model weights** often have their own license, apart from the code's: check both.

**Before adding one:** read its `LICENSE` file (and the weights' license), and pin the exact version, commit or file hash.

**Where it is recorded**
- The README's "Licenses" section: everything in the repo, in our image, or that our code downloads and runs. Tables of project (with its link) and license, grouped by where it's used; no reasons or descriptions. CC-BY entries keep the author and what was changed in the project column (Parakeet: converted to int8 ONNX). Update it in the same commit as the dependency.
- Code kept in the repo keeps its `LICENSE` and a patch log (`submission/nav2_graceful_controller/HARP_PATCHES.md`).
- Submodules: `third_party/README.md`.
- Each folder's README gives the setup commands; model weights are never committed. They go in `harp/models/` beside the main checkout ([[worktrees]]).

License findings by topic: [[speech-research]] (speech packages and models), [[gr00t-wbc]] (NVIDIA Open Model License), [[person-perception-sources]].
