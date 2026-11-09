---
name: architecture-guide
description: Read when someone wants the architecture explained or a shareable document made, or before changing processes, topics or folders the guide describes
metadata:
  type: project
  updated: 2026-11-09T03:43:46Z
  covers: [docs/architecture/**, submission/entrypoint.sh]
---

`docs/architecture/` is the source of a PDF guide to the whole repo: what each part does, how the parts connect (9 diagrams), and a table of what is used, opt-in, separate, duplicated or unused. Build it with `python3 docs/architecture/build.py` (`--html` keeps the page too).

**Files:** `content.html` is the text; each `<figure data-diagram="NAME">` gets `diagrams.py`'s `NAME()` drawing (hand-placed SVG on a 700-wide frame: `box`, `arrow`, `label`, `legend`); `style.css` is the page; `build.py` stitches them and prints with headless Chrome, Chromium or Edge (`CHROME=` to pick one). Standard library only.

**Rules (a teammate's choice, 2026-11-08):** documents to share are made this way, as a local PDF built from committed source, not as Claude Docs artifacts. Commit only the source: `architecture.pdf` and `architecture.html` are gitignored. When a change alters what the guide says (a process `entrypoint.sh` starts, a topic, a folder, the used/duplicated table), update `content.html` or `diagrams.py` in the same commit.

**Gotchas**
- Headless Chrome on macOS can keep running after it writes the PDF; `build.py` waits for the file to stop growing, then ends it.
- To look at the pages without poppler on macOS, render them with PDFKit from a short Swift script (`PDFPage.thumbnail`), then view the PNGs. Check every diagram after a change: labels are placed by hand and can land on a line.
- Writing follows [[readme-and-memory-guidelines]]: bullets and tables, no dashes or arrows in prose.
