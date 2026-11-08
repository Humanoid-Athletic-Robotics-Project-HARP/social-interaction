"""Build the architecture guide as a PDF (gitignored) from this folder's sources:
  content.html  the text; each <figure data-diagram="NAME"> gets diagrams.NAME()'s drawing
  diagrams.py   the drawings, as SVG
  style.css     the page style

  python3 docs/architecture/build.py            writes docs/architecture/architecture.pdf
  python3 docs/architecture/build.py --html     also keeps the page as architecture.html

Prints with headless Chrome, Chromium or Edge (CHROME=/path/to/browser to pick one).
Standard library only.
"""
import argparse
import datetime
import os
import re
import shutil
import subprocess
import sys
import tempfile
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import diagrams  # noqa: E402

BROWSERS = [
    "/Applications/Google Chrome.app/Contents/MacOS/Google Chrome",
    "/Applications/Chromium.app/Contents/MacOS/Chromium",
    "/Applications/Microsoft Edge.app/Contents/MacOS/Microsoft Edge",
    r"C:\Program Files\Google\Chrome\Application\chrome.exe",
    r"C:\Program Files (x86)\Google\Chrome\Application\chrome.exe",
    r"C:\Program Files (x86)\Microsoft\Edge\Application\msedge.exe",
]
ON_PATH = ["google-chrome", "google-chrome-stable", "chromium", "chromium-browser", "microsoft-edge", "msedge"]


def browser():
    if os.environ.get("CHROME"):
        return os.environ["CHROME"]
    for path in BROWSERS:
        if Path(path).exists():
            return path
    for name in ON_PATH:
        if shutil.which(name):
            return shutil.which(name)
    sys.exit("No Chrome, Chromium or Edge found: set CHROME=/path/to/browser")


def commit():
    try:
        return subprocess.run(["git", "rev-parse", "--short", "HEAD"], cwd=HERE, capture_output=True,
                              text=True, check=True).stdout.strip()
    except (OSError, subprocess.CalledProcessError):
        return "unknown"


def page():
    body = (HERE / "content.html").read_text(encoding="utf-8")

    def figure(m):
        draw = getattr(diagrams, m.group(1), None)
        if draw is None:
            sys.exit(f"content.html names a diagram diagrams.py doesn't have: {m.group(1)}")
        return m.group(0) + draw()

    body = re.sub(r'<figure data-diagram="(\w+)">', figure, body)
    body = body.replace("{{DATE}}", datetime.date.today().isoformat()).replace("{{COMMIT}}", commit())
    css = (HERE / "style.css").read_text(encoding="utf-8")
    return ('<!doctype html><html lang="en"><head><meta charset="utf-8">'
            '<title>HARP social interaction: architecture guide</title>'
            f"<style>{css}</style></head><body>{body}</body></html>")


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--html", action="store_true", help="also keep architecture.html")
    parser.add_argument("--out", default=str(HERE / "architecture.pdf"), help="PDF path")
    args = parser.parse_args()

    html = page()
    with tempfile.TemporaryDirectory() as tmp:
        src = Path(tmp) / "architecture.html"
        src.write_text(html, encoding="utf-8")
        if args.html:
            shutil.copy(src, HERE / "architecture.html")
        out = Path(args.out).resolve()
        out.unlink(missing_ok=True)
        chrome = subprocess.Popen(
            [browser(), "--headless=new", "--disable-gpu", "--no-first-run", "--no-default-browser-check",
             "--no-pdf-header-footer", "--print-to-pdf-no-header", f"--user-data-dir={Path(tmp) / 'profile'}",
             f"--print-to-pdf={out}", src.as_uri()],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        print_done(chrome, out)
    print(out)


def print_done(chrome, out, limit=60.0):
    """Wait for the PDF, then end the browser: headless Chrome on macOS can stay running after
    it has written the file."""
    start, size = time.monotonic(), -1
    while time.monotonic() - start < limit:
        if chrome.poll() is not None and not out.exists():
            sys.exit("The browser exited without writing the PDF")
        if out.exists() and out.stat().st_size == size and size > 0:
            break
        size = out.stat().st_size if out.exists() else -1
        time.sleep(1.0)
    else:
        chrome.kill()
        sys.exit(f"No PDF after {limit:.0f} s")
    if chrome.poll() is None:
        chrome.terminate()
        try:
            chrome.wait(5)
        except subprocess.TimeoutExpired:
            chrome.kill()


if __name__ == "__main__":
    main()
