#!/usr/bin/env python3
"""Find memories in context/ that may be out of date. Times are UTC only.

Each memory's front matter says when it was last confirmed true and what it
describes:

  metadata:
    type: reference
    updated: 2026-10-26T21:30:00Z        # UTC, written by this script, never by hand
    covers: [ros2/src/harp_ros/config/nav2_params.yaml, ros2/src/harp_sim/worlds/*.xml]

A memory is flagged when something it covers changed after its `updated`
time: in a commit that didn't also change the memory, or uncommitted while
the memory itself is unchanged. It prints "All memory current" when none are flagged.

  python3 context/check_memory.py               # once per session, before relying on a memory
  python3 context/check_memory.py --touch FILE  # after editing a memory, or confirming it's right
  python3 context/check_memory.py --pre-commit  # for an optional git pre-commit hook: stamps
                                                # `updated` on memories in the commit and warns
                                                # about covered files committed without them
See context/readme-and-memory-guidelines.md.

Plain Python 3 and git, no other dependencies, so it runs on any machine.
"""

import argparse
import datetime
import re
import subprocess
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
CONTEXT = REPO / "context"
UTC = datetime.timezone.utc
STAMP = re.compile(r"^\d{4}-\d{2}-\d{2}T\d{2}:\d{2}:\d{2}Z$")   # the only accepted form


def git(*args):
    return subprocess.run(["git", "-C", str(REPO), *args], capture_output=True, text=True).stdout


def now():
    return datetime.datetime.now(UTC).strftime("%Y-%m-%dT%H:%M:%SZ")


def front_matter(path):
    """(updated as a UTC datetime, or an error string; covers list or None) of one memory."""
    text = path.read_text()
    head = text.split("---", 2)[1] if text.startswith("---") else ""
    updated = re.search(r"^\s*updated:\s*(\S+)", head, re.M)
    covers = re.search(r"^\s*covers:\s*\[(.*)\]", head, re.M)
    paths = [p.strip() for p in covers.group(1).split(",") if p.strip()] if covers else None
    if not updated:
        return "no `updated`", paths
    if not STAMP.match(updated.group(1)):
        return f"`updated: {updated.group(1)}` isn't UTC in the form 2026-10-26T21:30:00Z", paths
    return datetime.datetime.strptime(updated.group(1), "%Y-%m-%dT%H:%M:%SZ").replace(tzinfo=UTC), paths


def specs(covers):
    return [f":(glob){p}" for p in covers]


def commits_touching(pathspecs):
    """(hash, UTC commit time, subject) of commits that changed these paths, newest first."""
    out = git("log", "--format=%H%x09%ct%x09%s", "--", *pathspecs)
    return [(h, datetime.datetime.fromtimestamp(int(t), UTC), s)
            for h, t, s in (line.split("\t", 2) for line in out.splitlines())]


def dirty(pathspecs):
    """Files with uncommitted changes (new files included) among these paths."""
    out = git("status", "--porcelain", "--untracked-files=all", "--", *pathspecs)
    return [line[3:] for line in out.splitlines()]


def memories():
    return [p for p in sorted(CONTEXT.glob("*.md")) if p.name != "MEMORY.md"]


def check(memory):
    """Reasons this memory may be out of date; empty if it looks current."""
    updated, covers = front_matter(memory)
    if isinstance(updated, str):
        return [updated]
    if covers is None:
        return ["no `covers`"]
    if not covers:
        return []
    rel = str(memory.relative_to(REPO))
    reasons = []
    changed = dirty(specs(covers))
    if changed and not dirty([rel]):
        reasons.append("uncommitted changes to " + ", ".join(changed[:4])
                       + (f" and {len(changed) - 4} more" if len(changed) > 4 else ""))
    with_memory = {h for h, _, _ in commits_touching([rel])}
    for h, when, subject in commits_touching(specs(covers)):
        if when <= updated:
            break
        if h not in with_memory:
            reasons.append(f"commit {h[:7]} ({when:%Y-%m-%dT%H:%MZ}) \"{subject}\"")
    return reasons


def report(flagged):
    lines = ["Memories that may be out of date (what they cover changed after their `updated`, UTC):"]
    for memory, reasons in flagged:
        lines.append(f"- {memory.relative_to(REPO)}: " + "; ".join(reasons))
    lines.append("Only the ones your task relies on matter: read their covered files instead of "
                 "trusting them, fix them if the task touches that topic, and run "
                 "`python3 context/check_memory.py --touch FILE` after fixing or confirming one.")
    return "\n".join(lines)


def stamp(paths, when):
    for path in paths:
        text = path.read_text()
        if re.search(r"^(\s*)updated:\s*\S+", text, re.M):
            text = re.sub(r"^(\s*)updated:\s*\S+", rf"\g<1>updated: {when}", text, count=1, flags=re.M)
        else:
            text = re.sub(r"^(\s*)type:(.*)$", rf"\g<0>\n\g<1>updated: {when}", text, count=1, flags=re.M)
        path.write_text(text)


def pre_commit():
    staged = git("diff", "--cached", "--name-only").splitlines()
    staged_memories = [REPO / p for p in staged
                       if p.startswith("context/") and p.endswith(".md") and p != "context/MEMORY.md"
                       and (REPO / p).exists()]
    if staged_memories:
        stamp(staged_memories, now())
        git("add", *[str(p.relative_to(REPO)) for p in staged_memories])
    names = {str(m.relative_to(REPO)) for m in staged_memories}
    missing = []
    for memory in memories():
        rel = str(memory.relative_to(REPO))
        _, covers = front_matter(memory)
        if not covers or rel in names:
            continue
        hit = git("diff", "--cached", "--name-only", "--", *specs(covers)).split()
        if hit:
            missing.append(f"- {rel} covers {', '.join(hit[:3])}")
    if missing:
        print("check_memory: this commit changes files these memories describe, but not the "
              "memories (update them if they're now wrong):\n" + "\n".join(missing), file=sys.stderr)
    return 0


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--pre-commit", action="store_true")
    mode.add_argument("--touch", nargs="+", metavar="FILE", help="set updated to now (UTC)")
    args = parser.parse_args()
    if args.pre_commit:
        return pre_commit()
    if args.touch:
        when = now()
        stamp([Path(p) for p in args.touch], when)
        print(f"updated: {when}")
        return 0
    flagged = [(m, r) for m in memories() if (r := check(m))]
    print(report(flagged) if flagged else "All memory current")
    return 0


if __name__ == "__main__":
    sys.exit(main())
