"""Look through recorded runs (runs/, see run_recorder.py) and compare them.

  ros2 run harp_eval runs list [--world slalom] [--tag rangecut] [--last 20]
  ros2 run harp_eval runs show RUN            # summary.json of one run (name or unique part)
  ros2 run harp_eval runs compare --world slalom [--tag a --tag b] [--plot paths.png]
  ros2 run harp_eval runs check [--world slalom] [--tag a]   # pass rate against the course criteria
  ros2 run harp_eval runs resummarize RUN... | --all # recompute summaries (after adding a metric)
  ros2 run harp_eval runs set RUN tether=gantry floor_surface=carpet battery_percent=80
                                 manual.end_position_measured="2.95 0.10" note="..."
  ros2 run harp_eval runs pin RUN / unpin RUN        # pinned runs are never pruned
  ros2 run harp_eval runs prune [--dry-run]          # apply the retention limits in logging.yaml
"""

import argparse
import json
import statistics
import sys

from harp_control import runlog
from harp_eval import run_summary
from harp_eval.run_recorder import load_config


def summaries(world=None, tags=None, last=None):
    found = []
    for run in run_summary.run_dirs(runlog.runs_dir()):
        path = run / "summary.json"
        if not path.exists():
            continue
        s = json.loads(path.read_text())
        if (world and s.get("world") != world) or (tags and s.get("tag") not in tags):
            continue
        found.append((run, s))
    return found[-last:] if last else found


def find_run(text):
    matches = [r for r in run_summary.run_dirs(runlog.runs_dir()) if text in r.name]
    if len(matches) != 1:
        raise SystemExit(f"{len(matches)} runs match {text!r}; give more of the name")
    return matches[0]


def fmt(v, spec="{:.2f}"):
    return "-" if v is None else spec.format(v)


def cmd_list(args):
    rows = summaries(args.world, args.tag, args.last)
    print(f"{'run':52} {'outcome':10} {'pass':>5} {'time':>6} {'path':>6} {'closest':>8} {'contacts':>8} {'recov':>5}")
    for run, s in rows:
        passed = {True: "yes", False: "NO", None: "-"}[s.get("passed")]
        print(f"{run.name:52} {s['outcome']:10} {passed:>5} {fmt(s.get('duration_s'), '{:.0f}'):>6} "
              f"{fmt(s.get('path_m')):>6} {fmt(s.get('closest_m')):>8} {s.get('contacts', 0):>8} "
              f"{fmt(s.get('recoveries'), '{}'):>5}")


def cmd_show(args):
    run = find_run(args.run)
    print(run_summary.one_screen(json.loads((run / "summary.json").read_text())))
    print(f"\n{run}")


def spread(values, spec="{:.2f}"):
    values = [v for v in values if v is not None]
    if not values:
        return "-"
    if len(values) == 1:
        return spec.format(values[0])
    return f"{spec.format(statistics.mean(values))} ± {spec.format(statistics.stdev(values))}"


def cmd_compare(args):
    rows = summaries(args.world, args.tag, args.last)
    groups = {}
    for run, s in rows:
        groups.setdefault((s.get("world"), s.get("tag")), []).append((run, s))
    print(f"{'world':16} {'tag':16} {'n':>3} {'ok':>3} {'err':>3} {'time s':>13} {'path m':>13} "
          f"{'closest m':>13} {'contacts':>8} {'recoveries':>12}")
    print("(means and spreads leave out runs that ended in an error or were stopped)")
    for (world, tag), group in groups.items():
        everything = [s for _, s in group]
        ss = [s for s in everything if s["outcome"] not in ("error", "stopped")]
        print(f"{world or '-':16} {tag or '-':16} {len(everything):>3} "
              f"{sum(s['outcome'] == 'succeeded' for s in everything):>3} "
              f"{len(everything) - len(ss):>3} "
              f"{spread([s.get('duration_s') for s in ss], '{:.0f}'):>13} "
              f"{spread([s.get('path_m') for s in ss]):>13} "
              f"{spread([s.get('closest_m') for s in ss]):>13} "
              f"{sum(s.get('contacts', 0) for s in ss):>8} "
              f"{spread([s.get('recoveries') for s in ss], '{:.1f}'):>12}")
    if args.plot:
        plot(groups, args.plot)


def plot(groups, out):
    """True paths of every run, one colour per tag, over the course's obstacles."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.patches as patches
    import matplotlib.pyplot as plt
    import mujoco
    import numpy as np
    from harp_sim.movers import Movers
    from harp_sim.run_sim import load_model

    worlds = sorted({w for w, _ in groups}, key=str)
    fig, axes = plt.subplots(len(worlds), 1, figsize=(10, 5 * len(worlds)), squeeze=False)
    for ax, world in zip(axes[:, 0], worlds):
        drawn = False
        for i, ((w, tag), group) in enumerate(groups.items()):
            if w != world:
                continue
            for j, (run, s) in enumerate(group):
                truth = run_summary.load_truth(run)
                meta = run_summary.read_meta(run)
                if truth is None:
                    continue
                if not drawn and (meta.get("sim") or {}).get("scene"):
                    sim = meta["sim"]
                    model = load_model(sim["scene"], [tuple(b) for b in sim.get("boxes") or []],
                                       sim.get("world"))
                    data = mujoco.MjData(model)
                    mujoco.mj_kinematics(model, data)
                    movers = Movers(model)
                    for start, end in zip(movers.start, movers.end):
                        ax.plot([start[0], end[0]], [start[1], end[1]], color="0.5", ls="--",
                                lw=2, label="moving obstacle's track")
                    for g in range(model.ngeom):
                        if model.geom_bodyid[g] != 0 or model.geom(g).name == "floor":
                            continue
                        pos, size = data.geom_xpos[g], model.geom_size[g]
                        if model.geom_type[g] == mujoco.mjtGeom.mjGEOM_CYLINDER:
                            ax.add_patch(patches.Circle(pos[:2], size[0], color="0.5"))
                        elif model.geom_type[g] == mujoco.mjtGeom.mjGEOM_BOX:
                            R = data.geom_xmat[g].reshape(3, 3)[:2, :2]
                            ax.add_patch(patches.Polygon([pos[:2] + R @ np.array([a * size[0], b * size[1]])
                                                          for a, b in ((-1, -1), (1, -1), (1, 1), (-1, 1))],
                                                         color="0.5"))
                    drawn = True
                xy = run_summary.smoothed_xy(truth["qpos"])
                ax.plot(xy[:, 0], xy[:, 1], color=f"C{i}", alpha=0.8,
                        label=f"{tag or '-'} ({len(group)} runs)" if j == 0 else None)
        ax.set_title(world or "no world")
        ax.set_aspect("equal")
        ax.grid(alpha=0.3)
        ax.legend(loc="upper left", fontsize=8)
    fig.tight_layout()
    fig.savefig(out, dpi=120)
    print(f"Plot: {out}")


def cmd_check(args):
    """Pass rate per course and tag, and which criteria failed in which runs."""
    groups = {}
    for run, s in summaries(args.world, args.tag, args.last):
        if s.get("check"):
            groups.setdefault((s.get("world"), s.get("tag")), []).append((run, s))
    if not groups:
        print("No course runs with criteria (harp_sim/worlds/<world>.yaml) match.")
        return
    for (world, tag), group in groups.items():
        passed = sum(s["passed"] for _, s in group)
        print(f"{world} [{tag or '-'}]: {passed}/{len(group)} passed "
              f"(criteria {group[0][1]['check']['criteria']})")
        for run, s in group:
            if not s["passed"]:
                failed = ", ".join(f"{k} {s['check']['results'][k]['value']} "
                                   f"(limit {s['check']['results'][k]['limit']})"
                                   for k in s["check"]["failed"])
                print(f"  FAIL {run.name}: {failed}")


def cmd_set(args):
    """Edit run.yaml fields afterwards: conditions by name, or dotted keys (manual.video)."""
    from harp_eval.run_recorder import CONDITIONS
    run = find_run(args.run)
    meta = run_summary.read_meta(run)
    for item in args.values:
        key, _, value = item.partition("=")
        if key in CONDITIONS:
            key = "conditions." + key
        if key == "manual.end_position_measured":
            value = [float(v) for v in value.split()]
        target = meta
        *parents, leaf = key.split(".")
        for part in parents:
            if not isinstance(target.get(part), dict):
                target[part] = {}
            target = target[part]
        target[leaf] = value
        print(f"{key} = {value}")
    run_summary.write_meta(run, meta)
    run_summary.summarize(run)


def cmd_summarize(args):
    runs = run_summary.run_dirs(runlog.runs_dir()) if args.all else [find_run(r) for r in args.runs]
    for run in runs:
        print(run_summary.one_screen(run_summary.summarize(run)))


def cmd_pin(args, pinned):
    run = find_run(args.run)
    meta = run_summary.read_meta(run)
    meta["pinned"] = pinned
    run_summary.write_meta(run, meta)
    run_summary.rebuild_index(run.parent)
    print(f"{'Pinned' if pinned else 'Unpinned'} {run.name}")


def cmd_prune(args):
    config = load_config()[1]
    actions = run_summary.prune(runlog.runs_dir(), config["retention"], config["replay"], args.dry_run)
    for run, what, _ in actions:
        print(f"{'would ' if args.dry_run else ''}{what}: {run.name}")
    if not actions:
        print("Nothing to prune.")


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    sub = parser.add_subparsers(dest="command", required=True)
    for name in ("list", "compare"):
        p = sub.add_parser(name)
        p.add_argument("--world")
        p.add_argument("--tag", action="append", help="repeatable")
        p.add_argument("--last", type=int, help="only the last N matching runs")
        if name == "compare":
            p.add_argument("--plot", metavar="FILE.png")
    sub.add_parser("show").add_argument("run")
    for name in ("summarize", "resummarize"):
        p = sub.add_parser(name)
        p.add_argument("runs", nargs="*")
        p.add_argument("--all", action="store_true")
    p = sub.add_parser("check")
    p.add_argument("--world")
    p.add_argument("--tag", action="append", help="repeatable")
    p.add_argument("--last", type=int, help="only the last N matching runs")
    p = sub.add_parser("set")
    p.add_argument("run")
    p.add_argument("values", nargs="+", metavar="KEY=VALUE")
    sub.add_parser("pin").add_argument("run")
    sub.add_parser("unpin").add_argument("run")
    sub.add_parser("prune").add_argument("--dry-run", action="store_true")
    args = parser.parse_args()
    if not runlog.runs_dir().exists():
        sys.exit(f"No runs yet ({runlog.runs_dir()} doesn't exist).")
    run_summary.rebuild_index(runlog.runs_dir())
    {"list": cmd_list, "show": cmd_show, "compare": cmd_compare, "summarize": cmd_summarize,
     "resummarize": cmd_summarize, "check": cmd_check, "set": cmd_set,
     "pin": lambda a: cmd_pin(a, True), "unpin": lambda a: cmd_pin(a, False),
     "prune": cmd_prune}[args.command](args)


if __name__ == "__main__":
    main()
