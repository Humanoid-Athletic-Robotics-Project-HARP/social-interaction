#!/usr/bin/env python3
"""A fast closed-loop benchmark of the predictive planner cores, without HIM or physics.

    python3 submission/planner_bench.py --cores mpc game --seeds 1-40 --crowds aware scripted

Each scenario is a generated world (generate_world.py, seeds of its own: never a held-out
seed) or a named world. In it:
- the walkers are the same code HIM's runs use (harp/movers.py: AwareMovers give way,
  scripted Movers don't), driven by the robot's true position;
- the robot is a kinematic G1: HIM's measured GR00T response (0.85 forward, 0.67 sideways),
  0.1 s delay, 0.25 s lag, acceleration limits;
- the route comes from a stand-in for Nav2: A* on the known fixed obstacles (inflated
  0.5 m) and a pure-pursuit follower that turns on the spot when far off its heading;
- the planner sees tracks (true positions and velocities with small noise; with --noise
  real, the tracker's typical faults too: velocity noise up to 0.2 m/s, most on slow
  objects, and people near the robot sometimes left behind as remembered still objects for
  a few seconds, as the filter's lost-object memory does) and wall tiles within 6 m, and
  runs at 5 Hz with Responsiveness evidence as in the filter.

So it tests the local avoidance core in closed loop with reacting or non-reacting crowds,
not perception, Nav2 or the real walker. Results are statistics over many scenarios in
minutes; HIM confirms the winner. Prints one JSON line per run and a table per core and crowd.
"""

import argparse
import heapq
import json
import math
import os
import sys
import tempfile
import time
from multiprocessing import Pool

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import mujoco  # noqa: E402
import generate_world  # noqa: E402
from harp import events  # noqa: E402
from harp.game_planner import GamePlanner  # noqa: E402
from harp.movers import AwareMovers, Movers  # noqa: E402
from harp.obstacle_tracker import STRUCTURE_ID, Track  # noqa: E402
from harp.predictive_mpc import Config, Planner, Responsiveness  # noqa: E402

DT = .05                 # s plant step
PLAN_EVERY = 4           # plant steps per plan (0.2 s, the filter's period)
SCAN_EVERY = 2           # plant steps per tracker update (0.1 s)
SENSE = 6.0              # m: wall tiles and people seen within this
TILE = .2                # m between wall tiles
GAINS = np.array([.85, .67, 1.])   # GR00T's measured response in HIM
LAG, DELAY, ACCEL = .25, .1, .6
ARMS = .355              # m from base_link: the scorer's clearance is from the arms
SPEED = .5
CORES = {"mpc": Planner, "game": GamePlanner}


def fixed_shapes(model):
    """Fixed obstacles' footprints: ('box', cx, cy, hx, hy) or ('cyl', cx, cy, r)."""
    out = []
    for g in range(model.ngeom):
        name = model.geom(g).name
        if model.geom_bodyid[g] != 0 or not name.startswith(("wall", "block")):
            continue
        x, y = model.geom_pos[g][:2]
        if model.geom_type[g] == mujoco.mjtGeom.mjGEOM_BOX:
            out.append(("box", x, y, *model.geom_size[g][:2]))
        elif model.geom_type[g] == mujoco.mjtGeom.mjGEOM_CYLINDER:
            out.append(("cyl", x, y, model.geom_size[g][0]))
    return out


def distance_to(shapes, p):
    best = math.inf
    for s in shapes:
        if s[0] == "box":
            best = min(best, math.hypot(max(abs(p[0]-s[1])-s[3], 0.), max(abs(p[1]-s[2])-s[4], 0.)))
        else:
            best = min(best, math.hypot(p[0]-s[1], p[1]-s[2])-s[3])
    return best


def tiles(shapes):
    """Wall tiles along every fixed footprint's edge, as the tracker's structure: (x, y, r)."""
    out = []
    for s in shapes:
        if s[0] == "cyl":
            out.append((s[1], s[2], s[3]))
            continue
        _, cx, cy, hx, hy = s
        r = min(hx, hy, TILE/2)
        for t in np.arange(-hx, hx+1e-9, TILE):
            out += [(cx+t, cy-hy+r, r), (cx+t, cy+hy-r, r)]
        for t in np.arange(-hy, hy+1e-9, TILE):
            out += [(cx-hx+r, cy+t, r), (cx+hx-r, cy+t, r)]
    return np.array(out)


def plan_route(shapes, start, goal, bounds, inflate=.5, cell=.1):
    """A* on a cell grid over the fixed obstacles grown by inflate: a list of (x, y)."""
    x0, x1, y0, y1 = bounds
    nx, ny = int((x1-x0)/cell)+1, int((y1-y0)/cell)+1
    gx, gy = x0+np.arange(nx)*cell, y0+np.arange(ny)*cell
    free = np.ones((nx, ny), bool)
    cost = np.zeros((nx, ny))
    X, Y = np.meshgrid(gx, gy, indexing="ij")
    for s in shapes:
        if s[0] == "box":
            d = np.hypot(np.maximum(np.abs(X-s[1])-s[3], 0.), np.maximum(np.abs(Y-s[2])-s[4], 0.))
        else:
            d = np.hypot(X-s[1], Y-s[2])-s[3]
        free &= d > inflate
        cost += np.maximum(0., 1.-d/(inflate+.5))*2.
    cell_of = lambda p: (int(round((p[0]-x0)/cell)), int(round((p[1]-y0)/cell)))
    s, g = cell_of(start), cell_of(goal)
    for c in (s, g):
        free[c] = True
    heap, came, best = [(0., s)], {s: None}, {s: 0.}
    while heap:
        _, c = heapq.heappop(heap)
        if c == g:
            break
        for di in (-1, 0, 1):
            for dj in (-1, 0, 1):
                n = (c[0]+di, c[1]+dj)
                if (di or dj) and 0 <= n[0] < nx and 0 <= n[1] < ny and free[n]:
                    step = math.hypot(di, dj)*(1.+cost[n])
                    if best[c]+step < best.get(n, math.inf):
                        best[n] = best[c]+step
                        came[n] = c
                        heapq.heappush(heap, (best[n]+math.hypot(n[0]-g[0], n[1]-g[1]), n))
    if g not in came:
        return None
    path, c = [], g
    while c is not None:
        path.append((gx[c[0]], gy[c[1]]))
        c = came[c]
    return path[::-1]


def follow(route, pose, goal):
    """The Nav2 stand-in's request (vx, 0, wz): pure pursuit 0.8 m ahead, turning on the
    spot when over 0.9 rad off, slowing in the last metre; zero within 0.2 m of the goal."""
    if math.hypot(goal[0]-pose[0], goal[1]-pose[1]) < .2:
        return np.zeros(3)
    pts = np.asarray(route)
    k = int(np.argmin(np.linalg.norm(pts-pose[:2], axis=1)))
    ahead = np.cumsum(np.r_[0., np.linalg.norm(np.diff(pts[k:], axis=0), axis=1)])
    target = pts[k+int(np.searchsorted(ahead, .8))] if len(ahead) and ahead[-1] > .8 else pts[-1]
    err = math.atan2(target[1]-pose[1], target[0]-pose[0])-pose[2]
    err = (err+math.pi) % (2*math.pi)-math.pi
    if abs(err) > .9:
        return np.array([0., 0., math.copysign(.6, err)])
    slow = min(1., math.hypot(goal[0]-pose[0], goal[1]-pose[1]))
    return np.array([SPEED*max(0., math.cos(err))*max(slow, .3), 0., float(np.clip(1.5*err, -.6, .6))])


def run(job):
    seed, crowd, core, world = job["seed"], job["crowd"], job["core"], job.get("world")
    rng = np.random.default_rng(seed*7+1)
    with tempfile.TemporaryDirectory() as folder:
        if world:
            path = os.path.join(HERE, "worlds", world+".xml")
        else:
            path, _, _ = generate_world.write(seed, f"bench_{seed}", folder)
        # The walkers read the robot's true position from qpos[:2]: a free body for it.
        xml = open(path).read().replace("<worldbody>", '<worldbody>\n    <body name="bench_robot"><freejoint/>'
                                        '<geom type="sphere" size=".05" contype="0" conaffinity="0"/></body>', 1)
        model = mujoco.MjModel.from_xml_string(xml)
    data = mujoco.MjData(model)
    shapes = fixed_shapes(model)
    wall = tiles(shapes)
    goal = np.array(model.site("goal").pos[:2])
    sizes = AwareMovers(model).radius
    movers = (AwareMovers if crowd == "aware" else Movers)(model)
    movers.update(np.zeros(2), 0., True, True)
    xs = [p for s in shapes for p in ((s[1]-s[3], s[1]+s[3]) if s[0] == "box" else (s[1]-s[3], s[1]+s[3]))]
    ys = [p for s in shapes for p in ((s[2]-s[4], s[2]+s[4]) if s[0] == "box" else (s[2]-s[3], s[2]+s[3]))]
    bounds = (min(xs+[-2.])-1, max(xs+[goal[0]])+1, min(ys+[-2.])-1, max(ys+[2.])+1)
    route = plan_route(shapes, (0., 0.), goal, bounds)
    if route is None:
        return dict(job, error="no route")
    planner = CORES[core](Config(speed=SPEED, tracking_gain=.85, lateral_tracking_gain=.6))
    resp = Responsiveness()
    pose, vel = np.zeros(3), np.zeros(3)
    queue = [np.zeros(3)]*int(round(DELAY/DT))
    command = np.zeros(3)
    limit = 120.
    closest, fixed_close, contacts, touching = math.inf, math.inf, 0, False
    asked = fought = 0
    plan_times, reached_at, standing = [], None, 0.
    path_len, last_people, ghosts = 0., None, []
    for step in range(int(limit/DT)):
        t = step*DT
        data.qpos[:2] = pose[:2]
        movers.apply(data, t)
        people = np.array([data.mocap_pos[m][:2] for m in movers.mocap])
        if step % PLAN_EVERY == 0:
            if step % (PLAN_EVERY*10) == 0:
                route = plan_route(shapes, tuple(pose[:2]), goal, bounds) or route
            desired = follow(route, pose, goal)
            pv = (people-last_people)/(DT*PLAN_EVERY) if last_people is not None else np.zeros_like(people)
            last_people = people.copy()
            real = job.get("noise") == "real"
            spread = lambda v: (.08 + (.12 if real and np.linalg.norm(v) < .3 else 0.))
            tracks = [Track(i+1, *(people[i]+rng.normal(0, .05 if real else .03, 2)),
                            *(pv[i]+rng.normal(0, spread(pv[i]), 2)), float(sizes[i]), .05, t, 20)
                      for i in range(len(people)) if np.linalg.norm(people[i]-pose[:2]) < SENSE]
            if real:
                # Lost-object memory: a person near the robot sometimes stays behind as a
                # still object (radius grown, as planning_tracks keeps one) for 3 s.
                for i in range(len(people)):
                    if np.linalg.norm(people[i]-pose[:2]) < 1.5 and rng.random() < .02:
                        ghosts.append((t+3., Track(1000+len(ghosts), *people[i], 0., 0., float(sizes[i]), .35, t, 20)))
                ghosts[:] = [g for g in ghosts if g[0] > t]
                tracks += [g[1] for g in ghosts]
            planner.credits = resp.update(tracks, tuple(pose), tuple(vel))
            if hasattr(planner, "stubborn"):
                planner.stubborn = resp.stubborn
            near = wall[np.linalg.norm(wall[:, :2]-pose[:2], axis=1) < SENSE] if len(wall) else wall
            tracks += [Track(STRUCTURE_ID+k, float(x), float(y), 0., 0., float(r), .05, t, 9)
                       for k, (x, y, r) in enumerate(near)]
            started = time.perf_counter()
            result = planner.plan(tuple(pose), tuple(vel), tuple(desired), tracks, t, tuple(goal))
            plan_times.append(time.perf_counter()-started)
            # As the filter does with every result (a numpy type here crashed it in HIM).
            events.record("predictive", "avoidance", mode=result.mode, alert=bool(result.alert) and result.alert,
                          solver=result.solver, clearance=result.clearance if math.isfinite(result.clearance) else None)
            command = np.asarray(result.command, dtype=float)
            if abs(desired[0]) < .05 and abs(desired[2]) > .2:
                asked += 1
                fought += command[0] > .25
            if not desired.any() and reached_at is None:
                reached_at = t
        queue.append(command.copy())
        target = queue.pop(0)*GAINS
        change = np.clip((target-vel)*DT/LAG, -ACCEL*DT, ACCEL*DT)
        vel = vel+change
        c, s = math.cos(pose[2]), math.sin(pose[2])
        move = np.array([c*vel[0]-s*vel[1], s*vel[0]+c*vel[1]])*DT
        pose[:2] += move
        pose[2] += vel[2]*DT
        path_len += float(np.linalg.norm(move))
        standing += DT if np.linalg.norm(vel[:2]) < .05 and reached_at is None else 0.
        if len(people):
            gap = float(np.min(np.linalg.norm(people-pose[:2], axis=1)-sizes-ARMS))
            closest = min(closest, gap)
            if gap < 0 and not touching:
                contacts += 1
            touching = gap < 0
        fixed_close = min(fixed_close, distance_to(shapes, pose[:2])-ARMS)
        if reached_at is not None:
            break
    end = float(np.linalg.norm(pose[:2]-goal))
    return dict(job, reached=reached_at is not None and end < .35, time=reached_at, end=round(end, 2),
                closest=round(closest, 3), contacts=contacts, wall=round(fixed_close, 3),
                fight=round(fought/max(asked, 1), 3), path=round(path_len, 1), standing=round(standing, 1),
                plan_p95_ms=round(1000*float(np.percentile(plan_times, 95)), 1))


def seeds(text):
    out = []
    for part in text.split(","):
        a, _, b = part.partition("-")
        out += list(range(int(a), int(b or a)+1))
    return out


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--cores", nargs="+", default=["mpc", "game"], choices=sorted(CORES))
    parser.add_argument("--seeds", default="1-20", help="generate_world seeds, e.g. 1-40 (not held-out ones)")
    parser.add_argument("--worlds", nargs="*", default=[], help="named worlds too (e.g. building office atrium)")
    parser.add_argument("--crowds", nargs="+", default=["aware"], choices=["aware", "scripted"])
    parser.add_argument("--noise", default="clean", choices=["clean", "real"])
    parser.add_argument("--jobs", type=int, default=max(1, (os.cpu_count() or 2)-2))
    parser.add_argument("--out", help="also write the runs as JSON lines here")
    args = parser.parse_args()
    held = set(range(20261103, 20261111))
    jobs = [dict(seed=s, crowd=c, core=k, noise=args.noise) for s in seeds(args.seeds) if s not in held
            for c in args.crowds for k in args.cores]
    jobs += [dict(seed=0, world=w, crowd=c, core=k, noise=args.noise)
             for w in args.worlds for c in args.crowds for k in args.cores]
    os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")
    with Pool(args.jobs) as pool:
        rows = pool.map(run, jobs)
    if args.out:
        with open(args.out, "w") as f:
            f.writelines(json.dumps(r)+"\n" for r in rows)
    for core in args.cores:
        for crowd in args.crowds:
            rs = [r for r in rows if r["core"] == core and r["crowd"] == crowd and "error" not in r]
            if not rs:
                continue
            n = len(rs)
            reached = [r for r in rs if r["reached"]]
            print(f"{core:5} {crowd:8} reached {len(reached)}/{n}  contacts {sum(r['contacts'] for r in rs)} "
                  f"(runs with any {sum(r['contacts'] > 0 for r in rs)})  clearance>=0.30 {sum(r['closest'] >= .3 for r in rs)}/{n}"
                  f"  median time {np.median([r['time'] for r in reached]) if reached else None}"
                  f"  fight {np.mean([r['fight'] for r in rs]):.3f}  standing {np.mean([r['standing'] for r in rs]):.1f}s"
                  f"  plan p95 {np.median([r['plan_p95_ms'] for r in rs]):.0f} ms")


if __name__ == "__main__":
    main()
