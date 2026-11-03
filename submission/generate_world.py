#!/usr/bin/env python3
"""A random course world for a held-out test: submission/generate_world.py --seed N [--name NAME].

Writes submission/worlds/NAME.xml and NAME.yaml (default NAME: heldout_N), a world no one has
tuned against: a walled hall split by a partition with one or two doorways, pillars and
boxes, a goal beyond the partition, and walkers of every kind the hand-made worlds have
(people crossing, coming head-on, overtaking from behind, standing in a group, walking out
and stopping, a cart, a child). Everything comes from the seed; the same seed gives the
same world. A world is kept only if the robot's footprint (Nav2's 0.4 m radius plus 5 cm)
can reach the goal past the fixed obstacles and the people who stand; else the seed's next
draw is tried. Movers' routes keep their bodies clear of walls and blocks.

    python3 submission/generate_world.py --seed 7
    python3 submission/course.py heldout_7 --avoidance predictive --crowd aware
"""

import argparse
import math
import random
from pathlib import Path

HERE = Path(__file__).resolve().parent
WORLDS = HERE / "worlds"
ROBOT = .45          # m: Nav2's robot_radius and a little, for the reachability check
CELL = .05           # m: reachability grid
WALL = .075          # m: half a wall's thickness
PERSON, CHILD = .2, .14


class Box:
    def __init__(self, name, x, y, hx, hy, hz, kind="box"):
        self.name, self.x, self.y, self.hx, self.hy, self.hz, self.kind = name, x, y, hx, hy, hz, kind

    def gap(self, px, py):
        """Distance from a point to this obstacle's footprint (0 inside)."""
        if self.kind == "cylinder":
            return max(0., math.hypot(px-self.x, py-self.y)-self.hx)
        return math.hypot(max(abs(px-self.x)-self.hx, 0.), max(abs(py-self.y)-self.hy, 0.))

    def xml(self, color):
        if self.kind == "cylinder":
            return (f'    <geom name="{self.name}" type="cylinder" size="{self.hx:.2f} {self.hz:.2f}" '
                    f'pos="{self.x:.2f} {self.y:.2f} {self.hz:.2f}" rgba="{color}"/>')
        return (f'    <geom name="{self.name}" type="box" size="{self.hx:.3f} {self.hy:.3f} {self.hz:.3f}" '
                f'pos="{self.x:.3f} {self.y:.3f} {self.hz:.3f}" rgba="{color}"/>')


def clear(fixed, x, y, margin):
    return all(o.gap(x, y) >= margin for o in fixed)


def segment_clear(fixed, a, b, margin):
    n = max(2, int(math.dist(a, b) / .1))
    return all(clear(fixed, a[0]+(b[0]-a[0])*k/n, a[1]+(b[1]-a[1])*k/n, margin) for k in range(n+1))


def reachable(fixed, standing, bounds, start, goal):
    """Breadth-first search on a CELL grid: can a ROBOT-radius disc get from start to goal?"""
    x0, x1, y0, y1 = bounds
    nx, ny = int((x1-x0)/CELL)+1, int((y1-y0)/CELL)+1
    discs = [(x, y, r) for x, y, r in standing]
    def free(i, j):
        x, y = x0+i*CELL, y0+j*CELL
        return clear(fixed, x, y, ROBOT) and all(math.hypot(x-a, y-b) >= ROBOT+r for a, b, r in discs)
    cell = lambda p: (round((p[0]-x0)/CELL), round((p[1]-y0)/CELL))
    s, g = cell(start), cell(goal)
    if not free(*s) or not free(*g):
        return False
    seen, frontier = {s}, [s]
    while frontier:
        nxt = []
        for i, j in frontier:
            for di, dj in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                c = (i+di, j+dj)
                if c == g:
                    return True
                if 0 <= c[0] < nx and 0 <= c[1] < ny and c not in seen and free(*c):
                    seen.add(c)
                    nxt.append(c)
        frontier = nxt
    return False


def draw(rng):
    """One candidate world: (fixed obstacles, movers, goal, bounds, description) or None."""
    length, depth = rng.uniform(13, 16), rng.uniform(9, 12)
    x0, x1, y0, y1 = -2., -2.+length, -depth/2, depth/2
    fixed = [Box("wall0", (x0+x1)/2, y0, length/2+WALL, WALL, 1.),
             Box("wall1", (x0+x1)/2, y1, length/2+WALL, WALL, 1.),
             Box("wall2", x0, 0., WALL, depth/2+WALL, 1.),
             Box("wall3", x1, 0., WALL, depth/2+WALL, 1.)]
    # The partition, across the hall, with one or two doorways.
    xp = rng.uniform(4., 6.5)
    doors = sorted(rng.uniform(y0+1.5, y1-1.5) for _ in range(rng.choice((1, 2))))
    if len(doors) == 2 and doors[1]-doors[0] < 2.5:
        doors = doors[:1]
    widths = [rng.uniform(1.3, 2.0) for _ in doors]
    edges, y = [], y0
    for d, w in zip(doors, widths):
        edges.append((y, d-w/2))
        y = d+w/2
    edges.append((y, y1))
    for k, (a, b) in enumerate(edges):
        if b-a > .1:
            fixed.append(Box(f"wall{4+k}", xp, (a+b)/2, WALL, (b-a)/2, 1.))
    goal = (rng.uniform(xp+3.5, min(x1-1.5, 9.5)), rng.uniform(y0+1.5, y1-1.5))
    blocks = []
    def place(name, kind, size, region):
        for _ in range(50):
            x, y = rng.uniform(*region[:2]), rng.uniform(*region[2:])
            b = Box(name, x, y, *size, kind=kind)
            corners = ((x+size[0], y+size[1]), (x-size[0], y-size[1]), (x+size[0], y-size[1]), (x-size[0], y+size[1]))
            if (b.gap(0., 0.) > 1.5 and b.gap(*goal) > 1.2 and all(b.gap(xp, d) > .9 for d in doors)
                    and all(o.gap(x, y) > max(size[:2])+.9 for o in fixed)
                    and all(y0+.5 < cy < y1-.5 and x0+.5 < cx < x1-.5 for cx, cy in corners)):
                blocks.append(b)
                fixed.append(b)
                return
    for k in range(rng.randint(3, 6)):
        r = rng.uniform(.2, .35)
        place(f"block{len(blocks)}", "cylinder", (r, r, 1.), (x0+1, x1-1, y0+1, y1-1))
    for k in range(rng.randint(1, 3)):
        place(f"block{len(blocks)}", "box", (rng.uniform(.2, .8), rng.uniform(.2, .5), rng.choice((.225, .4, .5))),
              (x0+1, x1-1, y0+1, y1-1))
    return fixed, blocks, goal, (x0, x1, y0, y1), xp, doors


def movers(rng, fixed, goal, bounds, xp, doors):
    """Walkers on routes clear of walls and blocks: list of dicts (name, start, route, speed,
    delay, trips, shape)."""
    x0, x1, y0, y1 = bounds
    out = []
    def add(name, points, speed, delay=0., trips=None, shape=("capsule", PERSON)):
        margin = shape[1]+.1
        if not all(clear(fixed, *p, margin) for p in points):
            return False
        if not all(segment_clear(fixed, a, b, margin) for a, b in zip(points, points[1:])):
            return False
        length = sum(math.dist(a, b) for a, b in zip(points, points[1:]))
        out.append(dict(name=name, start=points[0], route=points[1:], duration=max(1., length/speed),
                        delay=delay, trips=trips, shape=shape))
        return True
    def tries(fn, n=40):
        for _ in range(n):
            if fn():
                return
    k = [0]
    def person():
        k[0] += 1
        return f"person{k[0]-1}"
    # Crossers: across the hall, some through a doorway.
    for _ in range(rng.randint(2, 3)):
        def crosser():
            x = rng.uniform(1.5, x1-1.5)
            pts = [(x, y0+.6), (x+rng.uniform(-1.5, 1.5), y1-.6)]
            if rng.random() < .5:
                pts.reverse()
            return add(person(), pts, rng.uniform(.7, 1.3), rng.uniform(0, 8))
        tries(crosser)
    # Through a doorway, from one side of the partition to the other.
    def through():
        d = rng.choice(doors)
        pts = [(xp-rng.uniform(2, 4), d+rng.uniform(-2, 2)), (xp, d), (xp+rng.uniform(2, 4), d+rng.uniform(-2, 2))]
        if rng.random() < .5:
            pts.reverse()
        return add(person(), pts, rng.uniform(.8, 1.2), rng.uniform(0, 6))
    tries(through)
    # Head-on: from near the goal toward the start, through the robot's doorway.
    def head_on():
        d = min(doors, key=lambda d: abs(d-goal[1]/2))
        pts = [(goal[0]+rng.uniform(-1, 1), goal[1]+rng.uniform(-1, 1)), (xp, d+rng.uniform(-.3, .3)),
               (rng.uniform(-1, .5), rng.uniform(-1, 1))]
        return add(person(), pts, rng.uniform(.7, 1.1), rng.uniform(0, 4))
    tries(head_on)
    # Overtaking from behind, faster than the robot.
    def overtaker():
        d = rng.choice(doors)
        pts = [(x0+.6, rng.uniform(-1, 1)), (xp, d), (x1-.8, rng.uniform(y0+1, y1-1))]
        return add(person(), pts, rng.uniform(1.2, 1.6), rng.uniform(4, 10))
    tries(overtaker)
    # A group standing talking, somewhere in the hall.
    def group():
        cx, cy = rng.uniform(1.5, x1-2), rng.uniform(y0+1.5, y1-1.5)
        if math.hypot(cx, cy) < 2.5 or math.dist((cx, cy), goal) < 1.8 or any(abs(cx-xp) < 1.5 and abs(cy-d) < 1.5 for d in doors):
            return False
        before = len(out)
        for a in range(rng.randint(2, 3)):
            ang = a*2*math.pi/3+rng.uniform(-.3, .3)
            p = (cx+.55*math.cos(ang), cy+.55*math.sin(ang))
            if not add(person(), [p, (p[0]+.05, p[1])], .1, trips=0):
                del out[before:]
                return False
        return True
    tries(group)
    # Walks out and stops (trips 1), in the robot's half of the hall.
    def stopper():
        a = (rng.uniform(1, x1-2), rng.choice((y0+.6, y1-.6)))
        b = (a[0]+rng.uniform(-1, 1), rng.uniform(-1.5, 1.5))
        return add(person(), [a, b], rng.uniform(.6, 1.), rng.uniform(2, 10), trips=1)
    tries(stopper)
    # A cart pushed along the hall, out and back.
    def cart():
        y = rng.uniform(y0+1, y1-1)
        pts = [(x1-1, y), (x0+1.5, y+rng.uniform(-1, 1))]
        return add("cart", pts, rng.uniform(.4, .6), rng.uniform(2, 8), shape=("box", .46))
    tries(cart)
    # A child darting to and fro.
    def child():
        a = (rng.uniform(2, x1-2), rng.uniform(y0+1, y1-1))
        b = (a[0]+rng.uniform(-2, 2), a[1]+rng.uniform(-2, 2))
        return add("child", [a, b], rng.uniform(1., 1.6), rng.uniform(0, 6), trips=rng.choice((4, 6, None)),
                   shape=("capsule", CHILD))
    tries(child)
    return out


def generate(seed):
    rng = random.Random(seed)
    for attempt in range(200):
        drawn = draw(rng)
        fixed, blocks, goal, bounds, xp, doors = drawn
        walkers = movers(rng, fixed, goal, bounds, xp, doors)
        standing = [(w["start"][0], w["start"][1], w["shape"][1]) for w in walkers if w["trips"] == 0]
        stops = [(w["route"][-1][0], w["route"][-1][1], w["shape"][1]) for w in walkers if w["trips"] == 1]
        if len(walkers) >= 7 and reachable(fixed, standing+stops, bounds, (0., 0.), goal):
            return fixed, goal, walkers, xp, doors, attempt
    raise SystemExit(f"seed {seed}: no reachable world in 200 draws")


def write(seed, name, folder=WORLDS):
    fixed, goal, walkers, xp, doors, attempt = generate(seed)
    people = [w for w in walkers if w["name"].startswith("person")]
    for i, w in enumerate(people):
        w["name"] = f"person{i}"
    wall, block = "0.78 0.78 0.74 1", "0.2 0.4 0.8 1"
    custom, bodies = [], []
    for w in walkers:
        n = w["name"]
        custom.append(f'    <numeric name="{n}_duration" data="{w["duration"]:.1f}"/>')
        if w["delay"]:
            custom.append(f'    <numeric name="{n}_delay" data="{w["delay"]:.1f}"/>')
        if w["trips"] is not None:
            custom.append(f'    <numeric name="{n}_trips" data="{w["trips"]}"/>')
        route = " ".join(f"{x:.2f} {y:.2f}" for x, y in w["route"])
        custom.append(f'    <numeric name="{n}_route" data="{route}"/>')
        x, y = w["start"]
        kind, r = w["shape"]
        if kind == "box":
            geom = f'<geom name="{n}" type="box" size="0.3 0.35 0.5" pos="0 0 0.5" rgba="0.55 0.35 0.2 1"/>'
        elif r == CHILD:
            geom = (f'<geom name="{n}" type="capsule" size="0.14 0.3" pos="0 0 0.49" rgba="1 0.45 0.1 1"/>'
                    f'\n      <geom name="{n}_head" type="sphere" size="0.11" pos="0 0 0.89" rgba="1 0.7 0.4 1"/>')
        else:
            geom = (f'<geom name="{n}" type="capsule" size="0.2 0.5" pos="0 0 0.75" rgba="1 0.45 0.1 1"/>'
                    f'\n      <geom name="{n}_head" type="sphere" size="0.15" pos="0 0 1.39" rgba="1 0.7 0.4 1"/>')
        bodies.append(f'    <body name="{n}" mocap="true" pos="{x:.2f} {y:.2f} 0">\n      {geom}\n    </body>')
    text = "\n".join([
        f"<!-- Generated by submission/generate_world.py --seed {seed} (draw {attempt}): a hall with a",
        f"     partition at x {xp:.1f} (doorways at y {', '.join(f'{d:.1f}' for d in doors)}), goal "
        f"({goal[0]:.1f}, {goal[1]:.1f}), {len(walkers)} movers. Held out: not used to tune the planner.",
        f"     python3 submission/course.py {name} --avoidance predictive [--crowd aware] -->",
        "<mujoco>", "  <custom>", *custom, "  </custom>", "  <worldbody>",
        *(o.xml(wall if o.name.startswith("wall") else block) for o in fixed), *bodies,
        f'    <site name="goal" pos="{goal[0]:.2f} {goal[1]:.2f} 0.01" size="0.15 0.01" type="cylinder" rgba="0.2 0.8 0.3 0.6"/>',
        "  </worldbody>", "</mujoco>", ""])
    folder = Path(folder)
    (folder / f"{name}.xml").write_text(text)
    (folder / f"{name}.yaml").write_text(
        f"# Pass criteria for course tests on this generated world (seed {seed}); as building's.\n"
        "must_succeed: true\nmax_time_s: 120\nmin_clearance_m: 0.30\nmax_contacts: 0\n"
        "max_recoveries: 3\nmax_end_error_m: 0.35\n")
    return folder / f"{name}.xml", goal, len(walkers)


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--seed", type=int, required=True)
    parser.add_argument("--name", help="world name (default heldout_SEED)")
    args = parser.parse_args()
    path, goal, n = write(args.seed, args.name or f"heldout_{args.seed}")
    print(f"{path}: goal ({goal[0]:.2f}, {goal[1]:.2f}), {n} movers")


if __name__ == "__main__":
    main()
