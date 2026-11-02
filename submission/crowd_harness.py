#!/usr/bin/env python3
"""Fast closed-loop crowd check of the real avoidance pipeline, without ROS or joints.

MuJoCo supplies the world, the seeded pedestrians and LiDAR rays in the head
lidar's 480 x 40 pattern. The real tracker, occlusion memory
(PredictiveAvoidance.process_scan), native route driver and MPC run on them.
A delayed, acceleration-limited planar plant with walker-like response gains
stands in for the G1, deliberately a little unlike the planner's own model.
A run takes seconds instead of minutes, for iterating across many seeds;
native_g1_sim.py remains the walking-G1 check. Odometry is ideal here.

It imports the ROS node module, so run it with the native Jazzy Python
(as native_g1_sim.sh does); submission/native_g1_sim.sh is the G1 run:
  PYTHONPATH=submission <native python> submission/crowd_harness.py --world building --seeds 1-8

--course NAME runs a course world instead (submission/worlds/NAME.xml): its movers
exactly as HIM's course scores them (harp/movers.py, started when the goal is sent,
at t = 0), its goal site, and contacts as geometry touching the robot's 0.3 m
cylinder. With --speed and --plant matching HIM's image and walker it reproduces
course runs in seconds; HIM's Nav2 path is approximated by the native route driver.
"""
import os
os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')   # before numpy: one BLAS thread per job
import argparse
from concurrent.futures import ProcessPoolExecutor
import json
import math
from pathlib import Path
import sys
import time
from types import SimpleNamespace

import numpy as np

CONTACT_PERSON = .5   # m between centres: G1 shoulders/arms (~0.3) plus a 0.2 m person
CONTACT_WALL = .3     # m from the robot centre to a wall or block surface
DT = .02
PLANNING_DEADLINE = .15   # s, as the node: a slower plan publishes zero instead


class _Node:
    """The parts of an rclpy node PredictiveAvoidance touches, as no-ops."""
    def create_publisher(self, *args, **kwargs):
        return SimpleNamespace(publish=lambda message: None)

    def create_subscription(self, *args, **kwargs):
        return None

    def create_timer(self, *args, **kwargs):
        return None


def build(world, seed, crowd, flow, count=7):
    import mujoco
    import native_g1_sim as sim
    sys.path.insert(0, str(sim.prepare()/'harp_sim'))
    os.environ.update(HARP_CROWD_SEED=str(seed), HARP_CROWD_MODE=crowd, HARP_CROWD_SPEED=str(flow))
    from harp.crowd_motion import CrowdMotion
    path = Path(os.environ.get('TMPDIR', '/tmp'))/f'harness-{os.getpid()}-{seed}.xml'
    sim.world_file(path, count, layout=world)
    spec = mujoco.MjSpec.from_string(
        '<mujoco><worldbody><geom name="floor" type="plane" size="30 30 .1"/>'
        '<body name="robot"><freejoint/><geom type="cylinder" size=".3 .7" pos="0 0 .7" '
        'contype="0" conaffinity="0"/></body></worldbody></mujoco>')
    spec.attach(mujoco.MjSpec.from_file(str(path)), prefix='course/', frame=spec.worldbody.add_frame())
    model = spec.compile()
    path.unlink()
    data = mujoco.MjData(model)
    people = CrowdMotion(model)
    people.started = [float(sim.BUILDING_ROUTES[i][0]) if world == 'building' else float(d or 0.)
                      for i, d in enumerate(people.delay)]
    return model, data, people


class CourseMovers:
    """A course world's movers (harp/movers.py) behind the crowd interface the loop reads."""
    def __init__(self, model):
        from harp.movers import Movers
        self.movers = Movers(model)
        self.movers.started = [d or 0. for d in self.movers.delay]   # goal sent at t = 0
        self.positions = model.body_pos[model.body_mocapid >= 0].copy()   # Movers' order
        self.velocity = np.zeros((len(self.positions), 2))
        self.at = None

    def apply(self, data, t):
        self.movers.apply(data, t)
        now = data.mocap_pos[self.movers.mocap].copy()
        if self.at is not None and t > self.at:
            self.velocity = (now[:, :2]-self.positions[:, :2])/(t-self.at)
        self.positions, self.at = now, t


def build_course(name):
    import mujoco
    spec = mujoco.MjSpec.from_string(
        '<mujoco><worldbody><geom name="floor" type="plane" size="30 30 .1"/>'
        '<body name="robot"><freejoint/><geom name="robot" type="cylinder" size=".3 .7" pos="0 0 .7" '
        'contype="0" conaffinity="0"/></body></worldbody></mujoco>')
    world = Path(__file__).resolve().parent/'worlds'/(name+'.xml')
    spec.attach(mujoco.MjSpec.from_file(str(world)), prefix='course/', frame=spec.worldbody.add_frame())
    model = spec.compile()
    return model, mujoco.MjData(model), CourseMovers(model)


def run(args):
    world, crowd, seed, flow, seconds = args['world'], args['crowd'], args['seed'], args['flow'], args['seconds']
    course = args.get('course')
    gain_x, gain_y, delay = args.get('plant') or (.85, .55, .25)
    os.environ.update(HARP_PREDICTIVE_SPEED=str(args.get('speed') or .5),
                      HARP_PREDICTIVE_TRACKING_GAIN='.85', HARP_PREDICTIVE_LATERAL_GAIN='.60')
    import mujoco
    from harp import lidar_mount
    from harp.local_route import route_command, route_direction
    from harp.predictive_avoidance import PredictiveAvoidance
    if course:
        model, data, people = build_course(course)
        default_goal = model.site('course/goal').pos[:2].copy()
        # Each mover's geoms, in Movers' (body) order.
        mover_geoms = [np.flatnonzero(model.geom_bodyid == b) for b in np.flatnonzero(model.body_mocapid >= 0)]
        robot_geom = model.geom('robot').id
    else:
        model, data, people = build(world, seed, crowd, flow, args['people'])
        default_goal = (9., 0.) if world == 'building' else (6., 0.)
    goal = np.array(args['goal'] or default_goal)
    cap = float(os.environ['HARP_PREDICTIVE_SPEED'])
    guard = PredictiveAvoidance(_Node(), buffer=SimpleNamespace())
    if args.get('params'):
        # Tuning: module constants of predictive_mpc, or Config fields.
        from dataclasses import fields, replace
        import harp.predictive_mpc as mpc
        names = {f.name for f in fields(mpc.Config)}
        config = {k: v for k, v in args['params'].items() if k in names}
        for k, v in args['params'].items():
            if k not in names:
                if not hasattr(mpc, k):
                    raise SystemExit(f'unknown parameter {k}')
                setattr(mpc, k, v)
        guard.planner = mpc.Planner(replace(guard.planner.config, **config))
        guard.config = guard.planner.config
    rng = np.random.default_rng(seed+1000)
    robot = model.body('robot').id
    walls = [(model.geom_pos[g, :2].copy(), model.geom_size[g, :2].copy()) for g in range(model.ngeom)
             if model.geom(g).name.split('/')[-1].startswith(('wall', 'block', 'pillar'))]
    az = np.radians(np.arange(-180., 180., .75))
    el = np.radians(np.arange(-7., 52.75, 1.5))
    a, e = (v.ravel() for v in np.meshgrid(az, el))
    rays = np.stack([np.cos(e)*np.cos(a), np.cos(e)*np.sin(a), np.sin(e)], axis=1)
    geomid = np.zeros(len(rays), np.int32)
    dist = np.zeros(len(rays))
    # Plant: walker response unlike the planner's assumed .2 s / .85 / .60, with
    # the native G1's gait-dependent sidestep variability (lateral gain .3-.7);
    # 12 steps of .02 s is a .24 s delay.
    delay_steps, gains, accel, yaw_accel = int(round(delay/DT)), np.array([gain_x, gain_y, 1.]), .4, 1.
    pose, body_velocity = np.zeros(3), np.zeros(3)
    pending = [np.zeros(3)]*delay_steps
    command, desired, route_vector = np.zeros(3), np.zeros(3), np.array([1., 0.])
    tracks = []
    stats = dict(people_contacts=0, wall_contacts=0, scan_errors=0, deadline_stops=0,
                 modes={}, plan_ms=[], events=[])
    phantoms = []   # per plan: moving tracks within 4 m that are no real mover
    solver, scan_ok, at_wall = None, True, False
    touching, minimum_person, minimum_wall = set(), math.inf, math.inf
    standstill = longest = path = 0.
    status, mode = 'TIMEOUT', None
    for step in range(int(seconds/DT)):
        t = step*DT
        c, s = math.cos(pose[2]), math.sin(pose[2])
        data.qpos[:3] = (pose[0], pose[1], 0.)
        data.qpos[3:7] = (math.cos(pose[2]/2), 0., 0., math.sin(pose[2]/2))
        data.qvel[:2] = (c*body_velocity[0]-s*body_velocity[1], s*body_velocity[0]+c*body_velocity[1])
        people.apply(data, t)
        if step % 5 == 0:   # 10 Hz scans
            mujoco.mj_kinematics(model, data)
            origin = np.array([pose[0], pose[1], 1.3])
            mount = np.array([[c, -s, 0.], [s, c, 0.], [0., 0., 1.]]) @ lidar_mount.rotation()
            mujoco.mj_multiRay(model, data, origin, (rays @ mount.T).ravel(), None, 1, robot,
                               geomid, dist, None, len(rays), 40.)
            noisy = dist+rng.normal(0., .02, len(dist))
            keep = (geomid >= 0) & (noisy > .1) & (rng.random(len(dist)) > .05)
            points = origin+(rays[keep] @ mount.T)*noisy[keep, None]
            guard.pose = tuple(pose)
            try:
                tracks = guard.process_scan(points, origin, np.array([pose[0], pose[1], .8]), t)
                scan_ok = True
            except ValueError:
                # As the node: zero, and no plan until a usable scan arrives.
                stats['scan_errors'] += 1
                command[:] = 0.
                scan_ok = False
        if step % 25 == 0:   # route every .5 s, as the native trial
            route_vector = route_direction(pose[:2], goal, tracks)
        if step % 5 == 0:
            desired = np.array(route_command(pose, goal, route_vector, cap))
        if step % 10 == 0 and scan_ok:   # 5 Hz avoidance
            planning = guard.planning_tracks(t)
            movers_xy = people.positions[:, :2]
            phantoms.append(sum(1 for tr in planning if 0 <= tr.id < 10**9 and math.hypot(tr.vx, tr.vy) > .15
                                and math.hypot(tr.x-pose[0], tr.y-pose[1]) < 4.
                                and np.min(np.linalg.norm(movers_xy-[tr.x, tr.y], axis=1)) > .6))
            began = time.perf_counter()
            result = guard.planner.plan(pose, body_velocity, desired, planning, t, goal)
            elapsed = time.perf_counter()-began
            stats['plan_ms'].append(1000*elapsed)
            command = np.array(result.command)
            if elapsed > PLANNING_DEADLINE:
                stats['deadline_stops'] += 1
                command[:] = 0.
                guard.planner.previous[:] = 0.
            if args.get('trace') and args['trace'][0] <= t <= args['trace'][1]:
                near = sorted(range(len(people.positions)), key=lambda k: np.linalg.norm(people.positions[k, :2]-pose[:2]))[:2]
                print(f"t={t:5.1f} pose=({pose[0]:.2f},{pose[1]:.2f},{pose[2]:+.2f}) v=({body_velocity[0]:+.2f},{body_velocity[1]:+.2f}) "
                      f"desired=({desired[0]:+.2f},{desired[1]:+.2f},{desired[2]:+.2f}) cmd=({command[0]:+.2f},{command[1]:+.2f},{command[2]:+.2f}) "
                      f"{result.mode}/{result.solver} clr={result.clearance:+.2f} | "
                      + ' '.join(f"p{k}({people.positions[k,0]:.2f},{people.positions[k,1]:.2f} v{people.velocity[k,0]:+.2f},{people.velocity[k,1]:+.2f})" for k in near),
                      file=sys.stderr, flush=True)
            solver = result.solver
            if result.mode != mode:
                stats['modes'][result.mode] = stats['modes'].get(result.mode, 0)+1
                mode = result.mode
        if step % 25 == 0:
            gains[1] = rng.uniform(gain_y-.2, gain_y+.2)
        pending.append(command*gains)
        target = pending.pop(0)
        change = target-body_velocity
        linear = np.linalg.norm(change[:2])
        if linear > accel*DT:
            change[:2] *= accel*DT/linear
        change[2] = float(np.clip(change[2], -yaw_accel*DT, yaw_accel*DT))
        body_velocity += change
        world_velocity = np.array([c*body_velocity[0]-s*body_velocity[1], s*body_velocity[0]+c*body_velocity[1]])
        pose[:2] += world_velocity*DT
        pose[2] += body_velocity[2]*DT
        speed = float(np.linalg.norm(world_velocity))
        path += speed*DT
        standstill = standstill+DT if speed < .06 else 0.
        longest = max(longest, standstill)
        if course:
            # Surface distances from the robot's cylinder to each mover (person, trolley, ...).
            data.qpos[:2] = pose[:2]
            data.qpos[3:7] = (math.cos(pose[2]/2), 0., 0., math.sin(pose[2]/2))
            mujoco.mj_kinematics(model, data)
            gaps = np.array([min(mujoco.mj_geomDistance(model, data, robot_geom, g, 2., None)
                                 for g in geoms) for geoms in mover_geoms])
            now_touching = set(np.flatnonzero(gaps <= 0.))
        else:
            gaps = np.linalg.norm(people.positions[:, :2]-pose[:2], axis=1).reshape(-1)
            now_touching = set(np.flatnonzero(gaps < CONTACT_PERSON))
        minimum_person = min([minimum_person, *gaps.tolist()])
        stats['people_contacts'] += len(now_touching-touching)
        for i in now_touching-touching:
            # Context for diagnosis: was the actor tracked, how, and what was planned.
            q, v = people.positions[i, :2], people.velocity[i, :2]
            near = min(tracks, key=lambda tr: math.hypot(tr.x-q[0], tr.y-q[1]), default=None)
            toward = (q-pose[:2])/max(float(np.linalg.norm(q-pose[:2])), 1e-9)
            stats['events'].append(dict(
                t=round(t, 2), person=int(i), yields=bool(crowd != 'adversarial' or i % 3),
                # Who closed the gap: robot speed toward the actor, actor speed toward the robot.
                robot_closing=round(float(world_velocity @ toward), 2), actor_closing=round(float(-v @ toward), 2),
                robot=[round(float(v_), 2) for v_ in pose], robot_v=[round(float(v_), 2) for v_ in world_velocity],
                person_xy=[round(float(v_), 2) for v_ in q], person_v=[round(float(v_), 2) for v_ in v],
                mode=mode, solver=solver, command=[round(float(v_), 2) for v_ in command],
                track=None if near is None else dict(dist=round(math.hypot(near.x-q[0], near.y-q[1]), 2),
                    v=[round(near.vx, 2), round(near.vy, 2)], r=round(near.radius, 2),
                    u=round(near.uncertainty, 2), id=near.id)))
        touching = now_touching
        touching_wall = None
        for center, half in walls:
            gap = float(np.linalg.norm(pose[:2]-(center+np.clip(pose[:2]-center, -half, half))))
            minimum_wall = min(minimum_wall, gap)
            if gap < CONTACT_WALL and touching_wall is None:
                touching_wall = (center, gap)
        if touching_wall is not None and not at_wall:   # count each contact once
            center, gap = touching_wall
            stats['wall_contacts'] += 1
            stats['events'].append(dict(t=round(t, 2), wall=[round(float(v_), 2) for v_ in center],
                robot=[round(float(v_), 2) for v_ in pose], gap=round(gap, 2),
                robot_v=[round(float(v_), 2) for v_ in world_velocity], mode=mode, solver=solver,
                command=[round(float(v_), 2) for v_ in command]))
        at_wall = touching_wall is not None
        if np.linalg.norm(goal-pose[:2]) < .25:
            status = 'SUCCEEDED'
            break
    plan = np.array(stats.pop('plan_ms'))
    straight = float(np.linalg.norm(goal))
    return dict(world=course or world, crowd=None if course else crowd, seed=seed, flow=flow, params=args.get('params') or {},
                status=status, seconds=round(t, 1),
                passed=status == 'SUCCEEDED' and not stats['people_contacts'] and not stats['wall_contacts'],
                **stats, min_person_m=round(minimum_person, 3), min_wall_m=round(minimum_wall, 3),
                longest_standstill_s=round(longest, 2),
                phantom_movers=round(float(np.mean(phantoms)) if phantoms else 0., 2), path_efficiency=round(straight/max(path, 1e-9), 3),
                final=[round(float(v), 2) for v in pose[:2]],
                plan_ms_p50=round(float(np.percentile(plan, 50)), 1),
                plan_ms_p95=round(float(np.percentile(plan, 95)), 1),
                plan_ms_max=round(float(plan.max()), 1))


def seeds(text):
    if '-' in text:
        low, high = text.split('-')
        return list(range(int(low), int(high)+1))
    return [int(v) for v in text.split(',')]


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--world', choices=('open', 'building'), default='building')
    parser.add_argument('--crowd', choices=('mixed', 'adversarial'), default='mixed')
    parser.add_argument('--seeds', type=seeds, default=[1])
    parser.add_argument('--flow-speed', type=float, default=1.)
    parser.add_argument('--seconds', type=float, default=75.)
    parser.add_argument('--goal', type=float, nargs=2)
    parser.add_argument('--people', type=int, choices=range(0, 8), default=7)
    parser.add_argument('--jobs', type=int, default=os.cpu_count() or 4)
    parser.add_argument('--output', type=Path, help='append one JSON line per run')
    parser.add_argument('--param', action='append', default=[], metavar='NAME=VALUE',
                        help='override a predictive_mpc constant or Config field (tuning)')
    parser.add_argument('--trace', type=float, nargs=2, metavar=('FROM', 'TO'),
                        help='print every planning step in this sim-time window')
    parser.add_argument('--course', help='a course world (submission/worlds/NAME.xml) instead')
    parser.add_argument('--speed', type=float, default=.5,
                        help='predictive speed cap, m/s (HARP_PREDICTIVE_SPEED; the image: 0.3)')
    parser.add_argument('--plant', type=float, nargs=3, metavar=('GAIN_X', 'GAIN_Y', 'DELAY'),
                        help='walker response: forward and lateral gain, delay s (default .85 .55 .25)')
    args = parser.parse_args()
    jobs = [dict(world=args.world, crowd=args.crowd, seed=s, flow=args.flow_speed,
                 seconds=args.seconds, goal=args.goal, people=args.people, trace=args.trace,
                 course=args.course, speed=args.speed, plant=args.plant,
                 params={k: (int(v) if v.lstrip('-').isdigit() else float(v))
                         for k, v in (p.split('=', 1) for p in args.param)})
            for s in args.seeds]
    with ProcessPoolExecutor(max_workers=min(args.jobs, len(jobs))) as pool:
        results = list(pool.map(run, jobs))
    for r in results:
        line = json.dumps(r)
        print(line, flush=True)
        if args.output:
            with open(args.output, 'a') as out:
                out.write(line+'\n')
    passed = sum(r['passed'] for r in results)
    print(f'{passed}/{len(results)} passed; people contacts {sum(r["people_contacts"] for r in results)}, '
          f'wall contacts {sum(r["wall_contacts"] for r in results)}, '
          f'median time {np.median([r["seconds"] for r in results]):.1f} s', file=sys.stderr)
    return 0 if passed == len(results) else 1


if __name__ == '__main__':
    sys.exit(main())
