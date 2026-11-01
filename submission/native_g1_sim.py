#!/usr/bin/env python3
"""Native G1/GR00T moving-obstacle trial using the retained Jazzy environment.

Uses pinned historical native simulator/walker source in an ignored cache.
The current tracker/filter is unchanged. This is a simulator-only integration
test; the current Foxy/HIM deployment is tested separately.
"""
import argparse
import hashlib
import shutil
import fcntl
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
from datetime import datetime, timezone
from collections import deque
from types import SimpleNamespace
from harp.local_route import route_direction

import numpy as np
import rclpy
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import Float64MultiArray, String

HERE = Path(__file__).resolve().parent
NATIVE_REV = '9ee9255dffece6f5edf106a09c299d13edcb0f63'


def world_file(path, count, static_obstacles=False):
    # Independent, staggered trajectories; all bodies exist in the physics model.
    paths = [(1.5, -3, 1.5, 3, 12, 0), (2.2, 3, 2.2, -3, 15, 2),
             (3.0, -3, 3.0, 3, 15, 5), (4.5, .0, -2, .0, 20, 6),
             (3.8, 3, 1.0, -3, 18, 8), (4.2, -3, 2.0, 3, 18, 12),
             (5, 1.5, -.5, 1.5, 20, 3)]
    parts = ['<mujoco model="seven_movers"><custom>']
    for i, (_, _, _, _, duration, delay) in enumerate(paths[:count]):
        for suffix, value in [('duration', duration), ('delay', delay), ('trips', 1)]:
            parts.append(f'<numeric name="mover{i}_{suffix}" data="{value}"/>')
    parts.append('</custom><worldbody>')
    if static_obstacles:
        for i,(x,y) in enumerate(((2.,-1.7),(3.2,1.7))):
            parts.append(f'<geom name="pillar{i}" type="box" size=".2 .2 .6" '
                         f'pos="{x} {y} .6" rgba=".2 .4 .8 1"/>')
    for i, (x, y, ex, ey, _, _) in enumerate(paths[:count]):
        parts.append(f'<body name="mover{i}" mocap="true" pos="{x} {y} 0">'
                     '<geom type="capsule" size=".20 .5" pos="0 0 .75" rgba="1 .45 .1 1"/>'
                     '<geom type="sphere" size=".15" pos="0 0 1.5" rgba="1 .7 .4 1"/>'
                     f'</body><site name="mover{i}_end" pos="{ex} {ey} 0" size=".02"/>')
    parts.append('</worldbody></mujoco>')
    path.write_text('\n'.join(parts))


def prepare():
    cache = HERE / '.build/native-g1'
    if not (cache / 'ros2/src/harp_ros/harp_ros/walker_node.py').is_file():
        cache.mkdir(parents=True, exist_ok=True)
        archive = subprocess.check_output(['git', '-C', str(HERE.parent), 'archive', NATIVE_REV,
                                          'ros2/src/harp_control', 'ros2/src/harp_sim',
                                          'ros2/src/harp_ros'])
        subprocess.run(['tar', '-x', '-C', str(cache)], input=archive, check=True)
    return cache / 'ros2/src'


class Trial:
    def __init__(self, node, run_dir, seconds, obstacle_count, goal=(6.,0.), speed=.5):
        self.node, self.run_dir, self.seconds = node, run_dir, seconds
        self.obstacle_count = obstacle_count
        self.goal = goal
        self.speed = speed
        self.began = time.monotonic()
        self.started = self.finished = None
        self.odom = self.truth = None
        self.contacts = self.movers = 0
        self.modes = {}
        self.status = 'STARTUP'
        self.trajectory = open(run_dir / 'trajectory.jsonl', 'w')
        self.events_file = open(run_dir / 'events.jsonl', 'w')
        self.min_height = math.inf
        self.max_tilt = 0.0
        self.first_truth = None
        self.last_odom = None
        self.crowd = []
        self.received_crowd = False
        self.moving_people_seen = set()
        self.tracks = []
        self.next_route = 0.
        self.route_vector = None
        self.motion_window = deque(maxlen=11)
        self.first_meter = None
        self.longest_wait = self.waiting = self.crowd_progress = 0.
        self.frontier = 0.
        self.minimum_people_separation = math.inf
        qos = QoSProfile(depth=1000, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.events = node.create_publisher(String, '/harp/events', qos)
        self.commands = node.create_publisher(Twist, '/cmd_vel_nav', 10)
        node.create_subscription(Odometry, '/odom', self.on_odom, 10)
        node.create_subscription(Float64MultiArray, '/sim/qpos', self.on_truth, 10)
        node.create_subscription(String, '/harp/events', self.on_event, qos)
        node.create_subscription(String, '/harp/obstacle_tracks', self.on_tracks, 10)
        node.create_subscription(String, '/sim/crowd', self.on_crowd, 10)
        node.create_subscription(Twist, '/cmd_vel', self.on_command, 10)
        node.create_timer(.1, self.tick)

    def event(self, event, **kw):
        self.events.publish(String(data=json.dumps(dict(source='goal', event=event, **kw))))

    def on_command(self, msg):
        self.events_file.write(json.dumps(dict(command_stamp=time.monotonic(),
            command=[msg.linear.x,msg.linear.y,msg.angular.z]))+'\n')

    def on_odom(self, msg):
        self.odom = msg
        self.last_odom = time.monotonic()

    def on_tracks(self, msg):
        self.tracks = [SimpleNamespace(**v) for v in json.loads(msg.data)['tracks']]
        self.events_file.write(msg.data+'\n')
        self.events_file.flush()

    def on_crowd(self, msg):
        value = json.loads(msg.data)
        self.crowd = value['people']
        self.received_crowd = True
        self.moving_people_seen.update(i for i,p in enumerate(self.crowd)
                                      if math.hypot(p['vx'],p['vy']) > .1)
        if len(self.crowd) > 1:
            xy = np.array([[p['x'],p['y']] for p in self.crowd])
            distances = np.linalg.norm(xy[:,None]-xy[None],axis=-1)
            np.fill_diagonal(distances,math.inf)
            self.minimum_people_separation = min(self.minimum_people_separation,float(distances.min()))
            if value.get('minimum_separation') is not None:
                self.minimum_people_separation = min(self.minimum_people_separation,value['minimum_separation'])
        self.events_file.write(msg.data+'\n')

    def on_truth(self, msg):
        self.truth = list(msg.data)
        if self.started:
            q = self.truth[1:]
            self.min_height = min(self.min_height, q[2])
            w, x, y, z = q[3:7]
            self.max_tilt = max(self.max_tilt, math.acos(np.clip(1-2*(x*x+y*y), -1, 1)))
            self.trajectory.write(json.dumps(dict(time=self.truth[0], qpos=q))+'\n')
            self.trajectory.flush()
            if self.first_truth:
                progress = q[0]-self.first_truth[0]
                if progress >= 1. and self.first_meter is None:
                    self.first_meter = time.monotonic()-self.started
                near = any(math.hypot(p['x']-q[0],p['y']-q[1]) < 2.5 and
                           math.hypot(p['vx'],p['vy']) > .15 for p in self.crowd)
                if near and progress > self.frontier:
                    self.crowd_progress += progress-self.frontier
                self.frontier = max(self.frontier,progress)
            self.motion_window.append((self.truth[0],q[0],q[1]))
            if len(self.motion_window) == 11:
                old = self.motion_window[0]
                speed = math.hypot(q[0]-old[1],q[1]-old[2])/max(.01,self.truth[0]-old[0])
                self.waiting = self.waiting+.05 if speed < .06 else 0.
                self.longest_wait = max(self.longest_wait,self.waiting)

    def on_event(self, msg):
        try:
            record = json.loads(msg.data)
        except ValueError:
            return
        if not isinstance(record, dict):
            return
        self.events_file.write(msg.data+'\n')
        self.events_file.flush()
        if record.get('event') == 'contact':
            self.contacts += 1
        if record.get('event') == 'mover_start':
            self.movers += 1
        if record.get('source') == 'predictive':
            mode = record.get('mode')
            if mode:
                self.modes[mode] = self.modes.get(mode, 0)+1
                print('Avoidance:', mode, flush=True)

    def tick(self):
        now = time.monotonic()
        command = Twist()
        if self.started is None:
            if self.odom is not None and now-self.began > 15:
                self.started = now
                self.first_truth = self.truth[1:3] if self.truth else None
                self.event('release_band')
                self.event('goal_sent', x=self.goal[0], y=self.goal[1], yaw=0.0)
                self.status = 'RUNNING'
                print(f'Walking G1 trial started; target {self.goal}, {self.obstacle_count} moving obstacles.', flush=True)
            elif now-self.began > 60:
                self.finish('STARTUP_TIMEOUT')
        if self.started is not None and self.finished is None:
            if self.last_odom is None or now-self.last_odom > .5:
                self.finish('ODOMETRY_LOST')
            else:
                p, q = self.odom.pose.pose.position, self.odom.pose.pose.orientation
                yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
                dx, dy = self.goal[0]-p.x, self.goal[1]-p.y
                distance = math.hypot(dx, dy)
                if now >= self.next_route:
                    self.route_vector = route_direction((p.x,p.y),self.goal,self.tracks)
                    self.next_route = now+.5
                speed = min(.9*self.speed, distance*.5)
                if distance < .25:
                    self.finish('SUCCEEDED')
                else:
                    rx, ry = self.route_vector
                    command.linear.x = speed*(math.cos(yaw)*rx+math.sin(yaw)*ry)
                    command.linear.y = speed*(-math.sin(yaw)*rx+math.cos(yaw)*ry)
                    heading = math.atan2(ry,rx) if distance > .8 else 0.
                    error = math.atan2(math.sin(heading-yaw),math.cos(heading-yaw))
                    command.angular.z = float(np.clip(1.2*error, -.5, .5))
                if now-self.started > self.seconds:
                    self.finish('TIMEOUT')
        if self.finished:
            command = Twist()
        self.commands.publish(command)

    def finish(self, status):
        if self.finished is None:
            self.finished, self.status = time.monotonic(), status
            self.event('goal_result', status=status)

    def report(self):
        truth_xy = self.truth[1:3] if self.truth else None
        goal_error = math.dist(truth_xy,self.goal) if truth_xy else math.inf
        mover_count = (len(self.moving_people_seen) if getattr(self,'received_crowd',False)
                       else self.movers)
        passed = (self.status == 'SUCCEEDED' and self.contacts == 0
                  and mover_count == self.obstacle_count and self.min_height > .49
                  and self.max_tilt < math.radians(42))
        crowd_progress_ok = (not getattr(self,'crowd',[]) or
            (self.crowd_progress >= 1.5 and self.first_meter is not None
             and self.first_meter <= 20. and self.longest_wait <= 8.))
        passed = passed and crowd_progress_ok
        people_geometry_valid = self.minimum_people_separation >= .38
        passed = passed and people_geometry_valid and goal_error <= .4
        return dict(status=self.status, passed=passed, contacts=self.contacts,
                    obstacle_count=self.obstacle_count, movers_started=mover_count,
                    avoidance_mode_transitions=self.modes, true_final_xy=truth_xy,
                    true_displacement_m=(math.dist(self.first_truth, truth_xy)
                                         if self.first_truth and truth_xy else None),
                    goal=list(self.goal),true_goal_error_m=(goal_error if math.isfinite(goal_error) else None),
                    minimum_pelvis_height_m=(self.min_height if math.isfinite(self.min_height) else None),
                    maximum_tilt_deg=math.degrees(self.max_tilt),
                    time_to_first_meter_s=self.first_meter,
                    progress_with_moving_people_nearby_m=self.crowd_progress,
                    longest_standstill_s=self.longest_wait,
                    seconds=(self.finished-self.started if self.finished and self.started else None),
                    crowd_progress_ok=crowd_progress_ok,
                    people_geometry_valid=people_geometry_valid,
                    minimum_people_center_separation_m=(self.minimum_people_separation
                        if math.isfinite(self.minimum_people_separation) else None),
                    humanoid_physics=True, locomotion='native GR00T ONNX walker',
                    command_source='track-based geometric route, through current predictive ROS filter',
                    him_validation=False)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--headless', action='store_true')
    parser.add_argument('--rviz', action='store_true', help='also show the live LiDAR cloud')
    parser.add_argument('--obstacles', type=int, choices=range(0, 8), default=7)
    parser.add_argument('--seconds', type=float, default=75)
    parser.add_argument('--output', type=Path)
    parser.add_argument('--seed', type=int, default=1)
    parser.add_argument('--crowd', choices=('mixed','adversarial','legacy'),default='mixed')
    parser.add_argument('--static-obstacles',action='store_true')
    parser.add_argument('--flow-speed',type=float,default=1.,help='pedestrian speed/acceleration multiplier')
    parser.add_argument('--goal',type=float,nargs=2,default=[6.,0.],help='goal beyond the crowd (x,y) metres')
    parser.add_argument('--robot-speed',type=float,default=.5)
    args = parser.parse_args()
    if not math.isfinite(args.seconds) or args.seconds <= 0:
        parser.error('--seconds must be positive and finite')
    if not math.isfinite(args.flow_speed) or not .5 <= args.flow_speed <= 2.:
        parser.error('--flow-speed must be within .5 to 2')
    if not np.isfinite(args.goal).all():
        parser.error('--goal needs finite coordinates')
    if not math.isfinite(args.robot_speed) or not .1 <= args.robot_speed <= .6:
        parser.error('--robot-speed must be within .1 to .6')
    (HERE/'.build').mkdir(exist_ok=True)
    lock = open(HERE/'.build/native-g1.lock', 'a')
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        parser.error('a native G1 trial is already running; close it before starting another')
    sources = prepare()
    main_repo = Path(subprocess.check_output(['git', '-C', str(HERE), 'rev-parse',
                                             '--path-format=absolute', '--git-common-dir'], text=True).strip()).parent
    model = main_repo.parent / 'unitree_mujoco/unitree_robots/g1/scene_29dof.xml'
    if not model.is_file():
        parser.error('Unitree G1 model missing beside main checkout')
    run_dir = args.output or main_repo / 'runs' / datetime.now(timezone.utc).strftime('%Y-%m-%d_%H-%M-%SZ_native-g1-seven')
    run_dir.mkdir(parents=True, exist_ok=False)
    world = run_dir / 'world.xml'
    world_file(world, args.obstacles,args.static_obstacles)
    snapshot = run_dir/'source'
    (snapshot/'harp').mkdir(parents=True)
    for p in list((HERE/'harp').glob('*.py'))+[Path(__file__),HERE/'native_crowd_world.py']:
        dest = snapshot/p.relative_to(HERE)
        shutil.copyfile(p,dest)
    hashes = {str(p.relative_to(snapshot)):hashlib.sha256(p.read_bytes()).hexdigest()
              for p in snapshot.rglob('*.py')}
    (run_dir/'run.json').write_text(json.dumps(dict(native_source=NATIVE_REV,
        source_hashes=hashes, obstacles=args.obstacles, seconds=args.seconds,
        headless=args.headless, seed=args.seed, crowd=args.crowd,
        static_obstacles=args.static_obstacles,flow_speed=args.flow_speed,
        goal=args.goal,
        robot_speed=args.robot_speed,
        ros_domain=185, sdk_domain=184), indent=2)+'\n')
    env = dict(os.environ)
    env.update(ROS_DOMAIN_ID='185', ROS_LOCALHOST_ONLY='1', RMW_IMPLEMENTATION='rmw_cyclonedds_cpp',
               CYCLONEDDS_HOME=sys.prefix, AMENT_PREFIX_PATH=sys.prefix,
               HARP_G1_SCENE=str(model), OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1',
               HARP_PREDICTIVE_SPEED=str(args.robot_speed),
               HARP_PREDICTIVE_TRACKING_GAIN='.85',
               HARP_PREDICTIVE_LATERAL_GAIN='.60',
               HARP_CROWD_SEED=str(args.seed), HARP_CROWD_MODE=args.crowd,
               HARP_CROWD_SPEED=str(args.flow_speed),
               HARP_RUNS_DIR=str(run_dir / 'unused'),
               CYCLONEDDS_URI='<CycloneDDS><Domain Id="any"><General><Interfaces><NetworkInterface name="lo0"/></Interfaces></General></Domain></CycloneDDS>')
    env['PATH'] = str(Path(sys.prefix)/'bin')+os.pathsep+env.get('PATH','')
    env['PYTHONPATH'] = os.pathsep.join([str(snapshot)]+[str(sources/p) for p in ['harp_sim','harp_control','harp_ros']])
    # rclpy in this parent must use the same isolated local domain as its children.
    os.environ.update({k:env[k] for k in ['ROS_DOMAIN_ID','ROS_LOCALHOST_ONLY','RMW_IMPLEMENTATION','CYCLONEDDS_URI']})
    commands = {
        'sim': [sys.executable, '-u', '-m', 'native_crowd_world', '--lidar', '--world', str(world),
                '--scene', str(model), '--domain', '184', '--interface', 'lo0',
                '--auto-release', '5', '--hold-band'] + (['--headless'] if args.headless else []),
        'walker': [sys.executable, '-u', '-m', 'harp_ros.walker_node', '--ros-args',
                   '-p', 'auto_start:=true', '-p', 'domain:=184', '-p', 'real:=false'],
        'avoidance': [sys.executable, '-u', '-m', 'harp.predictive_avoidance']}
    if args.rviz:
        config = run_dir/'lidar.rviz'
        config.write_text((sources/'harp_sim/rviz/lidar.rviz').read_text())
        commands['rviz'] = [str(Path(sys.prefix)/'bin/rviz2'), '-d', str(config)]
    children, logs = [], []
    trial = node = None
    try:
        for name, command in commands.items():
            log = open(run_dir/(name+'.log'), 'w')
            logs.append(log)
            children.append(subprocess.Popen(command, env=env, stdout=log, stderr=subprocess.STDOUT,
                                               start_new_session=True))
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        node = rclpy.create_node('native_g1_trial')
        trial = Trial(node, run_dir, args.seconds, args.obstacles,args.goal,args.robot_speed)
        print('Run folder:', run_dir, flush=True)
        while trial.finished is None or time.monotonic()-trial.finished < 3:
            rclpy.spin_once(node, timeout_sec=.1)
            if any(p.poll() is not None for p in children):
                trial.finish('CHILD_EXIT')
                break
    except KeyboardInterrupt:
        if trial:
            trial.finish('INTERRUPTED')
    finally:
        signal.signal(signal.SIGINT,signal.SIG_IGN)
        if trial:
            if rclpy.ok():
                try:
                    trial.commands.publish(Twist())
                except Exception:
                    pass  # Always terminate our owned children, even during ROS failure.
        for p in reversed(children):
            if p.poll() is None:
                os.killpg(p.pid, signal.SIGINT)
        for p in children:
            try:
                p.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(p.pid, signal.SIGKILL)
                p.wait()
        for log in logs:
            log.close()
        if trial:
            report = trial.report()
            (run_dir/'summary.json').write_text(json.dumps(report, indent=2)+'\n')
            print(json.dumps(report, indent=2), flush=True)
            trial.trajectory.close()
            trial.events_file.close()
        if node:
            node.destroy_node()
            rclpy.shutdown()
    return 0 if trial and trial.report()['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
