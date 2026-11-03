"""Opt-in Nav2 command filter: stamped LiDAR tracks -> predictive velocity requests.

This process replaces approach_stop; it never runs beside it. HIM keeps motor
ownership. Alerts are JSON signals, ready for a separate speaker/indicator consumer.
It also republishes each scan without the returns of moving objects on STATIC_TOPIC,
which Nav2's costmaps mark from in this mode (nav2.launch.py): Nav2 plans around
fixed things, and moving ones are this filter's alone.
"""
import json
import math
import os
import sys
import time
from dataclasses import asdict, replace

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
from rclpy.duration import Duration
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

from harp import camera_mount, events
from harp.detour import DetourPatience, path_length
from harp.approach_stop import CAMERA_AGE, CAMERA_MIN_Z, CAMERA_TOPIC, TOPIC, rotation, xyz
from harp.lidar_mount import FRAME
from harp.make_way import MakeWay, toward
from harp.obstacle_tracker import STRUCTURE_ID, Tracker
from harp.game_planner import GamePlanner
from harp.predictive_mpc import CONTROL_PERIOD, Config, Planner, Responsiveness

PERIOD = CONTROL_PERIOD
TIMEOUT = 0.5
MAX_SCAN_AGE = 0.35
MAX_SOLVE_TIME = 0.15
MAKE_WAY_SHARE = .6   # of the planner's speed, walking to a make-way spot
STATIC_TOPIC = "/harp/static_cloud"
MOVING_SPEED = .15   # m/s: a fresh track at least this fast is left out of STATIC_TOPIC
MOVING_SIZE = .7     # m: larger tracks (a merged blob, a wall piece) always stay in it
MOVED_DISTANCE = .3  # m a track must have travelled from where it was first seen to count as
                     # moving: a pillar seen in part as the robot came had a filter velocity of
                     # 0.24-0.31 m/s for 5 s, so it left Nav2's map for good and Nav2 kept
                     # planning through it while the filter stood the robot (HIM atrium)
STILL_RETURN = 8.    # s a track must stay within STILL_RADIUS of one spot to count as still
STILL_RADIUS = .15   # again: being seen moving had been forever, so when a walker passing a
                     # pillar dragged its track 0.25 m (HIM atrium, Nov 3) the pillar left Nav2's
                     # map for the rest of the run, Nav2 planned through it and the filter stood
                     # the robot at the gap for 80 s. 8 s is longer than a pause to give way
                     # (HIM's walkers turn back after 6 s blocked); stiller than that is in the way.
SELF_RADIUS = .45    # m from base_link: the robot's own torso and arms (seen up to 0.42 m)
# The head depth camera (approach_stop.CAMERA_TOPIC) joins each lidar scan when its frame is
# within CAMERA_AGE: its points go to the tracker and, without moving objects, to STATIC_TOPIC.
# It looks 48 deg down and sees only legs (up to about 0.35 m high at 2.7 m), so its rays
# certify a lost object gone at leg height: whatever walks has legs on the floor.
CAMERA_HEIGHTS = (.2, .35)


def seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def transform_points(points, transform):
    q, p = transform.transform.rotation, transform.transform.translation
    quat = np.array([q.x, q.y, q.z, q.w])
    shift = np.array([p.x, p.y, p.z])
    norm = np.linalg.norm(quat)
    if not np.isfinite(shift).all() or not np.isfinite(norm) or norm < 1e-6:
        raise ValueError("invalid transform")
    return points @ rotation(quat / norm).T + shift


def untransform_points(points, transform):
    """The inverse of transform_points: points in the transform's target frame -> its source."""
    q, p = transform.transform.rotation, transform.transform.translation
    quat = np.array([q.x, q.y, q.z, q.w])
    norm = np.linalg.norm(quat)
    if not np.isfinite(norm) or norm < 1e-6:
        raise ValueError("invalid transform")
    return (points - np.array([p.x, p.y, p.z])) @ rotation(quat / norm)


def static_mask(points_xy, tracks, stamp, base_xy=None, moved=(), displaced=None):
    """Returns not on an object seen moving in this scan (a person, a trolley) or before
    (moved: track ids), nor on the robot itself (within SELF_RADIUS of base_xy).

    Only compact, freshly observed tracks that move, or have moved, are removed: anything
    never seen moving, unseen or large stays, so Nav2 still plans around it. A person
    pausing to give way had joined Nav2's map, and its replanned path swung the robot's
    heading. The robot's own returns lie just past Nav2's footprint; marked and inflated,
    they enclosed its start cell. displaced: ids that have travelled MOVED_DISTANCE (None: any
    fast track counts); a fast track that hasn't moved is a still object with a noisy velocity.
    """
    keep = np.ones(len(points_xy), bool)
    if base_xy is not None:
        keep &= np.hypot(points_xy[:, 0]-base_xy[0], points_xy[:, 1]-base_xy[1]) > SELF_RADIUS
    for t in tracks:
        if (t.id < STRUCTURE_ID and t.last_seen == stamp and t.radius < MOVING_SIZE
                and ((math.hypot(t.vx, t.vy) >= MOVING_SPEED and (displaced is None or t.id in displaced))
                     or t.id in moved)):
            keep &= np.hypot(points_xy[:, 0]-t.x, points_xy[:, 1]-t.y) > t.radius+.05
    return keep


class MotionHistory:
    """Which tracks count as moving, for STATIC_TOPIC: update(tracks, stamp) -> (moved,
    displaced). A track is displaced once it has travelled MOVED_DISTANCE from where it was
    first seen, and moved once fresh, compact, fast and displaced; it stays moved (a person
    pausing to give way stays out of Nav2's map) until it has stood within STILL_RADIUS of
    one spot for STILL_RETURN, which also makes that spot its new start."""

    def __init__(self, still_return=STILL_RETURN):
        self.still_return = still_return   # inf: seen moving is forever (the rule before)
        self.moved = set()
        self.first_xy = {}   # id -> where it was first seen, or last came to rest
        self.still = {}      # id -> (spot, since): where it has stayed, from when

    def update(self, tracks, stamp):
        present = {t.id for t in tracks}
        for kept in (self.first_xy, self.still):
            for gone in set(kept)-present:
                kept.pop(gone)
        self.moved &= present
        for t in tracks:
            self.first_xy.setdefault(t.id, (t.x, t.y))
            spot, since = self.still.get(t.id, ((t.x, t.y), stamp))
            if math.hypot(t.x-spot[0], t.y-spot[1]) > STILL_RADIUS:
                spot, since = (t.x, t.y), stamp
            self.still[t.id] = (spot, since)
            if stamp-since >= self.still_return and t.id in self.moved:
                self.moved.discard(t.id)
                self.first_xy[t.id] = (t.x, t.y)
        displaced = {t.id for t in tracks
                     if math.hypot(t.x-self.first_xy[t.id][0], t.y-self.first_xy[t.id][1]) > MOVED_DISTANCE}
        self.moved |= {t.id for t in tracks if t.id < STRUCTURE_ID and t.last_seen == stamp
                       and t.radius < MOVING_SIZE and math.hypot(t.vx, t.vy) >= MOVING_SPEED
                       and t.id in displaced}
        return set(self.moved), displaced


def cloud_message(header, points):
    """A PointCloud2 of float32 x, y, z under header (the scan's frame and stamp)."""
    message = PointCloud2(header=header, height=1, width=len(points), is_bigendian=False,
                          point_step=12, row_step=12*len(points), is_dense=True)
    message.fields = [PointField(name=n, offset=4*i, datatype=PointField.FLOAT32, count=1)
                      for i, n in enumerate("xyz")]
    message.data = np.asarray(points, dtype="<f4").tobytes()
    return message


def observed_free(track, origin, points, heights=(.5, 1.0)):
    """Multiple measured rays must pass beyond the lost occupancy at two heights.

    Empty space/no returns, background receipt alone and an occluding return
    cannot certify clearance. This is a sensor-resolution test, not object truth.
    """
    delta = points-origin
    ranges = np.linalg.norm(delta,axis=1)
    valid = ranges > .1
    delta, ranges = delta[valid], ranges[valid]
    if len(ranges) < 10:
        return False
    directions = delta/ranges[:,None]
    # Clear the formerly observed shape; prediction uncertainty is not a new
    # measured object and must not turn into a permanent wall around the robot.
    radius = track.radius
    offsets = [(0,0),(radius,0),(-radius,0),(0,radius),(0,-radius)]
    for z in heights:
        for dx,dy in offsets:
            ray = np.array([track.x+dx,track.y+dy,z])-origin
            distance = np.linalg.norm(ray)
            if distance < .6:
                return False
            aligned = directions @ (ray/distance) > math.cos(.025)
            if np.count_nonzero(aligned) < 3 or np.min(ranges[aligned]) < distance+.15:
                return False
    return True


class PredictiveAvoidance:
    def __init__(self, node, buffer=None):
        self.node = node
        self.tf = buffer if buffer is not None else Buffer(cache_time=Duration(seconds=2.0))
        self.listener = None if buffer is not None else TransformListener(self.tf, node)
        self.tracker = Tracker()
        speed = float(os.environ.get('HARP_PREDICTIVE_SPEED', '.3'))
        if not .05 <= speed <= .6:
            raise ValueError('predictive speed must be within .05 to .6 m/s')
        gain = float(os.environ.get('HARP_PREDICTIVE_TRACKING_GAIN','.85'))
        lateral_gain = float(os.environ.get('HARP_PREDICTIVE_LATERAL_GAIN','.60'))
        # The game-theoretic core (harp/game_planner.py) by default; HARP_PREDICTIVE_PLANNER=mpc,
        # the sampling MPC (harp/predictive_mpc.py). Benchmark, 90 worlds of walkers who give way:
        # as often past every check (26 against 27), 31% faster, half the fight with Nav2's turns;
        # HIM: crossing 2/2, building reached 2/2, a held-out world passed every check.
        core = os.environ.get('HARP_PREDICTIVE_PLANNER', 'game')
        if core not in ('mpc', 'game'):
            raise ValueError('HARP_PREDICTIVE_PLANNER must be mpc or game')
        self.planner = (GamePlanner if core == 'game' else Planner)(Config(speed=speed,tracking_gain=gain,lateral_tracking_gain=lateral_gain,
            cooperative=os.environ.get('HARP_PREDICTIVE_COOPERATIVE', '1') != '0',
            smoothing=os.environ.get('HARP_PREDICTIVE_SMOOTHING', '1') != '0',
            heading=os.environ.get('HARP_PREDICTIVE_HEADING', '0') == '1',
            people_margin=float(os.environ.get('HARP_PREDICTIVE_PEOPLE_MARGIN', '0'))))
        # HARP_PREDICTIVE_STILL_RETURN=inf (image layer): seen moving stays forever, for A/B runs.
        self.motion = MotionHistory(float(os.environ.get('HARP_PREDICTIVE_STILL_RETURN', STILL_RETURN)))
        self.moved = set()       # ids of tracks seen moving: people, still people when they pause
        self.displaced = set()   # ids that have travelled MOVED_DISTANCE since they were still
        # Out of a tight spot it is stuck in (HARP_PREDICTIVE_MAKE_WAY=1). Off by default: in HIM's
        # office it made way 5-6 times a run but reached the goal no more often (0/3 either way).
        self.make_way = (MakeWay(self.planner.config.robot_radius)
                         if os.environ.get('HARP_PREDICTIVE_MAKE_WAY', '0') == '1' else None)
        self.making_way = False
        # Stands a while for a blocked short way instead of taking Nav2's long detour
        # (harp/detour.py; HARP_PREDICTIVE_DETOUR=0 turns it off).
        self.detour = (DetourPatience()
                       if os.environ.get('HARP_PREDICTIVE_DETOUR', '1') != '0' else None)
        self.responsiveness = Responsiveness()   # who gives way: the planner's forecast credit
        self.config = self.planner.config
        self.out = node.create_publisher(Twist, "/cmd_vel", 10)
        latched = QoSProfile(depth=1000, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.events = node.create_publisher(String, "/harp/events", latched)
        self.alerts = node.create_publisher(String, "/harp/obstacle_alert", 10)
        self.tracks_out = node.create_publisher(String, "/harp/obstacle_tracks", 10)
        self.static_out = node.create_publisher(
            PointCloud2, STATIC_TOPIC, QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        node.create_subscription(String, "/harp/events", self.on_event, latched)
        node.create_subscription(Twist, "/cmd_vel_nav", self.on_nav, 10)
        node.create_subscription(Path, "/plan", self.on_plan, 10)
        node.create_subscription(Odometry, "/odom", self.on_odom, 10)
        node.create_subscription(PointCloud2, TOPIC, self.on_cloud,
                                 QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        node.create_subscription(PointCloud2, CAMERA_TOPIC, self.on_camera,
                                 QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.camera_cloud = None   # the latest head camera frame, for the next lidar scan
        node.create_timer(PERIOD, self.on_timer)
        node.create_timer(0.05, self.on_pending)
        self.pending_cloud = None
        self.pose = self.velocity = None
        self.odom_receipt = self.scan_receipt = self.nav_receipt = None
        self.odom_stamp = self.scan_stamp = None
        self.desired = (0.0, 0.0, 0.0)
        self.active = self.failed = False
        self.goal = None
        self.perception_failed = False
        self.last_tracks, self.tracks_stamp = [], None
        self.missing_detail = {}
        self.unresolved = {}
        self.surfaces = {}
        self.mode = None
        self.last_alert = -math.inf
        self.alert_active = False

    def clock(self):
        return self.node.get_clock().now().nanoseconds * 1e-9

    def status(self, mode, alert=False, **fields):
        if mode != self.mode:
            text = events.record("predictive", "avoidance", mode=mode, **fields)
            self.events.publish(String(data=text))
            print(text, flush=True)
            self.mode = mode
        now = time.monotonic()
        if alert != self.alert_active or (alert and now - self.last_alert >= 1.0):
            self.alerts.publish(String(data=json.dumps(dict(active=alert, reason=mode,
                                                            stamp=self.clock()))))
            self.alert_active, self.last_alert = alert, now

    def stop(self, reason, alert=False, **fields):
        self.out.publish(Twist())
        if hasattr(self.planner,'previous'):
            self.planner.previous[:] = 0.
            self.planner.escape_direction = None
        self.status(reason, alert, **fields)

    def on_event(self, msg):
        try:
            value = json.loads(msg.data)
        except (TypeError, ValueError):
            return
        if not isinstance(value, dict) or value.get("source") != "goal":
            return
        event = value.get("event")
        if event in ("goal_sent", "waypoint_sent") and not self.failed:
            self.active = True
            if self.detour is not None:
                self.detour.reset()
            xy = (value.get('x'),value.get('y'))
            if all(isinstance(v,(float,int)) and math.isfinite(v) for v in xy):
                self.goal = xy
        elif event == "goal_result":
            self.active = False
            self.failed |= value.get("status") != "SUCCEEDED"
            self.stop("route_failed" if self.failed else "idle")

    def on_nav(self, msg):
        command = (msg.linear.x, msg.linear.y, msg.angular.z)
        if not np.isfinite(command).all():
            self.nav_receipt = None
            self.stop("invalid_command", True)
            return
        self.desired = command
        self.nav_receipt = time.monotonic()

    def on_plan(self, msg):
        if self.detour is None or not self.active:
            return
        length = path_length([(p.pose.position.x, p.pose.position.y) for p in msg.poses])
        change = self.detour.update(self.clock(), length)
        if change:
            print(events.record('predictive', 'detour', action=change, length=round(length, 2),
                                short=round(self.detour.short or length, 2)), flush=True)

    def on_odom(self, msg):
        stamp, now = seconds(msg.header.stamp), self.clock()
        p, q, v = msg.pose.pose.position, msg.pose.pose.orientation, msg.twist.twist
        values = (p.x, p.y, q.x, q.y, q.z, q.w, v.linear.x, v.linear.y, v.angular.z)
        norm = math.sqrt(sum(a*a for a in (q.x, q.y, q.z, q.w)))
        if (msg.header.frame_id != "odom" or msg.child_frame_id != "base_link"
                or not np.isfinite(values).all() or norm < 1e-6
                or not 0 <= now - stamp <= TIMEOUT
                or (self.odom_stamp is not None and stamp <= self.odom_stamp)):
            return
        x, y, z, w = (q.x/norm, q.y/norm, q.z/norm, q.w/norm)
        yaw = math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))
        self.pose = (p.x, p.y, yaw)
        self.velocity = (v.linear.x, v.linear.y, v.angular.z)
        self.odom_stamp, self.odom_receipt = stamp, time.monotonic()

    def on_cloud(self, cloud):
        self.pending_cloud = cloud
        self.on_pending()

    def on_camera(self, cloud):
        if cloud.header.frame_id == camera_mount.FRAME:
            self.camera_cloud = cloud

    def camera_points(self, stamp):
        """(odom points, camera origin) of the camera frame nearest this scan, or None: the
        camera is optional, so a stale frame, a missing transform or bad data skip it."""
        cloud = self.camera_cloud
        if cloud is None or abs(seconds(cloud.header.stamp) - stamp) > CAMERA_AGE:
            return None
        try:
            if cloud.width * cloud.height > 100000:
                return None
            local = xyz(cloud)
            local = local[np.isfinite(local).all(axis=1)]
            if not len(local):
                return None
            world = self.tf.lookup_transform("odom", camera_mount.FRAME, Time.from_msg(cloud.header.stamp))
            return transform_points(local, world), transform_points(np.zeros((1, 3)), world)[0]
        except (TransformException, ValueError, TypeError):
            return None

    def on_pending(self):
        cloud = self.pending_cloud
        if cloud is None:
            return
        stamp, now = seconds(cloud.header.stamp), self.clock()
        if (cloud.header.frame_id != FRAME or not 0 <= now - stamp <= MAX_SCAN_AGE
                or (self.scan_stamp is not None and stamp <= self.scan_stamp)):
            self.pending_cloud = None
            return
        try:
            if cloud.width * cloud.height > 100000:
                raise ValueError("cloud exceeds input limit")
            local = xyz(cloud)
            local = local[np.isfinite(local).all(axis=1)]
            if not len(local):
                raise ValueError("empty cloud")
            at = Time.from_msg(cloud.header.stamp)
            # Both transforms use observation time, including the moving head mount.
            world = self.tf.lookup_transform("odom", FRAME, at)
            body = self.tf.lookup_transform("odom", "base_link", at)
            points = transform_points(local, world)
            origin = transform_points(np.zeros((1,3)), world)[0]
            base = transform_points(np.zeros((1, 3)), body)[0]
            camera = self.camera_points(stamp)
            tracks = self.process_scan(points, origin, base, stamp, camera)
            if self.pose is not None and self.velocity is not None and self.config.cooperative:
                self.planner.credits = self.responsiveness.update(tracks, self.pose, self.velocity)
                if hasattr(self.planner, 'stubborn'):
                    self.planner.stubborn = self.responsiveness.stubborn
            self.moved, self.displaced = self.motion.update(tracks, stamp)
        except TransformException:
            # DDS can deliver a scan before its TF. Retry one bounded pending scan,
            # keeping its original timestamp; the watchdog still holds zero if stale.
            return
        except (ValueError, TypeError, OverflowError) as error:
            self.pending_cloud = None
            self.scan_receipt = None
            self.stop("unusable_scan", True, message=str(error))
            return
        self.pending_cloud = None
        self.scan_stamp, self.scan_receipt = stamp, time.monotonic()
        moved = self.moved if self.config.smoothing else ()
        static = local[static_mask(points[:, :2], self.last_tracks, stamp, base[:2], moved, self.displaced)]
        if camera is not None:
            # The camera's still points too (its floor left out), in the lidar's frame: Nav2
            # marks only from here in predictive mode, so walkers' legs stay out of its costmaps.
            seen = camera[0][camera[0][:, 2] >= CAMERA_MIN_Z]
            seen = seen[static_mask(seen[:, :2], self.last_tracks, stamp, base[:2], moved, self.displaced)]
            static = np.vstack([static, untransform_points(seen, world)]).astype(np.float32)
        self.static_out.publish(cloud_message(cloud.header, static))
        credits = getattr(self.planner, 'credits', {})
        # unresolved: lost objects kept for planning (planning_tracks), for recordings.
        self.tracks_out.publish(String(data=json.dumps(dict(frame="odom", stamp=stamp, tracks=[
            dict(asdict(t), credit=credits.get(t.id)) for t in tracks],
            unresolved=[asdict(t) for t in self.unresolved.values()]))))

    def process_scan(self, points, origin, base, stamp, camera=None):
        """Odom-frame points, lidar origin and base position (and the camera's (odom points,
        origin), or None) -> planning tracks.

        Free of ROS types, so offline harnesses run exactly this pipeline.
        Raises ValueError (tracker capacity, bad data) without changing memory.
        """
        distance = np.linalg.norm(points[:, :2] - base[:2], axis=1)
        keep = ((points[:, 2] >= 0.15) & (points[:, 2] <= 1.8)
                & (distance > SELF_RADIUS) & (distance <= 6.0))
        seen = points[keep, :2]
        if camera is not None:
            near = np.linalg.norm(camera[0][:, :2] - base[:2], axis=1)
            low = max(.15, CAMERA_MIN_Z)
            seen = np.vstack([seen, camera[0][(camera[0][:, 2] >= low) & (camera[0][:, 2] <= 1.8)
                                              & (near > SELF_RADIUS) & (near <= 6.0), :2]])
        tracks = self.tracker.update(seen, stamp, origin)
        observed = {t.id for t in tracks if t.last_seen == stamp}
        # Static wall tiles cannot approach while unseen, and no ray ever passes
        # beyond a wall, so they never enter lost-occupancy memory (it grew past
        # 200 retained cells in a building and stalled planning).
        candidates = {t.id:t for t in tracks if stamp-t.last_seen > .25 and t.id < STRUCTURE_ID}
        candidates.update(self.unresolved)
        cleared = {ident for ident,t in candidates.items() if observed_free(t,origin,points)
                   or (camera is not None and observed_free(t,camera[1],camera[0],CAMERA_HEIGHTS))}
        vectors = points-origin
        ranges = np.linalg.norm(vectors,axis=1)
        directions = vectors/np.maximum(ranges[:,None],1e-9)
        fresh = [t for t in tracks if t.id in observed]
        for ident in candidates:
            if ident not in self.surfaces:
                continue
            remaining = []
            for surface in self.surfaces[ident]:
                # Match Nav2's footprint clearing for observations now occupied
                # by the robot itself; elsewhere require measured rays beyond it.
                if np.linalg.norm(surface[:2]-base[:2]) <= .4:
                    continue
                # A remembered cell inside fresh occupancy is accounted for by
                # that observation (for example a wall piece now seen whole).
                if any(math.hypot(surface[0]-u.x,surface[1]-u.y)+surface[3] <= u.radius
                       for u in fresh if u.id != ident):
                    continue
                ray = surface[:3]-origin
                length = np.linalg.norm(ray)
                aligned = directions@(ray/max(length,1e-9)) > math.cos(.025)
                if np.count_nonzero(aligned) >= 3 and np.min(ranges[aligned]) > length+.15:
                    continue
                # Returns within the cell itself, nothing in front: it is observed again and
                # this scan's tracks hold whatever is there. Rays never pass beyond a wall
                # end or block corner, so without this such pieces stayed remembered.
                if (np.count_nonzero(aligned) >= 3
                        and np.min(ranges[aligned]) >= length-surface[3]-.05):
                    continue
                remaining.append(surface)
            self.surfaces[ident] = remaining
            if not remaining:
                cleared.add(ident)
        # Reassociation after a merge can assign a new ID to the same occupied
        # shape. Retire the old ID only if fresh occupancy fully encloses it;
        # this transfers occupancy rather than interpreting loss as free space.
        for ident,t in candidates.items():
            if any(u.id != ident and math.hypot(t.x-u.x,t.y-u.y)+t.radius
                   <= u.radius+u.uncertainty for u in fresh):
                cleared.add(ident)
        self.tracker.clear_observed_free(cleared)
        tracks = [t for t in tracks if t.id not in cleared]
        for ident in observed | cleared:
            self.unresolved.pop(ident,None)
        self.perception_failed = bool(self.unresolved)
        observed |= cleared
        self.check_missing(stamp, observed)
        self.last_tracks, self.tracks_stamp = tracks, stamp
        occupied = points[keep]
        for t in tracks:
            if t.id not in observed or t.id in cleared or t.id >= STRUCTURE_ID:
                continue
            returns = occupied[np.linalg.norm(occupied[:,:2]-[t.x,t.y],axis=1) <= t.radius]
            if not len(returns):
                continue
            cells = np.floor(returns[:,:2]/.2).astype(int)
            _,labels = np.unique(cells,axis=0,return_inverse=True)
            surfaces = []
            for label in range(labels.max()+1):
                group = returns[labels == label]
                sample = group[np.argmin(abs(group[:,2]-.9))]
                radius = float(np.max(np.linalg.norm(group[:,:2]-sample[:2],axis=1))+.04)
                surfaces.append(np.r_[sample,radius])
            self.surfaces[t.id] = surfaces
        retained = {t.id for t in tracks}|set(self.unresolved)
        self.surfaces = {k:v for k,v in self.surfaces.items() if k in retained}
        return self.planning_tracks(stamp)

    def planning_tracks(self, stamp):
        tracks = self.tracker.tracks_at(stamp)
        present = {t.id for t in tracks}
        for ident,t in self.unresolved.items():
            if ident not in present:
                surfaces = self.surfaces.get(ident)
                if surfaces:
                    tracks.extend(replace(t,id=-100000-ident*100-i,x=float(p[0]),y=float(p[1]),
                        vx=0.,vy=0.,radius=float(p[3]),uncertainty=.05)
                        for i,p in enumerate(surfaces))
                else:
                    tracks.append(replace(t,vx=0.,vy=0.,uncertainty=.35))
        return tracks

    def check_missing(self, stamp, observed=()):
        """Never treat expiration of nearby unseen occupancy as evidence of clearance."""
        if self.pose is None or self.tracks_stamp is None:
            return
        dt = max(0.0, stamp - self.tracks_stamp)
        for track in self.last_tracks:
            # A brief occlusion is still represented by the track's expanding
            # occupancy in the planner. Latch before expiry, rather than at the
            # sensor receipt timeout (which also stopped on normal crowd occlusion).
            if (track.id in observed or track.id >= STRUCTURE_ID or
                    stamp - track.last_seen < self.tracker.max_age - PERIOD):
                continue
            x, y = track.x + track.vx*dt, track.y + track.vy*dt
            uncertainty = track.uncertainty + 0.15*dt + 0.05*dt*dt
            # Whole body plus maximum travel during the planning horizon.
            c = self.config
            reach = c.robot_radius + c.margin + c.speed*c.horizon + track.radius + uncertainty
            if math.hypot(x-self.pose[0], y-self.pose[1]) <= reach:
                self.missing_detail = dict(track_id=track.id, position=[x, y],
                                           radius=track.radius, age=stamp-track.last_seen,
                                           robot=list(self.pose))
                self.unresolved[track.id] = replace(track,x=x,y=y,uncertainty=uncertainty)
                self.perception_failed = True
                # Keep this occupied region for planning until positive evidence
                # clears/reassociates it. An occluded actor need not stop every
                # direction of travel elsewhere in the observed scene.

    def ready(self, now, stamp):
        return (self.pose is not None and all(t is not None and 0 <= now-t <= TIMEOUT
                for t in (self.scan_receipt, self.odom_receipt, self.nav_receipt))
                and all(t is not None and 0 <= stamp-t <= TIMEOUT
                        for t in (self.scan_stamp, self.odom_stamp)))

    def on_timer(self):
        now, stamp = time.monotonic(), self.clock()
        if self.failed or not self.active:
            self.stop("route_failed" if self.failed else "idle")
            return
        self.check_missing(stamp)
        if not self.ready(now, stamp):
            self.stop("stale_input", True)
            return
        try:
            # Retain lost geometry as occupancy memory, with a bounded local
            # spatial margin. Its future velocity is unknown, not a CV estimate
            # that can be extrapolated forever. Fresh scans still gate all motion.
            tracks = self.planning_tracks(stamp)
            desired, goal = self.desired, self.goal
            if self.detour is not None and self.detour.waiting(stamp):
                desired = (0., 0., 0.)   # stand for the short way; still checked as usual
            field = getattr(self.planner, 'field', None)
            spot = self.make_way and self.make_way.update(
                stamp, self.pose, self.goal, self.mode, tracks,
                None if field is None else (lambda p: self.planner._field_at(p, 0)))
            if spot is not None:
                # Walk there instead of Nav2's way, with every usual check (harp/make_way.py).
                desired, goal = toward(self.pose, spot, MAKE_WAY_SHARE*self.planner.config.speed), spot
                if not self.making_way:
                    print(events.record('predictive', 'make_way', spot=[round(float(v), 2) for v in spot]), flush=True)
            self.making_way = spot is not None
            result = self.planner.plan(self.pose, self.velocity, desired, tracks, stamp, goal)
            # A slow solve cannot renew an obsolete movement request.
            if time.monotonic()-now > MAX_SOLVE_TIME or not self.ready(time.monotonic(), self.clock()):
                self.stop("planning_deadline", True)
                return
            if not np.isfinite(result.command).all():
                raise ValueError("nonfinite plan")
            msg = Twist()
            msg.linear.x, msg.linear.y, msg.angular.z = map(float, result.command)
            self.out.publish(msg)
            self.status(result.mode, bool(result.alert) or bool(self.unresolved), solver=str(result.solver),
                        unresolved_tracks=len(self.unresolved),
                        clearance=(float(result.clearance) if math.isfinite(result.clearance) else None))
        except (ValueError, RuntimeError, FloatingPointError):
            self.stop("planning_failure", True)


def main():
    rclpy.init(args=sys.argv)
    node = rclpy.create_node("predictive_avoidance")
    guard = PredictiveAvoidance(node)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # SIGINT may already have shut down the ROS context. Do not publish or
        # shut it down twice (Foxy-compatible; no newer signal-handler API).
        if rclpy.ok():
            try:
                guard.out.publish(Twist())
            except RuntimeError:
                if rclpy.ok():
                    raise
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
