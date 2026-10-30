"""The robot on its own: wander, notice a person, walk up to them, talk, wander again.
entrypoint.sh starts it in place of harp/course_goal.py when G1_TASK_COMMAND is
{"behavior": {...}}, in a world's image in the sim and on the robot. It drives only
through Nav2 (harp/approach_stop.py still stands between Nav2 and our node), reusing
course_goal's Nav2 client.

It starts once HIM has handed control to our policy (in the sim, after HIM's start-up
balance; on the robot, D-pad Left + B), Nav2 is active and SETTLE s have passed, and
then loops until the container stops:
  - Wander: Nav2 goals to random points within radius m of where it started (in odom:
    there is no map, and leg odometry drifts), at least MIN_LEG from where it is,
    facing the way it walks. A failed or timed-out point is dropped for the next.
  - Notice: a person on /harp/people (harp/sim_people.py in the sim; nothing yet on
    the robot, which so only wanders) within NOTICE m, standing still (moved under
    STILL m/s over the last TRACK s), and not talked to in the last REGREET s. The
    nearest one cancels the wander goal.
  - Approach: one Nav2 goal STAND_OFF m from them on the robot's side, facing them,
    which the Graceful Controller reaches in one curve. If they move over REAIM m from
    where it aims, it aims again (at most REAIMS times), then gives up on them.
  - Talk: a placeholder until there is speech: a talk event with the line it would
    say, then TALK s standing (Nav2 has no goal, so our node gets no /cmd_vel and stands).

{"behavior": {"radius": 2.5, "seed": 1}}: both optional (RADIUS; a seed makes the
wander points the same every run).

Events on /harp/events (source "behavior"): loop_start (radius, seed, home),
wander_goal (x, y, yaw), wander_result (status, seconds), noticed (person, x, y,
distance), approach_goal (person, x, y, yaw), approach_result (person, status,
seconds), talk (person, line, seconds, distance), talk_done (person); error
(message) if it stops.
"""

import json
import math
import os
import random
import sys
import time
from collections import deque

import rclpy
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String

from harp.course_goal import NAV2_START, SETTLE, go_to, is_control_ready, spin_until, wait_for_active
from harp.leg_odometry import yaw_of

RADIUS = 2.5      # m from where it started that it wanders within, by default
MIN_LEG = 1.0     # m at least from where it is to the next wander point
WANDER_TIME = 60.0     # s for each wander point
APPROACH_TIME = 60.0   # s for each approach goal
NOTICE = 4.0      # m: people farther away are left alone (the detector we want reaches 4 m)
STILL = 0.3       # m/s: slower than this over TRACK counts as standing
TRACK = 1.0       # s of a person's track looked back over
SEEN = 0.5        # s since a person was last seen, after which they don't count as there
REGREET = 120.0   # s before it walks up to someone it talked with again
STAND_OFF = 1.0   # m from the person's centre to the robot's, to talk
REAIM = 0.5       # m a person may move from where it aims before it aims again
REAIMS = 3        # times it aims again before giving up on someone
TALK = 4.0        # s it stands talking
LINE = "Hi, I'm HARP, Stanford's humanoid. Nice to meet you!"


class Behavior:
    def __init__(self, node, radius=RADIUS, seed=None):
        self.node, self.radius, self.seed = node, radius, seed
        self.random = random.Random(seed)
        latched = QoSProfile(depth=1000, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.events = node.create_publisher(String, "/harp/events", latched)
        self.ready = False
        node.create_subscription(String, "/harp/events", self.on_event, latched)
        node.create_subscription(Odometry, "/odom", self.on_odom, 10)
        node.create_subscription(String, "/harp/people", self.on_people, 10)
        self.pose = None          # x, y, yaw in odom
        self.tracks = {}          # person id -> deque of (monotonic s, x, y in odom)
        self.talked = {}          # person id -> monotonic s of the last talk
        self.nav = ActionClient(node, NavigateToPose, "navigate_to_pose")

    def event(self, name, **fields):
        text = json.dumps({"source": "behavior", "event": name, **fields})
        self.events.publish(String(data=text))
        print(text, flush=True)

    def on_event(self, msg):
        self.ready = self.ready or is_control_ready(msg.data)

    def on_odom(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        self.pose = (p.x, p.y, yaw_of((q.w, q.x, q.y, q.z)))

    def on_people(self, msg):
        """Each detection into odom with the latest pose, appended to its person's track."""
        if self.pose is None:
            return
        now = time.monotonic()
        x0, y0, yaw = self.pose
        c, s = math.cos(yaw), math.sin(yaw)
        for p in json.loads(msg.data).get("people", []):
            track = self.tracks.setdefault(p["id"], deque())
            track.append((now, x0 + c * p["x"] - s * p["y"], y0 + s * p["x"] + c * p["y"]))
            while now - track[0][0] > 2 * TRACK:
                track.popleft()

    def where(self, person):
        """(x, y) in odom where the person was last seen, if within SEEN s, else None."""
        track = self.tracks.get(person)
        if not track or time.monotonic() - track[-1][0] > SEEN:
            return None
        return track[-1][1:]

    def standing(self, person):
        track = self.tracks.get(person)
        if self.where(person) is None or track[-1][0] - track[0][0] < TRACK:
            return False
        t1, x1, y1 = track[-1]
        t0, x0, y0 = next(p for p in track if t1 - p[0] <= TRACK)
        return t1 > t0 and math.hypot(x1 - x0, y1 - y0) / (t1 - t0) < STILL

    def distance(self, xy):
        return math.hypot(xy[0] - self.pose[0], xy[1] - self.pose[1])

    def someone(self):
        """The nearest person worth walking up to now, or None."""
        now = time.monotonic()
        near = [(self.distance(self.where(p)), p) for p in self.tracks
                if self.standing(p) and now - self.talked.get(p, -math.inf) > REGREET
                and self.distance(self.where(p)) <= NOTICE]
        return min(near)[1] if near else None

    def run(self):
        if not wait_for_active(self.node, "bt_navigator", NAV2_START):
            self.event("error", message="Nav2 not active")
            return 1
        spin_until(self.node, lambda: self.ready and self.pose is not None)
        spin_until(self.node, lambda: False, SETTLE)
        home = self.pose[:2]
        self.event("loop_start", radius=self.radius, seed=self.seed,
                   home=[round(v, 2) for v in home])
        while rclpy.ok():
            person = self.someone()
            if person is None:
                self.wander(home)
            elif self.approach(person):
                self.talk(person)

    def wander(self, home):
        """One wander goal, cut short as soon as someone is worth walking up to."""
        x, y = self.pose[:2]
        for _ in range(100):
            r, a = self.radius * math.sqrt(self.random.random()), self.random.uniform(-math.pi, math.pi)
            point = home[0] + r * math.cos(a), home[1] + r * math.sin(a)
            if math.hypot(point[0] - x, point[1] - y) >= MIN_LEG:
                break
        heading = math.degrees(math.atan2(point[1] - y, point[0] - x))
        self.event("wander_goal", x=round(point[0], 2), y=round(point[1], 2), yaw=round(heading, 1))
        began = time.monotonic()
        status, _ = go_to(self.node, self.nav, (*point, heading), began + WANDER_TIME,
                          lambda: self.someone() is not None)
        self.event("wander_result", status=status, seconds=round(time.monotonic() - began, 1))

    def approach(self, person):
        """Walk up to the person; True once there, facing them."""
        aim = self.where(person)
        self.event("noticed", person=person, x=round(aim[0], 2), y=round(aim[1], 2),
                   distance=round(self.distance(aim), 2))
        for _ in range(REAIMS + 1):
            x, y = self.pose[:2]
            away = max(self.distance(aim), 1e-3)
            goal = (aim[0] + (x - aim[0]) / away * STAND_OFF, aim[1] + (y - aim[1]) / away * STAND_OFF)
            heading = math.degrees(math.atan2(aim[1] - goal[1], aim[0] - goal[0]))
            self.event("approach_goal", person=person, x=round(goal[0], 2), y=round(goal[1], 2),
                       yaw=round(heading, 1))
            began = time.monotonic()

            def moved():
                now = self.where(person)
                return now is not None and math.dist(now, aim) > REAIM

            status, _ = go_to(self.node, self.nav, (*goal, heading), began + APPROACH_TIME, moved)
            self.event("approach_result", person=person, status=status,
                       seconds=round(time.monotonic() - began, 1))
            aim = self.where(person)
            if status != "INTERRUPTED" or aim is None:
                break
        if status != "SUCCEEDED":
            self.talked[person] = time.monotonic()   # leave them be for a while
        return status == "SUCCEEDED"

    def talk(self, person):
        """Placeholder until there is speech: say what it would say, and stand for TALK s."""
        where = self.where(person)
        self.event("talk", person=person, line=LINE, seconds=TALK,
                   distance=where and round(self.distance(where), 2))
        spin_until(self.node, lambda: False, TALK)
        self.talked[person] = time.monotonic()
        self.event("talk_done", person=person)


def settings():
    """radius and seed from G1_TASK_COMMAND {"behavior": {...}}."""
    task = json.loads(os.environ.get("G1_TASK_COMMAND") or "{}")
    options = task.get("behavior") or {}
    return float(options.get("radius", RADIUS)), options.get("seed")


def main():
    rclpy.init(args=sys.argv)
    node = rclpy.create_node("behavior")
    behavior = Behavior(node, *settings())
    try:
        return behavior.run()
    except Exception as e:
        behavior.event("error", message=str(e))
        raise
    finally:
        behavior.nav.destroy()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    sys.exit(main())
