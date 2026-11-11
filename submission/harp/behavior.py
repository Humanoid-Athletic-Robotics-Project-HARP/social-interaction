"""The robot on its own: wander, notice a person, walk up to them, talk, wander again.
entrypoint.sh starts it in place of harp/course_goal.py when G1_TASK_COMMAND is
{"behavior": {...}}, in a world's image in the sim (not yet on the robot). It drives only
through Nav2 (harp/approach_stop.py still stands between Nav2 and our node), reusing
course_goal's Nav2 client.

It starts once HIM has handed control to our policy (in the sim, after HIM's start-up
balance; on the robot, D-pad Left + B), Nav2 is active and SETTLE s have passed, and
then loops until the container stops:
  - Wander: Nav2 goals to a random point in front of it (MIN_LEG to MAX_LEG m away,
    within FRONT deg of where it faces), facing the way it walks, kept within radius m
    of where it started (in odom: there is no map, and leg odometry drifts): near the
    edge the cone widens until a point fits, and from outside it heads back to the
    start. A failed or timed-out point is dropped for the next.
  - Notice: someone on /harp/people (the stand-in harp/sim_people.py): a candidate the
    lidar found by its shape, or a person the camera confirmed by their legs (only within
    about 2 m ahead), within NOTICE m, standing still (moved under STILL m/s over the
    last TRACK s), and not walked up to in the last REGREET s. The nearest one cancels
    the wander goal. While it walks up to them, someone in the lidar's blind wedges is
    out of sight: it keeps to where it saw them.
  - Approach, in one walk: a candidate gets a Nav2 goal CHECK m from them on the
    robot's side, facing them, where the camera sees their legs. Once the camera has
    confirmed them a person (on the way, or within CONFIRM s of getting there), the goal
    is replaced without stopping by one STAND_OFF m from them, which the Graceful
    Controller reaches in one curve; someone already confirmed gets that one at once. A
    candidate never confirmed is not a person (a pillar, say): left alone for REGREET s.
    If they move over REAIM m from where it aims, it aims again (at most REAIMS times),
    then gives up on them.
  - Talk, standing (Nav2 has no goal, so our node gets no /cmd_vel), through speech's
    bridge (speech/bridge.py, context/talking.md) on /harp/talk: once the camera has
    confirmed someone it walks up to, {"prepare": person, "talk": id} (speech loads its
    voice and LLM); on arriving, {"start": person, "talk": id, "distance": m}. Until speech
    takes it (a talk_start event, source speech; never with no bridge, as in the sim), the
    placeholder: stand until the person leaves (out of sight for GONE s, or over LEFT m
    away) or TALK s pass, which stands in for them saying goodbye. Once speech has it, it
    stands until speech's talk_done (the conversation's end), or until the person leaves or
    LONGEST s pass. A talk that didn't end in speech's talk_done gets {"stop": person,
    "talk": id, "reason": left or time} (and up to STOP_WAIT s for that talk_done if speech
    had it); a walk up that fails after a prepare gets one with reason cancelled.
  - Turn away: right after the talk, a turn in place to face the other way (a Nav2 goal
    where it stands, which the Graceful Controller only turns to), then the loop again.

{"behavior": {"radius": 2.5, "seed": 1}}: both optional (RADIUS; a seed makes the
wander points the same every run).

Events on /harp/events (source "behavior"): loop_start (radius, seed, home),
wander_goal (x, y, yaw), wander_result (status, seconds), noticed (person, x, y,
distance, confirmed), approach_goal (person, stage: check or final, x, y, yaw),
approach_result (person, status, seconds), confirmed (person, distance), not_a_person
(person), talk (person, line, distance, talk), talk_done (person, ended, seconds,
speech: whether speech had it; ended is speech's reason, such as bye, goodbye, misses,
failed, or left or time; error from speech if it failed), turn_away (yaw), turn_result (status, seconds); resend (x, y, yaw, off, turn) when
Nav2 called a goal reached with the robot off m and turn deg from it; error
(message) if it stops.
"""

import json
import math
import os
import random
import sys
import time
import uuid
from collections import deque

import rclpy
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from std_msgs.msg import String

from harp import events
from harp.course_goal import NAV2_START, SETTLE, go_to, is_control_ready, spin_until, wait_for_active
from harp.leg_odometry import yaw_of

RADIUS = 2.5      # m from where it started that it wanders within, by default
MIN_LEG = 1.0     # m at least from where it is to the next wander point
MAX_LEG = 2.5     # m at most from where it is to the next wander point
FRONT = 45        # deg either side of where it faces that the next wander point is looked for
WANDER_TIME = 60.0     # s for each wander point
APPROACH_TIME = 60.0   # s for each approach goal
NOTICE = 4.0      # m: people farther away are left alone (the detector we want reaches 4 m)
STILL = 0.3       # m/s: slower than this over TRACK counts as standing
TRACK = 1.0       # s of a person's track looked back over
SEEN = 0.5        # s since a person was last seen, after which they don't count as there
REGREET = 120.0   # s before it walks up to someone it talked with again
STAND_OFF = 0.85  # m from the person's centre to the robot's goal: the goal checker's 0.25 m
                  # stops it short, at about 1.1 m
CHECK = 1.5       # m from a candidate's centre to the goal where the camera can confirm them
                  # (it sees legs within about 2 m; the goal checker stops it at about 1.75 m)
CONFIRM = 2.0     # s it waits at the check goal for the camera before it's not a person
REAIM = 0.5       # m a person may move from where it aims before it aims again
REAIMS = 3        # times it aims again before giving up on someone
TURN_TIME = 30.0  # s for the turn away after a talk
ARRIVED = 0.4     # m from a goal Nav2 calls reached beyond which it's sent again (the goal
ARRIVED_DEG = 30.0   # checker's 0.25 m and 20 deg, with room for the robot settling)
TALK = 30.0       # s at most it talks with someone (placeholder for them saying goodbye)
TALK_TOPIC = "/harp/talk"   # to speech's bridge (speech/bridge.py)
LONGEST = 240.0   # s at most it stands talking (talk.py says goodbye itself after 180 s)
STOP_WAIT = 12.0  # s for speech to finish after a stop (web.py gives talk.py 8 s)
GONE = 2.0        # s out of sight after which the person has left
LEFT = 2.0        # m away after which the person has left
LINE = "Hi, I'm HARP, Stanford's humanoid. Nice to meet you!"


class Behavior:
    def __init__(self, node, radius=RADIUS, seed=None):
        self.node, self.radius, self.seed = node, radius, seed
        self.random = random.Random(seed)
        self.event = events.Events(node, "behavior")
        self.ready = False
        node.create_subscription(String, events.TOPIC, self.on_event, events.latched())
        node.create_subscription(Odometry, "/odom", self.on_odom, 10)
        node.create_subscription(String, "/harp/people", self.on_people, 10)
        self.talk_pub = node.create_publisher(String, TALK_TOPIC, 10)
        self.speech = {}          # talk id -> {speech's event name: its fields}
        self.talk_id = None       # the talk a walk up has prepared
        self.pose = None          # x, y, yaw in odom
        self.tracks = {}          # person id -> deque of (monotonic s, x, y in odom)
        self.talked = {}          # person id -> monotonic s of the last talk
        self.people = set()       # ids the camera has confirmed are people
        self.nav = ActionClient(node, NavigateToPose, "navigate_to_pose")

    def on_event(self, msg):
        self.ready = self.ready or is_control_ready(msg.data)
        try:
            record = json.loads(msg.data)
        except ValueError:
            return
        if (isinstance(record, dict) and record.get("source") == "speech"
                and record.get("talk") in self.speech):   # only the talks it is running
            heard = self.speech[record["talk"]]
            if record.get("event") == "talk_start":
                heard.pop("talk_done", None)   # its prepared session's end, before this start
            heard[record.get("event")] = record

    def tell(self, what, person, talk, **fields):
        """A message to speech's bridge on /harp/talk."""
        self.talk_pub.publish(String(data=json.dumps({what: person, "talk": talk, **fields})))

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
            if "camera" in p.get("seen_by", []):
                self.people.add(p["id"])
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
        if not spin_until(self.node, lambda: self.ready and self.pose is not None, NAV2_START):
            self.event("error", message="control not handed over")
            return 1
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
                self.turn_away()

    def go(self, goal, deadline, interrupt=None, handoff=None):
        """course_goal.go_to's status, but a goal Nav2 calls reached while the robot is over
        ARRIVED m or ARRIVED_DEG from it is sent again (Nav2 can end a goal sent right after
        another at once), until the deadline."""
        while True:
            status, _ = go_to(self.node, self.nav, goal, deadline, interrupt, handoff)
            x, y, yaw = self.pose
            off = math.dist((x, y), goal[:2])
            turn = abs(math.degrees(math.atan2(math.sin(math.radians(goal[2]) - yaw),
                                               math.cos(math.radians(goal[2]) - yaw))))
            if (status != "SUCCEEDED" or (off <= ARRIVED and turn <= ARRIVED_DEG)
                    or time.monotonic() >= deadline):
                return status
            self.event("resend", x=round(goal[0], 2), y=round(goal[1], 2), yaw=round(goal[2], 1),
                       off=round(off, 2), turn=round(turn, 1))

    def wander(self, home):
        """One wander goal, cut short as soon as someone is worth walking up to."""
        x, y = self.pose[:2]
        point = wander_point(self.pose, home, self.radius, self.random)
        heading = math.degrees(math.atan2(point[1] - y, point[0] - x))
        self.event("wander_goal", x=round(point[0], 2), y=round(point[1], 2), yaw=round(heading, 1))
        began = time.monotonic()
        status = self.go((*point, heading), began + WANDER_TIME, lambda: self.someone() is not None)
        self.event("wander_result", status=status, seconds=round(time.monotonic() - began, 1))

    def approach(self, person):
        """Walk up to the person, checking a candidate with the camera on the way; True once
        there, facing a confirmed person."""
        aim = self.where(person)
        self.event("noticed", person=person, x=round(aim[0], 2), y=round(aim[1], 2),
                   distance=round(self.distance(aim), 2), confirmed=person in self.people)
        reaims = 0
        while True:
            final = person in self.people
            if final and self.talk_id is None:   # speech loads while it walks the rest
                self.talk_id = uuid.uuid4().hex[:12]
                self.speech[self.talk_id] = {}
                self.tell("prepare", person, self.talk_id)
            x, y = self.pose[:2]
            away = max(self.distance(aim), 1e-3)
            off = STAND_OFF if final else CHECK
            goal = (aim[0] + (x - aim[0]) / away * off, aim[1] + (y - aim[1]) / away * off)
            heading = math.degrees(math.atan2(aim[1] - goal[1], aim[0] - goal[0]))
            self.event("approach_goal", person=person, stage="final" if final else "check",
                       x=round(goal[0], 2), y=round(goal[1], 2), yaw=round(heading, 1))
            began = time.monotonic()

            def moved():
                now = self.where(person)
                return now is not None and math.dist(now, aim) > REAIM

            def confirmed():
                return not final and person in self.people

            status = self.go((*goal, heading), began + APPROACH_TIME,
                             lambda: moved() or confirmed(), confirmed)
            self.event("approach_result", person=person, status=status,
                       seconds=round(time.monotonic() - began, 1))
            if status == "SUCCEEDED" and not final:
                status = "HANDED_OFF" if spin_until(self.node, confirmed, CONFIRM) else "NOT_A_PERSON"
            if status == "HANDED_OFF":
                aim = self.where(person) or aim
                self.event("confirmed", person=person, distance=round(self.distance(aim), 2))
                continue
            if status == "NOT_A_PERSON":
                self.event("not_a_person", person=person)
            if status == "INTERRUPTED" and reaims < REAIMS and self.where(person) is not None:
                aim, reaims = self.where(person), reaims + 1
                continue
            break
        if status != "SUCCEEDED":
            self.talked[person] = time.monotonic()   # leave them be for a while
            if self.talk_id is not None:
                self.tell("stop", person, self.talk_id, reason="cancelled")
                self.speech.pop(self.talk_id, None)
                self.talk_id = None
        return status == "SUCCEEDED"

    def left(self, person):
        """True once the person is out of sight for GONE s or over LEFT m away."""
        track = self.tracks.get(person)
        return (not track or time.monotonic() - track[-1][0] > GONE
                or self.distance(track[-1][1:]) > LEFT)

    def talk(self, person):
        """Stand and talk with the person through speech; until speech takes it (never, with
        no bridge: the sim), the placeholder: stand until they leave or TALK s have passed."""
        talk, self.talk_id = self.talk_id or uuid.uuid4().hex[:12], None
        heard = self.speech.setdefault(talk, {})
        where = self.where(person)
        distance = where and round(self.distance(where), 2)
        self.tell("start", person, talk, distance=distance)
        self.event("talk", person=person, line=LINE, distance=distance, talk=talk)
        began = time.monotonic()
        spin_until(self.node, lambda: "talk_start" in heard or self.left(person), TALK)
        if "talk_start" in heard:
            spin_until(self.node, lambda: "talk_done" in heard or self.left(person),
                       LONGEST - (time.monotonic() - began))
        if "talk_start" in heard and "talk_done" in heard:
            ended = heard["talk_done"].get("reason", "stopped")
        else:
            ended = "left" if self.left(person) else "time"
            self.tell("stop", person, talk, reason=ended)   # also a speech that answers late
            if "talk_start" in heard:
                spin_until(self.node, lambda: "talk_done" in heard, STOP_WAIT)
        failed = heard.get("talk_done", {}).get("error")
        self.talked[person] = time.monotonic()
        self.speech.pop(talk, None)
        self.event("talk_done", person=person, ended=ended,
                   seconds=round(time.monotonic() - began, 1), speech="talk_start" in heard,
                   **({"error": failed} if failed else {}))

    def turn_away(self):
        """Turn in place to face away from where it faces now."""
        x, y, yaw = self.pose
        heading = math.degrees(math.atan2(-math.sin(yaw), -math.cos(yaw)))
        self.event("turn_away", yaw=round(heading, 1))
        began = time.monotonic()
        status = self.go((x, y, heading), began + TURN_TIME)
        self.event("turn_result", status=status, seconds=round(time.monotonic() - began, 1))


def wander_point(pose, home, radius, rng):
    """A point MIN_LEG to MAX_LEG m in front of the robot (within FRONT deg of where it
    faces) and within radius of home. Near the edge, facing out, none may fit: the cone
    widens FRONT deg at a time, up to all round. Outside the area, home."""
    x, y, yaw = pose
    for half in range(int(FRONT), 181, int(FRONT)):
        for _ in range(50):
            d, a = rng.uniform(MIN_LEG, MAX_LEG), yaw + math.radians(rng.uniform(-half, half))
            point = x + d * math.cos(a), y + d * math.sin(a)
            if math.dist(point, home) <= radius:
                return point
    return tuple(home)


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
