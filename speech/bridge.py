"""The ROS 2 bridge between the robot's behavior loop (submission/harp/behavior.py) and the
speech page's server (web.py), so the robot talks with the person it walked up to: the
handshake in context/talking.md.

  python3 bridge.py                              web.py on this computer
  python3 bridge.py --web http://LAPTOP:28733    web.py on a laptop (web.py --host 0.0.0.0)

It runs where ROS 2 reaches our image: on the robot, PC2, with the harness's settings
(ROS_DOMAIN_ID=97 ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_fastrtps_cpp), under ROS's own
Python, not speech's venv: it needs only rclpy and the standard library (3.8, PC2's). web.py
and the LLM server must already be running (README.md). --source, --voice, --device and
--output-device pick what web.py's Start would.

From behavior.py, JSON on /harp/talk (std_msgs/String), each with the talk's id:
  {"prepare": person, "talk": id}    walking up to someone the camera confirmed: start a
                                     session that loads but doesn't greet (web.py's wait)
  {"start": person, "talk": id, "distance": m}   arrived: greet now, or once loaded
  {"stop": person, "talk": id, "reason": r}      the person left, or the walk up failed
Its answers, on /harp/events (source "speech", harp/events.py's format) and stdout:
  talk_start (talk, person, prepared)   speech has the conversation
  greeting (talk, after_s)              its greeting's first sound, s after start
  talk_done (talk, person, reason, turns, seconds; error if failed)   over: talk.py's
      reason (bye, goodbye, time, misses), stopped (by behavior.py or the page) or failed
  prepare_failed (talk, error)          a prepare web.py refused (start tries again)
A prepared session not started within PREPARED_FOR s is stopped. A stop that comes while
the bridge is busy cancels a start not yet sent. If web.py's event stream drops, it
reconnects and carries on (web.py replays the session's events); after LOST s without it,
or if web.py has restarted, the talk has failed. Ctrl-C stops our session too.
One conversation at a time: a new talk ends one of ours still running first. A session
someone started from the page is never stopped: the start fails instead.
"""

import argparse
import json
import queue
import sys
import threading
import time
import urllib.error
import urllib.request

WEB = "http://127.0.0.1:28733"   # web.py's default
TALK_TOPIC = "/harp/talk"
EVENTS_TOPIC = "/harp/events"
RETRY = 15.0        # s it asks web.py to start again while its last session finishes
RETRY_EVERY = 0.5   # s between those asks
ENDING = 15.0       # s for a session it stops to end (web.py gives each program 8 s)
PREPARED_FOR = 300.0   # s a prepared session waits for its start before it's stopped
                       # (behavior.py's walk up: up to 4 goals of 60 s)
LOST = 10.0         # s without web.py's event stream before a talk counts as failed


def record(name, **fields):
    """One of our events, in harp/events.py's format."""
    return json.dumps({"source": "speech", "event": name, "wall": round(time.time(), 3), **fields})


class Web:
    """web.py's HTTP API."""

    def __init__(self, url=WEB):
        self.url = url.rstrip("/")

    def post(self, path, body=None):
        """(status, reply); status None if web.py can't be reached."""
        request = urllib.request.Request(self.url + path, json.dumps(body or {}).encode(),
                                         {"Content-Type": "application/json"})
        try:
            with urllib.request.urlopen(request, timeout=10) as response:
                return response.status, json.loads(response.read() or b"{}")
        except urllib.error.HTTPError as error:
            try:
                return error.code, json.loads(error.read() or b"{}")
            except ValueError:
                return error.code, {"error": "HTTP %d" % error.code}
        except (OSError, ValueError) as error:
            return None, {"error": "no speech server at %s (%s)" % (self.url, error)}

    def status(self):
        """web.py's /api/status, or {} if it can't be reached."""
        try:
            with urllib.request.urlopen(self.url + "/api/status", timeout=10) as response:
                return json.loads(response.read() or b"{}")
        except (OSError, ValueError):
            return {}

    def events(self):
        """web.py's events as they come, one dict each, until the stream ends."""
        with urllib.request.urlopen(self.url + "/events", timeout=60) as stream:
            for line in stream:   # web.py sends a keep-alive every 15 s
                if line.startswith(b"data: "):
                    try:
                        yield json.loads(line[6:])
                    except ValueError:
                        continue


class Talk:
    """One conversation web.py runs for us."""

    def __init__(self, talk, person, prepared):
        self.id, self.person, self.prepared = talk, person, prepared
        self.session = False   # its session's "started" event has come
        self.started = None    # monotonic s of behavior's start
        self.greeted = False
        self.reason = self.turns = self.seconds = None
        self.code = None       # talk.py's exit code
        self.finished = False  # its talk_done is sent or being sent
        self.over = threading.Event()   # set once its talk_done is out


class Bridge:
    """The handshake, without ROS: on_talk() takes /harp/talk's messages, publish(text) sends
    our events. Commands run in turn on one thread (run()); web.py's events on another
    (follow())."""

    def __init__(self, web, publish, config):
        self.web, self.publish, self.config = web, publish, config
        self.lock = threading.Lock()
        self.talk = None   # ours, while web.py runs it
        self.jobs = queue.Queue()
        self.closed = threading.Event()
        self.cancelled = set()   # talks behavior.py stopped, so a start still queued is dropped
        self.instance = None     # web.py's, to tell a restart from a dropped stream

    def close(self):
        """Stops taking commands, and stops our session if one runs."""
        self.closed.set()
        self.jobs.put(None)
        with self.lock:
            ours = self.talk
        if ours is not None:
            self.web.post("/api/stop")

    def event(self, name, **fields):
        self.publish(record(name, **fields))

    def on_talk(self, text):
        try:
            message = json.loads(text)
        except ValueError:
            return
        if isinstance(message, dict) and message.get("talk"):
            if "stop" in message:
                with self.lock:
                    self.cancelled.add(message["talk"])
            self.jobs.put(message)

    def run(self):
        while True:
            message = self.jobs.get()
            if message is None:
                return
            if "prepare" in message:
                self.prepare(message["talk"], message["prepare"])
            elif "start" in message:
                self.start(message["talk"], message["start"])
            elif "stop" in message:
                self.stop(message["talk"])
            elif "expire" in message:
                self.expire(message["talk"])

    def dropped(self, talk):
        with self.lock:
            return talk in self.cancelled

    def prepare(self, talk, person):
        with self.lock:
            if (self.talk and self.talk.id == talk) or talk in self.cancelled:
                return
        self.end_ours()
        error = self.begin(talk, person, True)
        if error:
            self.event("prepare_failed", talk=talk, error=error)
            return
        timer = threading.Timer(PREPARED_FOR, self.jobs.put, args=({"expire": talk, "talk": talk},))
        timer.daemon = True
        timer.start()

    def expire(self, talk):
        """A prepared session never started (behavior.py stopped sending): stop it. Runs in
        turn with the other commands, so not as a start comes in."""
        with self.lock:
            ours = self.talk if self.talk and self.talk.id == talk else None
        if ours and ours.started is None:
            self.web.post("/api/stop")

    def start(self, talk, person):
        with self.lock:
            ours = self.talk if self.talk and self.talk.id == talk else None
        if self.dropped(talk):   # stopped while it waited in the queue: greet nobody
            return
        if ours:
            ours.started = time.monotonic()
            code, _ = self.web.post("/api/go")
            if code == 200:
                self.event("talk_start", talk=talk, person=person, prepared=True)
                return
            self.end_ours()   # its session ended while it waited: start a fresh one
        else:
            self.end_ours()
        if self.dropped(talk):
            return
        error = self.begin(talk, person, False)
        if error:
            self.event("talk_done", talk=talk, person=person, reason="failed", turns=0,
                       seconds=0.0, error=error)
        else:
            self.event("talk_start", talk=talk, person=person, prepared=False)

    def stop(self, talk):
        with self.lock:
            ours = self.talk if self.talk and self.talk.id == talk else None
        if ours:
            self.web.post("/api/stop")

    def begin(self, talk, person, wait):
        """Asks web.py to start a session for this talk; None once it has, else why not."""
        deadline = time.monotonic() + RETRY
        while True:
            ours = Talk(talk, person, wait)
            if not wait:
                ours.started = time.monotonic()
            with self.lock:
                self.talk = ours   # before the request, so its "started" event is ours
            if self.dropped(talk):
                with self.lock:
                    if self.talk is ours:
                        self.talk = None
                return "stopped before it started"
            code, reply = self.web.post("/api/start", dict(self.config, wait=wait))
            if code == 200:
                return None
            with self.lock:
                if self.talk is ours:
                    self.talk = None
            if code == 409 and reply.get("stopping") and time.monotonic() < deadline:
                time.sleep(RETRY_EVERY)
                continue
            return reply.get("error") or "speech server answered %s" % code

    def end_ours(self):
        """Stops our running session, if any, and waits for it to end."""
        with self.lock:
            ours = self.talk
        if ours is None:
            return
        self.web.post("/api/stop")
        if not ours.over.wait(ENDING):
            self.finish(ours, error="its session didn't end")

    def follow(self):
        """web.py's events, for ever. A dropped stream is reconnected and the session's events
        replayed; after LOST s without it, or once web.py has restarted, our talk failed."""
        lost = None
        while not self.closed.is_set():
            try:
                for message in self.web.events():
                    if self.closed.is_set():
                        return
                    lost = None
                    if message.get("event") == "hello":
                        self.check_instance()
                    else:
                        self.on_web(message)
            except OSError:
                pass
            with self.lock:
                ours = self.talk
            if ours is not None and ours.session:
                lost = lost or time.monotonic()
                if time.monotonic() - lost >= LOST:
                    self.web.post("/api/stop")   # if it can still be reached
                    self.finish(ours, error="lost the speech server")
                    lost = None
            time.sleep(1.0)

    def check_instance(self):
        """On (re)connecting: a web.py that has restarted lost our session."""
        instance = self.web.status().get("instance")
        with self.lock:
            ours, before, self.instance = self.talk, self.instance, instance or self.instance
        if ours is not None and ours.session and None not in (before, instance) and before != instance:
            self.finish(ours, error="the speech server restarted")

    def on_web(self, message):
        source, name = message.get("source"), message.get("event")
        with self.lock:
            ours = self.talk
        if ours is None:
            return
        if source == "web" and name == "session" and message.get("state") == "started":
            ours.session = True
            return
        if not ours.session:
            return   # an earlier session's events (a new stream replays them)
        if (source == "talk" and name == "speaking" and message.get("first")
                and not ours.greeted and ours.started is not None):
            ours.greeted = True
            self.event("greeting", talk=ours.id, after_s=round(time.monotonic() - ours.started, 2))
        elif source == "talk" and name == "done":
            ours.reason, ours.turns, ours.seconds = (message.get("reason"), message.get("turns"),
                                                     message.get("seconds"))
        elif source == "talk" and name == "exit":
            ours.code = message.get("code")
        elif source == "web" and name == "session" and message.get("state") == "stopped":
            self.finish(ours)

    def finish(self, ours, error=None):
        """Its talk_done, once: talk.py's reason, else stopped, or failed."""
        with self.lock:
            if ours.finished:
                return
            ours.finished = True
            if self.talk is ours:
                self.talk = None
        if error is None and ours.reason is None and ours.code not in (None, 0):
            error = "talk.py ended with code %s" % ours.code
        reason = "failed" if error else ours.reason or "stopped"
        if reason in ("not_started", "input_ended"):   # its stdin closed: web.py's Stop
            reason = "stopped"
        fields = {"error": error} if error else {}
        self.event("talk_done", talk=ours.id, person=ours.person, reason=reason,
                   turns=ours.turns or 0, seconds=ours.seconds or 0.0, **fields)
        ours.over.set()   # after: a next talk's talk_start comes after this talk_done


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--web", default=WEB, help="web.py's address")
    parser.add_argument("--source", default="mic", choices=("mic", "robot"),
                        help="what listening hears: this computer's mic or the robot's")
    parser.add_argument("--voice", default="kokoro", choices=("kokoro", "piper"))
    parser.add_argument("--device", type=int, help="the microphone's number (listen.py --devices)")
    parser.add_argument("--output-device", help="the speaker's number or name")
    args = parser.parse_args()
    config = {"listen": True, "talk": True, "source": args.source, "voice": args.voice,
              "device": args.device, "output_device": args.output_device}

    import rclpy
    from rclpy.qos import DurabilityPolicy, QoSProfile
    from std_msgs.msg import String

    rclpy.init(args=sys.argv)
    node = rclpy.create_node("speech_bridge")
    # /harp/events' QoS (harp/events.py latched()): late joiners get every earlier event
    events = node.create_publisher(String, EVENTS_TOPIC,
                                   QoSProfile(depth=1000, durability=DurabilityPolicy.TRANSIENT_LOCAL))

    def publish(text):
        events.publish(String(data=text))
        print(text, flush=True)

    bridge = Bridge(Web(args.web), publish, config)
    node.create_subscription(String, TALK_TOPIC, lambda msg: bridge.on_talk(msg.data), 10)
    for work in (bridge.run, bridge.follow):
        threading.Thread(target=work, daemon=True).start()
    print("speech bridge: %s <-> %s" % (TALK_TOPIC, args.web), file=sys.stderr, flush=True)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        bridge.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
