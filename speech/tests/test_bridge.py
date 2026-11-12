"""speech/bridge.py against a real web.py server, with stand-ins for talk.py and listen.py
(no models, LLM or sound). The last test also runs it over ROS 2, if rclpy is there."""
import json
import os
import subprocess
import sys
import tempfile
import threading
import time
import unittest
from http.server import ThreadingHTTPServer
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import bridge  # noqa: E402
import web  # noqa: E402

# Like talk.py: with --wait it loads, says loaded and waits for go (its stdin closing first:
# not_started); then its greeting; each typed line a turn, "bye" ends it; stdin closing too.
TALK = """import json, sys, time
def say(**e): print(json.dumps(e), flush=True)
if "--wait" in sys.argv:
    time.sleep(0.3)
    say(event="loaded")
    for line in sys.stdin:
        if json.loads(line).get("go"):
            break
    else:
        say(event="done", reason="not_started", turns=0, seconds=0.0)
        sys.exit(0)
say(event="speaking", text="Hi, I'm HARP.", first=True, seconds=0.5)
say(event="said", text="Hi, I'm HARP.", canned=True)
turns = 0
for line in sys.stdin:
    turns += 1
    if "bye" in json.loads(line).get("text", ""):
        say(event="done", reason="bye", turns=turns, seconds=4.0)
        sys.exit(0)
say(event="done", reason="input_ended", turns=turns, seconds=2.0)
"""
LISTEN = "import sys\nfor line in sys.stdin: pass\n"   # until web.py's Ctrl-C


class Heard:
    """The bridge's events, and a wait for one."""

    def __init__(self):
        self.events, self.changed = [], threading.Condition()

    def __call__(self, text):
        with self.changed:
            self.events.append(json.loads(text))
            self.changed.notify_all()

    def wait(self, name, talk, timeout=10):
        with self.changed:
            self.changed.wait_for(lambda: self.find(name, talk), timeout)
            return self.find(name, talk)

    def find(self, name, talk):
        return next((e for e in self.events if e["event"] == name and e.get("talk") == talk), None)

    def names(self, talk):
        with self.changed:
            return [e["event"] for e in self.events if e.get("talk") == talk]


class Bridged(unittest.TestCase):
    def setUp(self):
        folder = tempfile.TemporaryDirectory()
        self.addCleanup(folder.cleanup)
        scripts = {}
        for name, text in (("talk.py", TALK), ("listen.py", LISTEN)):
            scripts[name] = Path(folder.name) / name
            scripts[name].write_text(text)
        self.llm = "ok"
        for target in (patch.object(web, "llm_status", lambda url: self.llm),
                       patch.object(web, "port_taken", return_value=False),
                       patch.object(web.Session, "command",
                                    lambda s, name, *args: [sys.executable, str(scripts[name])] + [str(a) for a in args])):
            target.start()
            self.addCleanup(target.stop)
        self.server = web.Server()
        self.httpd = ThreadingHTTPServer(("127.0.0.1", 0), web.handler(self.server))
        self.httpd.daemon_threads = True
        threading.Thread(target=self.httpd.serve_forever, daemon=True).start()
        self.addCleanup(self.close)
        self.url = "http://127.0.0.1:%d" % self.httpd.server_port
        self.heard = Heard()
        self.bridge = self.start_bridge(self.url)

    def start_bridge(self, url):
        b = bridge.Bridge(bridge.Web(url), self.heard, {"listen": True, "talk": True, "source": "mic"})
        for work in (b.run, b.follow):
            threading.Thread(target=work, daemon=True).start()
        self.addCleanup(b.close)
        time.sleep(0.2)   # its event stream connected
        return b

    def close(self):
        self.server.close()
        self.httpd.shutdown()
        self.httpd.server_close()

    def send(self, what, person, talk, b=None, **fields):
        (b or self.bridge).on_talk(json.dumps({what: person, "talk": talk, **fields}))

    def test_prepared_talk_greets_on_start_and_ends_with_its_goodbye(self):
        self.send("prepare", "person_a", "t1")
        self.send("start", "person_a", "t1", distance=1.1)   # before it has loaded (0.3 s): held
        start = self.heard.wait("talk_start", "t1")
        self.assertTrue(start and start["prepared"])
        self.assertIsNotNone(self.heard.wait("greeting", "t1"))
        self.assertTrue(self.server.session.go_asked)   # go came first, sent once loaded
        order = [(e["source"], e["event"]) for e in (json.loads(m) for m in list(self.server.hub.history))]
        self.assertLess(order.index(("talk", "loaded")), order.index(("web", "go")))
        logs = [json.loads(m) for m in list(self.server.hub.history)]
        self.assertIn("--wait", next(e["text"] for e in logs
                                     if e["event"] == "log" and "talk.py" in e.get("text", "")))
        self.assertEqual(self.server.say({"text": "bye then"})[0], 200)
        done = self.heard.wait("talk_done", "t1")
        self.assertEqual((done["reason"], done["turns"], done["person"]), ("bye", 1, "person_a"))
        self.assertEqual(self.heard.names("t1"), ["talk_start", "greeting", "talk_done"])

    def test_start_without_prepare_and_behaviors_stop(self):
        self.send("start", "person_b", "t2")
        start = self.heard.wait("talk_start", "t2")
        self.assertFalse(start["prepared"])
        self.assertIsNotNone(self.heard.wait("greeting", "t2"))
        self.send("stop", "person_b", "t2", reason="left")
        done = self.heard.wait("talk_done", "t2")
        self.assertEqual(done["reason"], "stopped")
        self.assertNotIn("error", done)

    def test_a_cancelled_prepare_ends_without_a_greeting(self):
        self.send("prepare", "person_c", "t3")
        time.sleep(0.6)   # loaded, waiting
        self.assertTrue(self.server.status()["waiting"])
        self.send("stop", "person_c", "t3", reason="cancelled")
        done = self.heard.wait("talk_done", "t3")
        self.assertEqual(done["reason"], "stopped")
        self.assertNotIn("greeting", self.heard.names("t3"))

    def test_a_new_talk_ends_the_last_one_first(self):
        self.send("start", "person_a", "t4")
        self.assertIsNotNone(self.heard.wait("greeting", "t4"))
        self.send("start", "person_b", "t5")
        self.assertEqual(self.heard.wait("talk_done", "t4")["reason"], "stopped")
        self.assertIsNotNone(self.heard.wait("greeting", "t5"))
        time.sleep(0.5)
        self.assertEqual(self.heard.names("t5"), ["talk_start", "greeting"])   # t4's end isn't t5's
        names = [e["event"] + ":" + e["talk"] for e in self.heard.events]
        self.assertLess(names.index("talk_done:t4"), names.index("talk_start:t5"))

    def test_a_stop_while_its_start_waits_greets_nobody(self):
        self.send("start", "person_a", "t10")
        self.send("stop", "person_a", "t10", reason="time")   # before the start's job ran
        time.sleep(1.5)
        self.assertEqual(self.heard.names("t10"), [])
        self.assertFalse(self.server.status()["running"])

    def test_without_the_llm_the_start_fails(self):
        self.llm = "down"
        self.send("prepare", "person_a", "t6")
        self.assertIn("LLM", self.heard.wait("prepare_failed", "t6")["error"])
        self.send("start", "person_a", "t6")
        done = self.heard.wait("talk_done", "t6")
        self.assertEqual(done["reason"], "failed")
        self.assertIn("LLM", done["error"])
        self.assertIsNone(self.heard.find("talk_start", "t6"))

    def test_a_session_started_from_the_page_is_left_alone(self):
        self.assertEqual(self.server.start({"listen": False})[0], 200)
        page = self.server.session
        self.send("start", "person_a", "t7")
        done = self.heard.wait("talk_done", "t7")
        self.assertEqual(done["reason"], "failed")
        self.assertIn("Already running", done["error"])
        self.assertTrue(page.running)

    def test_no_speech_server(self):
        b = self.start_bridge("http://127.0.0.1:9")
        self.send("start", "person_a", "t8", b)
        done = self.heard.wait("talk_done", "t8")
        self.assertEqual(done["reason"], "failed")
        self.assertIn("no speech server", done["error"])

    def test_a_prepare_never_started_is_stopped(self):
        with patch.object(bridge, "PREPARED_FOR", 0.8):
            self.send("prepare", "person_a", "t9")
            self.assertEqual(self.heard.wait("talk_done", "t9")["reason"], "stopped")

    def test_closing_the_bridge_stops_its_session(self):
        self.send("start", "person_a", "t11")
        self.assertIsNotNone(self.heard.wait("greeting", "t11"))
        self.bridge.close()
        self.assertTrue(self.server.session.done.wait(10))

    def test_over_ros(self):
        """bridge.py's own main: /harp/talk in, /harp/events out (needs rclpy)."""
        try:
            import rclpy
            from rclpy.qos import DurabilityPolicy, QoSProfile
            from std_msgs.msg import String
        except ImportError:
            self.skipTest("no ROS 2 here")
        env = dict(os.environ, ROS_DOMAIN_ID="173", ROS_LOCALHOST_ONLY="1")
        scoped = patch.dict(os.environ, ROS_DOMAIN_ID="173", ROS_LOCALHOST_ONLY="1")
        scoped.start()
        self.addCleanup(scoped.stop)
        process = subprocess.Popen([sys.executable, str(Path(bridge.__file__)), "--web", self.url],
                                   env=env, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE, text=True)
        self.addCleanup(process.wait, 10)
        self.addCleanup(process.terminate)
        lines = [process.stderr.readline() for _ in range(3)]   # Jazzy warns of ROS_LOCALHOST_ONLY first
        self.assertTrue(any("speech bridge" in line for line in lines), lines)
        rclpy.init()
        self.addCleanup(rclpy.try_shutdown)
        node = rclpy.create_node("behavior_stand_in")
        got = []
        node.create_subscription(String, bridge.EVENTS_TOPIC, lambda m: got.append(json.loads(m.data)),
                                 QoSProfile(depth=1000, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        talk = node.create_publisher(String, bridge.TALK_TOPIC, 10)
        deadline = time.monotonic() + 10
        while talk.get_subscription_count() == 0 and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        talk.publish(String(data=json.dumps({"start": "person_a", "talk": "r1"})))
        deadline = time.monotonic() + 15
        while not any(e["event"] == "greeting" for e in got) and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        self.assertEqual([e["event"] for e in got][:2], ["talk_start", "greeting"])
        self.assertEqual(got[0]["source"], "speech")
        node.destroy_node()


class FakeWeb:
    """A web.py whose event stream drops: each of streams is a list of events, or None for
    a stream that can't connect; after them, none can."""

    def __init__(self, streams, instances=("one",)):
        self.streams, self.instances, self.posts = list(streams), list(instances), []

    def post(self, path, body=None):
        self.posts.append(path)
        return None, {"error": "no speech server"}

    def status(self):
        return {"instance": self.instances.pop(0) if len(self.instances) > 1 else self.instances[0]}

    def events(self):
        stream = self.streams.pop(0) if self.streams else None
        if stream is None:
            raise OSError("refused")
        for message in stream:
            yield message


class Dropped(unittest.TestCase):
    def follow(self, web):
        heard = Heard()
        b = bridge.Bridge(web, heard, {})
        b.instance = "one"
        b.talk = bridge.Talk("t1", "person_a", False)
        b.talk.session = True
        threading.Thread(target=b.follow, daemon=True).start()
        self.addCleanup(b.closed.set)
        return heard.wait("talk_done", "t1", 15), web

    def test_a_dropped_stream_reconnects_and_carries_on(self):
        replay = [{"event": "hello"}, {"source": "web", "event": "session", "state": "started"},
                  {"source": "talk", "event": "done", "reason": "bye", "turns": 3, "seconds": 20.0},
                  {"source": "web", "event": "session", "state": "stopped"}]
        done, web = self.follow(FakeWeb([None, replay]))
        self.assertEqual((done["reason"], done["turns"]), ("bye", 3))
        self.assertNotIn("/api/stop", web.posts)

    def test_a_restarted_server_lost_the_talk(self):
        done, _ = self.follow(FakeWeb([None, [{"event": "hello"}]], instances=("two",)))
        self.assertEqual((done["reason"], done["error"]), ("failed", "the speech server restarted"))

    def test_no_stream_for_long_fails_it(self):
        with patch.object(bridge, "LOST", 1.5):
            done, web = self.follow(FakeWeb([]))
        self.assertEqual((done["reason"], done["error"]), ("failed", "lost the speech server"))
        self.assertIn("/api/stop", web.posts)


if __name__ == "__main__":
    unittest.main()
