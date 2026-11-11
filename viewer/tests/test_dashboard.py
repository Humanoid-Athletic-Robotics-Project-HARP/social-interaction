"""Exercise independent sources and the HTTP boundary without robot or model packages."""
import contextlib
import http.client
import json
from pathlib import Path
import sys
import threading
import time
import unittest
import urllib.error
import urllib.parse
import urllib.request
from unittest.mock import patch
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from feeds import Source, validate
from server import Dashboard, handler, page


@contextlib.contextmanager
def serving(handler_class):
    server = ThreadingHTTPServer(("127.0.0.1", 0), handler_class)
    server.daemon_threads = True
    thread = threading.Thread(target=server.serve_forever, kwargs={"poll_interval": .02}, daemon=True)
    thread.start()
    try:
        yield "http://127.0.0.1:%s" % server.server_port
    finally:
        server.shutdown()
        server.server_close()
        thread.join(2)


class Producer(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"
    instance = "first"
    lidar = True
    depth = False
    requests = []

    def log_message(self, *args):
        pass

    def send(self, body, content="application/json"):
        body = body if isinstance(body, bytes) else json.dumps(body).encode()
        self.send_response(200)
        self.send_header("Content-Type", content)
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        type(self).requests.append(self.path)
        if self.path == "/api/status":
            return self.send({"instance": self.instance, "running": False, "host": "fixture-machine"})
        if self.path == "/events":
            self.send_response(200)
            self.send_header("Content-Type", "text/event-stream")
            self.send_header("Connection", "close")
            self.end_headers()
            event = {"n": 1, "source": "listen", "event": "turn", "text": self.instance}
            self.wfile.write(("data: " + json.dumps(event) + "\n\n").encode())
            self.wfile.flush()
            for _ in range(20):
                time.sleep(.1)
                try:
                    self.wfile.write(b": heartbeat\n\n")
                    self.wfile.flush()
                except OSError:
                    break
            self.close_connection = True
            return
        if self.path.endswith("/calib"):
            return self.send({"connected": self.lidar, "why": None if self.lidar else "sensor stopped"})
        if self.path.endswith("/stats"):
            return self.send({"depth_connected": self.depth, "depth_why": "no depth frames", "colour_connected": True})
        if self.path.endswith("/points"):
            return self.send(b"", "application/octet-stream")
        return self.send({"inputs": [], "outputs": []})

    def do_POST(self):
        body = json.loads(self.rfile.read(int(self.headers["Content-Length"])))
        self.send({"accepted": body})


def eventually(fn, timeout=4):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if fn():
            return
        time.sleep(.02)
    raise AssertionError("condition did not become true")


class DashboardTests(unittest.TestCase):
    def test_empty_dashboard_has_no_optional_imports(self):
        dashboard = Dashboard({"sources": []})
        with serving(handler(dashboard)) as url:
            self.assertEqual(json.load(urllib.request.urlopen(url + "/api/state"))["sources"], [])
            html = urllib.request.urlopen(url).read()
            self.assertIn(b"HARP viewer", html)
        # Host health is read on start, from the standard library only.
        dashboard.start()
        eventually(lambda: dashboard.state()["host"] is not None)
        self.assertIn("ready", dashboard.state()["host"])
        dashboard.close()
        for package in ("numpy", "rerun", "cv2", "rclpy", "sounddevice", "sherpa_onnx"):
            self.assertNotIn(package, sys.modules)

    def test_source_validation(self):
        for sources in ([{"id": "bad/path", "kind": "speech", "url": "http://localhost"}],
                        [{"id": "a", "kind": "speech", "url": "file:///etc/passwd"}],
                        [{"id": "a", "kind": "speech", "url": "http://localhost"}] * 2,
                        [{"id": "a", "kind": "speech", "url": "http://localhost", "controls": "false"}]):
            with self.assertRaises(ValueError):
                validate({"sources": sources})

    def test_concurrent_feeds_partial_sensor_failure_and_recovery(self):
        with serving(Producer) as url:
            dashboard = Dashboard({"sources": [
                {"id": "sensors", "kind": "sensors", "url": url},
                {"id": "speech", "kind": "speech", "url": url},
                {"id": "missing", "kind": "speech", "url": "http://127.0.0.1:1"}]})
            dashboard.start()
            try:
                sensors, speech, missing = dashboard.sources.values()
                eventually(lambda: sensors.status()["reachable"] and speech.status()["feeds"].get("speech", {}).get("online"))
                self.assertTrue(sensors.status()["feeds"]["lidar"]["online"])
                self.assertFalse(sensors.status()["feeds"]["depth"]["online"])
                self.assertTrue(sensors.status()["feeds"]["colour"]["online"])
                self.assertFalse(missing.status()["reachable"])
                Producer.lidar = False
                eventually(lambda: not sensors.status()["feeds"]["lidar"]["online"])
                self.assertTrue(speech.status()["reachable"])
                self.assertTrue(sensors.status()["feeds"]["colour"]["online"])
                Producer.lidar = True
                eventually(lambda: sensors.status()["feeds"]["lidar"]["online"])
                # An empty but current cloud is a valid sample.
                self.assertEqual(sensors.request("lidar/points")[2], b"")
            finally:
                Producer.lidar = True
                dashboard.close()

    def test_history_is_bounded_deduplicated_and_separate(self):
        a = Source({"id": "a", "kind": "speech", "url": "http://localhost"})
        b = Source({"id": "b", "kind": "speech", "url": "http://localhost"})
        a.append({"text": "first"}, ("session", 1))
        a.append({"text": "duplicate"}, ("session", 1))
        b.append({"text": "other"}, ("session", 1))
        self.assertEqual(len(a.events), 1)
        self.assertEqual(b.events[0][1]["text"], "other")
        for n in range(2000):
            a.append({"n": n})
        self.assertEqual(len(a.events), 1500)
        self.assertTrue(a.since(0)[0])
        a.set_status({}, session="restarted")
        self.assertEqual(len(a.events), 0)
        self.assertEqual(len(b.events), 1)

    def test_voice_levels_do_not_push_a_conversation_out_of_history(self):
        a = Source({"id": "a", "kind": "speech", "url": "http://localhost"})
        for turn in range(3):
            a.append({"source": "listen", "event": "turn", "text": str(turn)})
            for n in range(1000):   # about 100 s of levels
                a.append({"source": "listen", "event": "level", "rms": n})
        gap, events = a.since(None)
        self.assertEqual([e["text"] for _, e in events if e["event"] == "turn"], ["0", "1", "2"])
        self.assertEqual(len(events), 6)
        self.assertEqual(events[-1][1]["rms"], 999)    # the newest level
        self.assertEqual(a.since(events[-2][0]), (False, events[-1:]))   # a page following along

    def test_sensor_endpoint_failure_does_not_hide_other_sensor(self):
        source = Source({"id": "a", "kind": "sensors", "url": "http://localhost"})
        def metadata(url):
            if "/camera/" in url:
                raise OSError("camera endpoint unavailable")
            return {"connected": True}
        with patch("feeds.get_json", side_effect=metadata):
            source.start()
            try:
                eventually(lambda: source.status()["reachable"])
                self.assertTrue(source.status()["feeds"]["lidar"]["online"])
                self.assertFalse(source.status()["feeds"]["depth"]["online"])
            finally:
                source.close()

    def test_speech_service_restart_replaces_history(self):
        with serving(Producer) as url:
            source = Source({"id": "a", "kind": "speech", "url": url})
            Producer.instance = "first"
            source.start()
            try:
                eventually(lambda: bool(source.events))
                self.assertEqual(source.events[-1][1]["text"], "first")
                Producer.instance = "second"
                eventually(lambda: source.session == "second" and bool(source.events) and source.events[-1][1]["text"] == "second")
                self.assertTrue(all(event["text"] == "second" for _, event in source.events))
            finally:
                source.close()
                Producer.instance = "first"

    def test_frozen_transport_expires_and_http_proxy_is_bounded(self):
        source = Source({"id": "a", "kind": "sensors", "url": "http://localhost"})
        source.set_status({"lidar": {"online": True}})
        source.updated -= 10
        self.assertFalse(source.status()["feeds"]["lidar"]["online"])
        self.assertEqual(source.request("http://example.com")[0], 404)
        self.assertEqual(source.request("../../etc/passwd")[0], 404)
        self.assertEqual(source.request("camera/measure?x=2&y=0")[0], 400)

    def test_source_pages_reuse_presentations_without_importing_drivers(self):
        source = Source({"id": "robot", "kind": "sensors", "url": "http://localhost"})
        for view in ("lidar", "camera", "hybrid"):
            html = page(source, view)
            self.assertIn("<!doctype html>", html)
            self.assertNotIn("https://cdn.jsdelivr", html)
        source = Source({"id": "mac", "kind": "speech", "url": "http://localhost"})
        html = page(source, "speech")
        self.assertIn('/source/mac/api/status', html)
        self.assertIn('/source/mac/events', html)
        self.assertNotIn('new EventSource("/events")', html)
        self.assertIn("#end-turn{display:none", html)
        source.config["controls"] = True
        self.assertNotIn("display:none!important", page(source, "speech"))

    def test_a_page_reaching_the_hub_under_another_name_is_refused(self):
        with serving(handler(Dashboard({"sources": []}))) as url:
            port = urllib.parse.urlsplit(url).port
            for host, code in (("evil.example:%d" % port, 403), ("localhost:%d" % port, 200),
                               ("127.0.0.1:%d" % port, 200), ("[::1]:%d" % port, 200)):
                connection = http.client.HTTPConnection("127.0.0.1", port, timeout=5)
                connection.putrequest("GET", "/api/state", skip_host=True)
                connection.putheader("Host", host)
                connection.endheaders()
                self.assertEqual(connection.getresponse().status, code, host)
                connection.close()

    def test_commands_require_enabled_source_and_same_origin(self):
        with serving(Producer) as upstream:
            config = {"sources": [{"id": "a", "kind": "speech", "url": upstream}]}
            dashboard = Dashboard(config)
            with serving(handler(dashboard)) as url:
                target = url + "/source/a/api/say"
                request = urllib.request.Request(target, data=b'{"text":"hello"}', headers={"Content-Type": "application/json"})
                with self.assertRaises(urllib.error.HTTPError) as denied:
                    urllib.request.urlopen(request)
                self.assertEqual(denied.exception.code, 403)
                dashboard.sources["a"].config["controls"] = True
                self.assertEqual(json.load(urllib.request.urlopen(request))["accepted"]["text"], "hello")
                end_turn = urllib.request.Request(url + "/source/a/api/end_turn", data=b"{}", headers={"Content-Type": "application/json"})
                self.assertEqual(json.load(urllib.request.urlopen(end_turn)), {"accepted": {}})
                request.add_header("Origin", "https://elsewhere.example")
                with self.assertRaises(urllib.error.HTTPError) as denied:
                    urllib.request.urlopen(request)
                self.assertEqual(denied.exception.code, 403)
                request = urllib.request.Request(url + "/api/state", data=b"{}", headers={"Content-Type": "text/plain"})
                with self.assertRaises(urllib.error.HTTPError) as denied:
                    urllib.request.urlopen(request)
                self.assertEqual(denied.exception.code, 415)


if __name__ == "__main__":
    unittest.main()
