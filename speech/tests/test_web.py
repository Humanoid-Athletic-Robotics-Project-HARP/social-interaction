"""speech/web.py's sessions, with a stand-in for talk.py (no models, LLM or sound)."""
import http.client
import json
import sys
import threading
import tempfile
import unittest
from http.server import ThreadingHTTPServer
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import web  # noqa: E402

# Like talk.py: reads turns until its stdin closes, then takes a moment to finish its line.
TALK = "import sys, time\nfor line in sys.stdin: pass\ntime.sleep(0.7)\n"


class Sessions(unittest.TestCase):
    def setUp(self):
        folder = tempfile.TemporaryDirectory()
        self.addCleanup(folder.cleanup)
        script = Path(folder.name) / "talk.py"
        script.write_text(TALK)
        for target in (patch.object(web, "llm_status", return_value="ok"),
                       patch.object(web.Session, "command", lambda self, name, *args: [sys.executable, str(script)])):
            target.start()
            self.addCleanup(target.stop)
        self.server = web.Server()

    def events(self):
        return [json.loads(m) for m in self.server.hub.history]

    def test_start_waits_until_the_last_session_has_stopped(self):
        self.assertEqual(self.server.start({"listen": False})[0], 200)
        first = self.server.session
        self.server.stop()
        code, body = self.server.start({"listen": False})   # talk.py is still finishing
        self.assertEqual(code, 409)
        self.assertIn("Still stopping", body["error"])
        self.assertTrue(first.done.wait(5))
        self.assertEqual(self.server.start({"listen": False})[0], 200)
        self.assertNotIn(("talk", "exit"), [(e["source"], e["event"]) for e in self.events()])
        self.assertNotIn("stopped", [e.get("state") for e in self.events()])
        self.server.stop()
        self.assertTrue(self.server.session.done.wait(5))

    def test_closing_during_a_stop_waits_for_it(self):
        import threading
        import time
        self.server.start({"listen": False})
        session = self.server.session
        threading.Thread(target=session.stop, daemon=True).start()   # Stop pressed
        while not session.over:
            time.sleep(0.01)
        self.server.close()                                          # then the terminal closed
        self.assertTrue(session.done.is_set())
        self.assertIsNotNone(session.talk.poll())

    def test_a_stopped_session_publishes_its_exit_before_stopped(self):
        self.server.start({"listen": False})
        self.server.stop()
        self.assertTrue(self.server.session.done.wait(5))
        order = [(e["source"], e["event"], e.get("state")) for e in self.events()]
        self.assertLess(order.index(("talk", "exit", None)), order.index(("web", "session", "stopped")))


class Hosts(unittest.TestCase):
    def test_a_page_reaching_it_under_another_name_is_refused(self):
        httpd = ThreadingHTTPServer(("127.0.0.1", 0), web.handler(web.Server()))
        threading.Thread(target=httpd.serve_forever, daemon=True).start()
        self.addCleanup(httpd.server_close)
        self.addCleanup(httpd.shutdown)
        port = httpd.server_port
        for method, host, code in (("GET", "evil.example", 403), ("POST", "evil.example", 403),
                                   ("GET", "localhost", 200), ("POST", "127.0.0.1", 200)):
            connection = http.client.HTTPConnection("127.0.0.1", port, timeout=5)
            body = b"{}" if method == "POST" else None
            connection.request(method, "/api/stop" if body else "/api/status", body,
                               {"Host": "%s:%d" % (host, port), "Content-Type": "application/json"})
            self.assertEqual(connection.getresponse().status, code, (method, host))
            connection.close()

    def test_loopback_names(self):
        for host in ("127.0.0.1:1", "localhost", "[::1]:28733", "app.localhost:8", "127.5.0.1"):
            self.assertTrue(web.loopback_host(host), host)
        for host in ("evil.example", "127.0.0.1.evil.example", "10.0.0.2:28733", ""):
            self.assertFalse(web.loopback_host(host), host)


class Signals(unittest.TestCase):
    def test_hangup_and_terminate_end_it_like_ctrl_c(self):
        import signal
        import subprocess
        for sig in (signal.SIGHUP, signal.SIGTERM):
            process = subprocess.Popen([sys.executable, "-u", str(Path(web.__file__)), "--port", "0"],
                                       stderr=subprocess.PIPE, text=True)
            self.assertIn("speech page at", process.stderr.readline())   # serving
            process.send_signal(sig)
            self.assertEqual(process.wait(10), 0, sig)   # its finally ran: not killed by the signal
            process.stderr.close()


if __name__ == "__main__":
    unittest.main()
