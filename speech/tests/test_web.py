"""speech/web.py's sessions, with a stand-in for talk.py (no models, LLM or sound)."""
import json
import sys
import tempfile
import unittest
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

    def test_a_stopped_session_publishes_its_exit_before_stopped(self):
        self.server.start({"listen": False})
        self.server.stop()
        self.assertTrue(self.server.session.done.wait(5))
        order = [(e["source"], e["event"], e.get("state")) for e in self.events()]
        self.assertLess(order.index(("talk", "exit", None)), order.index(("web", "session", "stopped")))


if __name__ == "__main__":
    unittest.main()
