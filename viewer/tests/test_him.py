"""Embed HIM's own local page without controlling or changing the simulator."""
from pathlib import Path
import sys
import unittest
from http.server import BaseHTTPRequestHandler

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from feeds import Source, validate
from test_dashboard import eventually, serving


class ViewerFixture(BaseHTTPRequestHandler):
    correct = True
    gets = []
    posts = []

    def log_message(self, *args):
        pass

    def do_GET(self):
        type(self).gets.append(self.path)
        body = b'<html><head><title>G1 sim replay</title></head><body>Fixture viewer</body></html>' if self.correct else b'<html>Another service</html>'
        self.send_response(200)
        self.send_header('Content-Type', 'text/html; charset=utf-8')
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_POST(self):
        type(self).posts.append(self.path)
        self.send_error(405)


class HimTests(unittest.TestCase):
    def test_local_viewer_appears_and_recovers_without_starting_a_run(self):
        ViewerFixture.correct, ViewerFixture.gets, ViewerFixture.posts = True, [], []
        with serving(ViewerFixture) as url:
            source = Source({'id': 'him', 'kind': 'him', 'url': url})
            source.start()
            try:
                eventually(lambda: source.status()['reachable'])
                self.assertTrue(source.status()['feeds']['viewer']['online'])
                self.assertEqual(source.status()['detail']['viewer'], url + '/')
                # Availability is independent of whether the run is playing or paused.
                self.assertTrue(all(path == '/' for path in ViewerFixture.gets))
                self.assertEqual(ViewerFixture.posts, [])
                ViewerFixture.correct = False
                eventually(lambda: not source.status()['reachable'])
                ViewerFixture.correct = True
                eventually(lambda: source.status()['reachable'])
            finally:
                source.close()

    def test_missing_viewer_has_an_actionable_offline_message(self):
        source = Source({'id': 'him', 'kind': 'him', 'url': 'http://127.0.0.1:1'})
        source.start()
        try:
            eventually(lambda: 'Start a simulation' in source.status()['reason'])
            self.assertFalse(source.status()['reachable'])
        finally:
            source.close()

    def test_remote_him_addresses_are_not_supported_by_local_embedding(self):
        with self.assertRaises(ValueError):
            validate({'sources': [{'id': 'him', 'kind': 'him', 'url': 'http://remote.example:8765'}]})
        for host in ('127.0.0.1', 'localhost', '[::1]'):
            validate({'sources': [{'id': 'him', 'kind': 'him', 'url': 'http://' + host + ':8765'}]})


if __name__ == '__main__':
    unittest.main()
