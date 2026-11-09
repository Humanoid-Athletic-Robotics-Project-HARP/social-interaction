"""Resolve private Jetson addresses without storing or sending them to the browser."""
import json
from pathlib import Path
import sys
import tempfile
import types
import unittest
import urllib.request
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from configuration import env_file, resolve, robot_host
from server import Dashboard, handler
from test_dashboard import serving


class ConfigurationTests(unittest.TestCase):
    def test_default_sources_resolve_ssh_login_without_modifying_env(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            content = 'OTHER_SETTING=unchanged\nG1_HOST=unitree@192.0.2.71\n'
            (root / '.env').write_text(content)
            config = resolve(root=root)
            self.assertEqual([s['id'] for s in config['sources']],
                             ['mac-sim', 'mac-speech', 'pc2-sensors', 'pc2-speech'])
            self.assertEqual(config['sources'][2]['url'], 'http://192.0.2.71:8082')
            self.assertEqual(config['sources'][3]['url'], 'http://192.0.2.71:28733')
            self.assertFalse(config['sources'][3]['controls'])
            self.assertEqual((root / '.env').read_text(), content)

    def test_missing_env_keeps_offline_jetson_sources(self):
        with tempfile.TemporaryDirectory() as folder:
            dashboard = Dashboard(root=Path(folder))
            sensors = dashboard.sources['pc2-sensors']
            sensors.start()
            self.assertEqual(sensors.threads, [])
            state = sensors.status()
            self.assertFalse(state['reachable'])
            self.assertEqual(set(state['feeds']), {'lidar', 'depth', 'colour'})
            self.assertIn('G1_HOST', state['reason'])
            self.assertIn('pc2-speech', dashboard.sources)
            dashboard.close()

    def test_quoted_values_plain_hostname_and_ipv6(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / '.env'
            for text, expected in [('export G1_HOST="unitree@robot.local" # lab\n', 'robot.local'),
                                   ('G1_HOST=robot.local\n', 'robot.local'),
                                   ("G1_HOST='unitree@[2001:db8::71]'\n", '2001:db8::71')]:
                path.write_text(text)
                self.assertEqual(robot_host(path)[0], expected)
            config = resolve({'sources': [{'id': 'robot', 'kind': 'sensors', 'host_env': 'G1_HOST', 'port': 8082}]}, Path(folder))
            self.assertEqual(config['sources'][0]['url'], 'http://[2001:db8::71]:8082')

    def test_invalid_values_are_not_executed_or_echoed(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / '.env'
            for value in ('unitree@private.example:22', 'unitree@private.example/path', '$(touch secret-marker)', '"unclosed'):
                path.write_text('G1_HOST=' + value + '\n')
                with self.assertRaises(ValueError) as error:
                    robot_host(path)
                self.assertNotIn('private.example', str(error.exception))
            self.assertFalse((Path(folder) / 'secret-marker').exists())

    def test_worktree_uses_main_checkout_env(self):
        with tempfile.TemporaryDirectory() as folder:
            main = Path(folder).resolve() / 'main'
            git = main / '.git'
            git.mkdir(parents=True)
            (main / '.env').write_text('G1_HOST=unitree@192.0.2.71\n')
            result = types.SimpleNamespace(returncode=0, stdout=str(git) + '\n')
            with patch('configuration.subprocess.run', return_value=result):
                self.assertEqual(env_file(Path(folder) / 'worktree'), main / '.env')
                self.assertEqual(resolve(root=Path(folder) / 'worktree')['sources'][2]['url'], 'http://192.0.2.71:8082')

    def test_public_status_errors_responses_and_events_redact_private_address(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            (root / '.env').write_text('G1_HOST=unitree@192.0.2.71\n')
            dashboard = Dashboard(root=root)
            source = dashboard.sources['pc2-sensors']
            source.failed('Cannot connect to http://192.0.2.71:8082')
            source.detail = {'nested': {'host': '192.0.2.71'}}
            public = json.dumps(dashboard.state())
            self.assertNotIn('192.0.2.71', public)
            self.assertNotIn('unitree@', public)
            self.assertNotIn('192.0.2.71', dashboard.encode({'event': 'log', 'text': '192.0.2.71'}))
            with serving(handler(dashboard)) as url:
                status = urllib.request.urlopen(url + '/api/state').read()
                self.assertNotIn(b'192.0.2.71', status)
                speech = dashboard.sources['pc2-speech']
                with patch.object(speech, 'request', return_value=(200, 'application/json', b'{"host":"192.0.2.71"}')):
                    body = urllib.request.urlopen(url + '/source/pc2-speech/api/status').read()
                    self.assertNotIn(b'192.0.2.71', body)
                    self.assertIn(b'Jetson address', body)
                with patch.object(speech, 'request', side_effect=OSError('Cannot reach 192.0.2.71')):
                    with self.assertRaises(urllib.error.HTTPError) as failure:
                        urllib.request.urlopen(url + '/source/pc2-speech/api/status')
                    self.assertNotIn(b'192.0.2.71', failure.exception.read())
                speech.append({'source':'listen','event':'log','text':'unitree@192.0.2.71'})
                response = urllib.request.urlopen(url + '/source/pc2-speech/events', timeout=2)
                try:
                    data = response.readline() + response.readline()
                    self.assertIn(b'data:', data)
                    self.assertNotIn(b'192.0.2.71', data)
                finally:
                    response.close()
            dashboard.close()

    def test_custom_config_is_not_mutated_or_implicitly_expanded(self):
        custom = {'sources': []}
        self.assertEqual(resolve(custom), custom)
        self.assertIsNot(resolve(custom), custom)
        with self.assertRaises(ValueError):
            resolve({'sources': [{'id': 'robot', 'kind': 'sensors', 'host_env':'OTHER_SECRET'}]})


if __name__ == '__main__':
    unittest.main()
