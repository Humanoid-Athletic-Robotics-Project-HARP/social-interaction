"""Run isolation, freshness and proxy behavior independent of Rerun installation."""
import json
import http.client
import io
from pathlib import Path
import struct
import sys
import tempfile
import time
import threading
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from diagnostics import scope_requests, serve
from feeds import Source
from simulation import Simulation, record
from test_dashboard import eventually


class SimulationTests(unittest.TestCase):
    def test_cancelled_rerun_stream_closes_cleanly(self):
        source = Simulation({'id': 'sim', 'kind': 'simulation'})
        source.ports = {'web_port': 1, 'grpc_port': 2}
        handler = Mock()
        handler.headers = {}
        handler.wfile = io.BytesIO()
        response = Mock()
        response.status = 200
        response.getheaders.return_value = [('Content-Type', 'application/grpc-web+proto')]
        response.read1.side_effect = [b'data', http.client.IncompleteRead(b'')]
        with patch('diagnostics.http.client.HTTPConnection') as connection:
            connection.return_value.sock = None
            connection.return_value.getresponse.return_value = response
            serve(handler, source, 'grpc/MessageProxyService/ReadMessages', '', b'', threading.Event())
            connection.return_value.close.assert_called_once()
        self.assertTrue(handler.close_connection)
        self.assertIn(b'data', handler.wfile.getvalue())

    def test_log_parsing_accepts_docker_and_compose(self):
        event = {'source': 'view', 'event': 'frame', 'sim_time': 1}
        for line in (json.dumps(event), 'submission-1 | ' + json.dumps(event)):
            self.assertEqual(record(line), event)
        for line in ('plain log', '{bad', '[]', '{}'):
            self.assertIsNone(record(line))

    def test_session_restart_drops_previous_pending_frames(self):
        source = Simulation({'id': 'sim', 'kind': 'simulation'})
        source.new_session('first')
        source.error = None
        source.ingest({'source': 'view', 'event': 'frame', 'sim_time': 1})
        self.assertTrue(source.status()['feeds']['diagnostics']['online'])
        source.last_frame -= 10
        self.assertFalse(source.status()['feeds']['diagnostics']['online'])
        for n in range(500):
            source.ingest({'source': 'view', 'event': 'frame', 'sim_time': n})
        self.assertLessEqual(source.render_queue.qsize(),128)
        self.assertGreater(source.render_dropped,0)
        source.new_session('second')
        self.assertEqual(source.render_queue.qsize(),1)
        self.assertEqual(source.render_queue.get()['_session'],'second')
        self.assertIsNone(source.last_frame)
        self.assertFalse(source.status()['feeds']['diagnostics']['online'])

    def test_recording_does_not_expire_like_live_data(self):
        source = Simulation({'id': 'saved', 'kind': 'recording', 'path': '/example'})
        source.error = None
        source.finished = True
        source.last_frame = time.monotonic()-1000
        state = source.status()
        self.assertTrue(state['feeds']['diagnostics']['online'])
        self.assertEqual(state['mode'],'replay')

    def test_saved_run_replays_frames_and_events_in_wall_order(self):
        with tempfile.TemporaryDirectory() as folder:
            run = Path(folder)
            frames = [{'source': 'view', 'event': 'frame', 'wall': w, 'sim_time': w} for w in (1, 3)]
            events = [{'source': 'predictive', 'event': 'started', 'wall': 0},
                      {'source': 'predictive', 'event': 'avoidance', 'wall': 2, 'mode': 'yield'}]
            (run / 'view.jsonl').write_text(''.join(json.dumps(f) + '\n' for f in frames))
            (run / 'events.jsonl').write_text(''.join(json.dumps(e) + '\n' for e in events))
            source = Simulation({'id': 'saved', 'kind': 'recording', 'path': folder})
            source.replay()
            sent = [source.render_queue.get_nowait() for _ in range(source.render_queue.qsize())]
            self.assertEqual([v.get('wall') for v in sent if 'wall' in v], [0, 1, 2, 3])
            self.assertEqual(source.status()['detail']['pieces'], ['predictive avoidance'])

    def test_live_run_names_its_pieces_and_a_new_run_forgets_them(self):
        source = Simulation({'id': 'sim', 'kind': 'simulation'})
        source.new_session('first')
        source.error = None
        for value in ({'source': 'people', 'event': 'started', 'detector': 'sim_people'},
                      {'source': 'approach', 'event': 'started'},
                      {'source': 'behavior', 'event': 'loop_start'}):
            source.ingest(value)
        self.assertEqual(source.status()['detail']['pieces'],
                         ['approach stop', 'sim_people', 'behavior loop'])
        source.new_session('second')
        self.assertEqual(source.status()['detail']['pieces'], [])

    def test_late_states_are_counted_per_run(self):
        source = Simulation({'id': 'sim', 'kind': 'simulation'})
        source.new_session('first')
        source.error = None
        for age in (.049, .113):
            source.ingest({'source': 'policy', 'event': 'late_state', 'age': age})
        detail = source.status()['detail']
        self.assertEqual((detail['late_states'], detail['max_late_s']), (2, .113))
        source.new_session('second')
        self.assertEqual(source.status()['detail']['late_states'], 0)

    def test_home_tile_tally_follows_the_run(self):
        source = Simulation({'id': 'sim', 'kind': 'simulation'})
        source.new_session('first')
        source.error = None
        for event in ({'source': 'behavior', 'event': 'noticed', 'person': 'a', 'confirmed': False},
                      {'source': 'behavior', 'event': 'noticed', 'person': 'a'},
                      {'source': 'behavior', 'event': 'noticed', 'person': 'b'},
                      {'source': 'behavior', 'event': 'talk', 'person': 'a', 'line': 'Hi', 'distance': 1.2},
                      {'source': 'approach', 'event': 'approach_stop', 'range': 1.1},
                      {'source': 'approach', 'event': 'lidar_lost'},
                      {'source': 'policy', 'event': 'timing', 'sim_time': 12.5}):
            source.ingest(event)
        detail = source.status()['detail']
        self.assertEqual(detail['activity'], {'event': 'talk', 'person': 'a', 'distance': 1.2})
        self.assertEqual((detail['talks'], detail['people'], detail['holding'], detail['lidar_lost'], detail['sim_time']),
                         (1, 2, 1.1, True, 12.5))
        source.ingest({'source': 'approach', 'event': 'approach_go'})
        source.ingest({'source': 'approach', 'event': 'lidar_back'})
        detail = source.status()['detail']
        self.assertEqual((detail['holding'], detail['lidar_lost']), (None, False))
        source.new_session('second')
        detail = source.status()['detail']
        self.assertEqual((detail['activity'], detail['talks'], detail['people']), (None, 0, 0))

    def test_a_saved_run_says_why_him_stopped_it(self):
        with tempfile.TemporaryDirectory() as folder:
            (Path(folder) / 'view.jsonl').write_text('')
            (Path(folder) / 'summary.json').write_text(json.dumps({'him_reason': 'action_timeout'}))
            source = Simulation({'id': 'saved', 'kind': 'recording', 'path': folder})
            source.replay()
            self.assertEqual(source.status()['detail']['stopped'], 'action_timeout')

    def test_rerun_request_scoping_keeps_sources_separate(self):
        html = '<html><head><script src="./re_viewer.js"></script></head></html>'
        first, second = scope_requests(html, 'one'), scope_requests(html, 'two')
        self.assertIn('/source/one',first)
        self.assertNotIn('/source/two',first)
        self.assertIn('/source/two',second)
        self.assertIn('await r.arrayBuffer()',first)  # finite HTTP/1 request bodies
        self.assertIn('./re_viewer.js',first)

    def test_hybrid_filters_stale_layer_and_keeps_live_layer(self):
        source = Source({'id':'pc2','kind':'sensors','url':'http://localhost'})
        source.set_status({'lidar':{'online':True},'depth':{'online':False}})
        lidar = struct.pack('<hhhBB',100,200,300,0,0)
        depth = struct.pack('<hhhBB',400,500,600,1,0)
        with patch('feeds.fetch',return_value=(200,'application/octet-stream',lidar+depth)):
            self.assertEqual(source.request('hybrid/points')[2],lidar)
        source.failed('transport lost')
        self.assertEqual(source.request('hybrid/points')[2],b'')

    def test_docker_lifecycle_recovers_independently(self):
        # A tiny fake CLI exercises the actual inspect/logs subprocess boundary.
        with tempfile.TemporaryDirectory() as temp:
            folder = Path(temp)
            state = folder/'state.json'
            def write(active, generation):
                other=state.with_suffix('.new')
                other.write_text(json.dumps({'active':active,'generation':generation}))
                other.replace(state)
            write(False,1)
            cli=folder/'docker'
            cli.write_text('#!'+sys.executable+'\n'+'''import json,sys,time
from pathlib import Path
state=Path(STATE)
def read(): return json.loads(state.read_text())
s=read()
if sys.argv[1]=='inspect':
 print(json.dumps([{'Id':'same-container','State':{'Running':s['active'],'StartedAt':str(s['generation'])}}]))
else:
 while True:
  now=read()
  if not now['active'] or now['generation']!=s['generation']: break
  print(json.dumps({'source':'view','event':'frame','sim_time':time.time()}),flush=True)
  time.sleep(.05)
'''.replace('STATE',repr(str(state))))
            cli.chmod(0o755)
            source=Simulation({'id':'sim','kind':'simulation','docker':str(cli),'python':'/missing/python'})
            source.start()
            try:
                eventually(lambda: source.error and 'stopped' in source.error)
                write(True,1)
                eventually(lambda: source.status()['feeds']['diagnostics']['online'])
                self.assertEqual(source.session,'same-container:1')
                write(False,1)
                eventually(lambda: not source.status()['feeds']['diagnostics']['online'])
                write(True,2)
                eventually(lambda: source.session=='same-container:2' and source.status()['feeds']['diagnostics']['online'])
            finally:
                source.close()

    def fake_docker(self, folder, first_lines=()):
        """A docker CLI whose one container runs and logs first_lines, then frames."""
        cli = folder/'docker'
        body = ("import json,sys,time\n"
                "if sys.argv[1]=='inspect':\n"
                " print(json.dumps([{'Id':'c','State':{'Running':True,'StartedAt':'1'}}]))\n"
                "else:\n"
                " for line in FIRST: print(json.dumps(line),flush=True)\n"
                " while True:\n"
                "  print(json.dumps({'source':'view','event':'frame','sim_time':time.time()}),flush=True)\n"
                "  time.sleep(.05)\n").replace('FIRST', repr(list(first_lines)))
        cli.write_text('#!'+sys.executable+'\n'+body)
        cli.chmod(0o755)
        return Simulation({'id':'sim','kind':'simulation','docker':str(cli),'python':'/missing/python'})

    def test_a_malformed_event_does_not_stop_following(self):
        with tempfile.TemporaryDirectory() as temp:
            bad = {'source':'policy','event':'late_state','age':[1]}
            source = self.fake_docker(Path(temp), [bad, {'source':'policy','event':'late_state','age':.05}])
            source.start()
            try:
                eventually(lambda: source.status()['feeds']['diagnostics']['online'])
                detail = source.status()['detail']
                self.assertEqual((detail['late_states'], detail['max_late_s']), (2, .05))
            finally:
                source.close()

    def test_a_failed_log_follow_is_retried(self):
        import simulation
        real, failed = simulation.subprocess.Popen, []
        def popen(args, *rest, **kwargs):
            if 'logs' in args and not failed:   # the follow, not docker inspect
                failed.append(args)
                raise OSError('docker logs failed once')
            return real(args, *rest, **kwargs)
        with tempfile.TemporaryDirectory() as temp:
            source = self.fake_docker(Path(temp))
            with patch.object(simulation.subprocess, 'Popen', side_effect=popen):
                source.start()
                try:
                    # A 1 s retry wait and three fake docker starts: more than the usual 4 s.
                    eventually(lambda: source.status()['feeds']['diagnostics']['online'], timeout=10)
                    self.assertTrue(failed)
                finally:
                    source.close()


if __name__=='__main__':
    unittest.main()
