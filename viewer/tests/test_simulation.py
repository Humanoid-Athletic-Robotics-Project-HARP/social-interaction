"""Run isolation, freshness and proxy behavior independent of Rerun installation."""
import json
from pathlib import Path
import struct
import sys
import tempfile
import time
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from diagnostics import scope_requests
from feeds import Source
from simulation import Simulation, record
from test_dashboard import eventually


class SimulationTests(unittest.TestCase):
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


if __name__=='__main__':
    unittest.main()
