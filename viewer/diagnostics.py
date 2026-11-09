"""Same-origin access to the optional, loopback-only Rerun renderer."""
import http.client
import json
from pathlib import Path
import shutil
import select
import socket
import threading


PAGE = '''<!doctype html><html><head><meta charset="utf-8"><title>HARP diagnostics</title>
<style>body{margin:0;height:100vh;display:flex;flex-direction:column;background:#10151b;color:#dce5ee;font:13px system-ui}header{padding:10px 16px;display:flex;flex-wrap:wrap;justify-content:space-between;gap:4px 20px}.hint{color:#91a0b0}.hint b{color:#dce5ee;font-weight:500}iframe{flex:1;min-height:0;width:100%;border:0}#waiting{padding:30px}</style></head><body>
<header><span id="status">Connecting to our diagnostics feed</span><span class="hint">3D view follows the robot · <b>drag</b> to turn · <b>right-drag</b> to pan · <b>scroll</b> to zoom · <b>double-click</b> empty space to reset</span></header>
<div id="waiting">Waiting for diagnostics.</div><script>
const id=SOURCE_ID, prefix=PREFIX;
async function poll(){try{const state=await(await fetch('/api/state')).json(),s=state.sources.find(s=>s.id===id);
document.getElementById('status').textContent=s.kind==='recording'?'Replay · '+s.label:(s.detail.finished?'Run ended · ':s.feeds.diagnostics?.online?'Live · ':'Offline · ')+s.label;
if(s.detail.renderer==='ready'&&!document.querySelector('iframe')){
const u=new URL(prefix+'/rerun/',location.origin);
u.searchParams.set('url',s.kind==='recording'&&s.detail.rrd?new URL(prefix+'/recording.rrd',location.origin).href:'rerun+'+location.origin+'/proxy');
u.searchParams.set('persist','0');
const f=document.createElement('iframe');f.title='Rerun diagnostics';f.src=u.href;document.getElementById('waiting').hidden=true;document.body.append(f);
}else if(s.detail.renderer!=='ready'){document.getElementById('waiting').textContent=s.detail.renderer;}
}catch(e){document.getElementById('status').textContent='Dashboard disconnected';}setTimeout(poll,1000);}poll();
</script></body></html>'''


def scope_requests(html, source_id):
    # Rerun 0.41 accepts only /proxy as the URI path. Its gRPC-web calls are rooted
    # at the origin. Scope those fetches to this source, keeping concurrent viewers
    # on one dashboard origin without exposing additional network listeners.
    prefix = '/source/' + source_id
    script = '''<script>
{const original=globalThis.fetch.bind(globalThis),prefix=PREFIX;
globalThis.fetch=async function(input,init){const r=new Request(input,init),u=new URL(r.url);
if(u.origin===location.origin&&!u.pathname.startsWith(prefix+'/')){
u.pathname=prefix+'/grpc'+u.pathname;return original(u.href,{method:r.method,headers:r.headers,
body:['GET','HEAD'].includes(r.method)?undefined:await r.arrayBuffer(),signal:r.signal,credentials:r.credentials});}
return original(r);};}
</script>'''.replace('PREFIX', json.dumps(prefix))
    return html.replace('<head>', '<head>' + script, 1)


def serve(handler, source, path, query, data, stop):
    if data is None and path == 'diagnostics/':
        page = PAGE.replace('SOURCE_ID', json.dumps(source.id)).replace('PREFIX', json.dumps('/source/' + source.id))
        return handler.reply(200, page.encode(), 'text/html; charset=utf-8')
    if data is None and path == 'recording.rrd' and source.kind == 'recording':
        file = Path(source.config['path']).expanduser().resolve()
        if file.suffix.lower() != '.rrd' or not file.is_file():
            return handler.reply(404, {'error': 'no .rrd file configured'})
        handler.send_response(200)
        handler.send_header('Content-Type', 'application/octet-stream')
        handler.send_header('Content-Length', str(file.stat().st_size))
        handler.end_headers()
        with file.open('rb') as stream:
            shutil.copyfileobj(stream, handler.wfile, 65536)
        return
    if not path.startswith(('rerun/', 'grpc/')):
        return handler.reply(404, {'error': 'unknown diagnostics endpoint'})
    if source.ports is None:
        return handler.reply(503, {'error': source.worker_error or 'Rerun is starting'})
    prefix, _, rest = path.partition('/')
    port = source.ports['web_port' if prefix == 'rerun' else 'grpc_port']
    headers = {k: v for k, v in handler.headers.items() if k.lower() in
               ('content-type', 'x-grpc-web', 'x-user-agent', 'grpc-timeout', 'accept', 'range')}
    # The hub checks browser origins; the loopback backend does not receive Origin.
    connection = http.client.HTTPConnection('127.0.0.1', port, timeout=30)
    begun = False
    done = threading.Event()
    try:
        connection.request('POST' if data is not None else 'GET', '/' + rest + ('?' + query if query else ''), body=data, headers=headers)
        response = connection.getresponse()
        if prefix == 'rerun' and 'text/html' in response.getheader('Content-Type', ''):
            html = response.read(2 * 1024 * 1024).decode()
            return handler.reply(response.status, scope_requests(html, source.id).encode(), 'text/html; charset=utf-8')
        if prefix == 'grpc' and connection.sock:
            upstream = connection.sock
            upstream.settimeout(None)
            def watch_client():
                # Quiet replay streams are healthy. Cancel on client disconnect or
                # hub shutdown rather than imposing a data-arrival timeout.
                while not done.wait(.5):
                    try:
                        readable = select.select([handler.connection], [], [], 0)[0]
                        closed = readable and not handler.connection.recv(1, socket.MSG_PEEK)
                        if closed or stop.is_set():
                            upstream.shutdown(socket.SHUT_RDWR)
                            return
                    except OSError:
                        return
            threading.Thread(target=watch_client, daemon=True).start()
        handler.send_response(response.status)
        for key, value in response.getheaders():
            if key.lower() in ('content-type', 'content-range', 'accept-ranges', 'grpc-status', 'grpc-message'):
                handler.send_header(key, value)
        handler.send_header('Transfer-Encoding', 'chunked')
        handler.send_header('Cache-Control', 'no-store')
        handler.end_headers()
        begun = True
        while not stop.is_set():
            chunk = response.read1(65536)
            if not chunk:
                break
            handler.wfile.write(('%x\r\n' % len(chunk)).encode() + chunk + b'\r\n')
            handler.wfile.flush()
        handler.wfile.write(b'0\r\n\r\n')
    except (OSError, http.client.HTTPException) as error:
        if not begun:
            handler.reply(502, {'error': str(error)})
        handler.close_connection = True
    finally:
        done.set()
        connection.close()
