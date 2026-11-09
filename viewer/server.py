#!/usr/bin/env python3
"""One dashboard for independent HARP feeds. python3 viewer/server.py --help"""
import argparse
import ast
import json
from pathlib import Path
import signal
import threading
import urllib.parse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

from feeds import MAX_BODY, Source, validate

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
DEFAULT = {"sources": [
    {"id": "mac-speech", "kind": "speech", "label": "Laptop speech", "machine": "MacBook", "url": "http://127.0.0.1:28733", "controls": True},
]}


def page(source, view):
    """Read our existing presentation without importing device or model dependencies."""
    base = "/source/" + source.id + "/"
    if source.kind == "speech" and view == "speech":
        html = (ROOT / "speech/web.html").read_text()
        html = html.replace('"/api/', '"' + base + 'api/').replace('"/events"', '"' + base + 'events"')
        html = html.replace('"harp-speech-"', json.dumps("harp-speech-" + source.id + "-"))
        html = html.replace('serverHost = here ? "this computer" : (s.host || location.hostname);',
                            'serverHost = s.host || ' + json.dumps(source.config.get("machine", source.id)) + ';')
        if not source.config.get("controls", False):
            html = html.replace('<script>', '<script>document.addEventListener("DOMContentLoaded", () => { document.querySelectorAll("button,input,select").forEach(e => e.disabled = true); });', 1)
        return html
    paths = {"lidar": "testground/lidar/lidar_viewer.py", "camera": "testground/camera/camera_viewer.py",
             "hybrid": "testground/hybrid/hybrid_viewer.py"}
    if view not in paths or source.kind != "sensors":
        raise ValueError("view is unavailable for this source")
    if source.config.get("layout", "hybrid") not in ("hybrid", view):
        raise ValueError("view is unavailable for this sensor layout")
    tree = ast.parse((ROOT / paths[view]).read_text())
    for node in tree.body:
        if isinstance(node, ast.Assign) and any(isinstance(t, ast.Name) and t.id == "PAGE" for t in node.targets):
            html = ast.literal_eval(node.value)
            html = html.replace("https://cdn.jsdelivr.net/npm/three@0.160.0/build/three.module.js", "/assets/three.module.js")
            html = html.replace("https://cdn.jsdelivr.net/npm/three@0.160.0/examples/jsm/", "/assets/addons/")
            for name in ("g1lidar.", "g1hybrid."):
                html = html.replace(name, name + source.id + ".")
            return html
    raise ValueError("presentation is missing")


class Dashboard:
    def __init__(self, config):
        self.sources = {item["id"]: Source(item) for item in validate(config)}
        self.stop = threading.Event()

    def start(self):
        for source in self.sources.values():
            source.start()

    def state(self):
        return {"sources": [s.status() for s in self.sources.values()]}

    def close(self):
        self.stop.set()
        for source in self.sources.values():
            source.close()


def handler(dashboard):
    class Handler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"

        def log_message(self, *args):
            pass

        def reply(self, code, body, kind="application/json"):
            if not isinstance(body, bytes):
                body = json.dumps(body).encode()
            self.send_response(code)
            self.send_header("Content-Type", kind)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.send_header("X-Content-Type-Options", "nosniff")
            self.end_headers()
            self.wfile.write(body)

        def sse(self, source=None):
            self.send_response(200)
            self.send_header("Content-Type", "text/event-stream")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Connection", "close")
            self.end_headers()
            self.close_connection = True
            cursor = self.headers.get("Last-Event-ID")
            cursor = int(cursor) if cursor and cursor.isdigit() else None
            session = source.session if source else None
            while not dashboard.stop.is_set():
                if source:
                    gap, events = source.since(cursor)
                    if gap or session != source.session:
                        self.wfile.write(b'event: reset\ndata: {}\n\n')
                        session = source.session
                    for n, event in events:
                        self.wfile.write(("id: %s\ndata: %s\n\n" % (n, json.dumps(event))).encode())
                        cursor = n
                    self.wfile.write(b": heartbeat\n\n")
                else:
                    self.wfile.write(("data: " + json.dumps(dashboard.state()) + "\n\n").encode())
                self.wfile.flush()
                dashboard.stop.wait(1)

        def route(self, data=None):
            url = urllib.parse.urlsplit(self.path)
            if data is None and url.path == "/api/state":
                return self.reply(200, dashboard.state())
            if data is None and url.path == "/events":
                return self.sse()
            if url.path.startswith("/source/"):
                parts = url.path.split("/", 3)
                source = dashboard.sources.get(parts[2])
                if source is None or len(parts) != 4:
                    return self.reply(404, {"error": "unknown source"})
                path = parts[3]
                if data is None and path == "events" and source.kind == "speech":
                    return self.sse(source)
                views = {"speech/": "speech", "lidar/": "lidar", "camera/": "camera", "hybrid/": "hybrid"}
                if data is None and path in views:
                    html = page(source, views[path])
                    # A source restart or a gap in bounded history requires rebuilding the speech page.
                    if views[path] == "speech":
                        html = html.replace('source.onopen =', 'source.addEventListener("reset", () => location.reload());\n  source.onopen =')
                    return self.reply(200, html.encode(), "text/html; charset=utf-8")
                if source.kind == "sensors" and source.config.get("layout", "hybrid") != "hybrid":
                    path = path.split("/", 1)[-1]
                code, kind, body = source.request(path + ("?" + url.query if url.query else ""), data)
                return self.reply(code, body, kind)
            if data is None:
                assets = {"/": (HERE / "index.html", "text/html; charset=utf-8"),
                          "/app.js": (HERE / "app.js", "text/javascript"), "/style.css": (HERE / "style.css", "text/css"),
                          "/assets/three.module.js": (HERE / ".assets/three.module.js", "text/javascript"),
                          "/assets/addons/controls/OrbitControls.js": (HERE / ".assets/OrbitControls.js", "text/javascript")}
                if url.path in assets:
                    path, kind = assets[url.path]
                    if path.is_file():
                        return self.reply(200, path.read_bytes(), kind)
                    return self.reply(503, {"error": "Install viewer assets: python3 viewer/setup_assets.py"})
            self.reply(404, {"error": "not found"})

        def do_GET(self):
            try:
                self.route()
            except (BrokenPipeError, ConnectionResetError):
                pass
            except (OSError, ValueError, KeyError, TypeError) as error:
                self.reply(502, {"error": str(error)})

        def do_POST(self):
            try:
                # Commands remain an explicit connection to the owning speech service.
                origin = self.headers.get("Origin")
                host = self.headers.get("Host")
                if origin and urllib.parse.urlsplit(origin).netloc != host:
                    return self.reply(403, {"error": "cross-origin commands are disabled"})
                if self.headers.get("Content-Type", "").split(";")[0] != "application/json":
                    return self.reply(415, {"error": "send JSON"})
                size = int(self.headers.get("Content-Length", "0"))
                if not 0 < size <= min(MAX_BODY, 65536):
                    self.close_connection = True
                    return self.reply(413, {"error": "command must be between 1 and 65536 bytes"})
                self.connection.settimeout(5)
                body = self.rfile.read(size)
                if not isinstance(json.loads(body), dict):
                    return self.reply(400, {"error": "send a JSON object"})
                self.route(body)
            except (BrokenPipeError, ConnectionResetError):
                pass
            except (OSError, ValueError, TypeError) as error:
                self.close_connection = True
                self.reply(400, {"error": str(error)})
    return Handler


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, help="JSON source configuration (see config.example.json)")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=28734)
    args = parser.parse_args()
    try:
        config = json.loads(args.config.read_text()) if args.config else DEFAULT
        dashboard = Dashboard(config)
    except (OSError, ValueError, KeyError, TypeError) as error:
        parser.error(str(error))
    server = ThreadingHTTPServer((args.host, args.port), handler(dashboard))
    server.daemon_threads = True
    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))
    dashboard.start()
    print("HARP viewer at http://%s:%s (Ctrl-C to stop)" % (args.host, server.server_port), flush=True)
    try:
        server.serve_forever(poll_interval=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        dashboard.close()
        server.server_close()


if __name__ == "__main__":
    main()
