#!/usr/bin/env python3
"""One dashboard for independent HARP feeds. python3 viewer/server.py --help"""
import argparse
import ast
import hashlib
import ipaddress
import json
from pathlib import Path
import signal
import sys
import threading
import urllib.parse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

from feeds import MAX_BODY, Source, validate
from configuration import resolve

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.append(str(ROOT / "submission"))
import host_health  # noqa: E402  (standard library only, shared with course.py)

HOST_EVERY = 5.0   # s between host health readings
MAX_OPEN = 2       # saved runs open at once from the Runs page: each runs a Rerun worker


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
            html = html.replace('</head>', '<style>#go,#llm-button,#composer,#end-turn{display:none!important}#setup{pointer-events:none;opacity:.6}</style></head>', 1)
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
    def __init__(self, config=None, root=ROOT):
        from simulation import Simulation
        config = resolve(config, root)
        self.private = sorted({value for source in config["sources"] if isinstance(source, dict)
                               for value in source.get("_private", []) if value}, key=len, reverse=True)
        self.sources = {item["id"]: (Simulation(item) if item["kind"] in ("simulation", "recording") else Source(item))
                        for item in validate(config)}
        self.stop = threading.Event()
        self.host = None   # host_health.health(): can this computer keep a sim's deadlines
        from runs import Runs, runs_folder
        self.runs = Runs(runs_folder(root))
        self.opened = []   # ids of saved runs opened from the Runs page, oldest first
        self.made = set()  # every id open_run created, the only ones close_run may close
        self.lock = threading.Lock()   # sources change as runs are opened and closed
        self.started = False

    def start(self):
        self.started = True
        for source in self.sources.values():
            source.start()
        threading.Thread(target=self.watch_host, daemon=True, name="host").start()

    def watch_host(self):
        while not self.stop.is_set():
            self.host = host_health.health()
            self.stop.wait(HOST_EVERY)

    def open_run(self, name):
        """A saved run as a recording source, its id; the oldest opened one closes past
        MAX_OPEN (configured recordings stay)."""
        from simulation import Simulation
        path = self.runs.path(name)
        if path is None:
            raise KeyError("no such run")
        key = "run-" + hashlib.sha1(name.encode()).hexdigest()[:10]
        with self.lock:
            if key in self.sources:
                self.opened.remove(key)
                self.opened.append(key)
                return key
            meta = next((r for r in self.runs.list() if r["name"] == name), {})
            source = Simulation({"id": key, "kind": "recording", "path": str(path), "run": name,
                                 "label": " · ".join(str(v) for v in (meta.get("world"), meta.get("tag"),
                                                                     name[:16].replace("_", " ")) if v),
                                 "machine": "Saved run"})
            self.sources[key] = source
            self.made.add(key)
            self.opened.append(key)
            closing = self.opened[:-MAX_OPEN]
            del self.opened[:-MAX_OPEN]
        for old in closing:
            self.close_run(old)
        if self.started:
            source.start()
        return key

    def close_run(self, key):
        """Close a run the Runs page opened; configured sources are never closed here."""
        with self.lock:
            source = self.sources.pop(key, None) if key in self.made else None
            self.made.discard(key)
            if key in self.opened:
                self.opened.remove(key)
        if source is not None:
            source.close()

    def state(self):
        with self.lock:
            sources = list(self.sources.values())
        return json.loads(self.encode({"sources": [s.status() for s in sources], "host": self.host,
                                       "runs": len(self.runs.list()), "opened": list(self.opened)}))

    def encode(self, value):
        return self.redact(json.dumps(value))

    def redact(self, text):
        for value in self.private:
            text = text.replace(value, "[Jetson address]")
        return text

    def close(self):
        self.stop.set()
        with self.lock:
            sources = list(self.sources.values())
        for source in sources:
            source.close()


def loopback_host(host):
    """Whether a request's Host header names this computer's loopback. A page on another
    site whose name it re-resolves to 127.0.0.1 (DNS rebinding) sends its own name here."""
    name = urllib.parse.urlsplit("//" + host).hostname or ""
    if name == "localhost" or name.endswith(".localhost"):
        return True
    try:
        return ipaddress.ip_address(name).is_loopback
    except ValueError:
        return False


def handler(dashboard):
    class Handler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"

        def log_message(self, *args):
            pass

        def reply(self, code, body, kind="application/json"):
            if not isinstance(body, bytes):
                body = dashboard.encode(body).encode()
            elif "application/json" in kind:
                body = dashboard.redact(body.decode("utf-8", errors="replace")).encode()
            self.send_response(code)
            self.send_header("Content-Type", kind)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.send_header("X-Content-Type-Options", "nosniff")
            self.end_headers()
            self.wfile.write(body)

        def foreign_host(self):
            """Bound to loopback (the default), a request must name it: see loopback_host."""
            host = self.headers.get("Host")
            try:
                bound = ipaddress.ip_address(self.server.server_address[0]).is_loopback
            except ValueError:
                bound = False
            foreign = bound and host is not None and not loopback_host(host)
            if foreign:
                self.close_connection = True   # its body, if any, is left unread
            return foreign

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
                        self.wfile.write(("id: %s\ndata: %s\n\n" % (n, dashboard.encode(event))).encode())
                        cursor = n
                    self.wfile.write(b": heartbeat\n\n")
                else:
                    self.wfile.write(("data: " + json.dumps(dashboard.state()) + "\n\n").encode())
                self.wfile.flush()
                dashboard.stop.wait(1)

        def route(self, data=None):
            url = urllib.parse.urlsplit(self.path)
            if data is None and url.path == "/favicon.ico":
                return self.reply(204, b"", "image/x-icon")
            if data is None and url.path == "/api/state":
                return self.reply(200, dashboard.state())
            if data is None and url.path == "/events":
                return self.sse()
            if data is None and url.path == "/api/runs":
                return self.reply(200, {"runs": dashboard.runs.list(), "opened": list(dashboard.opened),
                                        "max_open": MAX_OPEN})
            if data is None and url.path == "/api/runs/timing":
                name = urllib.parse.parse_qs(url.query).get("name", [None])[0]
                try:
                    return self.reply(200, dashboard.runs.timing(name))
                except KeyError:
                    return self.reply(404, {"error": "no such run"})
            if data is not None and url.path == "/api/runs/open":
                try:
                    return self.reply(200, {"id": dashboard.open_run(json.loads(data).get("name"))})
                except KeyError:
                    return self.reply(404, {"error": "no such run"})
            if data is not None and url.path == "/api/runs/close":
                dashboard.close_run(str(json.loads(data).get("id")))
                return self.reply(200, {"opened": list(dashboard.opened)})
            if url.path.startswith("/source/"):
                parts = url.path.split("/", 3)
                source = dashboard.sources.get(parts[2])
                if source is None or len(parts) != 4:
                    return self.reply(404, {"error": "unknown source"})
                path = parts[3]
                if source.kind in ("simulation", "recording"):
                    from diagnostics import serve
                    return serve(self, source, path, url.query, data, dashboard.stop)
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
                          "/app.js": (HERE / "app.js", "text/javascript"), "/timing.js": (HERE / "timing.js", "text/javascript"), "/robot.js": (HERE / "robot.js", "text/javascript"), "/style.css": (HERE / "style.css", "text/css"),
                          "/assets/three.module.js": (HERE / ".assets/three.module.js", "text/javascript"),
                          "/assets/addons/controls/OrbitControls.js": (HERE / ".assets/OrbitControls.js", "text/javascript")}
                if url.path in assets:
                    path, kind = assets[url.path]
                    if path.is_file():
                        return self.reply(200, path.read_bytes(), kind)
                    return self.reply(503, {"error": "Install viewer assets: python3 viewer/setup_assets.py"})
            self.reply(404, {"error": "not found"})

        def do_GET(self):
            if self.foreign_host():
                return self.reply(403, {"error": "unexpected Host: open this page at 127.0.0.1 or localhost"})
            try:
                self.route()
            except (BrokenPipeError, ConnectionResetError):
                pass
            except (OSError, ValueError, KeyError, TypeError) as error:
                self.reply(502, {"error": str(error)})

        def do_POST(self):
            if self.foreign_host():
                return self.reply(403, {"error": "unexpected Host: open this page at 127.0.0.1 or localhost"})
            try:
                # Commands remain an explicit connection to the owning speech service.
                origin = self.headers.get("Origin")
                host = self.headers.get("Host")
                if origin and urllib.parse.urlsplit(origin).netloc != host:
                    return self.reply(403, {"error": "cross-origin commands are disabled"})
                if self.path.startswith("/source/") and "/grpc/" in self.path:
                    size = int(self.headers.get("Content-Length", "0"))
                    if not 0 <= size <= MAX_BODY:
                        self.close_connection = True
                        return self.reply(413, {"error": "request too large"})
                    self.connection.settimeout(5)
                    return self.route(self.rfile.read(size))
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
        config = json.loads(args.config.read_text()) if args.config else None
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
