"""Independent collectors for existing HARP services. Standard library only."""
import collections
import json
import re
import threading
import time
import urllib.error
import urllib.parse
import urllib.request

MAX_BODY = 16 * 1024 * 1024
HISTORY = 1500
TIMEOUT = 2.0
STALE = 4.0


class NoRedirect(urllib.request.HTTPRedirectHandler):
    def redirect_request(self, *args, **kwargs):
        return None


HTTP = urllib.request.build_opener(urllib.request.ProxyHandler({}), NoRedirect())


def fetch(url, data=None, timeout=TIMEOUT):
    request = urllib.request.Request(url, data=data, headers={"Content-Type": "application/json"})
    try:
        response = HTTP.open(request, timeout=timeout)
    except urllib.error.HTTPError as error:
        response = error
    with response:
        body = response.read(MAX_BODY + 1)
        if len(body) > MAX_BODY:
            raise ValueError("feed response exceeds 16 MB")
        return response.status, response.headers.get("Content-Type", "application/octet-stream"), body


def get_json(url):
    code, _, body = fetch(url)
    if code != 200:
        raise ValueError("feed returned HTTP %s" % code)
    value = json.loads(body)
    if not isinstance(value, dict):
        raise ValueError("expected a JSON object")
    return value


def validate(config):
    if not isinstance(config, dict) or not isinstance(config.get("sources"), list):
        raise ValueError('configuration needs a "sources" list')
    seen = set()
    for item in config["sources"]:
        if not isinstance(item, dict):
            raise ValueError("each source must be an object")
        key = item.get("id", "")
        if not isinstance(key, str) or not re.fullmatch(r"[a-zA-Z0-9_-]{1,60}", key) or key in seen:
            raise ValueError("source IDs must be unique letters, numbers, underscores or hyphens")
        seen.add(key)
        if "controls" in item and not isinstance(item["controls"], bool):
            raise ValueError("controls must be a JSON boolean")
        if item.get("kind") not in ("sensors", "speech", "simulation", "recording", "him", "robot"):
            raise ValueError("unknown source kind for " + key)
        if item["kind"] in ("sensors", "speech", "him", "robot"):
            if item.get("_configuration_error") and item.get("host_env") == "G1_HOST":
                continue
            if not isinstance(item.get("url"), str):
                raise ValueError("source URL must be a string")
            url = urllib.parse.urlsplit(item.get("url", ""))
            if url.scheme not in ("http", "https") or not url.hostname or url.username or url.password or url.query or url.fragment:
                raise ValueError("source URL must be an HTTP(S) service address without credentials or query")
            if item["kind"] == "him" and url.hostname not in ("127.0.0.1", "localhost", "::1"):
                raise ValueError("HIM embedding supports a local viewer address only")
            if item["kind"] == "sensors" and item.get("layout", "hybrid") not in ("hybrid", "lidar", "camera"):
                raise ValueError("sensor layout must be hybrid, lidar or camera")
        if item["kind"] == "recording" and not isinstance(item.get("path"), str):
            raise ValueError("recording source needs a path to a run folder or .rrd")
    return config["sources"]


ROBOT_FEEDS = ("motors", "battery", "harness")


def robot_feeds(value):
    """testground/robot/status_server.py's readings as feeds: fresh enough to trust or not.
    The harness publishes at least twice a second and the battery about once (HIM stops past 5 s)."""
    def fresh(part, most, missing):
        age = (value.get(part) or {}).get("age_s")
        if isinstance(age, (int, float)) and age < most:
            return {"online": True, "reason": "receiving"}
        return {"online": False, "reason": missing if age is None else "no reading for %.0f s" % age}
    harness = value.get("harness") or {}
    return {"motors": fresh("lowstate", 1.0, "no rt/lowstate"),
            "battery": fresh("battery", 5.0, "no rt/lf/bmsstate"),
            "harness": fresh("harness", 2.0, harness.get("error") or "HARP's harness is not running")}


class Source:
    def __init__(self, config):
        self.config = config
        self.id, self.kind = config["id"], config["kind"]
        self.url = config.get("url", "").rstrip("/")
        self.lock = threading.RLock()
        self.stop = threading.Event()
        self.threads = []
        self.updated = None
        self.error = config.get("_configuration_error", "waiting for source")
        names = ("viewer",) if self.kind == "him" else ("speech",) if self.kind == "speech" else ROBOT_FEEDS if self.kind == "robot" else (("lidar",) if config.get("layout") == "lidar" else
                ("depth", "colour") if config.get("layout") == "camera" else ("lidar", "depth", "colour"))
        self.feeds = {name: {"online": False, "reason": self.error} for name in names}
        self.detail = {}
        self.session = None
        self.events = collections.deque(maxlen=HISTORY)
        self.serial = 0
        self.event_ids = collections.deque(maxlen=HISTORY * 2)
        self.event_id_set = set()
        self.cache = {}
        self.cache_locks = {}
        self.stream_connected = False
        self.stream_at = None

    def start_thread(self, fn):
        thread = threading.Thread(target=fn, daemon=True, name=self.id)
        self.threads.append(thread)
        thread.start()

    def start(self):
        if self.config.get("_configuration_error"):
            return
        self.start_thread(self.poll)
        if self.kind == "speech":
            self.start_thread(self.listen)

    def close(self):
        self.stop.set()
        for thread in self.threads:
            thread.join(timeout=0.2)

    def set_status(self, feeds, detail=None, session=None):
        with self.lock:
            self.updated, self.error = time.monotonic(), None
            self.feeds = feeds
            self.detail = detail or {}
            if session is not None and session != self.session:
                self.session = session
                self.events.clear()
                self.event_ids.clear()
                self.event_id_set.clear()
                self.cache.clear()

    def failed(self, error):
        with self.lock:
            self.error = str(error)

    def status(self):
        with self.lock:
            age = None if self.updated is None else time.monotonic() - self.updated
            reachable = self.error is None and age is not None and age < STALE
            feeds = {k: dict(v) for k, v in self.feeds.items()}
            for feed in feeds.values():
                if not reachable:
                    feed.update(online=False, reason=self.error or "source stopped updating")
            return {"id": self.id, "kind": self.kind, "label": self.config.get("label", self.id),
                    "machine": self.config.get("machine", urllib.parse.urlsplit(self.url).hostname or "this computer"),
                    "reachable": reachable, "age": None if age is None else round(age, 1),
                    "reason": self.error or (None if reachable else "source stopped updating"),
                    "feeds": feeds, "session": self.session, "detail": self.detail,
                    "controls": bool(self.config.get("controls", False)),
                    "layout": self.config.get("layout", "hybrid")}

    def paths(self):
        if self.kind in ("him", "robot"):
            return set()  # HIM's iframe talks to its own local server; robot status is drawn by the hub.
        layout = self.config.get("layout", "hybrid")
        if self.kind == "speech":
            return {"api/status", "api/devices"}
        if layout == "hybrid":
            return {"lidar/points", "lidar/calib", "hybrid/points", "hybrid/stats",
                    "camera/depth.jpg", "camera/map.jpg", "camera/colour.jpg", "camera/stats", "camera/measure"}
        if layout == "lidar":
            return {"points", "calib"}
        return {"depth.jpg", "map.jpg", "colour.jpg", "stats", "measure"}

    def request(self, path, data=None):
        """Fixed paths on configured origins only. Share short-lived binary snapshots."""
        parsed = urllib.parse.urlsplit(path)
        commands = {"api/start", "api/stop", "api/say", "api/llm/start", "api/llm/stop"}
        if parsed.scheme or parsed.netloc or parsed.fragment or parsed.path not in (commands if data is not None else self.paths()):
            return 404, "application/json", b'{"error":"unknown feed endpoint"}'
        if data is not None and (self.kind != "speech" or not self.config.get("controls", False)):
            return 403, "application/json", b'{"error":"speech controls are disabled for this source"}'
        if self.kind == "sensors" and parsed.path.endswith("points"):
            health = self.status()
            if not health["reachable"] or not any(v["online"] for v in health["feeds"].values()):
                return 200, "application/octet-stream", b""
        if parsed.query:
            if not parsed.path.endswith("measure"):
                # Existing pages append cache busters. Do not multiply the shared cache.
                path = parsed.path
            else:
                query = urllib.parse.parse_qs(parsed.query, strict_parsing=True)
                if set(query) != {"x", "y"} or any(len(v) != 1 or not 0 <= float(v[0]) <= 1 for v in query.values()):
                    return 400, "application/json", b'{"error":"measurement coordinates must be between zero and one"}'
        if data is not None or parsed.path.endswith("measure") or self.kind == "speech":
            return fetch(self.url + "/" + path, data)
        with self.lock:
            guard = self.cache_locks.setdefault(path, threading.Lock())
        with guard:
            cached = self.cache.get(path)
            if cached and time.monotonic() - cached[0] < 0.09:
                return cached[1]
            result = fetch(self.url + "/" + path)
            if self.kind == "sensors" and parsed.path == "hybrid/points" and result[0] == 200:
                # Existing producers may retain their last cloud after a device stops.
                # Remove only the offline layer while a different sensor stays live.
                health = self.status()["feeds"]
                body = result[2]
                if len(body) % 8:
                    raise ValueError("invalid hybrid point record length")
                live = {0: health.get("lidar", {}).get("online", False), 1: health.get("depth", {}).get("online", False)}
                body = b"".join(body[i:i+8] for i in range(0, len(body), 8) if live.get(body[i+6], False))
                result = result[0], result[1], body
            with self.lock:
                self.cache[path] = time.monotonic(), result
            return result

    def poll(self):
        while not self.stop.is_set():
            try:
                if self.kind == "him":
                    code, kind, body = fetch(self.url + "/")
                    if code != 200 or "text/html" not in kind or b"G1 sim replay" not in body:
                        raise ValueError("HIM viewer not found at its configured local address")
                    self.set_status({"viewer": {"online": True, "reason": "HIM viewer is available"}},
                                    {"viewer": self.url + "/"})
                elif self.kind == "robot":
                    value = get_json(self.url + "/status")
                    self.set_status(robot_feeds(value), value)
                elif self.kind == "speech":
                    value = get_json(self.url + "/api/status")
                    with self.lock:
                        stream_ok = self.stream_connected and self.stream_at is not None and time.monotonic() - self.stream_at < 25
                    self.set_status({"speech": {"online": stream_ok, "reason": "running" if value.get("running") else "idle"} if stream_ok else
                                     {"online": False, "reason": "waiting for speech event stream"}}, value, value.get("instance"))
                else:
                    layout = self.config.get("layout", "hybrid")
                    feeds, detail, errors = {}, {}, []
                    if layout != "camera":
                        try:
                            s = get_json(self.url + ("/lidar/calib" if layout == "hybrid" else "/calib"))
                            feeds["lidar"] = {"online": bool(s.get("connected")), "reason": s.get("why")}
                            detail["lidar"] = s
                        except (OSError, ValueError, TypeError) as error:
                            feeds["lidar"] = {"online": False, "reason": str(error)}
                            errors.append(str(error))
                    if layout != "lidar":
                        try:
                            s = get_json(self.url + ("/camera/stats" if layout == "hybrid" else "/stats"))
                            for key in ("depth", "colour"):
                                feeds[key] = {"online": bool(s.get(key + "_connected")), "reason": s.get(key + "_why")}
                            detail["camera"] = s
                        except (OSError, ValueError, TypeError) as error:
                            for key in ("depth", "colour"):
                                feeds[key] = {"online": False, "reason": str(error)}
                            errors.append(str(error))
                    if not detail:
                        raise ValueError("; ".join(errors))
                    self.set_status(feeds, detail)
            except (OSError, ValueError, KeyError, TypeError) as error:
                offline = {"him": "HIM viewer is offline. Start a simulation with its viewer enabled.",
                           "robot": "Robot status is offline. Start testground/robot/status_server.py on PC2."}
                self.failed(offline[self.kind] if self.kind in offline and isinstance(error, OSError) else error)
            self.stop.wait(1)

    def append(self, message, identity=None):
        with self.lock:
            if identity is not None:
                if identity in self.event_id_set:
                    return
                if len(self.event_ids) == self.event_ids.maxlen:
                    self.event_id_set.discard(self.event_ids[0])
                self.event_ids.append(identity)
                self.event_id_set.add(identity)
            self.serial += 1
            self.events.append((self.serial, message))

    def since(self, cursor):
        with self.lock:
            gap = cursor is not None and bool(self.events) and cursor < self.events[0][0] - 1
            return gap, [(n, e) for n, e in self.events if cursor is None or n > cursor]

    def listen(self):
        while not self.stop.is_set():
            try:
                # Resolve instance before reading history to avoid crossing process restarts.
                value = get_json(self.url + "/api/status")
                instance = value.get("instance")
                with self.lock:
                    if instance is not None and instance != self.session:
                        self.session = instance
                        self.events.clear()
                        self.event_ids.clear()
                        self.event_id_set.clear()
                with HTTP.open(self.url + "/events", timeout=20) as response:
                    with self.lock:
                        self.stream_connected = True
                        self.stream_at = time.monotonic()
                    while not self.stop.is_set():
                        line = response.readline(256 * 1024 + 1)
                        if not line:
                            break
                        if len(line) > 256 * 1024:
                            raise ValueError("speech event too large")
                        with self.lock:
                            self.stream_at = time.monotonic()
                        if line.startswith(b"data:"):
                            message = json.loads(line[5:])
                            if not isinstance(message, dict) or message.get("event") == "hello":
                                continue
                            self.append(message, (instance, message["n"]) if "n" in message else None)
            except (OSError, ValueError, TypeError) as error:
                self.failed(error)
            finally:
                with self.lock:
                    self.stream_connected = False
            self.stop.wait(1)
