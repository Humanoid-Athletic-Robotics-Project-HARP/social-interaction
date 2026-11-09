"""A web page for the speech pipeline (context/speech-web.md): start listening and talking,
watch what listening hears (loudness, Silero's probability, its state) and the
conversation with each turn's timings, read both programs' logs, and type a line for the
person.

  .venv/bin/python web.py                   then open http://127.0.0.1:28733
  .venv/bin/python web.py --host 0.0.0.0    reachable from other computers too (PC2)

It runs listen.py --events and talk.py --heard as its own child processes, in place of
the shell pipe `listen.py --mic | talk.py --heard`: listening's turns go on to talking
(once talking has said its greeting), and everything both print goes to the page as
server-sent events. It can also start llama-server with the README's command, and stops
it again on exit if it started it. Plain stdlib Python that also runs on 3.8 (PC2's).

Anyone who can reach the page can start the mic and the speakers, so it listens only on
this computer unless --host says otherwise. Commands must come as JSON, which a page on
another site can't send here without the browser asking first.
"""

import argparse
import collections
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import threading
import time
import urllib.error
import urllib.parse
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import listen
import talk
from listen import MUTE_PORT, log, models_dir

HERE = Path(__file__).resolve().parent
PORT = 28733          # beside llama-server (28731) and the speaking signal (28732)
LLM_URL = "http://127.0.0.1:28731"   # talk.py's default
LLM_MODEL = "Qwen3-4B-Instruct-2507-Q4_K_M.gguf"
LLM_ARGS = ["--device", "none", "-ngl", "0", "-c", "4096", "-t", "4"]   # README's (M1: CPU), and --port
HISTORY = 3000        # events kept for a page that opens mid-conversation (levels aren't kept)
STOP_S = 8.0          # s a program gets to finish before it is ended
LIMITS = {   # listening's and talking's settings the page draws (their docstrings say what each is)
    "start_prob": listen.START_PROB, "end_prob": listen.END_PROB, "check": listen.CHECK,
    "finished": listen.FINISHED, "wait": listen.WAIT, "silence": listen.SILENCE,
    "short_turn": listen.SHORT_TURN, "max_speech": listen.MAX_SPEECH, "filler": talk.FILLER,
    "give_up": talk.GIVE_UP, "max_turns": talk.MAX_TURNS, "max_s": talk.MAX_S}


class Page(collections.deque):
    """One open page's events not yet sent, and a flag for new ones."""

    def __init__(self):
        super().__init__(maxlen=HISTORY)
        self.ready = threading.Event()


class Hub:
    """Events to every open page: each gets a numbered copy; a new page gets the history."""

    def __init__(self):
        self.lock = threading.Lock()
        self.pages = []
        self.history = collections.deque(maxlen=HISTORY)
        self.count = 0

    def publish(self, source, event, **values):
        with self.lock:
            self.count += 1
            message = json.dumps({"n": self.count, "time": round(time.time(), 3),
                                  "source": source, "event": event, **values})
            if event != "level":
                self.history.append(message)
            for page in self.pages:
                page.append(message)
            for page in self.pages:
                page.ready.set()

    def subscribe(self):
        page = Page()
        with self.lock:
            page.extend(self.history)
            page.ready.set()
            self.pages.append(page)
        return page

    def unsubscribe(self, page):
        with self.lock:
            self.pages.remove(page)

    def clear(self):
        with self.lock:
            self.history.clear()


def llm_status(url):
    """ok, loading (the model is still loading) or down."""
    try:
        with urllib.request.urlopen(url + "/health", timeout=1) as response:
            return "ok" if response.status == 200 else "loading"
    except urllib.error.HTTPError as error:
        return "loading" if error.code == 503 else "down"
    except OSError:
        return "down"


def port_taken(port):
    """True if another program holds the speaking signal's port: a listen.py started in a
    terminal, which this page can't see and whose port its own listen.py can't get."""
    probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        probe.bind(("127.0.0.1", port))
        return False
    except OSError:
        return True
    finally:
        probe.close()


def devices():
    """This computer's microphones and speakers, as sounddevice numbers them."""
    try:
        import sounddevice
        found = sounddevice.query_devices()
        default_in, default_out = sounddevice.default.device
    except Exception as error:   # no PortAudio (PC2) or no sounddevice
        return {"inputs": [], "outputs": [], "error": str(error)}
    inputs = [{"index": i, "name": d["name"], "default": i == default_in}
              for i, d in enumerate(found) if d["max_input_channels"] > 0]
    outputs = [{"index": i, "name": d["name"], "default": i == default_out}
               for i, d in enumerate(found) if d["max_output_channels"] > 0]
    return {"inputs": inputs, "outputs": outputs}


def end(process, interrupt=True):
    """End a child process: Ctrl-C first (listen.py stops its mic cleanly), then kill."""
    if process is None or process.poll() is not None:
        return
    try:
        process.send_signal(signal.SIGINT if interrupt and os.name == "posix" else signal.SIGTERM)
        process.wait(STOP_S)
    except subprocess.TimeoutExpired:
        process.kill()
    except OSError:
        pass


class Session:
    """One run of listen.py and/or talk.py, with the pipe between them."""

    def __init__(self, hub, config, llm_url, mute_port):
        self.hub, self.config = hub, config
        self.llm_url, self.mute_port = llm_url, mute_port
        self.lock = threading.Lock()
        self.listen = self.talk = None
        self.ready = False   # talk.py has said its greeting, so it's reading turns
        self.over = False

    def command(self, script, *args):
        return [sys.executable, "-u", str(HERE / script)] + [str(a) for a in args]

    def start(self):
        c = self.config
        if c["talk"]:
            args = ["--heard", "--voice", c["voice"], "--llm", self.llm_url,
                    "--mute-port", self.mute_port]
            if not c["play"]:
                args.append("--no-play")
            if c.get("output_device") is not None:
                args += ["--output-device", c["output_device"]]
            self.talk = self.spawn("talk", self.command("talk.py", *args), stdin=True)
        if c["listen"]:
            args = ["--events", "--mute-port", self.mute_port]
            if c["source"] == "wav":
                args += ["--wav", c["wav"]] + (["--realtime"] if c.get("realtime", True) else [])
            elif c["source"] == "robot":
                args += ["--robot"] + (["--iface", c["iface"]] if c.get("iface") else [])
            else:
                args.append("--mic")
                if c.get("device") is not None:
                    args += ["--device", c["device"]]
            if c.get("fixed_turn"):
                args.append("--fixed-turn")
            if not c["talk"]:
                args.append("--no-mute")
            self.listen = self.spawn("listen", self.command("listen.py", *args))
        self.hub.publish("web", "session", state="started", config=c)

    def spawn(self, name, command, stdin=False):
        self.hub.publish("web", "log", text="$ " + " ".join(Path(command[0]).name if i == 0 else a
                                                           for i, a in enumerate(command)))
        process = subprocess.Popen(command, cwd=str(HERE), stdin=subprocess.PIPE if stdin else
                                   subprocess.DEVNULL, stdout=subprocess.PIPE,
                                   stderr=subprocess.PIPE, universal_newlines=True, bufsize=1)
        threading.Thread(target=self.read_out, args=(name, process), daemon=True).start()
        threading.Thread(target=self.read_err, args=(name, process), daemon=True).start()
        return process

    def read_err(self, name, process):
        for line in process.stderr:
            if line.strip():
                self.hub.publish(name, "log", text=line.rstrip())

    def read_out(self, name, process):
        for line in process.stdout:
            try:
                message = json.loads(line)
            except ValueError:
                self.hub.publish(name, "log", text=line.rstrip())
                continue
            if name == "listen":
                self.from_listen(message, line)
            else:
                self.from_talk(message)
        code = process.wait()
        self.hub.publish(name, "exit", code=code)
        self.exited(name)

    def from_listen(self, message, line):
        if "event" in message:
            event = message.pop("event")
            self.hub.publish("listen", event, **message)
            return
        forwarded = self.send(line)
        self.hub.publish("listen", "turn", forwarded=forwarded, **message)

    def from_talk(self, message):
        event = message.pop("event", "log")
        if event == "said" and not self.ready:
            self.ready = True
            self.hub.publish("web", "talk_ready")
        self.hub.publish("talk", event, **message)

    def send(self, line):
        """A turn to talk.py; False when there is no talking, or it isn't ready yet."""
        with self.lock:
            if not self.ready or self.talk is None or self.talk.poll() is not None:
                return False
            try:
                self.talk.stdin.write(line if line.endswith("\n") else line + "\n")
                self.talk.stdin.flush()
                return True
            except (OSError, ValueError):
                return False

    def typed(self, text):
        sent = self.send(json.dumps({"text": text}))
        self.hub.publish("web", "typed", text=text, forwarded=sent)
        return sent

    def exited(self, name):
        """When talking ends (its goodbye) the session ends; when listening ends (a
        recording's end, or an error) talking goes on, for typed lines."""
        if name == "talk" or not self.config["talk"]:
            threading.Thread(target=self.stop, daemon=True).start()

    def stop(self):
        with self.lock:
            if self.over:
                return
            self.over = True
            if self.talk and self.talk.poll() is None:
                try:
                    self.talk.stdin.close()   # talk.py finishes what it's saying, then ends
                except OSError:
                    pass
        end(self.listen)
        if self.talk:
            try:
                self.talk.wait(STOP_S)
            except subprocess.TimeoutExpired:
                end(self.talk, interrupt=False)
        self.hub.publish("web", "session", state="stopped")

    @property
    def running(self):
        return not self.over and any(p is not None and p.poll() is None
                                     for p in (self.listen, self.talk))


class Server:
    def __init__(self, llm_url=LLM_URL, mute_port=MUTE_PORT):
        self.llm_url, self.mute_port = llm_url.rstrip("/"), mute_port
        self.hub = Hub()
        self.session = None
        self.llm = None   # llama-server, when started from the page
        self.lock = threading.Lock()

    def status(self):
        s = self.session
        ours = bool(s and s.listen and s.listen.poll() is None)
        return {"other_listen": not ours and port_taken(self.mute_port), "mute_port": self.mute_port,"llm": llm_status(self.llm_url), "llm_url": self.llm_url, "llm_started_here": self.llm is not None and self.llm.poll() is None,
                "running": bool(s and s.running), "talk_ready": bool(s and s.ready),
                "config": s.config if s else None,
                "models": str(models_dir()), "limits": LIMITS, "host": socket.gethostname()}

    def start(self, body):
        config = {"listen": bool(body.get("listen", True)), "talk": bool(body.get("talk", True)),
                  "source": body.get("source", "mic"), "device": body.get("device"),
                  "wav": body.get("wav", ""), "realtime": bool(body.get("realtime", True)),
                  "iface": body.get("iface") or None, "fixed_turn": bool(body.get("fixed_turn")),
                  "voice": body.get("voice", "kokoro"), "play": bool(body.get("play", True)),
                  "output_device": body.get("output_device")}
        if not (config["listen"] or config["talk"]):
            return 400, {"error": "Pick listening, talking or both."}
        if config["source"] not in ("mic", "robot", "wav") or config["voice"] not in ("kokoro", "piper"):
            return 400, {"error": "Unknown source or voice."}
        if config["listen"] and config["source"] == "wav":
            path = Path(config["wav"]).expanduser()
            path = path if path.is_absolute() else HERE / path
            if not path.is_file():
                return 400, {"error": f"No recording at {path}."}
            config["wav"] = str(path)
        if config["talk"] and llm_status(self.llm_url) != "ok":
            return 409, {"error": "The LLM server isn't ready: start it first (LLM, Start)."}
        if config["listen"] and config["source"] != "wav" and port_taken(self.mute_port):
            return 409, {"error": f"Another listen.py is running (port {self.mute_port} is taken): "
                                  "stop it in its terminal first."}
        with self.lock:
            if self.session and self.session.running:
                return 409, {"error": "Already running: stop it first."}
            self.hub.clear()
            self.session = Session(self.hub, config, self.llm_url, self.mute_port)
            self.session.start()
        return 200, {"ok": True}

    def stop(self):
        if self.session:
            threading.Thread(target=self.session.stop, daemon=True).start()
        return 200, {"ok": True}

    def say(self, body):
        text = str(body.get("text", "")).strip()
        if not text:
            return 400, {"error": "Nothing to say."}
        if not (self.session and self.session.running and self.session.config["talk"]):
            return 409, {"error": "Talking isn't running."}
        if not self.session.typed(text):
            return 409, {"error": "HARP isn't ready yet: wait for the greeting."}
        return 200, {"ok": True}

    def start_llm(self):
        with self.lock:
            if llm_status(self.llm_url) != "down" or (self.llm and self.llm.poll() is None):
                return 200, {"ok": True}
            model = models_dir() / LLM_MODEL
            if not model.exists():
                return 400, {"error": f"{model} is missing: README.md says how to get it."}
            port = urllib.parse.urlsplit(self.llm_url).port or 80
            command = ["llama-server", "-m", str(model)] + LLM_ARGS + ["--port", str(port)]
            try:
                self.llm = subprocess.Popen(command, stdin=subprocess.DEVNULL, stdout=subprocess.PIPE,
                                            stderr=subprocess.STDOUT, universal_newlines=True, bufsize=1)
            except OSError as error:
                return 400, {"error": f"Couldn't start llama-server ({error}): README.md says how to install it."}
        self.hub.publish("web", "log", text="$ " + " ".join(command))
        threading.Thread(target=self.read_llm, args=(self.llm,), daemon=True).start()
        return 200, {"ok": True}

    def read_llm(self, process):
        for line in process.stdout:   # llama-server is chatty: only what matters
            if any(word in line for word in ("error", "listening", "loaded", "failed")):
                self.hub.publish("llm", "log", text=line.rstrip())
        self.hub.publish("llm", "exit", code=process.wait())

    def stop_llm(self):
        if self.llm:
            end(self.llm, interrupt=False)
            self.llm = None
        return 200, {"ok": True}

    def close(self):
        if self.session:
            self.session.stop()
        self.stop_llm()


def handler(server):
    class Handler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"

        def log_message(self, *args):
            pass   # the page shows what matters

        def reply(self, code, body, kind="application/json"):
            data = body if isinstance(body, bytes) else json.dumps(body).encode()
            self.send_response(code)
            self.send_header("Content-Type", kind)
            self.send_header("Content-Length", str(len(data)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(data)

        def do_GET(self):
            path = self.path.split("?")[0]
            if path == "/":
                self.reply(200, (HERE / "web.html").read_bytes(), "text/html; charset=utf-8")
            elif path == "/api/status":
                self.reply(200, server.status())
            elif path == "/api/devices":
                self.reply(200, devices())
            elif path == "/events":
                self.events()
            else:
                self.reply(404, {"error": "not found"})

        def do_POST(self):
            if self.headers.get("Content-Type", "").split(";")[0] != "application/json":
                return self.reply(415, {"error": "send JSON"})   # so other sites' forms can't
            try:
                body = json.loads(self.rfile.read(int(self.headers.get("Content-Length", 0))) or b"{}")
            except ValueError:
                return self.reply(400, {"error": "bad JSON"})
            routes = {"/api/start": lambda: server.start(body), "/api/stop": server.stop,
                      "/api/say": lambda: server.say(body), "/api/llm/start": server.start_llm,
                      "/api/llm/stop": server.stop_llm}
            route = routes.get(self.path)
            self.reply(*(route() if route else (404, {"error": "not found"})))

        def events(self):
            self.send_response(200)
            self.send_header("Content-Type", "text/event-stream")
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            page = server.hub.subscribe()
            try:
                self.wfile.write(b'data: {"event": "hello"}\n\n')
                while True:
                    if not page.ready.wait(15):
                        self.wfile.write(b": still here\n\n")   # keeps proxies and browsers from timing out
                    page.ready.clear()
                    while page:
                        self.wfile.write(b"data: " + page.popleft().encode() + b"\n\n")
                    self.wfile.flush()
            except (OSError, ValueError):
                pass
            finally:
                server.hub.unsubscribe(page)
                self.close_connection = True

    return Handler


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--host", default="127.0.0.1", help="0.0.0.0 to reach it from other computers")
    parser.add_argument("--port", type=int, default=PORT)
    parser.add_argument("--llm", default=LLM_URL, help="llama-server's address (Start runs it on its port)")
    parser.add_argument("--mute-port", type=int, default=MUTE_PORT,
                        help="the speaking signal's port, for both programs")
    args = parser.parse_args()
    server = Server(args.llm, args.mute_port)
    httpd = ThreadingHTTPServer((args.host, args.port), handler(server))
    httpd.daemon_threads = True
    log(f"speech page at http://{'127.0.0.1' if args.host == '0.0.0.0' else args.host}:{args.port}"
        " (Ctrl-C to stop)")
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.close()
        httpd.server_close()


if __name__ == "__main__":
    main()
