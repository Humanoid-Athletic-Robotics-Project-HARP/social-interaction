"""Resolve private robot addresses on the server, never in tracked configuration."""
import copy
from pathlib import Path
import shlex
import subprocess
import urllib.parse

ROOT = Path(__file__).resolve().parents[1]
DEFAULT = {"sources": [
    {"id": "mac-sim", "kind": "simulation", "label": "Laptop simulation", "machine": "MacBook"},
    {"id": "mac-speech", "kind": "speech", "label": "Laptop speech", "machine": "MacBook",
     "url": "http://127.0.0.1:28733", "controls": True},
    {"id": "pc2-sensors", "kind": "sensors", "label": "Jetson sensors", "machine": "Jetson PC2",
     "host_env": "G1_HOST", "port": 8082, "layout": "hybrid"},
    {"id": "pc2-speech", "kind": "speech", "label": "Jetson speech", "machine": "Jetson PC2",
     "host_env": "G1_HOST", "port": 28733, "controls": False},
    {"id": "pc2-robot", "kind": "robot", "label": "Robot status", "machine": "Jetson PC2",
     "host_env": "G1_HOST", "port": 28735},
    {"id": "him-sim", "kind": "him", "label": "HIM simulator", "machine": "MacBook",
     "url": "http://127.0.0.1:8765"},
]}


def main_checkout(root):
    """The main checkout, also from a worktree: its .env and runs/ are shared by all of them."""
    try:
        result = subprocess.run(["git", "-C", str(root), "rev-parse", "--path-format=absolute", "--git-common-dir"],
                                capture_output=True, text=True, timeout=2)
        common = result.stdout.strip()
        if result.returncode == 0 and common:
            return Path(common).resolve().parent
    except (OSError, subprocess.TimeoutExpired):
        pass
    return Path(root)


def env_file(root):
    """Worktrees share the main checkout's gitignored .env, as testground/sync.sh does."""
    return main_checkout(root) / ".env"


def robot_host(path):
    """Parse one dotenv value as text. Never execute shell or expand variables."""
    if not path.is_file():
        return None, []
    for line in path.read_text().splitlines():
        line = line.strip()
        if line.startswith("export "):
            line = line[7:].lstrip()
        key, separator, raw = line.partition("=")
        if not separator or key.strip() != "G1_HOST":
            continue
        try:
            tokens = shlex.split(raw, comments=True)
            if not tokens:
                return None, []
            if len(tokens) != 1:
                raise ValueError()
            value = tokens[0]
            parsed = urllib.parse.urlsplit("//" + value)
            host = parsed.hostname
            if (not host or parsed.port is not None or parsed.password is not None or parsed.path or
                    parsed.query or parsed.fragment or any(c.isspace() for c in host)):
                raise ValueError()
            # Hostname text must not turn into a URL path, shell expression or template.
            if any(c in host for c in '\\/$`{}<>"\''):
                raise ValueError()
            return host, [value, host]
        except ValueError:
            raise ValueError("G1_HOST in .env must be a hostname or user@hostname (IPv6 needs brackets)") from None
    return None, []


def resolve(config=None, root=ROOT):
    result = copy.deepcopy(DEFAULT if config is None else config)
    if not isinstance(result, dict) or not isinstance(result.get("sources"), list):
        raise ValueError('configuration needs a "sources" list')
    targets = [s for s in result["sources"] if isinstance(s, dict) and "host_env" in s]
    if not targets:
        return result
    for source in targets:
        if source["host_env"] != "G1_HOST" or source.get("kind") not in ("sensors", "speech", "robot"):
            raise ValueError("host_env supports G1_HOST for sensor, speech and robot status services")
    host, private = robot_host(env_file(root))
    for source in targets:
        if "url" in source:
            raise ValueError("choose host_env or url for a source, not both")
        port = source.get("port", {"sensors": 8082, "robot": 28735}.get(source["kind"], 28733))
        if isinstance(port, bool) or not isinstance(port, int) or not 1 <= port <= 65535:
            raise ValueError("source port must be between 1 and 65535")
        source.setdefault("machine", "Jetson PC2")
        source["_private"] = private
        if host:
            source["url"] = "http://%s:%s" % ("[" + host + "]" if ":" in host else host, port)
        else:
            source["url"] = ""
            source["_configuration_error"] = "Set G1_HOST in the main checkout's .env, then restart the dashboard"
    return result
