#!/usr/bin/env python3
"""Optional Rerun renderer, isolated from the standard-library dashboard process."""
import json
from pathlib import Path
import signal
import socket
import subprocess
import sys
import time
import urllib.request

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "submission"))
from lidar_view import Recording


def port():
    with socket.socket() as sock:
        sock.bind(("127.0.0.1", 0))
        return sock.getsockname()[1]


def main():
    grpc, web = port(), port()
    while grpc == web:
        web = port()
    exe = Path(sys.executable).parent / "rerun"
    process = subprocess.Popen([str(exe), "--serve-web", "--bind", "127.0.0.1", "--port", str(grpc),
                                "--web-viewer-port", str(web), "--server-memory-limit", "200MB", "--threads", "2"],
                               stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))
    recording = None
    try:
        for _ in range(100):
            if process.poll() is not None:
                raise RuntimeError("Rerun server exited during startup")
            try:
                with urllib.request.urlopen("http://127.0.0.1:%s/" % web, timeout=.2):
                    break
            except OSError:
                time.sleep(.1)
        else:
            raise RuntimeError("Rerun server did not start within ten seconds")
        print(json.dumps({"ready": True, "web_port": web, "grpc_port": grpc}), flush=True)
        for line in sys.stdin:
            message = json.loads(line)
            if "_session" in message:
                if recording:
                    recording.rec.flush()
                recording = Recording(message["_session"], "rerun+http://127.0.0.1:%s/proxy" % grpc)
                recording.rec.send_recording_name(message.get("_name", message["_session"]))
            elif "_flush" in message:
                if recording:
                    recording.rec.flush()
            elif recording:
                recording.add(message)
                # Bound the application-side trails as well as Rerun's own buffer.
                del recording.truth_path[:-2000]
                del recording.odom_path[:-2000]
        if recording:
            recording.rec.flush()
        # Keep a completed recording available until the dashboard closes stdin.
    except KeyboardInterrupt:
        pass
    finally:
        process.terminate()
        try:
            process.wait(3)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()


if __name__ == "__main__":
    try:
        main()
    except Exception as error:
        print(json.dumps({"error": str(error)}), flush=True)
        raise
