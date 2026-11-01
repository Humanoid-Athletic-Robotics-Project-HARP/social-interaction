"""Record the G1's head lidar on PC2, exactly as it arrives, to copy to a laptop and
analyse there (capture.py, report.py, view.py). Read-only: it only listens.

The MID-360 pushes Livox SDK2 UDP packets to PC2 (192.168.123.164) by itself, from
192.168.123.120: points to port 56301 (about 2,080 packets/s, 2.9 MB/s), its IMU to
56401 (200 Hz) and a status message to 56201 (1 Hz). There is no DDS lidar topic on
this robot. Only one program can receive each port, so stop any other lidar reader
(a viewer, livox_ros_driver2) first. Optionally also rt/lowstate over DDS (the
joints, for the arm and waist pose), through Unitree's SDK, subscribing only.

  python3 record.py NAME [--seconds 60] [--note "wall at 2.00 m"] [--headers-only]

writes captures/NAME/ beside this file:
  meta.json     what was recorded and how, packet counts, the kernel's drops
  udp.bin       every packet: int64 arrival ns (this computer's clock), uint16 port,
                uint16 length, then the packet's bytes (only the 36-byte header of
                point packets with --headers-only: about 100 KB/s, for long runs)
  lowstate.csv  rt/lowstate (pelvis IMU and joints), thinned to --lowstate-hz

About 175 MB a minute in full. Python 3.8 (PC2's); runs at low priority beside
HARP's harness.
"""

import argparse
import datetime
import json
import os
from pathlib import Path
import platform
import select
import shutil
import signal
import socket
import struct
import subprocess
import sys
import time

PORTS = {56301: "points", 56401: "imu", 56201: "status"}   # PC2's ends of the lidar's streams
HEADER = 36          # bytes of Livox SDK2 header on point and IMU packets
RECORD = struct.Struct("<qHH")   # arrival ns, port, length, before each packet in udp.bin
RATE = 3e6           # bytes/s in full, for the space check
JOINTS = 35          # motor_state slots in unitree_hg's LowState_ (the G1 uses 29)


def udp_drops():
    """Datagrams the kernel dropped per local UDP port (a full receive buffer)."""
    drops = {}
    for line in Path("/proc/net/udp").read_text().splitlines()[1:]:
        cols = line.split()
        drops[int(cols[1].split(":")[1], 16)] = int(cols[-1])
    return drops


class LowState:
    """rt/lowstate to lowstate.csv, from the SDK's reader thread."""

    def __init__(self, path, hz, interface, domain):
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
        self.file = open(path, "w")
        self.file.write(",".join(["arrival_ns", "tick", "mode_machine", "qw", "qx", "qy", "qz",
                                  "gx", "gy", "gz", "ax", "ay", "az", "roll", "pitch", "yaw"]
                                 + [f"q{i}" for i in range(JOINTS)]) + "\n")
        self.period = 1e9 / hz
        self.last = 0
        self.received = self.kept = 0
        ChannelFactoryInitialize(domain, interface)
        self.sub = ChannelSubscriber("rt/lowstate", LowState_)
        self.sub.Init(self.on_msg, 0)

    def on_msg(self, msg):
        arrival = time.time_ns()
        self.received += 1
        if arrival - self.last < self.period:
            return
        self.last = arrival
        imu = msg.imu_state
        row = ([arrival, msg.tick, msg.mode_machine] + list(imu.quaternion) + list(imu.gyroscope)
               + list(imu.accelerometer) + list(imu.rpy) + [m.q for m in msg.motor_state])
        self.file.write(",".join(str(v) for v in row) + "\n")
        self.kept += 1

    def close(self):
        # The SDK's reader first: with it still open the interpreter hangs on the way out.
        self.sub.Close()
        self.file.close()


def commit(folder):
    """The git commit these scripts are at, if they're in a clone (else None)."""
    try:
        out = subprocess.run(["git", "-C", str(folder), "describe", "--always", "--dirty"],
                             capture_output=True, text=True, timeout=5)
    except Exception:
        return None
    return out.stdout.strip() or None


def utc():
    return datetime.datetime.now(datetime.timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")


def stop(signum, frame):
    raise KeyboardInterrupt


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("name", help="the capture's folder name, e.g. static-open")
    parser.add_argument("--seconds", type=float, default=60.0, help="how long to record")
    parser.add_argument("--note", default="", help="what the scene was (distances, objects)")
    parser.add_argument("--headers-only", action="store_true",
                        help="keep only point packets' headers (timing and loss, no points)")
    parser.add_argument("--no-lowstate", action="store_true", help="skip rt/lowstate")
    parser.add_argument("--lowstate-hz", type=float, default=50.0)
    parser.add_argument("--interface", default="eth0", help="PC2's port on the robot network")
    parser.add_argument("--domain", type=int, default=0)
    args = parser.parse_args()

    folder = Path(__file__).resolve().parent / "captures" / args.name
    if folder.exists():
        sys.exit(f"{folder} already exists; pick another name.")
    need = (0.1e6 if args.headers_only else RATE) * args.seconds + 1e9
    if shutil.disk_usage(folder.parent.parent).free < need:
        sys.exit(f"Not enough disk for {args.seconds:.0f} s.")
    os.nice(10)   # below HARP's harness

    folder.mkdir(parents=True)
    meta = {"name": args.name, "note": args.note, "started": utc(), "host": socket.gethostname(),
            "python": platform.python_version(), "commit": commit(folder.parent.parent),
            "ports": {str(p): n for p, n in PORTS.items()}, "headers_only": args.headers_only,
            "seconds_requested": args.seconds}
    # DDS first: it takes a moment to start, and the lidar's ports must be read from
    # the moment they open, or the kernel drops packets.
    low = None
    if not args.no_lowstate:
        try:
            low = LowState(folder / "lowstate.csv", args.lowstate_hz, args.interface, args.domain)
        except Exception as e:
            print(f"No rt/lowstate ({type(e).__name__}: {e}); recording the lidar only.", flush=True)
    socks = {}
    for port in PORTS:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 8 << 20)
        try:
            s.bind(("0.0.0.0", port))
        except OSError as e:
            if low is not None:
                low.close()
            sys.exit(f"Can't listen on UDP {port} ({e}): another lidar reader is running? "
                     f"`ss -uanp | grep {port}` shows which.")
        socks[s] = port
    meta["receive_buffer"] = next(iter(socks)).getsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF)
    drops_before = udp_drops()
    counts = {name: 0 for name in PORTS.values()}
    sources = {}
    out = open(folder / "udp.bin", "wb", buffering=1 << 20)
    print(f"Recording {folder} for {args.seconds:.0f} s (Ctrl-C to stop early).", flush=True)
    signal.signal(signal.SIGTERM, stop)
    start = time.monotonic()
    tick, before = start + 1.0, dict(counts)
    try:
        while time.monotonic() - start < args.seconds:
            for s in select.select(list(socks), [], [], 0.1)[0]:
                data, source = s.recvfrom(65535)
                arrival = time.time_ns()
                port = socks[s]
                name = PORTS[port]
                counts[name] += 1
                sources[name] = source[0]
                if args.headers_only and name == "points":
                    data = data[:HEADER]
                out.write(RECORD.pack(arrival, port, len(data)))
                out.write(data)
            if time.monotonic() >= tick:
                rates = "  ".join(f"{n} {counts[n] - before[n]:4d}/s" for n in PORTS.values())
                print(f"{tick - start:4.0f} s  {rates}  {out.tell() / 1e6:6.0f} MB", flush=True)
                if counts["points"] == before["points"]:
                    print("No point packets this second.", flush=True)
                tick, before = tick + 1.0, dict(counts)
    except KeyboardInterrupt:
        print("Stopped early.", flush=True)
    finally:
        if low is not None:
            low.close()
        out.close()
        drops_after = udp_drops()
        for s in socks:
            s.close()
        meta.update({"ended": utc(), "seconds": round(time.monotonic() - start, 3),
                     "counts": counts, "sources": sources,
                     "kernel_drops": {PORTS[p]: drops_after.get(p, 0) - drops_before.get(p, 0)
                                      for p in PORTS},
                     "lowstate": None if low is None else {"received": low.received, "kept": low.kept,
                                                           "hz": args.lowstate_hz}})
        (folder / "meta.json").write_text(json.dumps(meta, indent=2) + "\n")
        print(f"Wrote {folder}: {counts}, kernel drops {meta['kernel_drops']}.", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
