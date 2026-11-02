#!/usr/bin/env python3
"""Record the head camera on PC2, to study it on a laptop once the robot is gone: depth,
colour and the robot's joints (its posture decides where the camera looks). Read-only:
by default it shares the camera, reading depth from depth_node.py's ROS 2 topic and
colour from Unitree's videohub; --depth-source device opens the depth camera itself,
when nothing else has it.

  python3 record_camera.py NAME [--seconds 30] [--note "wall at 2.00 m"]

With the default (ROS 2) depth, source ROS 2 and use depth_node.py's settings, and put
Unitree's CycloneDDS first for videohub and the joints (camera/README.md):
  source /opt/ros/foxy/setup.bash
  export ROS_DOMAIN_ID=97 ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  export LD_LIBRARY_PATH=$HOME/cyclonedds_ws/install/cyclonedds/lib:$LD_LIBRARY_PATH

writes captures/NAME/ beside this file:
  meta.json      what was recorded and how, counts, the nominal intrinsics
  depth/N.png    each depth frame: 16-bit PNG in mm, 0 for no depth (lossless)
  depth.csv      frame number, arrival ns (this computer's clock), stamp ns, file
  colour/N.jpg   each colour frame as videohub sent it (1920 x 1080 JPEG)
  colour.csv     frame number, arrival ns, request ms, file
  lowstate.csv   rt/lowstate (pelvis IMU and joints), 50 Hz (as ../lidar/record.py)

About 5 MB/s with the defaults. Python 3.8 (PC2's).
"""
import argparse
import datetime
import json
import os
from pathlib import Path
import platform
import shutil
import signal
import socket
import sys
import threading
import time

# One BLAS thread: numpy's pool otherwise keeps 7 threads spinning on the Jetson's cores
os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
sys.path[:0] = [str(HERE), str(HERE.parent / "lidar")]
from camera_viewer import HFOV, VFOV, V4L2Stream  # noqa: E402
from record import LowState, commit  # noqa: E402  (../lidar/record.py)

RATE = 6e6   # bytes/s, for the space check


def utc():
    return datetime.datetime.now(datetime.timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")


class Writer:
    """Numbered frames into a folder, one CSV row each."""

    def __init__(self, folder, name, ext, columns):
        self.dir, self.ext = folder / name, ext
        self.dir.mkdir()
        self.csv = open(folder / f"{name}.csv", "w")
        self.csv.write(",".join(["frame"] + columns + ["file"]) + "\n")
        self.count, self.lock = 0, threading.Lock()

    def add(self, data, *values):
        with self.lock:
            name = f"{self.count:06d}.{self.ext}"
            (self.dir / name).write_bytes(data)
            self.csv.write(",".join(str(v) for v in [self.count, *values, f"{self.dir.name}/{name}"]) + "\n")
            self.count += 1

    def close(self):
        with self.lock:
            self.csv.close()


def depth_png(frame):
    ok, png = cv2.imencode(".png", frame, [cv2.IMWRITE_PNG_COMPRESSION, 1])
    return png.tobytes() if ok else None


class TopicDepth:
    """Depth frames from a ROS 2 Image topic (depth_node.py's), into the writer."""

    def __init__(self, topic, writer):
        import rclpy
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        from sensor_msgs.msg import Image
        rclpy.init()
        self.node = rclpy.create_node("g1_camera_recorder")
        self.writer, self.size = writer, None
        # reliable: best effort drops most 800 KB images (camera/README.md)
        self.node.create_subscription(Image, topic, self.on_image,
                                      QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE))
        self.running = True
        self.thread = threading.Thread(target=self.spin, daemon=True)
        self.thread.start()

    def spin(self):
        import rclpy
        while self.running and rclpy.ok():   # short steps, so close() can stop it first
            rclpy.spin_once(self.node, timeout_sec=0.1)

    def on_image(self, msg):
        arrival = time.time_ns()
        if msg.encoding not in ("16UC1", "mono16"):
            return
        row = msg.step // 2
        frame = np.frombuffer(bytes(msg.data), ">u2" if msg.is_bigendian else "<u2")
        frame = frame[:row * msg.height].reshape(msg.height, row)[:, :msg.width].astype("<u2")
        png = depth_png(frame)
        if png is not None:
            self.size = (msg.width, msg.height)
            stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
            self.writer.add(png, arrival, stamp)

    def close(self):
        import rclpy
        self.running = False
        self.thread.join(timeout=2)
        self.node.destroy_node()
        rclpy.try_shutdown()


class DeviceDepth(threading.Thread):
    """Depth frames straight from /dev/video0, into the writer (stamp: arrival)."""

    def __init__(self, writer, size, fps):
        super().__init__(daemon=True)
        self.writer, self.size = writer, size
        self.stream = V4L2Stream("/dev/video0", "Z16 ", size[0], size[1], fps, "<u2")
        self.stream.start()
        self.running = True

    def run(self):
        last = 0
        while self.running:
            if self.stream.count == last:
                time.sleep(0.005)
                continue
            last = self.stream.count
            frame = self.stream.latest()
            png = depth_png(frame) if frame is not None else None
            if png is not None:
                now = time.time_ns()
                self.writer.add(png, now, now)

    def close(self):
        self.running = False
        self.stream.close()


class Colour(threading.Thread):
    """videohub JPEGs at up to `hz`, into the writer."""

    def __init__(self, writer, hz):
        super().__init__(daemon=True)
        from unitree_sdk2py.go2.video.video_client import VideoClient
        self.client = VideoClient()
        self.client.SetTimeout(2.0)
        self.client.Init()
        self.writer, self.period, self.running, self.failed = writer, 1.0 / hz, True, 0

    def run(self):
        while self.running:
            start = time.monotonic()
            arrival = time.time_ns()
            try:
                code, data = self.client.GetImageSample()
            except Exception:
                code, data = -1, None
            if code == 0 and data:
                self.writer.add(bytes(data), arrival, round((time.monotonic() - start) * 1000))
            else:
                self.failed += 1
            time.sleep(max(0.0, self.period - (time.monotonic() - start)))


def stop(signum, frame):
    raise KeyboardInterrupt


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("name", help="the capture's folder name, e.g. wall-2m")
    ap.add_argument("--seconds", type=float, default=30.0)
    ap.add_argument("--note", default="", help="what the scene was (distances, objects)")
    ap.add_argument("--depth-source", choices=("ros", "device"), default="ros",
                    help="ros: depth_node.py's topic (shares the camera); device: /dev/video0")
    ap.add_argument("--depth-topic", default="/camera/depth/image_rect_raw")
    ap.add_argument("--size", default="848x480", help="with --depth-source device")
    ap.add_argument("--fps", type=int, default=15, help="with --depth-source device")
    ap.add_argument("--colour-hz", type=float, default=10.0, help="0 for no colour")
    ap.add_argument("--no-lowstate", action="store_true")
    ap.add_argument("--interface", default="eth0", help="PC2's port on the robot network")
    args = ap.parse_args()

    folder = HERE / "captures" / args.name
    if folder.exists():
        sys.exit(f"{folder} already exists; pick another name.")
    if shutil.disk_usage(HERE).free < RATE * args.seconds + 1e9:
        sys.exit(f"Not enough disk for {args.seconds:.0f} s.")
    if args.depth_source == "ros" and os.environ.get("ROS_DOMAIN_ID", "0") == "0":
        sys.exit("Set ROS_DOMAIN_ID to depth_node.py's (not 0: Unitree's DDS domain).")
    os.nice(10)   # below HARP's harness
    folder.mkdir(parents=True)
    size = tuple(int(v) for v in args.size.split("x"))
    fx = (size[0] / 2) / np.tan(np.radians(HFOV / 2))
    fy = (size[1] / 2) / np.tan(np.radians(VFOV / 2))
    meta = {"name": args.name, "note": args.note, "started": utc(), "host": socket.gethostname(),
            "python": platform.python_version(), "commit": commit(HERE),
            "depth_source": args.depth_source, "depth_topic": args.depth_topic,
            "colour_hz": args.colour_hz, "seconds_requested": args.seconds,
            "depth_units": "mm", "frame": "d435_depth_optical_frame (z forward, x right, y down)",
            "nominal_intrinsics": {"hfov_deg": HFOV, "vfov_deg": VFOV, "fx": fx, "fy": fy,
                                   "cx": (size[0] - 1) / 2, "cy": (size[1] - 1) / 2,
                                   "for_size": list(size), "note": "Intel's nominal FOV, not the "
                                   "camera's own calibration"}}
    depth_writer = Writer(folder, "depth", "png", ["arrival_ns", "stamp_ns"])
    colour_writer = Writer(folder, "colour", "jpg", ["arrival_ns", "request_ms"])
    low = colour = None
    if args.colour_hz > 0 or not args.no_lowstate:   # Unitree's DDS, for videohub and the joints
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize
        ChannelFactoryInitialize(0, args.interface)
    if not args.no_lowstate:
        try:
            low = LowState(folder / "lowstate.csv", 50.0, args.interface, 0)
        except Exception as e:
            print(f"No rt/lowstate ({type(e).__name__}: {e}).", flush=True)
    if args.colour_hz > 0:
        try:
            colour = Colour(colour_writer, args.colour_hz)
            colour.start()
        except Exception as e:
            print(f"No colour ({type(e).__name__}: {e}).", flush=True)
    depth = (TopicDepth(args.depth_topic, depth_writer) if args.depth_source == "ros"
             else DeviceDepth(depth_writer, size, args.fps))
    if args.depth_source == "device":
        depth.start()
    print(f"Recording {folder} for {args.seconds:.0f} s (Ctrl-C to stop early).", flush=True)
    signal.signal(signal.SIGTERM, stop)
    start, before = time.monotonic(), (0, 0)
    try:
        while time.monotonic() - start < args.seconds:
            time.sleep(1.0)
            now = (depth_writer.count, colour_writer.count)
            print(f"{time.monotonic() - start:4.0f} s  depth {now[0] - before[0]:2d}/s  "
                  f"colour {now[1] - before[1]:2d}/s", flush=True)
            if depth_writer.count == 0 and time.monotonic() - start > 5:
                print("No depth yet: is depth_node.py running with these ROS settings?", flush=True)
            before = now
    except KeyboardInterrupt:
        print("Stopped early.", flush=True)
    finally:
        if colour:
            colour.running = False
            colour.join(timeout=3)
        depth.close()
        if low is not None:
            low.close()
        depth_writer.close()
        colour_writer.close()
        meta.update({"ended": utc(), "seconds": round(time.monotonic() - start, 3),
                     "counts": {"depth": depth_writer.count, "colour": colour_writer.count,
                                "colour_failed": colour.failed if colour else None,
                                "lowstate_kept": low.kept if low else None},
                     "depth_size": list(getattr(depth, "size", None) or size)})
        (folder / "meta.json").write_text(json.dumps(meta, indent=2) + "\n")
        print(f"Wrote {folder}: {meta['counts']}.", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
