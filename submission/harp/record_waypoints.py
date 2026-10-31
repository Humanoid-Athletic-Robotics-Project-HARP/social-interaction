"""Capture /odom poses on demand as a route for course.py --route.

Run in the running submission's ROS environment, or use record_waypoints.sh
from the host. Enter/r records, u undoes, s saves and exits, q discards.
This subscribes only: it never starts a move or publishes a command.
Coordinates belong to this odom origin; start a new recording if it resets.
"""

import argparse
import math
import os
import queue
import sys
import tempfile
import threading
import time
from pathlib import Path

import yaml

from harp.route import Route, route_from_data

MAX_AGE = 0.5       # s: both the sample stamp and local receipt must be fresh
FUTURE_TOLERANCE = 0.1  # s allowed for clock jitter


class Recorder:
    def __init__(self):
        self.waypoints = []
        self.pose = None
        self.received_at = None
        self.stamp = None
        self.reset = False
        self.error = "Waiting for valid /odom."

    def update(self, msg, received_at, ros_now_ns):
        """Accept one odometry sample, using the node's ROS clock for stamp age."""
        self.pose = None   # an invalid new sample must not leave an older pose usable
        if self.reset:
            return
        stamp = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
        if self.stamp is not None and stamp < self.stamp:
            self.reset = True
            self.error = "Odometry clock moved backwards; restart the recording."
            return
        age = (ros_now_ns - stamp) / 1e9
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        values = (p.x, p.y, p.z, q.x, q.y, q.z, q.w)
        if msg.header.frame_id != "odom" or msg.child_frame_id != "base_link":
            self.error = "Expected odom -> base_link."
        elif not all(math.isfinite(v) for v in values):
            self.error = "Odometry contains a non-finite position or orientation."
        elif stamp <= 0 or not -FUTURE_TOLERANCE <= age <= MAX_AGE:
            self.error = "Odometry stamp is stale or outside the node's ROS clock."
        else:
            norm = math.hypot(q.x, q.y, q.z, q.w)
            if not math.isfinite(norm) or norm < 1e-6:
                self.error = "Odometry has an invalid quaternion."
                return
            x, y, z, w = (v / norm for v in (q.x, q.y, q.z, q.w))
            yaw = math.degrees(math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))
            self.pose = (p.x, p.y, yaw)
            self.received_at, self.stamp = received_at, stamp
            self.error = ""

    def record(self, now):
        if self.pose is None:
            raise ValueError(self.error)
        if now - self.received_at > MAX_AGE:
            raise ValueError("No fresh odometry; wait for /odom before recording.")
        point = [round(v, 6) for v in self.pose]
        self.waypoints.append(point)
        return point

    def undo(self):
        if not self.waypoints:
            raise ValueError("No waypoint to undo.")
        return self.waypoints.pop()

    def save(self, output):
        if self.reset:
            raise ValueError(self.error)
        data = route_from_data(Route(self.waypoints).data()).data()
        output = Path(output)
        # Same directory: linking is atomic and refuses to overwrite an existing file,
        # including a symlink. A failed save leaves no partial destination.
        temporary = None
        try:
            with tempfile.NamedTemporaryFile(mode="w", encoding="utf-8", dir=output.parent,
                                             prefix=".harp-route-", delete=False) as stream:
                temporary = stream.name
                stream.write("# Poses in the recording session's odom frame (meters, degrees).\n")
                yaml.safe_dump(data, stream, sort_keys=False)
                stream.flush()
                os.fsync(stream.fileno())
            os.link(temporary, output)
        finally:
            if temporary is not None:
                os.unlink(temporary)


def read_commands(commands):
    # A daemon input thread lets ROS keep receiving while the terminal waits for a
    # whole line. Queue commands also work with piped input on macOS and Linux.
    for line in sys.stdin:
        commands.put(line.strip().lower())
    commands.put(None)


def run(node, output, commands):
    import rclpy
    from nav_msgs.msg import Odometry

    recorder = Recorder()
    ready = False

    def on_odom(msg):
        nonlocal ready
        recorder.update(msg, time.monotonic(), node.get_clock().now().nanoseconds)
        if recorder.pose is not None and not ready:
            print("Odometry ready; choose a position and press Enter/r.", flush=True)
            ready = True

    node.create_subscription(Odometry, "/odom", on_odom, 10)
    print("Enter/r: record | u: undo | s: save and exit | q: discard and exit", flush=True)
    print("Use poses from this odom session; recording does not drive the robot.", flush=True)
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)
        try:
            command = commands.get_nowait()
        except queue.Empty:
            continue
        try:
            if command in ("", "r", "record"):
                print(f"Recorded {len(recorder.waypoints) + 1}: {recorder.record(time.monotonic())}",
                      flush=True)
            elif command in ("u", "undo"):
                print(f"Removed: {recorder.undo()}", flush=True)
            elif command in ("s", "save"):
                recorder.save(output)
                print(f"Saved {len(recorder.waypoints)} waypoints to {output}", flush=True)
                return 0
            elif command in (None, "q", "quit"):
                print("Recording discarded; no file written.", flush=True)
                return 1
            else:
                print("Use Enter/r, u, s, or q.", flush=True)
        except (ValueError, OSError) as error:
            print(f"Cannot complete command: {error}", flush=True)
    return 1


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True, type=Path, help="new route YAML file")
    args = parser.parse_args(argv)
    if args.output.exists() or args.output.is_symlink():
        parser.error(f"output already exists: {args.output}")
    if not args.output.parent.is_dir():
        parser.error(f"output directory does not exist: {args.output.parent}")

    import rclpy

    rclpy.init(args=[])
    node = rclpy.create_node("waypoint_recorder")
    commands = queue.Queue()
    threading.Thread(target=read_commands, args=(commands,), daemon=True).start()
    try:
        return run(node, args.output, commands)
    except KeyboardInterrupt:
        return 130
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
