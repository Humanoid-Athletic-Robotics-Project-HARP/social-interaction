"""Record one run into its own folder under runs/ (gitignored): what the run
was (run.yaml), what happened (events.csv), the standard time series, the
light Nav2 data, and, on request, more topics in a rosbag. Our programs
publish; this node writes. What it records is in config/logging.yaml.

  Subscribes  /harp/events  JSON events from run_sim, walker_node, course_goal
              /sim/info     the sim's world, boxes and course goal (latched)
              the topics in logging.yaml: /odom, /cmd_vel, /sim/qpos,
              /behavior_tree_log, /plan and lidar statistics always; extras
              (nav, lidar, joints, ...) and replay only when asked for

course (the course test command) starts it from course.launch.py. On its own,
for any session (driving by hand, move_distance, the real robot):
  ros2 run harp_ros run_recorder --ros-args -p tag:=teleop -p note:="first try"
  ros2 run harp_ros run_recorder --ros-args -p kind:=robot -p tether:=gantry -p floor_surface:=carpet
  ros2 run harp_ros run_recorder --ros-args -p extras:=nav,lidar -p extras_reason:="why"
Ctrl-C ends the run and writes summary.json. Programs without ROS
(move_distance, walk) find the run through runs/ACTIVE (harp_control.runlog).

Replay recording (replay:=true) is only for when a person explicitly asks for
it, with replay_reason; see logging.yaml and context/run-logging.md.
"""

import csv
import datetime
import getpass
import json
import os
import platform
import shutil
import socket
import subprocess
import time
from pathlib import Path

import numpy as np
import rclpy
import rclpy.executors
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.serialization import deserialize_message, serialize_message
from rosidl_runtime_py.utilities import get_message
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import String

from harp_control import runlog
from harp_ros import run_summary

SCHEMA = 2
DIFF_LIMIT = 1_000_000    # bytes of uncommitted diff kept in git.diff
CONDITIONS = ("floor_surface", "tether", "battery_percent", "people_nearby", "lighting")


def utc(t=None):
    """A wall-clock time as UTC text, 2026-10-26T21:30:00Z. Runs use UTC only."""
    when = datetime.datetime.fromtimestamp(t, datetime.timezone.utc) if t is not None \
        else datetime.datetime.now(datetime.timezone.utc)
    return when.strftime("%Y-%m-%dT%H:%M:%SZ")


def run_name(kind, world, tag, when=None):
    """runs/ folder name: UTC date and time, sim or robot, world, tag."""
    when = when or datetime.datetime.now(datetime.timezone.utc)
    parts = [when.astimezone(datetime.timezone.utc).strftime("%Y-%m-%d_%H-%M-%SZ"), kind, world or "noworld"]
    if tag:
        parts.append("".join(c if c.isalnum() or c in "-." else "-" for c in tag))
    return "_".join(parts)


def load_config():
    path = Path(get_package_share_directory("harp_ros")) / "config" / "logging.yaml"
    return path, yaml.safe_load(path.read_text())


def replay_dir_for(run_dir, name, override=""):
    """Where a replay recording goes: replay_dir or $HARP_REPLAY_DIR, else the run folder."""
    base = override or os.environ.get("HARP_REPLAY_DIR", "")
    return Path(base) / name / "replay" if base else Path(run_dir) / "replay"


def git_info(run_dir):
    """Commit, branch and uncommitted changes of the repo this code runs from."""
    repo = Path(__file__).resolve().parents[4]

    def git(*args):
        try:
            return subprocess.run(["git", "-C", str(repo), *args], capture_output=True,
                                  text=True, timeout=10).stdout
        except (OSError, subprocess.TimeoutExpired):
            return ""
    status = git("status", "--porcelain")
    info = {"commit": git("rev-parse", "HEAD").strip() or None,
            "branch": git("rev-parse", "--abbrev-ref", "HEAD").strip() or None,
            "dirty": bool(status.strip()), "user": git("config", "user.name").strip() or None}
    if info["dirty"]:
        diff = git("diff", "HEAD") + "".join(
            f"\n# untracked: {line[3:]}" for line in status.splitlines() if line.startswith("??"))
        (run_dir / "git.diff").write_text(diff[:DIFF_LIMIT])
        info["diff"] = "git.diff" + (" (cut at 1 MB)" if len(diff) > DIFF_LIMIT else "")
    return info


class CsvStream:
    def __init__(self, path, fields):
        self.file = open(path, "w", newline="")
        self.writer = csv.writer(self.file)
        self.writer.writerow(fields)

    def write(self, row, flush=False):
        self.writer.writerow(row)
        if flush:
            self.file.flush()

    def close(self):
        self.file.close()


class BagStream:
    """Some topics into one rosbag (MCAP, zstd), each thinned to a rate, up to a size cap.
    Topics are subscribed to once they exist (Nav2 starts after the walker)."""

    def __init__(self, node, name, path, streams, cap_bytes, transform=None):
        import rosbag2_py
        self.node, self.name, self.cap = node, name, cap_bytes
        self.transform = transform or {}      # topic -> function(raw bytes) -> raw bytes
        Path(path).parent.mkdir(parents=True, exist_ok=True)
        self.writer = rosbag2_py.SequentialWriter()
        self.writer.open(rosbag2_py.StorageOptions(uri=str(path), storage_id="mcap",
                                                   storage_preset_profile="zstd_fast"),
                         rosbag2_py.ConverterOptions("cdr", "cdr"))
        self.pending = [dict(s) for s in streams]
        self.topics = 0
        self.bytes = 0
        self.last = {}
        node.create_timer(1.0, self.subscribe)

    def subscribe(self):
        import rosbag2_py
        if self.writer is None:
            return
        types = dict(self.node.get_topic_names_and_types())
        for stream in list(self.pending):
            topic = stream["topic"]
            if topic not in types:
                continue
            self.pending.remove(stream)
            publishers = self.node.get_publishers_info_by_topic(topic)
            qos = QoSProfile(depth=50)
            if any(i.qos_profile.durability == DurabilityPolicy.TRANSIENT_LOCAL for i in publishers):
                qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
            if any(i.qos_profile.reliability == ReliabilityPolicy.BEST_EFFORT for i in publishers):
                qos.reliability = ReliabilityPolicy.BEST_EFFORT
            type_name = types[topic][0]
            self.writer.create_topic(rosbag2_py.TopicMetadata(
                id=self.topics, name=topic, type=type_name, serialization_format="cdr"))
            self.topics += 1
            rate = stream.get("rate") or (1.0 / stream["period_s"] if stream.get("period_s") else None)
            self.node.create_subscription(
                get_message(type_name), topic,
                lambda data, topic=topic, rate=rate: self.on_message(topic, rate, data), qos, raw=True)

    def on_message(self, topic, rate, data):
        if self.writer is None:
            return
        now = time.time()
        if rate and now - self.last.get(topic, 0.0) < 0.95 / rate:
            return
        self.last[topic] = now
        if topic in self.transform:
            data = self.transform[topic](data)
        self.writer.write(topic, data, time.time_ns())
        self.bytes += len(data)
        if self.bytes > self.cap:
            self.stop(f"{self.name} reached its cap ({self.cap / 1e6:.0f} MB)")

    def stop(self, reason):
        if self.writer is not None:
            self.node.record_event(f"{self.name}_stopped", reason=reason, mb=round(self.bytes / 1e6, 1))
            self.node.get_logger().warn(f"Stopped recording {self.name}: {reason}")
            self.writer = None    # deleting the writer closes the bag


def xyz_only(data):
    """A PointCloud2 cut down to x, y, z (float32), for smaller replay recordings."""
    cloud = deserialize_message(data, PointCloud2)
    xyz = point_cloud2.read_points_numpy(cloud, field_names=("x", "y", "z"))
    return serialize_message(point_cloud2.create_cloud_xyz32(cloud.header, xyz))


class RunRecorder(Node):
    def __init__(self):
        super().__init__("run_recorder")
        defaults = {"run_dir": "", "tag": "", "note": "", "world": "", "kind": "sim",
                    "extras": "", "extras_reason": "", "command": "", "summarize": True,
                    "replay": False, "replay_reason": "", "replay_dir": "",
                    "lidar_fields": "", "lidar_rate": 0.0,
                    "end_position_measured": "", "video": "", "nav2_params_file": "",
                    **{c: "" for c in CONDITIONS}}
        for name, default in defaults.items():
            self.declare_parameter(name, default)
        p = {name: self.get_parameter(name).value for name in defaults}
        self.summarize = p["summarize"]
        config_path, self.config = load_config()
        runs = runlog.runs_dir()
        runs.mkdir(parents=True, exist_ok=True)

        extras = [e for e in p["extras"].replace(" ", "").split(",") if e]
        unknown = [e for e in extras if e not in self.config["extras"]]
        if unknown:
            raise SystemExit(f"Unknown extras {unknown}; logging.yaml has {list(self.config['extras'])}")
        if extras and not p["extras_reason"]:
            self.get_logger().warn("Extras without extras_reason: say why in run.yaml next time.")
        replay = self.config["replay"]
        if p["replay"] and not p["replay_reason"]:
            raise SystemExit("Replay recording needs replay_reason: it is only for when a person "
                             "asks for it, and the reason goes in run.yaml.")

        run_summary.prune(runs, self.config["retention"], replay)
        self.dir = Path(p["run_dir"]) if p["run_dir"] else runs / run_name(p["kind"], p["world"], p["tag"])
        self.dir.mkdir(parents=True, exist_ok=True)

        replay_meta = None
        if p["replay"]:
            replay_path = replay_dir_for(self.dir, self.dir.name, p["replay_dir"])
            replay_path.parent.mkdir(parents=True, exist_ok=True)
            free = shutil.disk_usage(replay_path.parent).free / 1e9
            if free - replay["run_cap_gb"] < replay["min_free_gb"]:
                raise SystemExit(f"Not recording replay: {free:.1f} GB free on that disk, and a "
                                 f"replay run may take {replay['run_cap_gb']} GB, which would leave "
                                 f"under {replay['min_free_gb']} GB. Free space or use replay_dir.")
            replay_meta = {"reason": p["replay_reason"], "path": str(replay_path),
                           "lidar_fields": p["lidar_fields"] or replay["lidar_fields"],
                           "lidar_rate": p["lidar_rate"] or replay["lidar_rate"],
                           "free_gb_at_start": round(free, 1)}

        if not p["run_dir"]:
            # For programs without ROS started by hand. course gives its programs the run
            # folder (HARP_RUN_DIR) instead, so runs at the same time don't share this.
            (runs / runlog.ACTIVE).write_text(f"{self.dir.name} {os.getpid()}\n")
        self.started = time.time()
        shutil.copy(config_path, self.dir / "logging.yaml")
        # The Nav2 parameters this run used: ours, or another file (replay.launch.py).
        nav2 = p["nav2_params_file"] or Path(get_package_share_directory("harp_ros")) / "config" / "nav2_params.yaml"
        shutil.copy(nav2, self.dir / "nav2_params.yaml")
        measured = [float(v) for v in p["end_position_measured"].split()]
        self.meta = {
            "schema": SCHEMA, "run": self.dir.name, "kind": p["kind"],
            "started": utc(self.started),
            "ended": None, "tag": p["tag"] or None, "note": p["note"] or None,
            "world": p["world"] or None, "pinned": False,
            "extras": extras, "extras_reason": p["extras_reason"] or None,
            "replay": replay_meta,
            "command": p["command"] or None,
            "who": {"user": getpass.getuser()},
            "machine": {"host": socket.gethostname(), "os": platform.platform(),
                        "python": platform.python_version()},
            "git": git_info(self.dir),
            "config": {"nav2_params": "nav2_params.yaml", "logging": "logging.yaml"},
            "sim": None,
            # Conditions of the run: from the command line, or later with `runs set`.
            "conditions": {c: p[c] or None for c in CONDITIONS},
            # Robot runs: where the robot really ended (x y in odom, tape measure) and a video.
            "manual": {"end_position_measured": measured or None, "video": p["video"] or None},
            "result": None,
        }
        self.meta["who"]["git_user"] = self.meta["git"].pop("user")
        self.write_meta()

        latched = QoSProfile(depth=1000, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.events = CsvStream(self.dir / "events.csv", runlog.EVENT_FIELDS)
        self.create_subscription(String, "/harp/events", self.on_event, latched)
        self.create_subscription(String, "/sim/info", self.on_sim_info, QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))

        self.last = {}            # topic -> wall time of the last message kept
        self.baseline = self.config["baseline"]
        self.odom = CsvStream(self.dir / self.baseline["odom"]["file"],
                              ["t", "x", "y", "yaw", "vx", "vy", "wz"])
        self.cmd_vel = CsvStream(self.dir / self.baseline["cmd_vel"]["file"], ["t", "vx", "vy", "wz"])
        self.truth = []           # (wall time, sim time, qpos)
        for key, callback in (("odom", self.on_odom), ("cmd_vel", self.on_cmd_vel),
                              ("truth", self.on_truth)):
            stream = self.baseline[key]
            self.create_subscription(get_message(stream["type"]), stream["topic"], callback, 50)

        stats = self.config["lidar_stats"]
        self.lidar_stats = CsvStream(self.dir / stats["file"], ["t", "points", "min_range", "median_range"])
        self.create_subscription(PointCloud2, stats["topic"], self.on_lidar, QoSProfile(
            depth=5, reliability=ReliabilityPolicy.BEST_EFFORT))

        self.bags = [BagStream(self, "standard_bag", self.dir / "standard_bag",
                               self.config["standard_bag"], 50e6)]
        if extras:
            self.bags.append(BagStream(self, "extras", self.dir / "bag",
                                       [t for e in extras for t in self.config["extras"][e]],
                                       self.config["retention"]["run_budget_mb"] * 1e6))
        if replay_meta:
            topics = [dict(t, rate=replay_meta["lidar_rate"]) if t.get("lidar") else t
                      for t in replay["topics"]]
            transform = ({t["topic"]: xyz_only for t in topics if t.get("lidar")}
                         if replay_meta["lidar_fields"] == "xyz" else {})
            self.replay = BagStream(self, "replay", replay_meta["path"], topics,
                                    replay["run_cap_gb"] * 1e9, transform)
            self.bags.append(self.replay)
            self.create_timer(5.0, self.check_disk)
            self.record_event("replay_started", reason=replay_meta["reason"], path=replay_meta["path"])
        self.get_logger().info(f"Recording run {self.dir}")

    def write_meta(self):
        (self.dir / "run.yaml").write_text(yaml.safe_dump(self.meta, sort_keys=False))

    def keep(self, topic, rate):
        """Thin a stream to rate (Hz): True if this message should be kept."""
        now = time.time()
        if rate and now - self.last.get(topic, 0.0) < 0.95 / rate:
            return False
        self.last[topic] = now
        return True

    def check_disk(self):
        free = shutil.disk_usage(Path(self.meta["replay"]["path"]).parent).free / 1e9
        if free < self.config["replay"]["min_free_gb"]:
            self.replay.stop(f"only {free:.1f} GB free on the disk")

    def on_event(self, msg):
        record = json.loads(msg.data)
        # Flushed at once, so events.csv can be watched while the run goes on.
        self.events.write([record["t"], record["source"], record["event"],
                           json.dumps(record.get("details", {}))], flush=True)

    def on_sim_info(self, msg):
        self.meta["sim"] = json.loads(msg.data)
        if not self.meta["world"]:
            self.meta["world"] = self.meta["sim"].get("world")
        # What the sim knows about the conditions, unless they were given.
        c = self.meta["conditions"]
        c["floor_surface"] = c["floor_surface"] or "sim"
        c["tether"] = c["tether"] or "band until released (band_released in events.csv)"
        c["people_nearby"] = c["people_nearby"] or "0"
        self.write_meta()

    def on_odom(self, msg):
        if not self.keep("odom", self.baseline["odom"]["rate"]):
            return
        q = msg.pose.pose.orientation
        yaw = np.arctan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        t = msg.twist.twist
        self.odom.write([time.time(), msg.pose.pose.position.x, msg.pose.pose.position.y, yaw,
                         t.linear.x, t.linear.y, t.angular.z])

    def on_cmd_vel(self, msg):
        if self.keep("cmd_vel", self.baseline["cmd_vel"]["rate"]):
            self.cmd_vel.write([time.time(), msg.linear.x, msg.linear.y, msg.angular.z])

    def on_truth(self, msg):
        if self.keep("truth", self.baseline["truth"]["rate"]):
            self.truth.append((time.time(), msg.data[0], np.array(msg.data[1:])))

    def on_lidar(self, msg):
        """Statistics only: the points themselves are recorded only in extras or replay."""
        if not self.keep("lidar_stats", self.config["lidar_stats"]["rate"]):
            return
        xyz = point_cloud2.read_points_numpy(msg, field_names=("x", "y", "z"))
        r = np.linalg.norm(xyz, axis=1)
        self.lidar_stats.write([time.time(), len(xyz), round(float(r.min()), 3) if len(r) else "",
                                round(float(np.median(r)), 3) if len(r) else ""])

    def record_event(self, name, **details):
        self.events.write([time.time(), "recorder", name, json.dumps(details)], flush=True)

    def finish(self):
        """Close everything; summarize unless course does it after the launch."""
        for bag in self.bags:
            bag.writer = None     # deleting the writer closes the bag
        for stream in (self.odom, self.cmd_vel, self.lidar_stats):
            stream.close()
        self.events.close()
        if self.truth:
            wall, sim_t, qpos = zip(*self.truth)
            np.savez(self.dir / self.baseline["truth"]["file"], t=np.array(wall),
                     sim_t=np.array(sim_t), qpos=np.stack(qpos))
        self.meta["ended"] = utc()
        self.write_meta()
        marker = runlog.runs_dir() / runlog.ACTIVE
        if marker.exists() and marker.read_text().split()[0] == self.dir.name:
            marker.unlink()
        if self.summarize:
            summary = run_summary.summarize(self.dir)
            print(run_summary.one_screen(summary))


def main():
    rclpy.init()
    node = RunRecorder()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    node.finish()
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
