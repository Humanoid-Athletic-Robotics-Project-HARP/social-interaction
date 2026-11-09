"""The G1's health for the dashboard, read-only: battery charge, each motor's casing and
winding temperature, and HARP's harness state, as JSON on port 28735 (/status).

Listens only: rt/lowstate and rt/lf/bmsstate on Unitree's DDS (domain 0, eth0), and the
harness's /safety/hardware_status on its ROS 2 network (domain 97, localhost, Fast DDS).
It publishes nothing and switches no service. The limits are the ones the harness stops
the robot at (its robot_safety/joint_checks.py).

  source /opt/ros/foxy/setup.bash && export LD_LIBRARY_PATH=$HOME/cyclonedds_ws/install/cyclonedds/lib:$LD_LIBRARY_PATH && python3 status_server.py
"""
import argparse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import os
import sys
import threading
import time

PORT = 28735
# The G1's 29 joints in motor_state order (submission/joint_profile.json).
JOINTS = ("left_hip_pitch", "left_hip_roll", "left_hip_yaw", "left_knee", "left_ankle_pitch",
          "left_ankle_roll", "right_hip_pitch", "right_hip_roll", "right_hip_yaw", "right_knee",
          "right_ankle_pitch", "right_ankle_roll", "waist_yaw", "waist_roll", "waist_pitch",
          "left_shoulder_pitch", "left_shoulder_roll", "left_shoulder_yaw", "left_elbow",
          "left_wrist_roll", "left_wrist_pitch", "left_wrist_yaw", "right_shoulder_pitch",
          "right_shoulder_roll", "right_shoulder_yaw", "right_elbow", "right_wrist_roll",
          "right_wrist_pitch", "right_wrist_yaw")
LIMITS = {"casing_c": 85.0, "winding_c": 120.0, "battery_percent": 10.0}   # the harness stops here
# What the dashboard shows of the harness's status (it publishes more).
HARNESS = ("startup_phase", "startup_stable", "handoff_ready", "control_ready", "hardware_output_enabled",
           "armed", "handover_complete", "motion_released", "policy_launched", "arm_rejection_reason",
           "abort_reason", "reported_fsm_id", "pose_hold", "walk_stop", "limit_mode", "startup_policy")


class Latest:
    """The newest message of each kind, kept by the readers' threads, summed up on request."""

    def __init__(self):
        self.lock = threading.Lock()
        self.low = self.bms = self.harness = None
        self.low_at = self.bms_at = self.harness_at = None
        self.low_count, self.rate = 0, (None, 0)   # (monotonic s, count) a second ago
        self.harness_error = None

    def on_low(self, message):   # about 1 kHz: keep it to a reference and a count
        with self.lock:
            self.low, self.low_at = message, time.monotonic()
            self.low_count += 1

    def on_bms(self, message):
        with self.lock:
            self.bms, self.bms_at = message, time.monotonic()

    def on_harness(self, text):
        try:
            value = json.loads(text)
        except ValueError:
            return
        if isinstance(value, dict):
            with self.lock:
                self.harness, self.harness_at = value, time.monotonic()

    def status(self):
        now = time.monotonic()
        with self.lock:
            low, bms, harness = self.low, self.bms, self.harness
            ages = [None if t is None else round(now - t, 2) for t in (self.low_at, self.bms_at, self.harness_at)]
            since, count = self.rate
            hz = None if since is None or now - since <= 0 else round((self.low_count - count) / (now - since))
            if since is None or now - since >= 1.0:
                self.rate = (now, self.low_count)
            error = self.harness_error
        return summary(low, bms, harness, ages, hz, error)


def summary(low, bms, harness, ages, hz, harness_error=None):
    """The /status JSON from the newest messages (any may be None) and their ages in s."""
    motors = []
    for name, motor in zip(JOINTS, getattr(low, "motor_state", None) or []):
        temperature = list(getattr(motor, "temperature", None) or [])
        motors.append({"name": name, "casing_c": round(float(temperature[0]), 1) if len(temperature) > 0 else None,
                       "winding_c": round(float(temperature[1]), 1) if len(temperature) > 1 else None})
    soc = getattr(bms, "soc", None)
    return {"limits": LIMITS,
            "lowstate": {"age_s": ages[0], "hz": hz, "mode_machine": getattr(low, "mode_machine", None)},
            "motors": motors,
            "battery": {"age_s": ages[1], "percent": None if soc is None else float(soc)},
            "harness": {"age_s": ages[2], "error": harness_error,
                        **({k: harness.get(k) for k in HARNESS} if harness else {})}}


def read_robot(latest, interface, domain):
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import BmsState_, LowState_
    ChannelFactoryInitialize(domain, interface)
    low = ChannelSubscriber("rt/lowstate", LowState_)
    low.Init(latest.on_low, 0)
    bms = ChannelSubscriber("rt/lf/bmsstate", BmsState_)
    bms.Init(latest.on_bms, 0)
    return [low, bms]


def read_harness(latest, stop):
    """/safety/hardware_status on the harness's ROS 2 network, in its own thread."""
    # The harness's network (context/robot-services.md); set before rclpy starts.
    os.environ.setdefault("ROS_DOMAIN_ID", "97")
    os.environ.setdefault("ROS_LOCALHOST_ONLY", "1")
    os.environ.setdefault("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp")
    try:
        import rclpy
        from std_msgs.msg import String
        rclpy.init(args=[])
        node = rclpy.create_node("harp_status_reader")
        node.create_subscription(String, "/safety/hardware_status", lambda m: latest.on_harness(m.data), 1)
        while not stop.is_set():
            rclpy.spin_once(node, timeout_sec=0.2)
        node.destroy_node()
        rclpy.shutdown()
    except Exception as error:   # ROS 2 not sourced, or rclpy failed: the robot's readings still come
        with latest.lock:
            latest.harness_error = f"{type(error).__name__}: {error}"


def handler(latest):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def do_GET(self):
            if self.path != "/status":
                self.send_error(404)
                return
            body = json.dumps(latest.status()).encode()
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)
    return Handler


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--port", type=int, default=PORT)
    parser.add_argument("--interface", default="eth0")
    parser.add_argument("--domain", type=int, default=0)
    parser.add_argument("--no-harness", action="store_true", help="skip HARP's harness status")
    args = parser.parse_args()
    os.nice(10)   # below HARP's harness
    latest, stop = Latest(), threading.Event()
    readers = read_robot(latest, args.interface, args.domain)
    if args.no_harness:
        latest.harness_error = "not read (--no-harness)"
    else:
        threading.Thread(target=read_harness, args=(latest, stop), daemon=True).start()
    server = ThreadingHTTPServer(("0.0.0.0", args.port), handler(latest))
    print(f"Robot status at http://0.0.0.0:{args.port}/status (Ctrl-C to stop)", flush=True)
    try:
        server.serve_forever(poll_interval=0.5)
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        server.server_close()
        for reader in readers:   # the SDK's readers first, or the interpreter hangs on exit
            reader.Close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
