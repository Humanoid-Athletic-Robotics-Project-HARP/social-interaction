"""HARP's policy node: him-sim2real-management's shared walker client, steered by
/cmd_vel, with the robot's state republished for Nav2 (harp/state.py).

The client (/example/client.py, copied in at build time) reads /safety/policy_state,
arms the safety checker and answers every state with the NVIDIA walker's joint
command. This replaces its fixed velocity with the latest /cmd_vel. A velocity
schedule (HIM's --command FILE) still wins over /cmd_vel, as in HIM's client.

Events go out as JSON on /harp/events (std_msgs/String, transient local):
{"source": ..., "event": ..., ...}: control_ready once HIM hands control to our
policy.
"""

import json
import math
import sys
import time

import numpy as np

sys.path.insert(0, "/example")
sys.path.insert(0, "/harp")
import client  # noqa: E402
from geometry_msgs.msg import Twist  # noqa: E402
from rclpy.qos import DurabilityPolicy, QoSProfile  # noqa: E402
from std_msgs.msg import String  # noqa: E402

from harp.state import StatePublisher  # noqa: E402

CMD_LIMITS = np.array([0.6, 0.4, 1.0])  # vx, vy (m/s), yaw rate (rad/s)
CMD_TIMEOUT = 0.5                       # s of /cmd_vel silence before the walker stands
MODEL = "/harp/model/g1_robot.xml"      # him-sim2real-management's G1, for leg kinematics


class Timing:
    """Worst case each second of how HIM's 0.2 s state deadline is spent, as a "timing" line:
    age (s from HIM receiving the state to it reaching us), reply (our joint command),
    extra (republishing the state), gap (between states)."""

    def __init__(self):
        self.start, self.last, self.n = None, None, 0
        self.worst = dict(age=0.0, reply=0.0, extra=0.0, gap=0.0)

    def add(self, state, arrived, replied, done):
        if state is None:
            return
        self.start = arrived if self.start is None else self.start
        sample = dict(age=arrived - state.get("received_at_secs", arrived), reply=replied - arrived,
                      extra=done - replied, gap=0.0 if self.last is None else arrived - self.last)
        self.worst = {k: max(v, sample[k]) for k, v in self.worst.items()}
        self.last, self.n = arrived, self.n + 1
        if done - self.start >= 1.0:
            self.flush(state)

    def flush(self, state):
        if self.n:
            print(json.dumps({"event": "timing", "wall": round(time.time(), 2),
                              "sim_time": (state or {}).get("sim_time"), "n": self.n,
                              **{k: round(v, 3) for k, v in self.worst.items()}}), flush=True)
        self.__init__()


class HarpPolicyNode(client.PolicyNode):
    def __init__(self, policy, mode="sim"):
        super().__init__(policy, mode)
        # Without a task velocity the shared walker walks forward at 0.3 m/s; stand instead.
        # A {"moves": ...} or {"to": ...} task is for harp/moves.py, which drives /cmd_vel.
        if self.task_command is None or isinstance(self.task_command, dict):
            self.task_command = [0.0, 0.0, 0.0]
        self.cmd_time = None
        self.create_subscription(Twist, "/cmd_vel", self.on_cmd_vel, 10)
        self.state_out = StatePublisher(self, MODEL)
        latched = QoSProfile(depth=1000, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.events_pub = self.create_publisher(String, "/harp/events", latched)
        self.control_ready = False
        self.timing = Timing()

    def event(self, source, name, **fields):
        text = json.dumps({"source": source, "event": name, **fields})
        self.events_pub.publish(String(data=text))
        print(text, flush=True)

    def on_cmd_vel(self, msg):
        cmd = [msg.linear.x, msg.linear.y, msg.angular.z]
        if not all(math.isfinite(v) for v in cmd):
            # Stand at once rather than keep the last command until the timeout.
            self.task_command, self.cmd_time = [0.0, 0.0, 0.0], None
            print("Non-finite /cmd_vel: standing.", file=sys.stderr, flush=True)
            return
        self.task_command = np.clip(cmd, -CMD_LIMITS, CMD_LIMITS).tolist()
        self.cmd_time = time.monotonic()

    def publish_command(self):
        if self.cmd_time is not None and time.monotonic() - self.cmd_time > CMD_TIMEOUT:
            self.task_command, self.cmd_time = [0.0, 0.0, 0.0], None
        super().publish_command()

    def on_state(self, message):
        arrived = time.monotonic()
        super().on_state(message)    # the joint command goes out first
        replied = time.monotonic()
        if not self.done:
            if self.state.get("control_ready") and not self.control_ready:
                self.control_ready = True     # HIM's startup balance handed over to us
                self.event("policy", "control_ready")
            self.state_out.publish(self.state)
        self.timing.add(self.state, arrived, replied, time.monotonic())

    def on_abort(self, message):
        self.timing.flush(self.state)
        super().on_abort(message)


if __name__ == "__main__":
    client.PolicyNode = HarpPolicyNode
    sys.exit(client.main())
