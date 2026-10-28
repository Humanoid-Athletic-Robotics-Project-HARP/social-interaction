"""HARP's policy node: him-sim2real-management's shared walker client, steered by
/cmd_vel, with the robot's state republished for Nav2 (harp/state.py).

The client (/example/client.py, copied in at build time) reads /safety/policy_state,
arms the safety checker and answers every state with the NVIDIA walker's joint
command. This replaces its fixed velocity with the latest /cmd_vel.

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


class HarpPolicyNode(client.PolicyNode):
    def __init__(self, policy, mode="sim"):
        super().__init__(policy, mode)
        # Without a task velocity the shared walker walks forward at 0.3 m/s; stand instead.
        # A {"to": ...} task is a target for harp/move_to_point.py, which drives /cmd_vel.
        if self.task_command is None or isinstance(self.task_command, dict):
            self.task_command = [0.0, 0.0, 0.0]
        self.cmd_time = None
        self.create_subscription(Twist, "/cmd_vel", self.on_cmd_vel, 10)
        self.state_out = StatePublisher(self, MODEL)
        latched = QoSProfile(depth=1000, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.events_pub = self.create_publisher(String, "/harp/events", latched)
        self.control_ready = False

    def event(self, source, name, **fields):
        text = json.dumps({"source": source, "event": name, **fields})
        self.events_pub.publish(String(data=text))
        print(text, flush=True)

    def on_cmd_vel(self, msg):
        cmd = [msg.linear.x, msg.linear.y, msg.angular.z]
        if not all(math.isfinite(v) for v in cmd):
            return
        self.task_command = np.clip(cmd, -CMD_LIMITS, CMD_LIMITS).tolist()
        self.cmd_time = time.monotonic()

    def publish_command(self):
        if self.cmd_time is not None and time.monotonic() - self.cmd_time > CMD_TIMEOUT:
            self.task_command, self.cmd_time = [0.0, 0.0, 0.0], None
        super().publish_command()

    def on_state(self, message):
        super().on_state(message)    # the joint command goes out first
        if not self.done:
            if self.state.get("control_ready") and not self.control_ready:
                self.control_ready = True     # HIM's startup balance handed over to us
                self.event("policy", "control_ready")
            self.state_out.publish(self.state)


if __name__ == "__main__":
    client.PolicyNode = HarpPolicyNode
    sys.exit(client.main())
