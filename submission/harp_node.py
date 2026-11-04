"""HARP's policy node: him-sim2real-management's shared walker client, steered by
/cmd_vel, with the robot's state republished for Nav2 (harp/state.py).

The client (/example/client.py, copied in at build time) reads /safety/policy_state,
arms the safety checker and answers every state with the NVIDIA walker's joint
command. This replaces its fixed velocity with the latest /cmd_vel. A velocity
schedule (HIM's --command FILE) still wins over /cmd_vel, as in HIM's client.

A {"wave": {}} task stands and waves the right arm (harp/wave.py): WavingPolicy
replaces only the walker's right-arm targets, so GR00T keeps the legs and waist.

Events (harp/events.py): control_ready once HIM hands control to our policy, and a
"timing" line each second; in a wave task wave_start, wave_phase and wave_done.
"""

import math
import sys
import time

import numpy as np

sys.path.insert(0, "/example")
sys.path.insert(0, "/harp")
import client  # noqa: E402
from geometry_msgs.msg import Twist  # noqa: E402

from harp import events, wave  # noqa: E402
from harp.state import StatePublisher  # noqa: E402

CMD_LIMITS = np.array([0.6, 0.4, 1.0])  # vx, vy (m/s), yaw rate (rad/s)
CMD_TIMEOUT = 0.5                       # s of /cmd_vel silence before the walker stands
MODEL = "/harp/model/g1_robot.xml"      # him-sim2real-management's G1, for leg kinematics
LATE = 0.04                             # s: a state this old gets its own late_state line


def clipped(cmd):
    """cmd ([vx, vy, yaw rate]) within CMD_LIMITS, or None unless it is three finite numbers."""
    try:
        cmd = [float(v) for v in cmd]
    except (TypeError, ValueError):
        return None
    if len(cmd) != 3 or not all(math.isfinite(v) for v in cmd):
        return None
    return np.clip(cmd, -CMD_LIMITS, CMD_LIMITS).tolist()


def wave_task(task):
    """True for a {"wave": ...} task."""
    return isinstance(task, dict) and "wave" in task


class WavingPolicy:
    """HIM's walker with the right arm's targets and velocities taken from harp/wave.py: the
    wave starts wave.SETTLE_S after control_ready, from the arm's measured pose, and runs once.
    Its clock is the state's (sim_time in the sim, received_at_secs on the robot)."""

    def __init__(self, policy, emit):
        self.policy, self.emit = policy, emit
        self.ready_at, self.wave, self.phase, self.done = None, None, "raise", False

    def command(self, state):
        frame = self.policy.command(state)
        if self.done or not state.get("control_ready"):
            return frame
        now, sim_time = state.get("sim_time", state["received_at_secs"]), state.get("sim_time")
        if self.ready_at is None:
            self.ready_at = now
        if self.wave is None:
            if now < self.ready_at + wave.SETTLE_S:
                return frame
            pose = [float(v) for v in state["q"][wave.RIGHT_ARM]]
            self.wave = wave.Wave(now, pose)
            self.emit("wave_start", sim_time, pose=pose)
        q, dq, phase = self.wave.at(now)
        if phase == "done":
            self.done = True
            self.emit("wave_done", sim_time)
            return frame
        if phase != self.phase:
            self.phase = phase
            self.emit("wave_phase", sim_time, phase=phase)
        frame["q"][wave.RIGHT_ARM] = [float(v) for v in q]
        frame["dq"][wave.RIGHT_ARM] = [float(v) for v in dq]
        return frame


class Timing:
    """Worst case each second of how HIM's 0.2 s state deadline is spent, as a "timing" line:
    age (s from HIM receiving the state to it reaching us), reply (our joint command),
    extra (republishing the state), gap (between states); and a "late_state" line for each
    state LATE s or more old (age, gap, and missed: states before it that never came)."""

    def __init__(self):
        self.last, self.last_id = None, None
        self.reset()

    def reset(self):
        self.start, self.n = None, 0
        self.worst = dict(age=0.0, reply=0.0, extra=0.0, gap=0.0)

    def add(self, state, arrived, replied, done):
        if state is None:
            return
        self.start = arrived if self.start is None else self.start
        sample = dict(age=arrived - state.get("received_at_secs", arrived), reply=replied - arrived,
                      extra=done - replied, gap=0.0 if self.last is None else arrived - self.last)
        self.worst = {k: max(v, sample[k]) for k, v in self.worst.items()}
        oid = state.get("observation_id")
        missed = oid - self.last_id - 1 if isinstance(oid, int) and self.last_id is not None else None
        if sample["age"] >= LATE:
            events.emit("policy", "late_state", state.get("sim_time"), age=round(sample["age"], 3),
                        gap=round(sample["gap"], 3), missed=missed)
        self.last, self.last_id, self.n = arrived, oid, self.n + 1
        if done - self.start >= 1.0:
            self.flush(state)

    def flush(self, state):
        if self.n:
            events.emit("policy", "timing", (state or {}).get("sim_time"), n=self.n,
                        **{k: round(v, 3) for k, v in self.worst.items()})
        self.reset()


class HarpPolicyNode(client.PolicyNode):
    def __init__(self, policy, mode="sim"):
        super().__init__(policy, mode)
        waving = wave_task(self.task_command)
        # Without a task velocity the shared walker walks forward at 0.3 m/s; stand instead.
        # A {"moves": ...} or {"to": ...} task is for harp/moves.py, which drives /cmd_vel.
        if self.task_command is None or isinstance(self.task_command, dict):
            self.task_command = [0.0, 0.0, 0.0]
        else:
            # A velocity task ([vx, vy, yaw rate]) gets the /cmd_vel limits; a malformed one stands.
            self.task_command = clipped(self.task_command) or [0.0, 0.0, 0.0]
        self.cmd_time = None
        self.create_subscription(Twist, "/cmd_vel", self.on_cmd_vel, 10)
        self.state_out = StatePublisher(self, MODEL)
        self.events = events.Events(self, "policy")
        if waving:
            self.policy = WavingPolicy(self.policy, self.events)
        self.control_ready = False
        self.timing = Timing()

    def on_cmd_vel(self, msg):
        cmd = clipped([msg.linear.x, msg.linear.y, msg.angular.z])
        if cmd is None:
            # Stand at once rather than keep the last command until the timeout.
            self.task_command, self.cmd_time = [0.0, 0.0, 0.0], None
            print("Non-finite /cmd_vel: standing.", file=sys.stderr, flush=True)
            return
        self.task_command = cmd
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
                self.events("control_ready", self.state.get("sim_time"))
            self.state_out.publish(self.state)
        self.timing.add(self.state, arrived, replied, time.monotonic())

    def on_abort(self, message):
        self.timing.flush(self.state)
        super().on_abort(message)


if __name__ == "__main__":
    client.PolicyNode = HarpPolicyNode
    sys.exit(client.main())
