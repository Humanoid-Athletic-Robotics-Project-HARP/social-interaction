"""Walk and balance the 29-joint G1 with NVIDIA's GR00T lower-body policy
(Decoupled WBC), in place of Unitree's built-in walking controller.

Talks to the robot or the MuJoCo sim over the same low-level DDS topics, so
the same code runs on both: reads rt/lowstate (unitree_hg), writes rt/lowcmd.
The policy drives the 15 leg and waist joints at 50 Hz; the arms are held in a
fixed pose. Commands go out at 200 Hz, one per sim step; the motor drivers
(and our sim, harp_sim) recompute the PD torque toward them at every step.

Observation layout, scales, gains, default pose and arm pose follow NVIDIA's
real-robot G1 deployment (GR00T-WholeBodyControl, decoupled_wbc:
g1_gear_wbc_policy.py, g1_gear_wbc.yaml, g1_29dof_gear_wbc.yaml; Apache-2.0,
see LICENSE-GR00T-WholeBodyControl.txt). The ONNX weights in policy/ are
NVIDIA's, under the NVIDIA Open Model License (policy/NOTICE.txt).

Sim (start it first: ros2 run harp_sim run_sim, see README):
  ros2 run harp_control walk
Real robot (untested; hang it, damp it, then debug mode L2+R2 first):
  ros2 run harp_control walk --real --interface eth0

Keys, typed in this terminal: ] start policy, w/s forward/back, a/d left/right, q/e turn left/right,
z zero velocity, 1/2 raise/lower, o stop (damp) and quit.

Every way of stopping goes through Walker.stop: o, Ctrl-C, L2+B on the robot's
remote (the e-stop), a lost rt/lowstate, a fall, or an error. It damps every
joint; on the real robot it then keeps damping until o or Ctrl-C, so the
joints never go uncommanded while someone secures the robot.
"""

import argparse
import math
import select
import sys
import termios
import time
import tty
from collections import deque
from pathlib import Path

import numpy as np
import onnxruntime as ort
from unitree_sdk2py.core.channel import (
    ChannelFactoryInitialize, ChannelPublisher, ChannelSubscriber)
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
from unitree_sdk2py.utils.crc import CRC

from harp_control import remote, runlog

POLICY_DIR = Path(__file__).resolve().parent / "policy"
NUM_MOTORS = 29
NUM_ACTIONS = 15          # 12 leg joints + 3 waist joints, motor order
G1_29DOF_REV_1_0 = 5      # lowstate.mode_machine NVIDIA's config is written for

# Stand pose: legs from NVIDIA's default_angles, arms from loco_upper_body_dof_pos.
STAND_POSE = np.array([
    -0.1, 0.0, 0.0, 0.3, -0.2, 0.0,          # left hip pitch/roll/yaw, knee, ankle pitch/roll
    -0.1, 0.0, 0.0, 0.3, -0.2, 0.0,          # right leg
    0.0, 0.0, 0.0,                           # waist yaw/roll/pitch
    0.0, 0.3, 0.0, 1.0, 0.0, 0.0, 0.0,       # left shoulder pitch/roll/yaw, elbow, wrist roll/pitch/yaw
    0.0, -0.3, 0.0, 1.0, 0.0, 0.0, 0.0,      # right arm
], dtype=np.float32)
# The policy's observation subtracts defaults for the 15 joints it controls only.
OBS_DEFAULTS = np.concatenate([STAND_POSE[:NUM_ACTIONS], np.zeros(NUM_MOTORS - NUM_ACTIONS)])

KP = np.array([150, 150, 150, 200, 40, 40, 150, 150, 150, 200, 40, 40,
               250, 250, 250,
               100, 100, 40, 40, 20, 20, 20, 100, 100, 40, 40, 20, 20, 20], dtype=np.float32)
KD = np.array([2, 2, 2, 4, 2, 2, 2, 2, 2, 4, 2, 2,
               5, 5, 5,
               5, 5, 2, 2, 2, 2, 2, 5, 5, 2, 2, 2, 2, 2], dtype=np.float32)
DAMPING_KD = 8.0          # kd for every joint when stopping (kp = 0)
Q_LOWER = np.array([-2.5307, -0.5236, -2.7576, -0.087267, -0.87267, -0.2618,
                    -2.5307, -2.9671, -2.7576, -0.087267, -0.87267, -0.2618,
                    -2.618, -0.52, -0.52,
                    -3.0892, -1.5882, -2.618, -1.0472, -1.972222, -1.61443, -1.61443,
                    -3.0892, -2.2515, -2.618, -1.0472, -1.972222, -1.61443, -1.61443])
Q_UPPER = np.array([2.8798, 2.9671, 2.7576, 2.8798, 0.5236, 0.2618,
                    2.8798, 0.5236, 2.7576, 2.8798, 0.5236, 0.2618,
                    2.618, 0.52, 0.52,
                    2.6704, 2.2515, 2.618, 2.0944, 1.972222, 1.61443, 1.61443,
                    2.6704, 1.5882, 2.618, 2.0944, 1.972222, 1.61443, 1.61443])

ACTION_SCALE = 0.25
ANG_VEL_SCALE = 0.5
DOF_VEL_SCALE = 0.05
CMD_SCALE = np.array([2.0, 2.0, 0.5], dtype=np.float32)
HISTORY_LEN = 6
OBS_DIM = 86              # 3 cmd + 1 height + 3 rpy + 3 gyro + 3 gravity + 29 q + 29 dq + 15 action
STAND_CMD = 0.05          # |cmd| below this runs the Balance policy instead of Walk
DEFAULT_HEIGHT = 0.74

# Our own limits, kept well inside what the policy was trained on.
MAX_VX, MAX_VY, MAX_YAW_RATE = 0.6, 0.4, 1.0
HEIGHT_RANGE = (0.6, 0.8)
CMD_STEP, HEIGHT_STEP = 0.2, 0.05

CONTROL_DT = 0.02         # policy rate, 50 Hz
PUBLISH_DT = 0.005        # lowcmd rate, 200 Hz
RAMP_TIME = 3.0           # s to move from the current pose to the stand pose
STATE_TIMEOUT_REAL = 0.1  # s without lowstate before damping
STATE_TIMEOUT_SIM = 0.5   # the sim's physics pauses when it stalls (macOS throttles its window), so a gap is harmless
FALL_TILT = math.radians(60.0)  # damp if the pelvis tilts more than this
STOP_TIME = 1.0           # s of damping commands sent when stopping
# Stops that aren't failures: the program exits 0 after these.
CLEAN_STOPS = ("quit", "ctrl_c", "remote_estop")


class Stop(Exception):
    """Raised in the control loop to stop; run() hands the reason to Walker.stop."""

    def __init__(self, reason, message):
        super().__init__(message)
        self.reason = reason


def require_finite(name, *values):
    if any(not np.isfinite(value).all() for value in values):
        raise ValueError(f"{name} must contain only finite values")


def stop_message(message, *, error=False):
    """A closed terminal must not interrupt damping or the real-robot hold."""
    try:
        print(message, file=sys.stderr if error else sys.stdout)
    except (Exception, KeyboardInterrupt):
        pass


def stop_event(name, **details):
    """Record a stop without letting failed telemetry bypass motor damping."""
    try:
        runlog.event("walker", name, **details)
    except (Exception, KeyboardInterrupt) as exc:
        stop_message(f"Could not record walker {name}: {exc}", error=True)


def gravity_in_body(quat):
    """Unit gravity vector in the pelvis frame, from a (w, x, y, z) quaternion."""
    w, x, y, z = quat
    return np.array([2 * (w * y - x * z), -2 * (w * x + y * z),
                     -(w * w - x * x - y * y + z * z)], dtype=np.float32)


class Gr00tPolicy:
    """GR00T Decoupled WBC lower body: two ONNX MLPs, Balance and Walk."""

    def __init__(self):
        opts = ort.SessionOptions()
        opts.intra_op_num_threads = opts.inter_op_num_threads = 1
        self.balance = ort.InferenceSession(
            str(POLICY_DIR / "GR00T-WholeBodyControl-Balance.onnx"), opts)
        self.walk = ort.InferenceSession(
            str(POLICY_DIR / "GR00T-WholeBodyControl-Walk.onnx"), opts)
        self.action = np.zeros(NUM_ACTIONS, dtype=np.float32)
        self.history = deque([np.zeros(OBS_DIM, dtype=np.float32)] * HISTORY_LEN,
                             maxlen=HISTORY_LEN)

    def step(self, q, dq, quat, gyro, cmd, height):
        """Return target positions for the 15 leg and waist joints."""
        obs = np.zeros(OBS_DIM, dtype=np.float32)
        obs[0:3] = cmd * CMD_SCALE
        obs[3] = height
        # obs[4:7] is the torso roll/pitch/yaw command, kept at zero.
        obs[7:10] = gyro * ANG_VEL_SCALE
        obs[10:13] = gravity_in_body(quat)
        obs[13:42] = q - OBS_DEFAULTS
        obs[42:71] = dq * DOF_VEL_SCALE
        obs[71:86] = self.action
        require_finite("policy observation", obs)
        self.history.append(obs)

        session = self.balance if np.linalg.norm(cmd) < STAND_CMD else self.walk
        stacked = np.concatenate(self.history)[None, :]
        action = session.run(None, {session.get_inputs()[0].name: stacked})[0][0]
        require_finite("policy action", action)
        self.action = action
        return STAND_POSE[:NUM_ACTIONS] + self.action * ACTION_SCALE


class Walker:
    """Stand-up ramp, then the GR00T policy driven by velocity commands."""

    def __init__(self, real=False):
        self.real = real
        self.state_timeout = STATE_TIMEOUT_REAL if real else STATE_TIMEOUT_SIM
        self.policy = Gr00tPolicy()
        self.state = None
        self.state_time = 0.0
        self.cmd = np.zeros(3, dtype=np.float32)   # vx, vy (m/s), yaw rate (rad/s)
        self.height = DEFAULT_HEIGHT
        self.standing = False     # stand-up ramp finished; the policy may start
        self.policy_on = False
        self.quit = False
        self.read_key = None
        self.on_policy_output = None   # (obs, action) after each policy step, for logging
        # Printed once the policy is on; move_distance.py replaces it.
        self.next_hint = ("Drive with w/s (forward/back), a/d (sideways), q/e (turn), "
                          "z (stop moving), 1/2 (raise/lower); o stops.")

        self.low_cmd = unitree_hg_msg_dds__LowCmd_()
        self.crc = CRC()
        self.cmd_pub = ChannelPublisher("rt/lowcmd", LowCmd_)
        self.cmd_pub.Init()
        self.state_sub = ChannelSubscriber("rt/lowstate", LowState_)
        self.state_sub.Init(self.on_state, 10)

    def on_state(self, msg):
        self.state = msg
        self.state_time = time.monotonic()

    def joints(self):
        motors = self.state.motor_state
        q = np.array([motors[i].q for i in range(NUM_MOTORS)], dtype=np.float32)
        dq = np.array([motors[i].dq for i in range(NUM_MOTORS)], dtype=np.float32)
        require_finite("joint state", q, dq)
        return q, dq

    def send(self, q, kp, kd):
        require_finite("motor target and gains", q, kp, kd)
        self.low_cmd.mode_pr = 0  # ankles commanded as pitch/roll joints
        self.low_cmd.mode_machine = self.state.mode_machine
        q = np.clip(q, Q_LOWER, Q_UPPER)
        for i in range(NUM_MOTORS):
            m = self.low_cmd.motor_cmd[i]
            m.mode = 1
            m.q, m.dq, m.tau = float(q[i]), 0.0, 0.0
            m.kp, m.kd = float(kp[i]), float(kd[i])
        self.low_cmd.crc = self.crc.Crc(self.low_cmd)
        self.cmd_pub.Write(self.low_cmd)

    def send_damping(self):
        self.send(np.zeros(NUM_MOTORS), np.zeros(NUM_MOTORS), np.full(NUM_MOTORS, DAMPING_KD))

    def damp(self):
        """Joints go soft (kp = 0) and the robot sinks; the standard safe state.
        Runs its full STOP_TIME even through Ctrl-C."""
        if self.state is None:
            return
        next_tick = time.monotonic()
        end = next_tick + STOP_TIME
        while time.monotonic() < end:
            try:
                self.send_damping()
                next_tick += PUBLISH_DT
                time.sleep(max(0.0, next_tick - time.monotonic()))
            except KeyboardInterrupt:
                pass
        stop_event("damped")
        stop_message("Stopped: all joints damped, so the robot sinks.")

    def stop(self, reason, message):
        """The one way to stop: say why, damp every joint, and on the real robot
        keep damping until o or Ctrl-C."""
        stop_message(message, error=reason not in CLEAN_STOPS)
        self.policy_on = False
        stop_event("stop", reason=reason)
        self.damp()
        if self.real and self.state is not None:
            self.hold()

    def hold(self):
        """Keep sending damping commands, so the joints stay soft and nothing
        else is left commanding them, until o or Ctrl-C."""
        stop_message("Still damping. Press o or Ctrl-C to quit once the robot is secured.")
        next_tick = time.monotonic()
        try:
            while True:
                self.send_damping()
                if self.read_key is not None and self.read_key() == "o":
                    break
                next_tick += PUBLISH_DT
                time.sleep(max(0.0, next_tick - time.monotonic()))
        except KeyboardInterrupt:
            pass

    def handle_key(self, key):
        steps = {"w": (0, CMD_STEP), "s": (0, -CMD_STEP), "a": (1, CMD_STEP),
                 "d": (1, -CMD_STEP), "q": (2, CMD_STEP), "e": (2, -CMD_STEP)}
        if key == "]" and not self.policy_on:
            if not self.standing:
                print("Not yet: wait for 'Stand pose reached' (about 3 s after start).")
                return
            self.policy_on = True
            runlog.event("walker", "policy_on")
            if self.real:
                print("Policy on: GR00T is balancing. Slacken the gantry, leaving the rope "
                      "just loose enough to catch a fall.")
            else:
                print("Policy on: GR00T is balancing. Release the band (9 in the MuJoCo "
                      "window) and give the robot a few seconds to settle.")
            print(self.next_hint)
            return
        if key == "o":
            self.quit = True
            return
        if key in steps:
            axis, step = steps[key]
            self.cmd[axis] += step
        elif key == "z":
            self.cmd[:] = 0.0
        elif key in ("1", "2"):
            self.height += HEIGHT_STEP if key == "1" else -HEIGHT_STEP
        else:
            return
        self.cmd = np.clip(self.cmd, [-MAX_VX, -MAX_VY, -MAX_YAW_RATE],
                           [MAX_VX, MAX_VY, MAX_YAW_RATE]).astype(np.float32)
        self.height = float(np.clip(self.height, *HEIGHT_RANGE))
        print(f"cmd vx={self.cmd[0]:+.1f} vy={self.cmd[1]:+.1f} "
              f"yaw={self.cmd[2]:+.1f}  height={self.height:.2f}")

    def check_mode_machine(self):
        mode = self.state.mode_machine
        if mode != G1_29DOF_REV_1_0:
            msg = (f"the robot reports mode_machine {mode}, but GR00T's config is for the "
                   f"29-joint G1 rev 1.0 (mode_machine {G1_29DOF_REV_1_0})")
            if self.real:
                raise RuntimeError(f"Not running: {msg}.")
            print(f"Note: {msg}. Carrying on, since this is a sim.")

    def run(self, read_key, on_policy_step=None):
        """Control loop. read_key() returns one pending key or None.

        on_policy_step(q, quat), if given, is called before each policy step
        (50 Hz, once the policy is on) and may change self.cmd or set self.quit.
        However it ends, it ends in stop(), and returns why: "quit" (o), "ctrl_c",
        "remote_estop", "state_lost", "fall" or "error" (CLEAN_STOPS aren't failures).
        """
        self.read_key = read_key
        try:
            self.loop(read_key, on_policy_step)
        except Stop as e:
            if e.reason not in CLEAN_STOPS and e.reason != "fall":   # a fall has its own outcome
                stop_event("error", message=str(e))
            self.stop(e.reason, str(e))
            return e.reason
        except KeyboardInterrupt:
            self.stop("ctrl_c", "Interrupted (Ctrl-C).")
            return "ctrl_c"
        except Exception as e:
            stop_event("error", message=str(e))
            self.stop("error", f"Error: {e}")
            return "error"
        self.stop("quit", "Stopping (o).")
        return "quit"

    def loop(self, read_key, on_policy_step):
        wait_until = time.monotonic() + 5.0
        while self.state is None:
            if time.monotonic() > wait_until:
                where = ("the robot. Check the Ethernet cable and that --interface is the one "
                         "on 192.168.123.x" if self.real else
                         "the sim. Start it in another terminal first: ros2 run harp_sim run_sim")
                raise RuntimeError(f"No robot state (rt/lowstate) after 5 s from {where}.")
            time.sleep(0.05)
        self.check_mode_machine()

        start_q, _ = self.joints()
        t0 = time.monotonic()
        next_tick = t0
        target = start_q
        tick = 0
        print("Moving to the stand pose (3 s). Keys go to this terminal; o stops at any time.")
        while not self.quit:
            now = time.monotonic()
            if now - self.state_time > self.state_timeout:
                stop_event("state_lost", ms=round((now - self.state_time) * 1000))
                raise Stop("state_lost", f"Robot state (rt/lowstate) stopped for "
                           f"{(now - self.state_time) * 1000:.0f} ms. Did the sim or the network stop?")
            if remote.ESTOP <= remote.pressed(self.state.wireless_remote):
                stop_event("estop", by="remote")
                raise Stop("remote_estop", "E-stop: L2+B on the remote.")
            key = read_key()
            if key:
                self.handle_key(key)

            q, dq = self.joints()
            quat = np.array(self.state.imu_state.quaternion, dtype=np.float32)
            gyro = np.array(self.state.imu_state.gyroscope, dtype=np.float32)
            require_finite("IMU state", quat, gyro)
            if self.policy_on:
                if gravity_in_body(quat)[2] > -math.cos(FALL_TILT):
                    stop_event("fall")
                    raise Stop("fall", f"Fall detected: the pelvis tilted more than "
                               f"{math.degrees(FALL_TILT):.0f} degrees.")
                if tick % round(CONTROL_DT / PUBLISH_DT) == 0:
                    if on_policy_step is not None:
                        on_policy_step(q, quat)
                    target = STAND_POSE.copy()
                    target[:NUM_ACTIONS] = self.policy.step(
                        q, dq, quat, gyro, self.cmd, self.height)
                    if self.on_policy_output is not None:
                        self.on_policy_output(self.policy.history[-1], self.policy.action, self.cmd)
            else:
                alpha = min((now - t0) / RAMP_TIME, 1.0)
                target = (1 - alpha) * start_q + alpha * STAND_POSE
                if alpha >= 1.0 and not self.standing:
                    self.standing = True
                    runlog.event("walker", "stand_pose")
                    if self.real:
                        print("Stand pose reached. Lower the gantry until the feet are flat on "
                              "the floor, then press ] here to start the policy.")
                    else:
                        print("Stand pose reached. Lower the robot onto its feet (press 8 a few "
                              "times in the MuJoCo window), then press ] here to start the policy.")
            self.send(target, KP, KD)

            tick += 1
            next_tick += PUBLISH_DT
            time.sleep(max(0.0, next_tick - time.monotonic()))


def check_unitree_controller_off():
    """Refuse to run if Unitree's own controller still owns the motors."""
    from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient
    client = MotionSwitcherClient()
    client.SetTimeout(3.0)
    client.Init()
    code, result = client.CheckMode()
    if code != 0:
        raise RuntimeError(f"Could not ask the robot which controller is active (motion_switcher "
                           f"error {code}). Is the robot on, and is --interface right?")
    if result and result.get("name"):
        raise RuntimeError(
            f"Not running: Unitree's own controller ({result['name']}) is still active. Hang "
            "the robot, press L1+A (damping), then L2+R2 (debug mode) on the remote, and run again.")


class TerminalKeys:
    """Non-blocking single-key reads from the terminal (macOS and Linux)."""

    def __enter__(self):
        self.fd = sys.stdin.fileno()
        self.saved = termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd)
        return self

    def __exit__(self, *exc):
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.saved)

    def read(self):
        if select.select([sys.stdin], [], [], 0)[0]:
            return sys.stdin.read(1)
        return None


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--real", action="store_true",
                        help="the real robot (DDS domain 0) instead of the sim (domain 1)")
    parser.add_argument("--interface", help="network interface; default lo0/lo in the sim")
    args = parser.parse_args()
    if args.real and not args.interface:
        parser.error("--real needs --interface (the one on 192.168.123.x)")
    interface = args.interface or ("lo0" if sys.platform == "darwin" else "lo")

    ChannelFactoryInitialize(0 if args.real else 1, interface)
    if args.real:
        check_unitree_controller_off()
    walker = Walker(real=args.real)
    with TerminalKeys() as keys:
        reason = walker.run(keys.read)
    return 0 if reason in CLEAN_STOPS else 1


if __name__ == "__main__":
    raise SystemExit(main())
