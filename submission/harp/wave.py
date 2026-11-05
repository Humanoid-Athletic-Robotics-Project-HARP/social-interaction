"""The right arm's hello wave as a function of time, for harp_node.py's wave task
(G1_TASK_COMMAND='{"wave": {}}'): GR00T keeps the legs and waist, this sets the arm.

From the arm's pose when it starts:
  raise  RAISE_S s along a minimum-jerk curve (speed and acceleration zero at both
         ends) to HAND_UP;
  wave   CYCLES swings at HZ: the elbow swings SWING rad each way about HAND_UP, and
         wrist roll WRIST_SHARE of that, WRIST_LAG of a cycle behind (the hand's
         flick); a raised cosine fades the first and last FADE swings in and out;
  lower  LOWER_S s, minimum jerk, to NEUTRAL (the walker's own arm targets);
then done, at NEUTRAL. Targets come with their velocities. Wrist pitch and yaw never
move: their 5 Nm caps leave about 0.11 rad of tracking room at HIM's kp 40.

HAND_UP is from forward kinematics on HIM's G1 model (2026-11-04): the right wrist
0.21 m above, 0.23 m out from and 0.03 m ahead of the shoulder, the forearm upright;
an elbow change of 0.4 rad moves it 0.07 m sideways. Arm joints are in Unitree order:
shoulder pitch, roll, yaw, elbow, wrist roll, pitch, yaw (indices 22-28 of the 29).
"""

import math

import numpy as np

RIGHT_ARM = slice(22, 29)                 # Unitree indices of the right arm's joints
NEUTRAL = (0.0,) * 7                      # the walker's arm targets
HAND_UP = (0.0, -1.6, -1.4, 0.0, 0.0, 0.0, 0.0)
LOW = (-3.0892, -2.2515, -2.618, -1.0472, -1.97222, -1.61443, -1.61443)   # rad, HIM's G1 model
HIGH = (2.6704, 1.5882, 2.618, 2.0944, 1.97222, 1.61443, 1.61443)
SETTLE_S = 2.0      # s after HIM hands control over before the wave starts
RAISE_S = 2.0       # s (1.0 moved the hand at up to 1.3 m/s in HIM: too fast, Lam)
LOWER_S = 2.0       # s
HZ = 0.8            # swings a second (1.25 rocked the torso 3.2° at the swing's ends, Lam saw it; 0.8 with SWING 0.3: 1.0°)
CYCLES = 3
FADE = 1.0          # swings the size fades in and out over (0.5 was over twice as jerky)
SWING = 0.3         # rad, the elbow's swing each way about HAND_UP (0.4 rocked the torso half as much again)
WRIST_SHARE = 0.25  # wrist roll's swing as a share of SWING
WRIST_LAG = 0.15    # cycles wrist roll trails the elbow
ELBOW, WRIST_ROLL = 3, 4


def min_jerk(u):
    """Progress and its rate (per unit of u) along a minimum-jerk curve, u from 0 to 1."""
    return 10 * u**3 - 15 * u**4 + 6 * u**5, 30 * u**2 - 60 * u**3 + 30 * u**4


class Wave:
    def __init__(self, start_time, start_pose):
        self.start = float(start_time)
        self.pose = np.array(start_pose, dtype=float)
        self.up, self.neutral = np.array(HAND_UP), np.array(NEUTRAL)
        self.wave_s = CYCLES / HZ
        self.fade_s = FADE / HZ
        self.duration = RAISE_S + self.wave_s + LOWER_S
        self.end = self.start + self.duration

    def at(self, t):
        """(targets, velocities, phase) at time t: 7 values each, phase raise/wave/lower/done."""
        tau = t - self.start
        if tau <= 0.0:
            return self.pose.copy(), np.zeros(7), "raise"
        if t >= self.end:    # absolute, so rounding in tau's sums can't leave the end in "lower"
            return self.neutral.copy(), np.zeros(7), "done"
        if tau < RAISE_S:
            return self.curve(self.pose, self.up, tau, RAISE_S) + ("raise",)
        tau -= RAISE_S
        if tau < self.wave_s:
            return self.swing(tau) + ("wave",)
        tau -= self.wave_s
        return self.curve(self.up, self.neutral, min(tau, LOWER_S), LOWER_S) + ("lower",)

    @staticmethod
    def curve(a, b, tau, length):
        s, rate = min_jerk(tau / length)
        return a + (b - a) * s, (b - a) * rate / length

    def envelope(self, tau):
        """The swing's size (0 to 1) and its rate: raised cosines over the first and last FADE swings."""
        k = math.pi / self.fade_s
        if tau < self.fade_s:
            return 0.5 * (1 - math.cos(k * tau)), 0.5 * k * math.sin(k * tau)
        left = self.wave_s - tau
        if left < self.fade_s:
            return 0.5 * (1 - math.cos(k * left)), -0.5 * k * math.sin(k * left)
        return 1.0, 0.0

    def swing(self, tau):
        q, dq = self.up.copy(), np.zeros(7)
        e, de = self.envelope(tau)
        w = 2 * math.pi * HZ
        for joint, size, lag in ((ELBOW, SWING, 0.0), (WRIST_ROLL, WRIST_SHARE * SWING, 2 * math.pi * WRIST_LAG)):
            angle = w * tau - lag
            q[joint] += size * e * math.sin(angle)
            dq[joint] = size * (de * math.sin(angle) + e * w * math.cos(angle))
        return q, dq
