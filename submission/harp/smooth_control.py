"""Smooth walk to a target pose, after Park & Kuipers, "A Smooth Control Law
for Graceful Motion of Differential Wheeled Mobile Robots in 2D Environments"
(ICRA 2011). The same law is Nav2's Graceful Controller
(nav2_graceful_controller, SmoothControlLaw), so its gains carry over.

The robot is treated as a unicycle: forward speed v and turn rate w, no
sideways steps. That's also how people mostly walk to a goal (Arechavaleta,
Laumond et al., "On the nonholonomic nature of human locomotion", 2008).
Instead of turning in place and then walking straight, the robot follows one
curve that ends at the target position, facing the target heading.

Poses are (x, y, yaw) in any one fixed frame, yaw in radians,
counterclockwise positive.
"""

import math

# Nav2's defaults for the Graceful Controller.
K_PHI = 2.0        # how strongly the target heading shapes the curve
K_DELTA = 1.0      # how fast the robot turns onto that curve
BETA = 0.4         # how fast speed drops on sharp curves,
LAMBDA = 2.0       #   v = v_max / (1 + BETA * |curvature|^LAMBDA)
SLOWDOWN_RADIUS = 0.6   # m, speed ramps down linearly inside this distance


def wrap_angle(a):
    """Wrap an angle in radians to [-pi, pi)."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def ego_polar(current, target):
    """Target in the robot's egocentric polar coordinates (r, phi, delta).

    r: distance to the target. phi: the target's heading relative to the line
    of sight from robot to target. delta: the robot's heading relative to it.
    """
    dx, dy = target[0] - current[0], target[1] - current[1]
    sight = math.atan2(dy, dx)
    return math.hypot(dx, dy), wrap_angle(target[2] - sight), wrap_angle(current[2] - sight)


def curvature(r, phi, delta):
    """Curvature (1/m, positive turns left) that steers onto the paper's
    exponentially stable path to the target pose."""
    return -(K_DELTA * (delta - math.atan(-K_PHI * phi))
             + (1.0 + K_PHI / (1.0 + (K_PHI * phi) ** 2)) * math.sin(delta)) / r


def command(current, target, v_max, w_max, v_min=0.0):
    """Forward speed (m/s) and turn rate (rad/s) toward the target pose."""
    r, phi, delta = ego_polar(current, target)
    k = curvature(r, phi, delta)
    v = v_max / (1.0 + BETA * abs(k) ** LAMBDA)   # slow down on sharp curves
    v = min(v, v_max * r / SLOWDOWN_RADIUS)       # and near the target
    v = max(v, v_min)
    w = k * v
    if abs(w) > w_max:   # keep the curve, slow down to stay within the turn rate
        w = math.copysign(w_max, w)
        v = w / k
    return v, w
