"""Where the head lidar (Livox MID-360) sits on the G1. One place for these
numbers, used by the sim's lidar (harp_sim) and by walker_node, which
publishes them as the base_link -> livox_frame transform.

From mid360_joint in Unitree's g1_29dof.urdf (unitree_ros), the revision our
MuJoCo model matches. That URDF leaves out the 180 deg roll, but Unitree's
lidar docs and the community G1 drivers have the lidar upside down. Check
both against our robot: the newer g1_29dof_rev_1_0.urdf puts it at 0.428434 m
with pitch 0.0511.
"""

import math

import numpy as np

PARENT = "torso_link"
POSITION = np.array([0.0002835, 0.00003, 0.40618])   # m, in torso_link
RPY = (math.pi, 0.04014257279586953, 0.0)             # rad, URDF order (fixed axes x, y, z)
FRAME = "livox_frame"

# For base_link, the point on the floor under the pelvis: how high torso_link
# is while GR00T balances. The pelvis is at about 0.75 m in the sim (GR00T
# commands 0.74), and torso_link is 0.054 m above it in the G1 model.
STANDING_TORSO_HEIGHT = 0.75 + 0.054   # m


def rotation():
    """The lidar's orientation in torso_link: Rz(yaw) Ry(pitch) Rx(roll)."""
    r, p, y = RPY
    rx = np.array([[1, 0, 0], [0, math.cos(r), -math.sin(r)], [0, math.sin(r), math.cos(r)]])
    ry = np.array([[math.cos(p), 0, math.sin(p)], [0, 1, 0], [-math.sin(p), 0, math.cos(p)]])
    rz = np.array([[math.cos(y), -math.sin(y), 0], [math.sin(y), math.cos(y), 0], [0, 0, 1]])
    return rz @ ry @ rx


def from_base_link():
    """base_link -> lidar for a robot standing upright: translation (m) and
    quaternion (x, y, z, w). Leaning and swaying of the torso aren't in it."""
    r, p, y = (a / 2 for a in RPY)
    quat = (math.sin(r) * math.cos(p) * math.cos(y) - math.cos(r) * math.sin(p) * math.sin(y),
            math.cos(r) * math.sin(p) * math.cos(y) + math.sin(r) * math.cos(p) * math.sin(y),
            math.cos(r) * math.cos(p) * math.sin(y) - math.sin(r) * math.sin(p) * math.cos(y),
            math.cos(r) * math.cos(p) * math.cos(y) + math.sin(r) * math.sin(p) * math.sin(y))
    return POSITION + np.array([0.0, 0.0, STANDING_TORSO_HEIGHT]), quat
