"""Where the head lidar (Livox MID-360) sits on the G1: mid360_joint in Unitree's
g1_29dof_rev_1_0.urdf (unitree_ros), the revision of our robot (mode_machine 5)
and of him-sim2real-management's G1 model. The URDF leaves out the 180 deg roll;
Unitree's lidar docs and the community G1 drivers have the lidar upside down.
"""

import math

import numpy as np

PARENT = "torso_link"
POSITION = np.array([0.0003, 0.0, 0.428434])   # m, in torso_link
RPY = (math.pi, 0.0511, 0.0)                    # rad, URDF order (fixed axes x, y, z)
FRAME = "livox_frame"


def rotation():
    """The lidar's orientation in torso_link: Rz(yaw) Ry(pitch) Rx(roll)."""
    r, p, y = RPY
    rx = np.array([[1, 0, 0], [0, math.cos(r), -math.sin(r)], [0, math.sin(r), math.cos(r)]])
    ry = np.array([[math.cos(p), 0, math.sin(p)], [0, 1, 0], [-math.sin(p), 0, math.cos(p)]])
    rz = np.array([[math.cos(y), -math.sin(y), 0], [math.sin(y), math.cos(y), 0], [0, 0, 1]])
    return rz @ ry @ rx
