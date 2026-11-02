"""Where the head depth camera (Intel RealSense D435i) sits on the G1: d435_joint in
Unitree's g1_29dof_rev_1_0.urdf, 5.7 cm ahead of the lidar and pitched 47.6 deg down
(Unitree's depth camera drawing agrees). Its points are in the camera's optical frame
(z forward, x right, y down), as a RealSense driver publishes them.
"""

import numpy as np

from harp.lidar_mount import rpy_matrix

PARENT = "torso_link"
POSITION = np.array([0.0576235, 0.01753, 0.42987])   # m, in torso_link
RPY = (0.0, 0.8307767239493009, 0.0)                 # rad, URDF order (fixed axes x, y, z)
FRAME = "d435_depth_optical_frame"
# The optical axes (x right, y down, z forward) in the camera link's (x forward, z up)
OPTICAL = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])


def rotation():
    """The camera's optical frame's orientation in torso_link."""
    return rpy_matrix(*RPY) @ OPTICAL
