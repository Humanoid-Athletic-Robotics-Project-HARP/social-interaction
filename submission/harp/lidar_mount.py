"""Where the head lidar (Livox MID-360) sits on the G1: mid360_joint in Unitree's
g1_29dof_rev_1_0.urdf (unitree_ros), the revision of our robot (mode_machine 5)
and of him-sim2real-management's G1 model. It is upside down (the URDF's 180 deg
roll), as measured on the robot; the older g1_29dof.urdf leaves the roll out.

What the head lets it see around the front depends on the face visor (VISOR,
context/g1-face-visor.md): with it on, nothing ahead; with it off, a window ahead
between two blind wedges, the face frame's sides. sees() is that shadow, for the
simulated lidar (harp/lidar.py), the stand-in person detector (harp/sim_people.py)
and Nav2's clearing views (nav2/nav2.launch.py). Switching VISOR needs a rebuild
(submission/build.sh).
"""

import math

import numpy as np

PARENT = "torso_link"
POSITION = np.array([0.0003, 0.0, 0.428434])   # m, in torso_link
RPY = (math.pi, 0.0511, 0.0)                    # rad, URDF order (fixed axes x, y, z)
ELEVATION = (-7.0, 52.0)   # deg, its field of view in its own frame (upside down: 7 up to 52 down)
FRAME = "livox_frame"
# livox_frame turned 180 deg about its z: its +x points back, where the lidar sees, so
# Nav2's clearing view (centred on +x) can leave out the blind cone ahead
REAR_FRAME = "livox_rear"
VISOR = False   # the face visor on the robot: off since 2026-11-02 (check the robot)
# The head's shadow by torso_link elevation (deg, + up), symmetric left and right: (elevation,
# open, blind): with the visor off it sees within `open` deg of straight ahead, and nothing
# from there out to `blind` deg (the face frame); with it on, nothing within `blind`. Above
# about +2 deg the band holding the camera blocks the window. Fitted to the visor-off map
# (robot-captures 2026-11-02/lidar/visor-off-standing-60s, testground/lidar/blind_map.py);
# the visor-on one (lidar/standing-60s) has the same `blind` edge.
VIEW = ((6.0, 0.0, 68.0), (2.0, 0.0, 67.0), (1.0, 36.0, 66.0), (-5.0, 42.0, 64.0),
        (-17.0, 39.0, 60.0), (-29.0, 35.0, 55.0), (-41.0, 27.0, 51.0), (-49.0, 11.0, 52.0))
TOPIC = "/utlidar/cloud_livox_mid360"   # the cloud, in FRAME
REAR_TOPIC = "/utlidar/cloud_rear"      # the same, in REAR_FRAME


def rpy_matrix(r, p, y):
    """A URDF rpy as a rotation matrix: Rz(yaw) Ry(pitch) Rx(roll)."""
    rx = np.array([[1, 0, 0], [0, math.cos(r), -math.sin(r)], [0, math.sin(r), math.cos(r)]])
    ry = np.array([[math.cos(p), 0, math.sin(p)], [0, 1, 0], [-math.sin(p), 0, math.cos(p)]])
    rz = np.array([[math.cos(y), -math.sin(y), 0], [math.sin(y), math.cos(y), 0], [0, 0, 1]])
    return rz @ ry @ rx


def rotation():
    """The lidar's orientation in torso_link."""
    return rpy_matrix(*RPY)


def sees(directions, visor=None):
    """Which directions from the lidar (N x 3, in torso_link) the head leaves open, with
    the visor on or off (VISOR by default)."""
    visor = VISOR if visor is None else visor
    d = np.asarray(directions, dtype=float).reshape(-1, 3)
    azimuth = np.abs(np.degrees(np.arctan2(d[:, 1], d[:, 0])))
    elevation = np.degrees(np.arctan2(d[:, 2], np.hypot(d[:, 0], d[:, 1])))
    el, open_, blind = (np.array(c[::-1]) for c in zip(*VIEW))   # np.interp wants rising
    window = 0.0 if visor else np.interp(elevation, el, open_)
    return (azimuth < window) | (azimuth >= np.interp(elevation, el, blind))
