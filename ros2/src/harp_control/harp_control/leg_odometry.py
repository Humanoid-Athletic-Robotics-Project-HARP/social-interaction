"""Leg odometry for the G1: pelvis position on the floor and heading, from
the joint angles and the pelvis IMU. Unitree's odometry is part of the
built-in controller we no longer use, so we estimate our own:
  - Heading: yaw from the pelvis IMU quaternion.
  - Position: joint angles and the IMU orientation give each foot's position
    relative to the pelvis (forward kinematics of the G1 model in MuJoCo).
    The lower foot is taken as planted, so the pelvis moves by minus that
    foot's motion.
"""

import math
import os

import mujoco
import numpy as np

from harp_control.gr00t.walk import NUM_MOTORS
from harp_control.smooth_control import wrap_angle

# Unitree's G1 model (unitree_mujoco, cloned next to this repo). The ROS 2
# environment (ros2/pixi.toml) sets HARP_G1_SCENE to it.
DEFAULT_SCENE = os.environ.get("HARP_G1_SCENE")
FEET = ("left_ankle_roll_link", "right_ankle_roll_link")


def yaw_of(quat):
    w, x, y, z = quat
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


class LegOdometry:
    """Pelvis position on the floor and unwrapped heading, from joints and IMU."""

    def __init__(self, scene):
        self.model = mujoco.MjModel.from_xml_path(str(scene))
        self.data = mujoco.MjData(self.model)
        self.feet = [self.model.body(name).id for name in FEET]
        self.position = np.zeros(2)
        self.heading = 0.0        # sum of yaw changes, radians
        self.yaw = None
        self.stance = None        # index of the planted foot
        self.last_feet = None     # both feet relative to the pelvis, world-aligned, last update

    def update(self, q, quat):
        yaw = yaw_of(quat)
        if self.yaw is not None:
            self.heading += wrap_angle(yaw - self.yaw)
        self.yaw = yaw

        d = self.data
        d.qpos[:3] = 0.0          # pelvis at the origin, so foot positions are relative to it
        d.qpos[3:7] = quat
        d.qpos[7:7 + NUM_MOTORS] = q
        mujoco.mj_kinematics(self.model, d)
        feet = [d.xpos[f].copy() for f in self.feet]
        if self.last_feet is not None:
            # Use the foot that was planted since the last update, also on the update
            # where the planted foot switches. Skipping that one dropped its motion at
            # every footstep, about 7% of the distance walked.
            self.position -= feet[self.stance][:2] - self.last_feet[self.stance][:2]
        self.stance = 0 if feet[0][2] < feet[1][2] else 1
        self.last_feet = feet
