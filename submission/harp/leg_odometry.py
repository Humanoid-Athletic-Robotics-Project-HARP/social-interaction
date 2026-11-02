"""Leg odometry for the G1, our /odom in the sim and on the robot alike (harp/state.py):
  - Heading: yaw from the pelvis IMU quaternion.
  - Position: joint angles and the IMU orientation give each foot's position
    relative to the pelvis (forward kinematics of the G1 model in MuJoCo).
    The lower foot is taken as planted, so the pelvis moves by minus that
    foot's motion.
The same kinematics give the head lidar's and depth camera's poses for their transforms.
"""

import math

import mujoco
import numpy as np

from harp import camera_mount, lidar_mount

NUM_MOTORS = 29
FEET = ("left_ankle_roll_link", "right_ankle_roll_link")
ANKLE_HEIGHT = 0.035      # m, ankle_roll_link above the sole, in the G1 model


def yaw_of(quat):
    w, x, y, z = quat
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def wrap_angle(a):
    return math.atan2(math.sin(a), math.cos(a))


class LegOdometry:
    """Pelvis position on the floor and unwrapped heading, from joints and IMU."""

    def __init__(self, model_path):
        self.model = mujoco.MjModel.from_xml_path(str(model_path))
        self.data = mujoco.MjData(self.model)
        self.feet = [self.model.body(name).id for name in FEET]
        self.torso = self.model.body(lidar_mount.PARENT).id   # both head sensors' parent
        self.lidar_mount = lidar_mount.rotation()
        self.camera_mount = camera_mount.rotation()
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

    def lidar_pose(self):
        """The head lidar relative to base_link: see sensor_pose."""
        return self.sensor_pose(lidar_mount.POSITION, self.lidar_mount)

    def camera_pose(self):
        """The head depth camera's optical frame relative to base_link: see sensor_pose."""
        return self.sensor_pose(camera_mount.POSITION, self.camera_mount)

    def sensor_pose(self, position_in_torso, rotation_in_torso):
        """A sensor on the torso relative to base_link (the floor under the pelvis,
        turned by the pelvis yaw) at the last update: translation (m) and quaternion
        (x, y, z, w). Follows the torso's lean and sway and the pelvis height, so
        floor points stay on the floor in the costmap."""
        d = self.data
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        unyaw = np.array([[c, s, 0.0], [-s, c, 0.0], [0.0, 0.0, 1.0]])
        torso = d.xmat[self.torso].reshape(3, 3)
        position = unyaw @ (d.xpos[self.torso] + torso @ position_in_torso)
        position[2] += ANKLE_HEIGHT - self.last_feet[self.stance][2]   # pelvis above the floor
        quat = np.zeros(4)
        mujoco.mju_mat2Quat(quat, (unyaw @ torso @ rotation_in_torso).ravel())
        return position, (quat[1], quat[2], quat[3], quat[0])
