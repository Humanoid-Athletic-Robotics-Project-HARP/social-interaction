"""Robot state from /safety/policy_state as standard ROS topics, for Nav2 and rviz:

  /odom          nav_msgs/Odometry   odom -> base_link (planar, really a base_footprint)
  /tf            odom -> base_link, and base_link -> livox_frame (the head lidar's real
                 pose from the joints and IMU, so the floor stays flat in the costmap)
  /joint_states  sensor_msgs/JointState  the 29 joints
  /imu           sensor_msgs/Imu     the pelvis IMU

Odometry is always our leg odometry, in the sim too, so Nav2 and the course
results get the same drifting estimate as on the robot (where HIM usually has
no base position). HIM's `position` in the sim is the exact MuJoCo pose: it is
only ground truth, logged once a second beside the estimate (an "odom" line in
the container's output, HIM's policy.log) for drift checks and course scoring,
never used for /odom.
"""

import math

import numpy as np
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, JointState
from tf2_ros import TransformBroadcaster

from harp import events, lidar_mount
from harp.leg_odometry import LegOdometry, yaw_of

CONTROL_DT = 0.02         # s between states
VELOCITY_SMOOTHING = 0.2  # low-pass factor per state for leg-odometry velocity
TRUTH_PERIOD = 1.0        # s between "odom" lines (estimate beside the sim's truth)


class StatePublisher:
    def __init__(self, node, model_path):
        self.node = node
        self.legs = LegOdometry(model_path)
        model = self.legs.model
        self.joint_names = [model.joint(model.actuator_trnid[i, 0]).name for i in range(model.nu)]
        self.odom_pub = node.create_publisher(Odometry, "/odom", 10)
        self.joints_pub = node.create_publisher(JointState, "/joint_states", 10)
        self.imu_pub = node.create_publisher(Imu, "/imu", 10)
        self.tf = TransformBroadcaster(node)
        self.last = None          # leg-odometry x, y, heading at the previous state
        self.velocity = np.zeros(3)
        self.next_truth = 0.0

    def publish(self, state):
        stamp = self.node.get_clock().now().to_msg()
        self.legs.update(state["q"], state["quat"])
        x, y, yaw, velocity = self.from_legs()
        self.publish_odometry(stamp, x, y, yaw, velocity)
        self.publish_joints_imu(stamp, state)
        if "position" in state and state.get("sim_time", 0.0) >= self.next_truth:
            self.next_truth = state["sim_time"] + TRUTH_PERIOD
            truth = state["position"]
            events.emit("policy", "odom", state["sim_time"],
                        legs=[round(float(v), 3) for v in (x, y, yaw)],
                        truth=[round(truth[0], 3), round(truth[1], 3), round(yaw_of(state["quat"]), 3)])

    def from_legs(self):
        x, y = self.legs.position
        heading = self.legs.heading
        if self.last is not None:
            dx, dy = x - self.last[0], y - self.last[1]
            c, s = math.cos(self.legs.yaw), math.sin(self.legs.yaw)
            measured = np.array([c * dx + s * dy, -s * dx + c * dy,
                                 heading - self.last[2]]) / CONTROL_DT
            self.velocity += VELOCITY_SMOOTHING * (measured - self.velocity)
        self.last = (x, y, heading)
        return x, y, self.legs.yaw, tuple(self.velocity)

    def publish_odometry(self, stamp, x, y, yaw, velocity):
        rotation = (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x, odom.pose.pose.position.y = float(x), float(y)
        o = odom.pose.pose.orientation
        o.x, o.y, o.z, o.w = rotation
        odom.twist.twist.linear.x, odom.twist.twist.linear.y = float(velocity[0]), float(velocity[1])
        odom.twist.twist.angular.z = float(velocity[2])
        self.odom_pub.publish(odom)

        base = TransformStamped()
        base.header = odom.header
        base.child_frame_id = "base_link"
        base.transform.translation.x, base.transform.translation.y = float(x), float(y)
        r = base.transform.rotation
        r.x, r.y, r.z, r.w = rotation
        position, quat = self.legs.lidar_pose()
        lidar = TransformStamped()
        lidar.header.stamp = stamp
        lidar.header.frame_id = "base_link"
        lidar.child_frame_id = lidar_mount.FRAME
        t = lidar.transform.translation
        t.x, t.y, t.z = map(float, position)
        r = lidar.transform.rotation
        r.x, r.y, r.z, r.w = map(float, quat)
        self.tf.sendTransform([base, lidar])

    def publish_joints_imu(self, stamp, state):
        joints = JointState()
        joints.header.stamp = stamp
        joints.name = self.joint_names
        joints.position = [float(v) for v in state["q"]]
        joints.velocity = [float(v) for v in state["dq"]]
        self.joints_pub.publish(joints)
        imu = Imu()
        imu.header.stamp = stamp
        imu.header.frame_id = "imu_link"
        o = imu.orientation
        o.w, o.x, o.y, o.z = map(float, state["quat"])
        g = imu.angular_velocity
        g.x, g.y, g.z = map(float, state["gyro"])
        if "accel" in state:
            a = imu.linear_acceleration
            a.x, a.y, a.z = map(float, state["accel"])
        self.imu_pub.publish(imu)
