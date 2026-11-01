"""ROS message contracts and failure gates for the predictive Nav2 filter."""
import json
import math
import os
import subprocess
import sys
import tempfile
import time
import unittest
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
from tf2_ros import TransformException

from harp.predictive_avoidance import PredictiveAvoidance


def set_stamp(message, stamp):
    message.header.stamp.sec = int(stamp)
    message.header.stamp.nanosec = round((stamp-int(stamp))*1e9)


class Adapter(unittest.TestCase):
    def setUp(self):
        self.stamp, self.wall = 10.0, 100.0
        self.output, self.lookups = {}, []
        self.robot_x = self.robot_yaw = 0.0

        def publisher(kind, topic, qos):
            self.output[topic] = []
            return SimpleNamespace(publish=self.output[topic].append)

        def lookup(target, source, at):
            self.lookups.append((target, source, at.nanoseconds))
            transform = TransformStamped()
            transform.transform.translation.x = self.robot_x
            transform.transform.rotation.z = math.sin(self.robot_yaw/2)
            transform.transform.rotation.w = math.cos(self.robot_yaw/2)
            return transform

        node = SimpleNamespace(
            create_publisher=publisher, create_subscription=lambda *a: None,
            create_timer=lambda *a: None,
            get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(
                nanoseconds=round(self.stamp*1e9))))
        self.guard = PredictiveAvoidance(node, SimpleNamespace(lookup_transform=lookup))
        self.mock_clock = patch('harp.predictive_avoidance.time.monotonic',
                                side_effect=lambda: self.wall)
        self.mock_clock.start()
        self.addCleanup(self.mock_clock.stop)

    def cloud(self, points=None):
        points = [[2, 0, 1]] if points is None else points
        cloud = PointCloud2(height=1, width=len(points), point_step=12,
                            row_step=12*len(points))
        cloud.header.frame_id = 'livox_frame'
        set_stamp(cloud, self.stamp)
        cloud.fields = [PointField(name=name, offset=4*i,
                                  datatype=PointField.FLOAT32, count=1)
                        for i, name in enumerate(('x', 'y', 'z'))]
        cloud.data = np.asarray(points, dtype='<f4').tobytes()
        return cloud

    def odom(self):
        message = Odometry()
        message.header.frame_id, message.child_frame_id = 'odom', 'base_link'
        set_stamp(message, self.stamp)
        message.pose.pose.orientation.w = 1.0
        return message

    def event(self, event, **fields):
        self.guard.on_event(String(data=json.dumps(dict(source='goal', event=event, **fields))))

    def fresh_active(self):
        self.guard.on_cloud(self.cloud([[2, 0, 0]]))  # Valid floor-only scan: no obstacles.
        self.guard.on_odom(self.odom())
        command = Twist()
        command.linear.x = .2
        self.guard.on_nav(command)
        self.event('goal_sent')
        self.guard.planner = SimpleNamespace(plan=lambda *args: SimpleNamespace(
            command=(.1, 0, 0), mode='follow', alert=False,solver='test',clearance=1.))

    def last_command(self):
        message = self.output['/cmd_vel'][-1]
        return (message.linear.x, message.linear.y, message.angular.z)

    def test_stamp_time_transform_compensates_translation_and_turn(self):
        self.guard.on_cloud(self.cloud())
        self.stamp += .1
        self.robot_x, self.robot_yaw = .3, .4
        # Fixed world object observed from a translated, rotated robot.
        self.guard.on_cloud(self.cloud([[1.7*math.cos(.4), -1.7*math.sin(.4), 1]]))
        track = self.guard.tracker.tracks_at(self.stamp)[0]
        self.assertAlmostEqual(track.x, 2, places=6)
        self.assertAlmostEqual(track.y, 0, places=6)
        self.assertAlmostEqual(track.vx, 0, places=6)
        self.assertAlmostEqual(track.vy, 0, places=6)
        self.assertEqual(self.lookups, [
            ('odom', 'livox_frame', 10000000000), ('odom', 'base_link', 10000000000),
            ('odom', 'livox_frame', 10100000000), ('odom', 'base_link', 10100000000)])

    def test_each_stale_receipt_stops(self):
        for field in ('scan_receipt', 'odom_receipt', 'nav_receipt'):
            with self.subTest(field=field):
                self.fresh_active()
                self.guard.on_timer()
                self.assertGreater(self.last_command()[0], 0)
                setattr(self.guard, field, self.wall-.6)
                self.guard.on_timer()
                self.assertEqual(self.last_command(), (0, 0, 0))
                self.assertEqual(self.guard.mode, 'stale_input')
                self.stamp += .1

    def test_old_observation_timestamp_blocks_despite_fresh_receipt(self):
        self.fresh_active()
        self.guard.scan_stamp = self.stamp - .6
        self.guard.on_timer()
        self.assertEqual(self.last_command(), (0, 0, 0))

    def test_lost_nearby_obstacle_cannot_expire_into_clear_space(self):
        self.fresh_active()
        self.stamp += .1
        self.guard.on_cloud(self.cloud([[1.5, 0, 1]]))
        # Fresh floor/background returns do not prove an occluded object left.
        for age in (2.4, 3.0):
            self.stamp += age
            self.wall += age
            self.guard.on_cloud(self.cloud([[2, 0, 0]]))
            self.guard.on_odom(self.odom())
            command = Twist()
            command.linear.x = .2
            self.guard.on_nav(command)
            self.guard.on_timer()
            # Expired geometry remains an explicit obstacle for both route/MPC.
            tracks = self.guard.planning_tracks(self.guard.scan_stamp)
            self.assertTrue(any(abs(t.x-1.5) < .1 for t in tracks))
            self.assertTrue(self.guard.unresolved)

    def test_brief_occlusion_keeps_occupancy_without_permanent_latch(self):
        self.fresh_active()
        self.stamp += .1
        self.guard.on_cloud(self.cloud([[1.5, 0, 1]]))
        self.stamp += .7
        self.guard.on_cloud(self.cloud([[2, 0, 0]]))
        self.assertFalse(self.guard.perception_failed)
        tracks = self.guard.tracker.tracks_at(self.guard.scan_stamp)
        self.assertEqual(len(tracks), 1)
        self.assertGreater(tracks[0].uncertainty, .2)

    def test_failed_route_latches_and_success_goes_idle(self):
        self.fresh_active()
        self.event('goal_result', status='SUCCEEDED')
        self.guard.on_timer()
        self.assertEqual(self.last_command(), (0, 0, 0))
        self.assertFalse(self.guard.failed)
        self.event('goal_sent')
        self.guard.on_timer()
        self.assertGreater(self.last_command()[0], 0)
        self.event('goal_result', status='FAILED')
        self.event('goal_sent')
        self.guard.on_timer()
        self.assertEqual(self.last_command(), (0, 0, 0))
        self.assertTrue(self.guard.failed)

    def test_malformed_cloud_invalidates_prior_scan_immediately(self):
        self.fresh_active()
        self.stamp += .1
        cloud = self.cloud()
        cloud.data = b'truncated'
        self.guard.on_cloud(cloud)
        self.assertIsNone(self.guard.scan_receipt)
        self.assertEqual(self.last_command(), (0, 0, 0))
        self.assertEqual(self.guard.mode, 'unusable_scan')

    def test_invalid_odometry_cannot_renew_feedback_and_nan_command_stops(self):
        self.fresh_active()
        receipt, stamp = self.guard.odom_receipt, self.guard.odom_stamp
        self.wall += .1
        self.stamp += .1
        odom = self.odom()
        odom.twist.twist.linear.x = float('nan')
        self.guard.on_odom(odom)
        self.assertEqual((self.guard.odom_receipt, self.guard.odom_stamp), (receipt, stamp))
        command = Twist()
        command.linear.y = float('nan')
        self.guard.on_nav(command)
        self.assertIsNone(self.guard.nav_receipt)
        self.assertEqual(self.last_command(), (0, 0, 0))

    def test_solver_overrun_rejects_nonzero_result(self):
        self.fresh_active()

        def slow(*args):
            self.wall += .2
            return SimpleNamespace(command=(.1, 0, 0), mode='follow', alert=False,solver='test',clearance=1.)

        self.guard.planner = SimpleNamespace(plan=slow)
        self.guard.on_timer()
        self.assertEqual(self.last_command(), (0, 0, 0))
        self.assertEqual(self.guard.mode, 'planning_deadline')
        alert = json.loads(self.output['/harp/obstacle_alert'][-1].data)
        self.assertTrue(alert['active'])


    def test_tf_arrival_retries_original_stamp_and_expires_pending_scan(self):
        lookup = self.guard.tf.lookup_transform
        self.guard.tf.lookup_transform = lambda *a: (_ for _ in ()).throw(TransformException('late TF'))
        cloud = self.cloud()
        self.guard.on_cloud(cloud)
        self.assertIs(self.guard.pending_cloud, cloud)
        self.assertIsNone(self.guard.scan_receipt)
        self.stamp += .1
        self.guard.tf.lookup_transform = lookup
        self.guard.on_pending()
        self.assertIsNone(self.guard.pending_cloud)
        self.assertEqual(self.guard.scan_stamp, 10)
        self.assertEqual(self.lookups[0][2], 10000000000)
        previous_receipt = self.guard.scan_receipt
        self.guard.tf.lookup_transform = lambda *a: (_ for _ in ()).throw(TransformException('late TF'))
        self.guard.on_cloud(self.cloud())
        self.stamp += .4
        self.wall += .4
        self.guard.tf.lookup_transform = lookup
        self.guard.on_pending()
        self.assertIsNone(self.guard.pending_cloud)
        self.assertEqual(self.guard.scan_receipt, previous_receipt)
        self.assertEqual(self.guard.scan_stamp, 10)


@unittest.skipUnless(os.environ.get('ROS_LOCALHOST_ONLY') == '1'
                     and os.environ.get('ROS_DOMAIN_ID', '0') != '0',
                     'requires an isolated local ROS test domain')
class RosTransport(unittest.TestCase):
    """Real DDS smoke test; synthetic publishers only, no robot or motor interface."""
    def setUp(self):
        rclpy.init()
        self.node = rclpy.create_node('test_predictive_inputs')
        self.odom_pub = self.node.create_publisher(Odometry, '/odom', 10)
        self.tf_pub = self.node.create_publisher(TFMessage, '/tf', 10)
        self.cloud_pub = self.node.create_publisher(PointCloud2, '/utlidar/cloud_livox_mid360', 10)
        self.nav_pub = self.node.create_publisher(Twist, '/cmd_vel_nav', 10)
        latched = QoSProfile(depth=1000, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.event_pub = self.node.create_publisher(String, '/harp/events', latched)
        self.received = []
        self.node.create_subscription(Twist, '/cmd_vel', self.on_command, 10)
        self.log = tempfile.TemporaryFile(mode='w+t')
        self.process = subprocess.Popen([sys.executable, '-u', '-m', 'harp.predictive_avoidance'],
                                        stdout=self.log, stderr=subprocess.STDOUT)
        self.clouds = True
        self.result = None
        self.previous_stamp = None

    def on_command(self, message):
        self.received.append((time.monotonic(), message.linear.x,
                              message.linear.y, message.angular.z))

    def tearDown(self):
        if self.process.poll() is None:
            self.process.terminate()
        self.process.wait(timeout=5)
        self.log.close()
        self.node.destroy_node()
        rclpy.shutdown()

    def publish(self):
        stamp = self.node.get_clock().now().to_msg()
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id, odom.child_frame_id = 'odom', 'base_link'
        odom.pose.pose.orientation.w = 1.0
        self.odom_pub.publish(odom)
        transforms = []
        for child in ('base_link', 'livox_frame'):
            transform = TransformStamped()
            transform.header.stamp = stamp
            transform.header.frame_id, transform.child_frame_id = 'odom', child
            transform.transform.rotation.w = 1.0
            transforms.append(transform)
        self.tf_pub.publish(TFMessage(transforms=transforms))
        if self.clouds and self.previous_stamp is not None:
            cloud = PointCloud2(height=1, width=1, point_step=12, row_step=12)
            cloud.header.frame_id, cloud.header.stamp = 'livox_frame', self.previous_stamp
            cloud.fields = [PointField(name=n, offset=i*4, datatype=PointField.FLOAT32, count=1)
                            for i, n in enumerate(('x', 'y', 'z'))]
            cloud.data = np.array([[2, 0, 0]], dtype='<f4').tobytes()
            self.cloud_pub.publish(cloud)
        self.previous_stamp = stamp
        nav = Twist()
        nav.linear.x = .2
        self.nav_pub.publish(nav)
        event = dict(source='goal', event='goal_sent') if self.result is None else dict(
            source='goal', event='goal_result', status=self.result)
        self.event_pub.publish(String(data=json.dumps(event)))
        rclpy.spin_once(self.node, timeout_sec=.03)

    def wait_for(self, predicate, timeout=10):
        deadline = time.monotonic()+timeout
        while time.monotonic() < deadline and self.process.poll() is None:
            self.publish()
            if predicate():
                return
        self.log.seek(0)
        self.fail('ROS predictive smoke test timed out:\n'+self.log.read())

    def assert_stays_zero(self, duration=0.7):
        start = time.monotonic()
        self.wait_for(lambda: time.monotonic()-start >= duration, timeout=duration+2)
        recent = [values[1:] for values in self.received if values[0] >= start]
        self.assertGreaterEqual(len(recent), 2)
        self.assertTrue(all(values == (0, 0, 0) for values in recent), recent)

    def test_motion_cloud_dropout_recovery_and_failed_route(self):
        self.wait_for(lambda: any(value[1] > .01 for value in self.received))
        self.clouds = False
        lost_at = time.monotonic()
        self.wait_for(lambda: any(value[0] > lost_at+.7 and value[1:] == (0, 0, 0)
                                 for value in self.received))
        self.assert_stays_zero()
        self.clouds = True
        restored_at = time.monotonic()
        self.wait_for(lambda: any(value[0] > restored_at and value[1] > .01
                                 for value in self.received))
        self.result = 'FAILED'
        failed_at = time.monotonic()
        self.wait_for(lambda: any(value[0] > failed_at+.3 and value[1:] == (0, 0, 0)
                                 for value in self.received))
        self.result = None  # Late goal and fresh sensors must not clear the failure latch.
        self.assert_stays_zero()


if __name__ == '__main__':
    unittest.main()
