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

from harp.approach_stop import xyz
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

    def test_the_tasks_speed_caps_the_planner(self):
        self.assertEqual(self.guard.planner.config.speed, .5)
        node = SimpleNamespace(create_publisher=lambda *a: SimpleNamespace(publish=lambda m: None),
                               create_subscription=lambda *a: None, create_timer=lambda *a: None,
                               get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=0)))
        with patch.dict(os.environ, {"G1_TASK_COMMAND": '{"avoidance":"predictive","speed":0.3}'}):
            slow = PredictiveAvoidance(node, SimpleNamespace(lookup_transform=None))
        self.assertEqual(slow.planner.config.speed, .3)

    def test_the_view_feed_reads_the_tracks_the_filter_publishes(self):
        try:
            from harp.view_feed import tracks_of
        except ImportError:
            self.skipTest("needs MuJoCo")
        from dataclasses import asdict
        from harp.obstacle_tracker import STRUCTURE_ID, Track
        person = Track(7, 1.234, -.5, .8, 0., .27, .1, 9.9, 12)
        wall = Track(STRUCTURE_ID + 3, 2., 2., 0., 0., .1, .1, 9.9, 40)
        # As PredictiveAvoidance.on_pending sends them on /harp/obstacle_tracks.
        text = json.dumps(dict(frame="odom", stamp=10., unresolved=[],
                               tracks=[dict(asdict(t), credit=None) for t in (person, wall)]))
        self.assertEqual(tracks_of(text), [[7, 1.23, -.5, .8, 0., .27]])

    def test_with_a_map_the_goal_is_planned_for_in_odom(self):
        self.event('goal_sent', x=3., y=1.)
        self.assertEqual(self.guard.goal_in_odom(), (3., 1.))      # no map: as sent
        self.guard.mapped, self.robot_x, self.robot_yaw = True, .5, math.pi/2
        self.lookups.clear()
        x, y = self.guard.goal_in_odom()
        self.assertEqual(self.lookups[0][:2], ("odom", "map"))
        self.assertAlmostEqual(x, .5-1.)
        self.assertAlmostEqual(y, 3.)

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

    def camera(self, points):
        """A head camera frame (the test's transforms put every frame at the robot)."""
        cloud = self.cloud(points)
        cloud.header.frame_id = 'd435_depth_optical_frame'
        return cloud

    def test_the_camera_shows_the_tracker_a_person_the_lidar_misses(self):
        self.fresh_active()
        legs = [[1.2, y, z] for y in (-.1, -.05, .05, .1) for z in (.2, .3, .4)]
        for _ in range(3):
            self.stamp += .1
            self.wall += .1
            self.guard.on_camera(self.camera(legs))
            self.guard.on_cloud(self.cloud([[2, 0, 0]]))   # the lidar: floor only
        tracks = self.guard.planning_tracks(self.guard.scan_stamp)
        self.assertTrue(any(abs(t.x-1.2) < .15 and abs(t.y) < .15 for t in tracks))

    def test_a_stale_or_untransformable_camera_frame_is_left_out(self):
        self.fresh_active()
        legs = [[1.2, y, .3] for y in (-.1, 0., .1)]
        self.guard.on_camera(self.camera(legs))
        self.assertIsNone(self.guard.camera_points(self.stamp + 1.))   # older than CAMERA_AGE
        lookup = self.guard.tf.lookup_transform
        self.guard.tf.lookup_transform = lambda *a: (_ for _ in ()).throw(TransformException('late TF'))
        self.assertIsNone(self.guard.camera_points(self.stamp))
        self.guard.tf.lookup_transform = lookup
        self.assertIsNotNone(self.guard.camera_points(self.stamp))

    def test_camera_rays_at_leg_height_clear_a_lost_person(self):
        from harp.predictive_avoidance import CAMERA_HEIGHTS
        self.fresh_active()
        self.stamp += .1
        self.guard.on_cloud(self.cloud([[1.5, 0, 1]]))
        self.stamp += 2.4
        self.wall += 2.4
        self.guard.on_cloud(self.cloud([[2, 0, 0]]))
        self.assertTrue(self.guard.unresolved)       # lost nearby: remembered
        # The camera's rays pass beyond where it was, at leg height (3 rays per check).
        lost = next(iter(self.guard.unresolved.values()))
        r = lost.radius
        rays = [np.array([lost.x+dx, lost.y+dy, z])*1.6 + jitter for z in CAMERA_HEIGHTS
                for dx, dy in ((0, 0), (r, 0), (-r, 0), (0, r), (0, -r))
                for jitter in ((0, 0, 0), (0, .002, 0), (0, -.002, 0))]
        self.stamp += .1
        self.wall += .1
        self.guard.on_camera(self.camera(rays))
        self.guard.on_cloud(self.cloud([[2, 0, 0]]))
        self.assertFalse(self.guard.unresolved)

    def test_the_camera_s_still_points_reach_nav2_s_static_cloud(self):
        self.fresh_active()
        self.stamp += .1
        self.guard.on_camera(self.camera([[1.0, .3, .4], [1.0, .3, .02]]))   # a box, and floor
        self.guard.on_cloud(self.cloud([[2, 0, 1]]))
        static = xyz(self.output['/harp/static_cloud'][-1])
        self.assertTrue(np.any(np.linalg.norm(static - [1.0, .3, .4], axis=1) < 1e-3))
        self.assertFalse(np.any(np.linalg.norm(static - [1.0, .3, .02], axis=1) < 1e-3))

    HEIGHTS = (.5, .98, 1., 1.02, 1.4)   # dense near 1 m, as a real scan's rings

    def corner(self):
        """A wall end: an L of returns, compact and not thin, so tracked as an object."""
        return ([[1.5, y, z] for y in np.arange(0., .51, .02) for z in self.HEIGHTS]
                + [[x, .5, z] for x in np.arange(1.52, 1.91, .02) for z in self.HEIGHTS])

    def lose_object_then_scan(self, points):
        """Track the corner, lose it past the tracker's memory, then scan points."""
        self.fresh_active()
        for _ in range(2):
            self.stamp += .1
            self.guard.on_cloud(self.cloud(self.corner()))
        self.stamp += 2.4
        self.wall += 2.4
        self.guard.on_cloud(self.cloud([[2, 0, 0]]))
        self.guard.on_odom(self.odom())
        self.guard.on_timer()
        self.assertTrue(self.guard.unresolved)
        self.stamp += .1
        self.wall += .1
        self.guard.on_cloud(self.cloud(points))

    def wall_to_corner(self):
        """The same corner seen again, now with the wall it ends (a long, tiled component)."""
        return self.corner() + [[1.5, y, z] for y in np.arange(-1.5, 0., .02) for z in self.HEIGHTS]

    def test_remembered_occupancy_measured_again_is_no_longer_unseen(self):
        # Wall ends and block corners were tracked as objects and lost when people passed
        # in front. No ray can pass beyond a wall, so about 30 stayed remembered in the
        # building at 0.5 m/s, each a phantom obstacle on the wall. Returns measured at
        # the remembered spot, with nothing in front, mean it is observed again.
        self.lose_object_then_scan(self.wall_to_corner())
        self.assertFalse(self.guard.unresolved)

    def test_remembered_occupancy_behind_a_nearer_return_stays(self):
        # Something nearer on the same rays: the remembered spot is still unseen.
        self.lose_object_then_scan(self.wall_to_corner() + [[.9, y, z] for y in np.arange(-.1, .66, .02)
                                                             for z in self.HEIGHTS])
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



class StaticCloud(unittest.TestCase):
    def test_only_returns_on_objects_seen_moving_leave_the_cloud_for_nav2(self):
        from harp.obstacle_tracker import STRUCTURE_ID, Track
        from harp.predictive_avoidance import cloud_message, static_mask
        from harp.approach_stop import xyz
        stamp = 2.
        tracks = [Track(1, 0., 0., .5, 0., .3, .05, stamp, 9),      # walking, seen now
                  Track(2, 3., 0., 0., 0., .3, .05, stamp, 9),      # standing
                  Track(3, 6., 0., .5, 0., .3, .05, stamp-.3, 9),   # walking, unseen now
                  Track(4, 9., 0., .5, 0., .9, .05, stamp, 9),      # large: merged or a wall
                  Track(STRUCTURE_ID, 12., 0., 0., 0., .2, .05, stamp, 9)]
        points = np.array([[x, .1, .5] for x in (0., 3., 6., 9., 12.)])
        keep = static_mask(points[:, :2], tracks, stamp)
        self.assertEqual(keep.tolist(), [False, True, True, True, True])
        message = cloud_message(PointCloud2().header, points[keep])
        np.testing.assert_allclose(xyz(message), points[keep], atol=1e-6)

    def test_a_still_object_with_a_noisy_velocity_stays_in_nav2s_map(self):
        # HIM atrium: a pillar seen in part had a filter velocity of 0.24-0.31 m/s for 5 s; it
        # left Nav2's map, Nav2 planned through it and the filter stood the robot for good.
        from harp.obstacle_tracker import Track
        from harp.predictive_avoidance import static_mask
        stamp = 2.
        tracks = [Track(1, 0., 0., .3, 0., .3, .05, stamp, 9),     # pillar: fast, not displaced
                  Track(2, 3., 0., .8, 0., .3, .05, stamp, 9)]     # walker: fast and displaced
        points = np.array([[0., .1], [3., .1]])
        self.assertEqual(static_mask(points, tracks, stamp, displaced={2}).tolist(), [True, False])
        self.assertEqual(static_mask(points, tracks, stamp).tolist(), [False, False])  # old rule

    def test_a_still_object_dragged_and_back_returns_to_nav2s_map(self):
        # HIM atrium: a walker passing a pillar dragged its track 0.25 m at 0.43 m/s and back;
        # seen moving for good, the pillar left Nav2's map and the robot stood at the gap 80 s.
        from harp.obstacle_tracker import Track
        from harp.predictive_avoidance import MotionHistory
        for home, after in ((True, set()), (False, {1})):   # home=False: the rule before
            history = MotionHistory(home)
            history.update([Track(1, 6.4, 2.2, 0., 0., .37, .05, 0., 9)], 0.)
            moved, displaced = history.update([Track(1, 6.2, 2.45, .43, 0., .37, .05, .1, 9)], .1)
            self.assertEqual((moved, displaced), ({1}, {1}))
            moved, displaced = history.update([Track(1, 6.41, 2.2, .02, 0., .37, .05, .2, 9)], .2)
            self.assertEqual((moved, displaced), (after, set()))

    def test_a_walker_who_stopped_elsewhere_stays_out_of_nav2s_map(self):
        # Letting a track still for 8 s back in had stopped people join Nav2's map, and Nav2
        # without a map detoured through unseen space (held-out worlds, Nov 3).
        from harp.obstacle_tracker import Track
        from harp.predictive_avoidance import MotionHistory
        history = MotionHistory()
        for k in range(10):   # walks 1 m, then stands a minute
            history.update([Track(3, .1*k, 0., 1., 0., .25, .05, .1*k, 9)], .1*k)
        for k in range(600):
            moved, _ = history.update([Track(3, .9, 0., 0., 0., .25, .05, 1.+.1*k, 9)], 1.+.1*k)
        self.assertEqual(moved, {3})

    def test_the_robots_own_body_never_reaches_nav2(self):
        # HIM's lidar sees the torso and arms up to 0.42 m away, just past Nav2's 0.4 m
        # footprint. Marked and inflated, they boxed in the robot's own cell, so Nav2's
        # planner found no path from anywhere ("failed to create plan").
        from harp.predictive_avoidance import SELF_RADIUS, static_mask
        base = np.array([5., 2.])
        points = base + np.array([[.42, 0.], [0., -.3], [.6, 0.], [-1., 0.]])
        keep = static_mask(points, [], 2., base)
        self.assertEqual(keep.tolist(), [False, False, True, True])
        self.assertGreater(SELF_RADIUS, .42)

if __name__ == '__main__':
    unittest.main()
