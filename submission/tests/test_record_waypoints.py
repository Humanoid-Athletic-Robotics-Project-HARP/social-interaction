"""Recorder data checks; no ROS runtime needed (PyYAML is required)."""

import math
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace as NS

import yaml

from harp.record_waypoints import MAX_AGE, Recorder
from harp.route import route_from_data


def odom(x=1.0, y=2.0, yaw=90.0, stamp=10.0):
    angle = math.radians(yaw) / 2
    return NS(header=NS(frame_id="odom", stamp=NS(sec=int(stamp), nanosec=int(stamp % 1 * 1e9))),
              child_frame_id="base_link", pose=NS(pose=NS(
                  position=NS(x=x, y=y, z=0.0),
                  orientation=NS(x=0.0, y=0.0, z=math.sin(angle), w=math.cos(angle)))))


class Recording(unittest.TestCase):
    def setUp(self):
        self.recorder = Recorder()

    def update(self, msg=None, received_at=1.0, ros_now_ns=10 * 10**9):
        self.recorder.update(msg or odom(), received_at, ros_now_ns)

    def test_record_undo_and_route_round_trip(self):
        self.update()
        self.assertEqual(self.recorder.record(1.1), [1.0, 2.0, 90.0])
        self.update(odom(x=-2.0, yaw=-45.0, stamp=10.1), ros_now_ns=10_100_000_000)
        self.recorder.record(1.1)
        self.assertEqual(self.recorder.undo(), [-2.0, 2.0, -45.0])
        with tempfile.TemporaryDirectory() as folder:
            output = Path(folder) / "route.yaml"
            self.recorder.save(output)
            route = route_from_data(yaml.safe_load(output.read_text()))
            self.assertEqual(route.current(), (1.0, 2.0, 90.0))
            self.assertEqual(route.laps, 1)
            self.assertIsNone(route.advance())

    def test_no_odometry_and_empty_save_are_refused(self):
        with self.assertRaises(ValueError):
            self.recorder.record(0)
        with tempfile.TemporaryDirectory() as folder:
            output = Path(folder) / "route.yaml"
            with self.assertRaises(ValueError):
                self.recorder.save(output)
            self.assertFalse(output.exists())
        with self.assertRaises(ValueError):
            self.recorder.undo()

    def test_stale_receipt_and_old_or_future_stamps_are_refused(self):
        self.update()
        with self.assertRaises(ValueError):
            self.recorder.record(1 + MAX_AGE + 0.01)
        for stamp in (0.0, 8.0, 12.0):
            recorder = Recorder()
            recorder.update(odom(stamp=stamp), 1.0, 10 * 10**9)
            with self.subTest(stamp=stamp), self.assertRaises(ValueError):
                recorder.record(1.1)

    def test_bad_latest_sample_invalidates_old_pose(self):
        for kind in ("frame", "child", "nan", "quaternion"):
            self.update()
            msg = odom(stamp=10.1)
            if kind == "frame":
                msg.header.frame_id = "map"
            elif kind == "child":
                msg.child_frame_id = "camera"
            elif kind == "nan":
                msg.pose.pose.position.x = math.nan
            else:
                msg.pose.pose.orientation.z = msg.pose.pose.orientation.w = 0.0
            self.update(msg, ros_now_ns=10_100_000_000)
            with self.subTest(kind=kind), self.assertRaises(ValueError):
                self.recorder.record(1.1)

    def test_clock_reset_requires_new_session(self):
        self.update()
        self.recorder.record(1.1)
        self.update(odom(stamp=9.0), ros_now_ns=9 * 10**9)
        self.update(odom(stamp=11.0), ros_now_ns=11 * 10**9)
        with self.assertRaises(ValueError):
            self.recorder.record(1.1)

    def test_existing_file_and_symlink_are_never_overwritten(self):
        self.update()
        self.recorder.record(1.1)
        with tempfile.TemporaryDirectory() as folder:
            output = Path(folder) / "route.yaml"
            output.write_text("original")
            with self.assertRaises(FileExistsError):
                self.recorder.save(output)
            self.assertEqual(output.read_text(), "original")
            link = Path(folder) / "link.yaml"
            link.symlink_to(output)
            with self.assertRaises(FileExistsError):
                self.recorder.save(link)
            self.assertEqual(output.read_text(), "original")
            self.assertEqual(sorted(p.name for p in Path(folder).iterdir()), ["link.yaml", "route.yaml"])


if __name__ == "__main__":
    unittest.main()
