#!/usr/bin/env python3
"""The head depth camera (RealSense D435i) as ROS 2 topics: this node owns the device,
and any number of programs read the topics (hybrid_viewer.py --source ros, Nav2,
someone else's code) while it runs.

    source /opt/ros/foxy/setup.bash && export ROS_DOMAIN_ID=42 && python3 depth_node.py

Publishes, in d435_depth_optical_frame (z forward, x right, y down), stamped on arrival:
  /camera/depth/image_rect_raw   sensor_msgs/Image, 16UC1 in mm (848x480 at 15 fps)
  /camera/depth/camera_info      sensor_msgs/CameraInfo, nominal intrinsics
  /camera/depth/points           sensor_msgs/PointCloud2, x y z of every --step-th pixel
                                 from 0.2 m (as the sim's harp/depth_camera.py publishes)

Depth comes straight from /dev/video0 through V4L2 (camera_viewer.V4L2Stream), so
Unitree's videohub keeps the colour. The intrinsics are nominal (Intel's 87 x 58 deg
field of view), not the camera's own calibration. If another program has the camera,
it says so and tries again every 5 s. ROS_DOMAIN_ID must not be 0, Unitree's DDS
domain, where ROS 2 discovery runs out of memory.
"""
import argparse
import array
import math
import os
import sys
import time

# One BLAS thread: numpy's pool otherwise keeps 7 threads spinning on the Jetson's cores
os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")

import numpy as np
import rclpy
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField

from camera_viewer import HFOV, MIN_DEPTH, VFOV, V4L2Stream

FRAME = "d435_depth_optical_frame"
RETRY = 5.0   # s between tries while the camera is busy


def intrinsics(w, h):
    """fx, fy, cx, cy from the nominal field of view."""
    return ((w / 2) / math.tan(math.radians(HFOV / 2)), (h / 2) / math.tan(math.radians(VFOV / 2)),
            (w - 1) / 2, (h - 1) / 2)


class DepthNode:
    def __init__(self, node, size, fps, step):
        self.node, self.size, self.fps, self.step = node, size, fps, step
        self.image_pub = node.create_publisher(Image, "/camera/depth/image_rect_raw", 5)
        self.info_pub = node.create_publisher(CameraInfo, "/camera/depth/camera_info", 5)
        self.points_pub = node.create_publisher(PointCloud2, "/camera/depth/points", 5)
        fx, fy, cx, cy = intrinsics(*size)
        v, u = np.mgrid[0:size[1]:step, 0:size[0]:step]
        self.rays = np.stack([(u - cx) / fx, (v - cy) / fy], axis=-1).astype(np.float32)
        self.info = CameraInfo(width=size[0], height=size[1], distortion_model="plumb_bob",
                               d=[0.0] * 5, k=[fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0],
                               r=[1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
                               p=[fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0])
        self.fields = [PointField(name=c, offset=4 * i, datatype=PointField.FLOAT32, count=1)
                       for i, c in enumerate("xyz")]
        self.stream, self.last_count, self.next_try = None, 0, 0.0
        node.create_timer(0.5 / fps, self.tick)

    def tick(self):
        now = time.monotonic()
        if self.stream is None or (not self.stream.is_alive() and self.stream.why):
            if now >= self.next_try:
                self.open()
                self.next_try = now + RETRY
            return
        if self.stream.count == self.last_count:
            return
        self.last_count = self.stream.count
        frame = self.stream.latest()
        if frame is not None:
            self.publish(frame)

    def open(self):
        if self.stream is not None and self.stream.why:
            print(f"Not connected: {self.stream.why}; trying again in {RETRY:.0f} s.", flush=True)
        self.stream = V4L2Stream("/dev/video0", "Z16 ", self.size[0], self.size[1], self.fps, "<u2")
        self.stream.start()

    def publish(self, depth):
        stamp = self.node.get_clock().now().to_msg()
        h, w = depth.shape
        image = Image(height=h, width=w, encoding="16UC1", is_bigendian=0, step=w * 2)
        image.header.stamp, image.header.frame_id = stamp, FRAME
        image.data = array.array("B", depth.tobytes())  # array.array, not bytes: Foxy's rclpy checks bytes one by one (124 ms for 800 KB, 1 ms this way)
        self.image_pub.publish(image)
        self.info.header.stamp, self.info.header.frame_id = stamp, FRAME
        self.info_pub.publish(self.info)
        z = depth[::self.step, ::self.step].astype(np.float32) * 1e-3
        keep = z > MIN_DEPTH
        xyz = np.empty((int(keep.sum()), 3), np.float32)
        xyz[:, :2] = self.rays[keep] * z[keep, None]
        xyz[:, 2] = z[keep]
        cloud = PointCloud2(height=1, width=len(xyz), fields=self.fields, is_bigendian=False,
                            point_step=12, row_step=12 * len(xyz), is_dense=True)
        cloud.header.stamp, cloud.header.frame_id = stamp, FRAME
        cloud.data = array.array("B", xyz.tobytes())
        self.points_pub.publish(cloud)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--size", default="848x480", help="depth resolution, e.g. 640x480 or 1280x720")
    ap.add_argument("--fps", type=int, default=15, help="6, 15, 30, 60 or 90")
    ap.add_argument("--step", type=int, default=4, help="pixels between points in the cloud, each way")
    args = ap.parse_args()
    if os.environ.get("ROS_DOMAIN_ID", "0") == "0":
        sys.exit("Set ROS_DOMAIN_ID (not 0: Unitree's DDS domain, where ROS 2 discovery runs out of memory).")
    os.nice(10)   # below HARP's harness
    rclpy.init()
    node = rclpy.create_node("g1_depth_camera")
    size = tuple(int(v) for v in args.size.split("x"))
    depth = DepthNode(node, size, args.fps, args.step)
    print(f"Publishing the depth camera on ROS 2 (domain {os.environ['ROS_DOMAIN_ID']}): "
          "/camera/depth/image_rect_raw, camera_info, points.", flush=True)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if depth.stream is not None:
            depth.stream.close()
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
