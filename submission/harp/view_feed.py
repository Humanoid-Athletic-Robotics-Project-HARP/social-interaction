"""What a sim run looks like to the robot, a few times a second on the container's output,
for submission/lidar_view.py (Rerun) on the host: HIM's sim gives the container no network,
so its output is the only way out (docker logs -f, then HIM's policy.log). Its own process
at the lowest priority in a world's image (entrypoint.sh): the policy's process never
handles the cloud.

Each line is an event from source "view" (harp/events.py); course.py moves them out of
policy.log into the run's view.jsonl. First "world", once: the world's fixed obstacles
(geoms: type, size, pos, quat [w x y z], in the world). Then a "frame" every PERIOD s:
  truth     [x, y, z, qw, qx, qy, qz]  the pelvis in the world, from HIM's state
  odom      [x, y, yaw]  our leg odometry (/odom): what Nav2 sees
  lidar     [x, y, z, qw, qx, qy, qz]  livox_frame in base_link (/tf)
  points    the latest cloud in livox_frame, one point per VOXEL m cube, at most
            MAX_POINTS: int16 cm (x, y, z per point), zlib, base64
  people    {name: [x, y]}  each mover in the world (movers.py, from mover_start events)
  plan      [[x, y], ...]  Nav2's latest plan in odom (/plan), every PLAN_STEP-th pose
  costmap   every COSTMAP_EVERY frames: Nav2's local costmap in odom (origin [x, y],
            resolution, width, height, cells: OccupancyGrid values as uint8, zlib, base64)
"""

import base64
import json
import sys
import zlib
from pathlib import Path

import mujoco
import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry, Path as PlanPath
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage

from harp import events, lidar_mount
from harp.leg_odometry import yaw_of
from harp.lidar import TOPIC as CLOUD
from harp.movers import Movers

PERIOD = 0.25        # s (wall) between frames
VOXEL = 0.1          # m
MAX_POINTS = 3000
PLAN_STEP = 5
COSTMAP_EVERY = 4    # frames (Nav2 publishes the local costmap at 2 Hz)
WORLD = Path("/harp/model/world.xml")
SHAPES = {mujoco.mjtGeom.mjGEOM_BOX: "box", mujoco.mjtGeom.mjGEOM_CYLINDER: "cylinder",
          mujoco.mjtGeom.mjGEOM_SPHERE: "sphere", mujoco.mjtGeom.mjGEOM_CAPSULE: "capsule"}


def packed(array, dtype):
    return base64.b64encode(zlib.compress(np.ascontiguousarray(array, dtype=dtype).tobytes())).decode()


def thinned(cloud):
    """The cloud's xyz (float32 x, y, z first, as harp/lidar.py and the robot send it), one
    point per VOXEL cube, at most MAX_POINTS."""
    step = cloud.point_step // 4
    points = np.frombuffer(bytes(cloud.data), dtype=np.float32).reshape(-1, step)[:, :3]
    points = points[np.isfinite(points).all(axis=1)]
    _, first = np.unique(np.floor(points / VOXEL).astype(np.int32), axis=0, return_index=True)
    points = points[first]
    if len(points) > MAX_POINTS:
        points = points[np.random.default_rng().choice(len(points), MAX_POINTS, replace=False)]
    return points


def round_list(values, digits=3):
    return [round(float(v), digits) for v in values]


class ViewFeed:
    def __init__(self, node):
        self.node = node
        self.state = self.odom = self.lidar = self.cloud = self.plan = self.costmap = None
        self.frames = 0
        self.world = mujoco.MjModel.from_xml_path(str(WORLD)) if WORLD.is_file() else None
        self.world_data = self.world and mujoco.MjData(self.world)
        self.started = {}
        self.movers = self.world and Movers(self.world, started=self.started)
        best_effort = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        node.create_subscription(String, "/safety/policy_state", self.on_state, best_effort)
        node.create_subscription(String, events.TOPIC, self.on_event, events.latched())
        node.create_subscription(Odometry, "/odom", self.on_odom, 10)
        node.create_subscription(TFMessage, "/tf", self.on_tf, 50)
        node.create_subscription(PointCloud2, CLOUD, self.on_cloud, best_effort)
        node.create_subscription(PlanPath, "/plan", self.on_plan, 10)
        node.create_subscription(OccupancyGrid, "/local_costmap/costmap", self.on_costmap, 1)
        node.create_timer(PERIOD, self.on_timer)
        if self.world:
            events.emit("view", "world", geoms=self.fixed_geoms())

    def fixed_geoms(self):
        m, d = self.world, self.world_data
        mujoco.mj_kinematics(m, d)
        geoms = []
        for g in range(m.ngeom):
            if m.body_mocapid[m.geom_bodyid[g]] >= 0 or m.geom_type[g] not in SHAPES:
                continue
            quat = np.zeros(4)
            mujoco.mju_mat2Quat(quat, d.geom_xmat[g])
            geoms.append({"name": m.geom(g).name, "type": SHAPES[m.geom_type[g]],
                          "size": round_list(m.geom_size[g]), "pos": round_list(d.geom_xpos[g]),
                          "quat": round_list(quat, 4), "rgba": round_list(m.geom_rgba[g], 2)})
        return geoms

    def on_state(self, msg):
        self.state = msg.data      # parsed only when a frame goes out

    def on_event(self, msg):
        record = json.loads(msg.data)
        if record.get("event") == "mover_start" and self.world:
            self.started[record["mover"]] = record["starts_at"]
            self.movers = Movers(self.world, started=self.started)

    def on_odom(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        self.odom = round_list((p.x, p.y, yaw_of((q.w, q.x, q.y, q.z))))

    def on_tf(self, msg):
        for t in msg.transforms:
            if t.header.frame_id == "base_link" and t.child_frame_id == lidar_mount.FRAME:
                p, q = t.transform.translation, t.transform.rotation
                self.lidar = round_list((p.x, p.y, p.z, q.w, q.x, q.y, q.z), 4)

    def on_cloud(self, msg):
        self.cloud = msg            # thinned only when a frame goes out

    def on_plan(self, msg):
        poses = msg.poses[::PLAN_STEP] + msg.poses[-1:]
        self.plan = [round_list((p.pose.position.x, p.pose.position.y), 2) for p in poses]

    def on_costmap(self, msg):
        info = msg.info
        self.costmap = {"origin": round_list((info.origin.position.x, info.origin.position.y)),
                        "resolution": round(info.resolution, 4), "width": info.width,
                        "height": info.height, "cells": packed(np.array(msg.data, dtype=np.int8)
                                                               .view(np.uint8), np.uint8)}

    def on_timer(self):
        if self.state is None:
            return
        state = json.loads(self.state)
        t = state.get("sim_time")
        frame = {}
        if "position" in state:
            frame["truth"] = round_list(list(state["position"]) + list(state["quat"]), 4)
        if self.odom:
            frame["odom"] = self.odom
        if self.lidar:
            frame["lidar"] = self.lidar
        if self.cloud is not None:
            frame["points"] = packed(np.round(thinned(self.cloud) * 100), np.int16)
            self.cloud = None
        if self.movers and self.movers.names and t is not None:
            self.movers.apply(self.world_data, t)
            frame["people"] = {name: round_list(self.world_data.mocap_pos[m][:2])
                               for name, m in zip(self.movers.names, self.movers.mocap)}
        if self.plan is not None:
            frame["plan"], self.plan = self.plan, None
        if self.costmap is not None and self.frames % COSTMAP_EVERY == 0:
            frame["costmap"], self.costmap = self.costmap, None
        self.frames += 1
        events.emit("view", "frame", t, **frame)


def main():
    rclpy.init(args=sys.argv)
    node = rclpy.create_node("harp_view_feed")
    ViewFeed(node)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
