"""Stand-in for a person detector on the head camera (Intel RealSense D435i), for runs in
HIM's simulator, until a real one exists. Its own process in a world's image when the task
is the behavior loop (entrypoint.sh); on the robot nothing publishes /harp/people yet, so
harp/behavior.py only wanders there.

It takes every mover in the world (harp/movers.py: all of them are people so far) where
it is at that sim time, from the same copy of the world as the simulated lidar
(/harp/model/lidar_scene.xml) and its mover_start events, and the robot's true pose
from /safety/policy_state. Each PERIOD of sim time it publishes what a detector would
see: the people in the camera's field of view (FOV wide, out to RANGE), not hidden behind
something (one ray from the head to them, at head height), placed with NOISE.

/harp/people, JSON on std_msgs/String, every frame, people or not:
  {"frame": "base_link", "people": [{"id": "person", "x": 2.1, "y": -0.4}]}
x ahead and y left of the robot, in m, as a detector with depth gives them. The id
stays with a person, as a tracker's would.
"""

import json
import math
import sys

import mujoco
import numpy as np
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from harp import lidar_mount
from harp.leg_odometry import yaw_of
from harp.movers import Movers

SCENE = "/harp/model/lidar_scene.xml"
PERIOD = 0.2      # s of sim time between frames (detectors run at about 5-15 Hz)
FOV = 87.0        # deg wide: the D435i's colour camera
RANGE = 4.0       # m: the detector we want places people out to 4 m (README)
NOISE = 0.05      # m, 1 sigma, on x and y
SKIP = 0.4        # m of the ray to a person left out: the robot's own head and arms


class SimPeople:
    def __init__(self, node, scene):
        self.node = node
        self.model = mujoco.MjModel.from_xml_path(scene)
        self.data = mujoco.MjData(self.model)
        self.movers = Movers(self.model)
        self.torso = self.model.body(lidar_mount.PARENT).id
        self.bodies = {self.model.body_mocapid[b]: b for b in range(self.model.nbody)
                       if self.model.body_mocapid[b] >= 0}
        self.rng = np.random.default_rng()
        self.pub = node.create_publisher(String, "/harp/people", 10)
        latched = QoSProfile(depth=1000, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        node.create_subscription(String, "/harp/events", self.on_event, latched)
        node.create_subscription(String, "/safety/policy_state", self.on_state,
                                 QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.next_frame = 0.0

    def on_event(self, msg):
        """The lidar's mover_start events: when each mover sets off, as in its world."""
        try:
            record = json.loads(msg.data)
        except ValueError:
            return
        if isinstance(record, dict) and record.get("event") == "mover_start":
            self.movers.started[self.movers.names.index(record["mover"])] = record["starts_at"]

    def on_state(self, msg):
        state = json.loads(msg.data)
        t = state.get("sim_time")
        if t is None or "position" not in state or t < self.next_frame:
            return
        self.next_frame = t + PERIOD
        qpos = np.concatenate([state["position"], state["quat"], state["q"]])
        self.data.qpos[:len(qpos)] = qpos
        self.movers.apply(self.data, t)
        mujoco.mj_kinematics(self.model, self.data)
        people = self.seen(np.asarray(state["position"][:2]), yaw_of(state["quat"]))
        self.pub.publish(String(data=json.dumps({"frame": "base_link", "people": people})))

    def seen(self, robot, yaw):
        torso = self.data.xmat[self.torso].reshape(3, 3)
        head = self.data.xpos[self.torso] + torso @ lidar_mount.POSITION
        c, s = math.cos(yaw), math.sin(yaw)
        people = []
        for i, mocap in enumerate(self.movers.mocap):
            dx, dy = self.data.mocap_pos[mocap][:2] - robot
            x, y = c * dx + s * dy, -s * dx + c * dy
            if math.hypot(x, y) > RANGE or abs(math.degrees(math.atan2(y, x))) > FOV / 2:
                continue
            if not self.in_sight(head, self.data.mocap_pos[mocap], self.bodies[mocap]):
                continue
            x, y = np.array([x, y]) + self.rng.normal(0.0, NOISE, 2)
            people.append({"id": self.movers.names[i], "x": round(float(x), 3), "y": round(float(y), 3)})
        return people

    def in_sight(self, head, person, body):
        """True if a level ray from the head to the person hits them first."""
        to = np.array([person[0] - head[0], person[1] - head[1], 0.0])
        length = np.linalg.norm(to)
        if length <= SKIP:
            return True
        to /= length
        geom = np.array([-1], dtype=np.int32)
        mujoco.mj_ray(self.model, self.data, head + SKIP * to, to, None, 1, self.torso, geom)
        return geom[0] >= 0 and self.model.geom_bodyid[geom[0]] == body


def main():
    rclpy.init(args=sys.argv)
    node = rclpy.create_node("sim_people")
    SimPeople(node, SCENE)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
