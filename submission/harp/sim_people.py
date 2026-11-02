"""Stand-in for a person detector, for runs in HIM's simulator, until a real one exists.
Its own process in a world's image when the task is the behavior loop (entrypoint.sh).

It takes every mover in the world (harp/movers.py: all of them are people so far) where
it is at that sim time, from the same copy of the world as the simulated lidar
(/harp/model/lidar_scene.xml) and its mover_start events, and the robot's true pose
from /safety/policy_state. Each PERIOD of sim time it publishes the people the robot's
real sensors could find (context/g1-head-camera.md, context/seeing-ahead-research.md):
  - lidar: more than LIDAR_BLIND off the nose (the face visor blinds it ahead), out to
    LIDAR_RANGE, with a level ray from the lidar reaching them first;
  - camera: the D435i is pitched about 48 deg down (harp/camera_mount.py), so it sees a
    person ahead only by their legs: a point LEGS above the floor at their centre inside
    its colour view (COLOUR_FOV, where a detector would run, narrower than depth), within
    CAMERA_RANGE, with a ray from the camera reaching them first.
So people ahead are found only within about 2 m, and others off to the sides. Each is
placed with NOISE.

/harp/people, JSON on std_msgs/String, every frame, people or not:
  {"frame": "base_link", "people": [{"id": "person", "x": 2.1, "y": -0.4, "seen_by": "lidar"}]}
x ahead and y left of the robot, in m; seen_by is lidar or camera. The id stays with a
person, as a tracker's would.
"""

import json
import math
import sys

import mujoco
import numpy as np
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from harp import camera_mount, events, lidar_mount
from harp.leg_odometry import yaw_of
from harp.movers import Movers

SCENE = "/harp/model/lidar_scene.xml"
PERIOD = 0.2      # s of sim time between frames (detectors run at about 5-15 Hz)
LIDAR_BLIND = 60.0   # deg each side of the nose where the lidar finds nobody (visor on)
LIDAR_RANGE = 11.0   # m: where the lidar still finds a person indoors
LEGS = (0.1, 0.4, 0.7)   # m above the floor: points on a person the camera may see
COLOUR_FOV = (69.0, 42.0)   # deg across and down: the D435i's colour camera
CAMERA_RANGE = 2.0   # m: a detector finds people by their legs only this near
NOISE = 0.05      # m, 1 sigma, on x and y
SKIP = 0.4        # m of each ray left out: the robot's own head and arms


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
        node.create_subscription(String, events.TOPIC, self.on_event, events.latched())
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
        torso_at = self.data.xpos[self.torso]
        torso = self.data.xmat[self.torso].reshape(3, 3)
        lidar = torso_at + torso @ lidar_mount.POSITION
        camera, optical = torso_at + torso @ camera_mount.POSITION, torso @ camera_mount.rotation()
        c, s = math.cos(yaw), math.sin(yaw)
        people = []
        for i, mocap in enumerate(self.movers.mocap):
            at, body = self.data.mocap_pos[mocap], self.bodies[mocap]
            dx, dy = at[:2] - robot
            x, y = c * dx + s * dy, -s * dx + c * dy
            if (abs(math.degrees(math.atan2(y, x))) > LIDAR_BLIND and math.hypot(x, y) <= LIDAR_RANGE
                    and self.in_sight(lidar, np.array([at[0], at[1], lidar[2]]), body)):
                by = "lidar"
            elif any(self.in_view(camera, optical, np.array([at[0], at[1], h]))
                     and self.in_sight(camera, np.array([at[0], at[1], h]), body) for h in LEGS):
                by = "camera"
            else:
                continue
            x, y = np.array([x, y]) + self.rng.normal(0.0, NOISE, 2)
            people.append({"id": self.movers.names[i], "x": round(float(x), 3), "y": round(float(y), 3),
                           "seen_by": by})
        return people

    @staticmethod
    def in_view(camera, optical, point):
        """True if the point is in the colour camera's view, within CAMERA_RANGE (optical
        axes: x right, y down, z ahead)."""
        x, y, z = optical.T @ (point - camera)
        return (z > 0 and math.hypot(*(point - camera)[:2]) <= CAMERA_RANGE
                and abs(math.degrees(math.atan2(x, z))) <= COLOUR_FOV[0] / 2
                and abs(math.degrees(math.atan2(y, z))) <= COLOUR_FOV[1] / 2)

    def in_sight(self, origin, point, body):
        """True if a ray from origin to the point (on the person) hits them first."""
        to = point - origin
        length = np.linalg.norm(to)
        if length <= SKIP:
            return True
        to /= length
        geom = np.array([-1], dtype=np.int32)
        mujoco.mj_ray(self.model, self.data, origin + SKIP * to, to, None, 1, self.torso, geom)
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
