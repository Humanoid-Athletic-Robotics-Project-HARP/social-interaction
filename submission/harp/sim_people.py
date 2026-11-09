"""Stand-in for a person detector, for runs in HIM's simulator, until a real one exists.
Its own process in a world's image when the task is the behavior loop (entrypoint.sh).

The lidar finds person candidates by their shape; the camera confirms a person by their
legs. It takes every mover in the world (harp/movers.py: all of them are people so far)
where it is at that sim time, and every fixed geom a lidar shape test would take for a
person (a decoy: upright, CANDIDATE_WIDTH across and CANDIDATE_HEIGHT tall, standing on
the floor, such as a pillar), from the same copy of the world as the simulated lidar
(/harp/model/lidar_scene.xml) and its mover_start events, and the robot's true pose
from /safety/policy_state. Each PERIOD of sim time it publishes what the robot's real
sensors could find (context/g1-head-camera.md, context/seeing-ahead-research.md):
  - lidar, people and decoys: a point on them (BODY above the floor) in a direction its
    head leaves open (harp/lidar_mount.sees: with the face visor off, ahead and out past
    about 60 deg to the sides; with it on, only past about 60 deg), within LIDAR_RANGE,
    with a ray from the lidar reaching them first;
  - camera, people only (a decoy has no legs): the D435i is pitched about 48 deg down
    (harp/camera_mount.py), so it sees a person ahead only by their legs: a point LEGS
    above the floor at their centre inside its colour view (COLOUR_FOV, where a detector
    would run, narrower than depth), within CAMERA_RANGE, with a ray from the camera
    reaching them first.
So with the visor off, candidates are found ahead and to the sides, but not in the
wedges either side, and confirmed only within about 2 m ahead. Each is placed with NOISE.

/harp/people, JSON on std_msgs/String, every frame, people or not:
  {"frame": "base_link", "people": [{"id": "person", "x": 2.1, "y": -0.4, "seen_by": ["lidar"]}]}
x ahead and y left of the robot, in m; seen_by lists the sensors that see them now:
lidar (a candidate) and camera (a person). The id stays with a person or decoy, as a
tracker's would. At start, a "started" event (source "people", detector "sim_people") says
which detector this is, for viewers and recordings.
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
LIDAR_RANGE = 11.0   # m: where the lidar still finds a person indoors
BODY = (0.6, 1.0, 1.4)   # m above the floor: points on a person the lidar may see
LEGS = (0.1, 0.4, 0.7)   # m above the floor: points on a person the camera may see
COLOUR_FOV = (69.0, 42.0)   # deg across and down: the D435i's colour camera
CAMERA_RANGE = 2.0   # m: a detector finds people by their legs only this near
NOISE = 0.05      # m, 1 sigma, on x and y
CANDIDATE_WIDTH = (0.2, 0.7)    # m across: what a lidar shape test takes for a person
CANDIDATE_HEIGHT = (1.0, 2.1)   # m tall
SKIP = 0.4        # m of each ray left out: the robot's own head and arms


class SimPeople:
    def __init__(self, node, scene):
        self.node = node
        self.model = mujoco.MjModel.from_xml_path(scene)
        self.data = mujoco.MjData(self.model)
        self.movers = Movers(self.model)
        self.torso = self.model.body(lidar_mount.PARENT).id
        self.mount = lidar_mount.rotation()
        self.geoms = {self.model.body_mocapid[b]: self.geoms_of(b) for b in range(self.model.nbody)
                      if self.model.body_mocapid[b] >= 0}
        mujoco.mj_kinematics(self.model, self.data)
        self.decoys = [g for g in range(self.model.ngeom) if self.person_shaped(g)]
        self.rng = np.random.default_rng()
        self.pub = node.create_publisher(String, "/harp/people", 10)
        # Which detector publishes /harp/people (a real one says its own name the same way).
        events.Events(node, "people")("started", detector="sim_people", sensors=["lidar", "camera"],
                                      lidar_range=LIDAR_RANGE, camera_range=CAMERA_RANGE)
        node.create_subscription(String, events.TOPIC, self.on_event, events.latched())
        node.create_subscription(String, "/safety/policy_state", self.on_state,
                                 QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.next_frame = 0.0

    def geoms_of(self, body):
        return {g for g in range(self.model.ngeom) if self.model.geom_bodyid[g] == body}

    def person_shaped(self, g):
        """True if fixed geom g is upright, person-sized and on the floor (after kinematics)."""
        m = self.model
        if m.geom_bodyid[g] != 0 or m.geom_type[g] not in (mujoco.mjtGeom.mjGEOM_CYLINDER,
                                                             mujoco.mjtGeom.mjGEOM_BOX):
            return False
        size, half = m.geom_size[g], self.half_height(g)
        across = (2 * size[0], 2 * size[0] if m.geom_type[g] == mujoco.mjtGeom.mjGEOM_CYLINDER
                  else 2 * size[1])
        upright = self.data.geom_xmat[g].reshape(3, 3)[2, 2] > 0.99
        return (upright and all(CANDIDATE_WIDTH[0] <= w <= CANDIDATE_WIDTH[1] for w in across)
                and CANDIDATE_HEIGHT[0] <= 2 * half <= CANDIDATE_HEIGHT[1]
                and self.data.geom_xpos[g][2] - half < 0.3)

    def half_height(self, g):
        """Half the height of an upright cylinder or box geom."""
        return self.model.geom_size[g][1 if self.model.geom_type[g] == mujoco.mjtGeom.mjGEOM_CYLINDER else 2]

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
        found = [(self.movers.names[i], self.data.mocap_pos[m], 0.0, self.geoms[m], True)
                 for i, m in enumerate(self.movers.mocap)]
        found += [(self.model.geom(g).name or f"geom{g}", self.data.geom_xpos[g],
                   -self.half_height(g), {g}, False) for g in self.decoys]
        people = []
        for name, at, floor, geoms, person in found:
            base = at[2] + floor     # the floor under them: a decoy's position is its centre
            dx, dy = at[:2] - robot
            x, y = c * dx + s * dy, -s * dx + c * dy
            seen_by = []
            if math.hypot(x, y) <= LIDAR_RANGE and any(
                    self.lidar_sees(lidar, torso, np.array([at[0], at[1], base + h]), geoms) for h in BODY):
                seen_by.append("lidar")
            if person and any(self.in_view(camera, optical, np.array([at[0], at[1], h]))
                              and self.in_sight(camera, np.array([at[0], at[1], h]), geoms) for h in LEGS):
                seen_by.append("camera")
            if not seen_by:
                continue
            x, y = np.array([x, y]) + self.rng.normal(0.0, NOISE, 2)
            people.append({"id": name, "x": round(float(x), 3), "y": round(float(y), 3),
                           "seen_by": seen_by})
        return people

    def lidar_sees(self, lidar, torso, point, geoms):
        """True if the point (on a person or decoy, made of geoms) is in the lidar's field of
        view, where its head leaves it open, and nothing is in the way."""
        in_torso = torso.T @ (point - lidar)
        x, y, z = self.mount.T @ in_torso
        elevation = math.degrees(math.atan2(z, math.hypot(x, y)))
        return (lidar_mount.ELEVATION[0] <= elevation <= lidar_mount.ELEVATION[1]
                and lidar_mount.sees(in_torso)[0] and self.in_sight(lidar, point, geoms))

    @staticmethod
    def in_view(camera, optical, point):
        """True if the point is in the colour camera's view, within CAMERA_RANGE (optical
        axes: x right, y down, z ahead)."""
        x, y, z = optical.T @ (point - camera)
        return (z > 0 and math.hypot(*(point - camera)[:2]) <= CAMERA_RANGE
                and abs(math.degrees(math.atan2(x, z))) <= COLOUR_FOV[0] / 2
                and abs(math.degrees(math.atan2(y, z))) <= COLOUR_FOV[1] / 2)

    def in_sight(self, origin, point, geoms):
        """True if a ray from origin to the point (on a person or decoy) hits one of its geoms
        first."""
        to = point - origin
        length = np.linalg.norm(to)
        if length <= SKIP:
            return True
        to /= length
        geom = np.array([-1], dtype=np.int32)
        mujoco.mj_ray(self.model, self.data, origin + SKIP * to, to, None, 1, self.torso, geom)
        return geom[0] in geoms


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
