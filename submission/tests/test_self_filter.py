"""The lidar self-filter (harp/self_filter.py) against rays cast like the sim lidar's.

HIM's G1 model (/harp/model in the image) stands on a floor beside a post, posed by joint
angles, and rays from the head lidar hit it, as harp/lidar.py casts them (the head's own
geoms skipped, 2 cm range noise). The filter must drop the robot's hits and keep the
post's and the floor's, in the stand pose and with the arms where they reach farthest.
Skipped where the model or MuJoCo is missing.
"""

import unittest
from pathlib import Path

import numpy as np

try:
    import mujoco
except ImportError:
    mujoco = None

from harp import lidar_mount, wave

MODELS = (Path("/harp/model/g1_robot.xml"),
          Path(__file__).resolve().parents[1] / ".build/him/environment/g1_robot.xml")
MODEL = next((p for p in MODELS if p.exists()), None)
RAYS = 200_000
NOISE = 0.02


def pose(right_arm=None, left_arm=None):
    """The model's stand pose with the given arm joints (7 each)."""
    model = mujoco.MjModel.from_xml_path(str(MODEL))
    q = model.key("stand").qpos[7:36].copy()
    if left_arm is not None:
        q[15:22] = left_arm
    if right_arm is not None:
        q[22:29] = right_arm
    return q


@unittest.skipIf(mujoco is None or MODEL is None, "needs MuJoCo and HIM's G1 model")
class AgainstCastRays(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        from harp.self_filter import SelfFilter
        cls.filter = SelfFilter(MODEL)
        try:
            spec = mujoco.MjSpec.from_file(str(MODEL))
        except TypeError:           # MuJoCo 3.2 (the image's): from_file fills a spec
            spec = mujoco.MjSpec()
            spec.from_file(str(MODEL))
        # A post 0.6 m to the robot's right, through sensor height: inside the 0.6 m range
        # cut and close to the radii (0.45 m) that stand in for a filter elsewhere.
        for name, kind, size, pos in (("floor", mujoco.mjtGeom.mjGEOM_PLANE, (5, 5, 0.1), (0, 0, 0)),
                                      ("post", mujoco.mjtGeom.mjGEOM_BOX, (0.05, 0.05, 0.9), (0, -0.6, 0.9))):
            geom = spec.worldbody.add_geom()
            geom.name, geom.type, geom.size, geom.pos = name, kind, size, pos
        cls.model = spec.compile()
        cls.data = mujoco.MjData(cls.model)
        cls.stand = cls.model.key("stand").qpos.copy()
        cls.rng = np.random.default_rng(0)

    def cast(self, q):
        """Points (lidar frame) and what each hit: 'robot', 'floor' or 'post'."""
        m, d = self.model, self.data
        d.qpos[:] = self.stand
        d.qpos[7:36] = q
        mujoco.mj_kinematics(m, d)
        torso_id = m.body(lidar_mount.PARENT).id
        az = self.rng.uniform(-np.pi, np.pi, RAYS)
        el = np.radians(self.rng.uniform(*lidar_mount.ELEVATION, RAYS))
        rays = np.stack([np.cos(el) * np.cos(az), np.cos(el) * np.sin(az), np.sin(el)], axis=1)
        torso = d.xmat[torso_id].reshape(3, 3)
        origin = d.xpos[torso_id] + torso @ lidar_mount.POSITION
        geomid, dist = np.zeros(RAYS, np.int32), np.zeros(RAYS)
        mujoco.mj_multiRay(m, d, origin, (rays @ (torso @ lidar_mount.rotation()).T).ravel(),
                           None, True, torso_id, geomid, dist, RAYS, 40.0)
        hit = geomid >= 0
        points = rays[hit] * (dist[hit] + self.rng.normal(0, NOISE, hit.sum()))[:, None]
        names = np.array(["post" if m.geom(g).name == "post" else "floor" if m.geom(g).name == "floor"
                          else "robot" for g in geomid[hit]])
        return points, names

    def check(self, q, max_robot=0.003):
        points, names = self.cast(q)
        keep = self.filter.keep(points, [q])
        robot = names == "robot"
        self.assertGreater(robot.sum(), 500, "the lidar should see the arms")
        self.assertLessEqual(keep[robot].mean(), max_robot, "robot points got through")
        self.assertEqual(keep[names == "post"].mean(), 1.0, "post points were dropped")
        self.assertGreater(keep[names == "floor"].mean(), 0.999, "floor points were dropped")
        return points, names, keep

    def test_stand_pose(self):
        self.check(pose())

    def test_wave_hand_up(self):
        self.check(pose(right_arm=wave.HAND_UP))

    def test_arms_reaching_out(self):
        self.check(pose(right_arm=(-1.5, -1.2, 0, 0, 0, 0, 0), left_arm=(-1.5, 1.2, 0, 0, 0, 0, 0)))

    def test_a_scan_over_a_moving_arm_takes_every_pose(self):
        """A scan smeared over an arm moving fast (part of the wave's raise): one pose's
        boxes miss the arm where it was, the scan's poses together don't."""
        raise_ = [pose(right_arm=np.multiply(wave.HAND_UP, s)) for s in (0.3, 0.5, 0.7)]
        casts = [self.cast(q) for q in raise_]
        points = np.concatenate([p for p, _ in casts])
        robot = np.concatenate([n for _, n in casts]) == "robot"
        self.assertGreater(self.filter.keep(points, raise_[1:2])[robot].mean(), 0.01)
        self.assertLessEqual(self.filter.keep(points, raise_)[robot].mean(), 0.003)

    def test_no_points_and_points_far_away(self):
        self.assertEqual(len(self.filter.keep(np.zeros((0, 3)), [pose()])), 0)
        self.assertTrue(self.filter.keep([[5.0, 0.0, 0.0], [0.0, 3.0, 1.0]], [pose()]).all())


if __name__ == "__main__":
    unittest.main()
