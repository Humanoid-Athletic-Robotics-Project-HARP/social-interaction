"""submission/generate_world.py: random held-out worlds."""
import os
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import mujoco  # noqa: E402
import generate_world  # noqa: E402
from harp.movers import Movers  # noqa: E402


class GeneratedWorld(unittest.TestCase):
    def test_a_seed_gives_one_loadable_reachable_world(self):
        with tempfile.TemporaryDirectory() as folder:
            a, goal, n = generate_world.write(5, "a", folder)
            b, _, _ = generate_world.write(5, "b", folder)
            self.assertEqual(a.read_text().replace(" a ", " ").split("-->")[1],
                             b.read_text().replace(" b ", " ").split("-->")[1])   # same seed, same world
            self.assertTrue((Path(folder) / "a.yaml").is_file())
            model = mujoco.MjModel.from_xml_path(str(a))
            movers = Movers(model)
            self.assertEqual(len(movers.names), n)
            self.assertGreaterEqual(n, 7)
            self.assertTrue(all(abs(v) <= 10 for v in goal))   # inside Nav2's 20 m rolling map
            self.assertEqual(model.site("goal").pos[:2].round(2).tolist(), [round(v, 2) for v in goal])

    def test_reachability_sees_a_closed_doorway(self):
        wall = [generate_world.Box("w", 2., 0., .1, 5., 1.)]
        bounds = (-1., 4., -2., 2.)
        self.assertFalse(generate_world.reachable(wall, [], bounds, (0., 0.), (3., 0.)))
        gap = [generate_world.Box("w0", 2., -1.5, .1, 1., 1.), generate_world.Box("w1", 2., 1.5, .1, 1., 1.)]
        self.assertTrue(generate_world.reachable(gap, [], bounds, (0., 0.), (3., 0.)))   # a 1 m door
        self.assertFalse(generate_world.reachable(gap, [(2., 0., .2)], bounds, (0., 0.), (3., 0.)))


if __name__ == "__main__":
    unittest.main()
