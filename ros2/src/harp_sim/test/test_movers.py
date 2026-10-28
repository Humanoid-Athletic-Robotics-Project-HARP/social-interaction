"""Geometry-only mover checks: no simulation stepping, viewer, ROS, or DDS."""

from pathlib import Path
import unittest

import mujoco
import numpy as np

from harp_sim.movers import Movers


def model(duration="2", delay=None, trips=None):
    extra = "".join(f'<numeric name="person_{name}" data="{value}"/>'
                    for name, value in (("delay", delay), ("trips", trips)) if value is not None)
    return mujoco.MjModel.from_xml_string(f"""
        <mujoco><custom><numeric name="person_duration" data="{duration}"/>{extra}</custom>
        <worldbody><body name="person" mocap="true">
          <geom type="sphere" size=".2"/>
        </body><site name="person_end" pos="2 0 0"/></worldbody></mujoco>
    """)


class MoverTests(unittest.TestCase):
    def test_position_goes_to_end_and_back_without_stepping_simulation(self):
        m = model()
        movers, data = Movers(m), mujoco.MjData(m)
        for t, x in ((0, 0), (1, 1), (2, 2), (3, 1), (4, 0)):
            movers.apply(data, t)
            np.testing.assert_allclose(data.mocap_pos[0], [x, 0, 0])

    def test_delayed_one_trip_mover_waits_then_stops_at_end(self):
        m = model(delay="3", trips="1")
        movers, data = Movers(m), mujoco.MjData(m)
        data.time = 4
        self.assertEqual(movers.update(data, released=False, goal_sent=True), [])
        self.assertEqual(movers.update(data, released=True, goal_sent=True), [("person", 7)])
        for t, x in ((6, 0), (8, 1), (9, 2), (20, 2)):
            movers.apply(data, t)
            np.testing.assert_allclose(data.mocap_pos[0], [x, 0, 0])

    def test_duration_must_be_positive(self):
        for duration in ("0", "-1"):
            with self.subTest(duration=duration), self.assertRaisesRegex(SystemExit, "person_duration"):
                Movers(model(duration=duration))

    def test_delay_and_trip_count_cannot_be_negative(self):
        for name in ("delay", "trips"):
            with self.subTest(name=name), self.assertRaisesRegex(SystemExit, f"person_{name}"):
                Movers(model(**{name: "-1"}))

    def test_nonfinite_mover_values_are_rejected_before_positions_are_written(self):
        for name in ("duration", "delay", "trips"):
            for value in (float("nan"), float("inf"), -float("inf")):
                with self.subTest(name=name, value=value):
                    m = model(delay="0", trips="1")
                    i = m.numeric("person_" + name).id
                    m.numeric_data[m.numeric_adr[i]] = value
                    with self.assertRaisesRegex(SystemExit, "person_" + name):
                        Movers(m)

    def test_numeric_setting_must_have_one_value(self):
        with self.assertRaisesRegex(SystemExit, "person_duration"):
            Movers(model(duration="2 3"))

    def test_shipped_world_mover_metadata_is_valid(self):
        worlds = Path(__file__).resolve().parents[1] / "worlds"
        for world in worlds.glob("*.xml"):
            with self.subTest(world=world.name):
                Movers(mujoco.MjModel.from_xml_path(str(world)))


if __name__ == "__main__":
    unittest.main()
