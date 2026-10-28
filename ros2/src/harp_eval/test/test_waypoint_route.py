"""Offline checks for waypoint route state."""

from pathlib import Path
import tempfile
import unittest

import yaml

from harp_eval.waypoint_route import WaypointRoute, load_route


class WaypointRouteTests(unittest.TestCase):
    def test_initial_state(self):
        route = WaypointRoute(["A", "B", "C"], laps=2)

        self.assertEqual(route.waypoints, ("A", "B", "C"))
        self.assertEqual(route.lap_limit, 2)
        self.assertEqual(route.index, 0)
        self.assertEqual(route.completed_laps, 0)
        self.assertEqual(route.current(), "A")

    def test_reading_current_does_not_advance(self):
        route = WaypointRoute(["A", "B"], laps=2)

        for _ in range(3):
            self.assertEqual(route.current(), "A")

        self.assertEqual(route.index, 0)
        self.assertEqual(route.completed_laps, 0)

    def test_route_is_independent_of_original_list_mutation(self):
        original = ["A", "B"]
        route = WaypointRoute(original, laps=1)

        original[0] = "changed"
        original.append("C")

        self.assertEqual(route.waypoints, ("A", "B"))
        self.assertEqual(route.current(), "A")

    def test_tuple_pose_values_are_preserved(self):
        poses = ((2.0, 0.0, 90.0), (2.0, 2.0, 180.0))
        route = WaypointRoute(poses, laps=1)

        self.assertEqual(route.waypoints, poses)
        self.assertEqual(route.current(), (2.0, 0.0, 90.0))

    def test_nonempty_iterator_is_accepted(self):
        route = WaypointRoute(iter(["A", "B"]), laps=1)

        self.assertEqual(route.waypoints, ("A", "B"))
        self.assertEqual(route.current(), "A")

    def test_empty_route_is_rejected(self):
        for waypoints in ([], (), iter(())):
            with self.subTest(kind=type(waypoints).__name__):
                with self.assertRaises(ValueError):
                    WaypointRoute(waypoints, laps=1)

    def test_invalid_lap_limits_are_rejected(self):
        for laps in (0, -1, 1.5, 2.0, "2", True, False, None):
            with self.subTest(laps=laps):
                with self.assertRaises(ValueError):
                    WaypointRoute(["A"], laps=laps)

    def test_strings_and_bytes_are_not_routes(self):
        for waypoints in ("ABC", "", b"ABC", b""):
            with self.subTest(waypoints=waypoints):
                with self.assertRaises(TypeError):
                    WaypointRoute(waypoints, laps=1)

    def test_two_laps_visit_targets_in_order(self):
        route = WaypointRoute(["A", "B", "C"], laps=2)
        visited = []

        for expected in ("A", "B", "C", "A", "B", "C"):
            self.assertEqual(route.current(), expected)
            visited.append(route.current())
            route.advance()

        self.assertEqual(visited, ["A", "B", "C", "A", "B", "C"])
        self.assertEqual(route.completed_laps, 2)
        self.assertIsNone(route.current())

    def test_advance_returns_next_target_and_counts_a_lap(self):
        route = WaypointRoute(["A", "B"], laps=2)

        self.assertEqual(route.advance(), "B")
        self.assertEqual(route.completed_laps, 0)
        self.assertEqual(route.advance(), "A")
        self.assertEqual(route.completed_laps, 1)
        self.assertEqual(route.index, 0)

    def test_completed_route_stays_complete(self):
        route = WaypointRoute(["A"], laps=1)

        self.assertIsNone(route.advance())
        for _ in range(3):
            self.assertIsNone(route.current())
            self.assertIsNone(route.advance())
        self.assertEqual(route.completed_laps, 1)
        self.assertEqual(route.index, 0)

    def test_single_waypoint_can_repeat_for_several_laps(self):
        route = WaypointRoute(["A"], laps=3)

        self.assertEqual(route.advance(), "A")
        self.assertEqual(route.advance(), "A")
        self.assertIsNone(route.advance())
        self.assertEqual(route.completed_laps, 3)


class LoadRouteTests(unittest.TestCase):
    def load_text(self, text):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "route.yaml"
            path.write_text(text, encoding="utf-8")
            return load_route(path)

    def load_document(self, **overrides):
        document = {"frame": "odom", "laps": 2, "waypoints": [[2, 0], [2, 2, 90]]}
        document.update(overrides)
        return self.load_text(yaml.safe_dump(document))

    def test_loads_waypoint_pairs_and_triples_in_degrees(self):
        route = self.load_document()

        self.assertEqual(route.waypoints, ((2.0, 0.0, 0.0), (2.0, 2.0, 90.0)))
        self.assertEqual(route.lap_limit, 2)
        self.assertEqual(route.current(), (2.0, 0.0, 0.0))
        self.assertIsNone(route.criteria)

    def test_omitted_laps_default_to_one(self):
        route = self.load_text("frame: odom\nwaypoints: [[1, 0], [0, 0]]\n")
        self.assertEqual(route.lap_limit, 1)

    def test_valid_criteria_are_preserved(self):
        criteria = {"must_succeed": True, "max_time_s": 120, "min_clearance_m": 0.2,
                    "max_contacts": 0, "max_recoveries": 2, "max_end_error_m": 0.4}
        route = self.load_document(criteria=criteria)

        self.assertEqual(route.criteria, criteria)

    def test_null_criteria_mean_no_criteria(self):
        self.assertIsNone(self.load_document(criteria=None).criteria)

    def test_non_mapping_documents_are_rejected(self):
        for text in ("", "null", "[]", "42", "a string"):
            with self.subTest(text=text):
                with self.assertRaises(ValueError):
                    self.load_text(text)

    def test_malformed_or_unsafe_yaml_is_rejected(self):
        for text in ("frame: [", "!!python/tuple [1, 2]"):
            with self.subTest(text=text):
                with self.assertRaises(ValueError):
                    self.load_text(text)

    def test_odom_frame_is_required(self):
        for frame in (None, "map", "base_link", 1):
            with self.subTest(frame=frame):
                with self.assertRaises(ValueError):
                    self.load_document(frame=frame)
        with self.assertRaises(ValueError):
            self.load_text("laps: 1\nwaypoints: [[0, 0]]\n")

    def test_loader_rejects_invalid_laps(self):
        for laps in (None, 0, -1, True, 1.0, "2"):
            with self.subTest(laps=laps):
                with self.assertRaises(ValueError):
                    self.load_document(laps=laps)

    def test_waypoint_list_is_required(self):
        for points in (None, [], {}, "0 0", 1):
            with self.subTest(points=points):
                with self.assertRaises(ValueError):
                    self.load_document(waypoints=points)

    def test_waypoints_require_two_or_three_numbers(self):
        for point in (None, "1 2", {}, 4, [], [1], [1, 2, 3, 4]):
            with self.subTest(point=point):
                with self.assertRaises(ValueError):
                    self.load_document(waypoints=[point])

    def test_nonfinite_nonnumeric_and_boolean_values_are_rejected(self):
        for bad in (float("nan"), float("inf"), -float("inf"), "1", None, True):
            for column in range(3):
                point = [1, 2, 90]
                point[column] = bad
                with self.subTest(bad=bad, column=column):
                    with self.assertRaises(ValueError):
                        self.load_document(waypoints=[point])

    def test_invalid_criteria_are_rejected(self):
        invalid = [[], "pass", {"unknown": 1}, {"must_succeed": 1},
                   {"max_contacts": True}, {"max_contacts": 0.5},
                   {"max_contacts": -1}, {"max_recoveries": 1.0},
                   {"max_recoveries": -1}, {"max_time_s": -1},
                   {"max_time_s": float("inf")}, {"min_clearance_m": float("nan")},
                   {"min_clearance_m": True}, {"max_end_error_m": "0.2"},
                   {"max_end_error_m": -0.1}]
        for criteria in invalid:
            with self.subTest(criteria=criteria):
                with self.assertRaises(ValueError):
                    self.load_document(criteria=criteria)


if __name__ == "__main__":
    unittest.main()
