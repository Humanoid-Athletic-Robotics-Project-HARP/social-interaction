"""The Runs page's listing of saved run folders, and opening them as replays."""
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import runs  # noqa: E402
from server import Dashboard  # noqa: E402

NAMES = ("2026-11-09_06-27-47Z_sim_crossing_a", "2026-11-09_06-39-31Z_sim_people_b",
         "2026-11-08_10-00-00Z_sim_one_box_c")


def make(folder, name, summary=None, events=(), view=True):
    run = Path(folder) / name
    run.mkdir()
    (run / "run.json").write_text(json.dumps({"world": name.split("_sim_")[1].rsplit("_", 1)[0],
                                              "tag": name[-1], "commit": "0123456789abc", "dirty": True}))
    if summary is not None:
        (run / "summary.json").write_text(json.dumps(summary))
    (run / "events.jsonl").write_text("".join(json.dumps(e) + "\n" for e in events))
    if view:
        (run / "view.jsonl").write_text("")
    return run


class Listing(unittest.TestCase):
    def setUp(self):
        self.folder = tempfile.mkdtemp()
        make(self.folder, NAMES[0], {"outcome": "inconclusive", "him_reason": "action_timeout"},
             [{"source": "predictive", "event": "started"}])
        make(self.folder, NAMES[1], {"outcome": "succeeded", "passed": True},
             [{"source": "people", "event": "started", "detector": "sim_people"},
              {"source": "approach", "event": "started"}, {"source": "behavior", "event": "loop_start"}])
        make(self.folder, NAMES[2], view=False)
        (Path(self.folder) / "not-a-run").mkdir()
        self.runs = runs.Runs(self.folder)

    def test_newest_first_with_what_each_run_had(self):
        found = self.runs.list()
        self.assertEqual([r["name"] for r in found], [NAMES[1], NAMES[0], NAMES[2]])
        people, crossing, box = found
        self.assertEqual(people["pieces"], ["approach stop", "sim_people", "behavior loop"])
        self.assertEqual((crossing["him_reason"], crossing["pieces"]), ("action_timeout", ["predictive avoidance"]))
        self.assertEqual((box["scored"], box["replay"], box["commit"]), (False, False, "012345678"))

    def test_only_listed_folder_names_resolve(self):
        self.assertIsNotNone(self.runs.path(NAMES[0]))
        for name in ("../" + NAMES[0], "not-a-run", NAMES[0] + "/view.jsonl", None, 7):
            self.assertIsNone(self.runs.path(name))

    def test_a_rescored_run_is_read_again(self):
        self.runs.list()
        (Path(self.folder) / NAMES[2] / "summary.json").write_text(json.dumps({"outcome": "violation"}))
        self.runs.listed = (0.0, [])
        self.assertEqual(self.runs.list()[2]["outcome"], "violation")


class Timing(unittest.TestCase):
    def test_seconds_late_states_and_memory_on_one_clock(self):
        folder = tempfile.mkdtemp()
        start = 1000.0
        run = make(folder, NAMES[0], {"outcome": "violation", "him_reason": "action_timeout"}, [
            {"source": "people", "event": "started", "wall": start - 5},
            {"source": "policy", "event": "odom", "wall": start, "sim_time": 0.0},
            {"source": "policy", "event": "timing", "wall": start + 1, "sim_time": 1.0, "n": 50,
             "age": 0.003, "reply": 0.001, "extra": 0.002, "gap": 0.03},
            {"source": "policy", "event": "late_state", "wall": start + 1.5, "sim_time": 1.5,
             "age": 0.078, "gap": 0.11, "missed": 1},
            {"source": "policy", "event": "timing", "wall": start + 2, "sim_time": 2.0, "n": 50,
             "age": 0.08, "reply": 0.002, "extra": 0.001, "gap": 0.11}])
        meta = json.loads((run / "run.json").read_text())
        meta["host_memory"] = {"min_free_mb": 60, "most_decompressed_mb_in_1s": 300,
                               "seconds": [[start - 50, 900, 300], [start + 1.4, 400, 70], ["bad"]]}
        (run / "run.json").write_text(json.dumps(meta))
        listed = runs.Runs(folder)
        self.assertTrue(listed.list()[0]["timing"])
        found = listed.timing(NAMES[0])
        self.assertEqual([s["t"] for s in found["seconds"]], [1.0, 2.0])
        self.assertEqual((found["seconds"][1]["age"], found["end"]), (80.0, 2.0))
        self.assertEqual(found["late"], [{"t": 1.5, "sim_time": 1.5, "age": 78.0, "gap": 110.0, "missed": 1}])
        self.assertEqual(found["memory"], [{"t": 1.4, "free_mb": 400, "paged_mb": 70}])   # before the policy: left out
        self.assertEqual((found["him_reason"], found["deadline_ms"], found["host"]["min_free_mb"]),
                         ("action_timeout", 200.0, 60))
        with self.assertRaises(KeyError):
            listed.timing("../" + NAMES[0])

    def test_a_run_without_timing_lines(self):
        folder = tempfile.mkdtemp()
        make(folder, NAMES[2], events=[{"source": "approach", "event": "started"}])
        listed = runs.Runs(folder)
        self.assertFalse(listed.list()[0]["timing"])
        found = listed.timing(NAMES[2])
        self.assertEqual((found["seconds"], found["late"], found["memory"], found["end"]), ([], [], [], None))


class Opening(unittest.TestCase):
    def test_at_most_two_open_and_configured_sources_stay(self):
        folder = tempfile.mkdtemp()
        for name in NAMES:
            make(folder, name)
        with patch.object(runs, "runs_folder", return_value=Path(folder)):
            dashboard = Dashboard({"sources": [{"id": "saved", "kind": "recording", "path": folder}]})
        ids = [dashboard.open_run(name) for name in NAMES]
        self.assertEqual(dashboard.opened, ids[1:])
        self.assertNotIn(ids[0], dashboard.sources)
        self.assertEqual(dashboard.open_run(NAMES[1]), ids[1])     # already open: same id
        dashboard.close_run("saved")                                 # configured: never closed here
        self.assertIn("saved", dashboard.sources)
        dashboard.close_run(ids[2])
        self.assertEqual(dashboard.opened, [ids[1]])
        with self.assertRaises(KeyError):
            dashboard.open_run("../etc")
        dashboard.close()


if __name__ == "__main__":
    unittest.main()
