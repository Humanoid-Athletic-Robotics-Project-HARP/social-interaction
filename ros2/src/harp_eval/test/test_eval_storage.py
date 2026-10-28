"""Offline checks for recorder inputs and reader-safe run files."""

import csv
import json
import math
import os
import tempfile
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from harp_control import runlog
from harp_eval import run_recorder, run_summary


class ActiveRecordingTests(unittest.TestCase):
    def test_replay_preserves_the_manual_run_used_by_no_ros_programs(self):
        with tempfile.TemporaryDirectory() as directory:
            runs = Path(directory)
            manual, replay = runs / "manual", runs / "replay"
            manual.mkdir()
            replay.mkdir()
            run_recorder.mark_active(runs, manual, "robot")
            marker = (runs / runlog.ACTIVE).read_text()

            run_recorder.mark_active(runs, replay, "replay")

            self.assertEqual((runs / runlog.ACTIVE).read_text(), marker)
            with patch.object(runlog, "runs_dir", return_value=runs), \
                    patch.dict(os.environ, {"HARP_RUN_DIR": ""}):
                self.assertEqual(runlog.active_run(), manual)

    def test_manual_sim_and_robot_sessions_publish_their_own_marker(self):
        with tempfile.TemporaryDirectory() as directory:
            runs = Path(directory)
            for kind in ("sim", "robot"):
                with self.subTest(kind=kind):
                    run = runs / f"manual-{kind}"
                    run.mkdir()

                    run_recorder.mark_active(runs, run, kind)

                    name, pid = (runs / runlog.ACTIVE).read_text().split()
                    self.assertEqual((name, int(pid)), (run.name, os.getpid()))
                    with patch.object(runlog, "runs_dir", return_value=runs), \
                            patch.dict(os.environ, {"HARP_RUN_DIR": ""}):
                        self.assertEqual(runlog.active_run(), run)


class EventInputTests(unittest.TestCase):
    def test_malformed_fields_survive_recording_and_summary_parsing(self):
        bad = [
            {"t": None, "source": "walker", "event": "error", "details": {}},
            {"t": "bad", "source": "walker", "event": "error", "details": {}},
            {"t": float("nan"), "source": "walker", "event": "error", "details": {}},
            {"t": 10, "source": "goal", "event": "goal_result", "details": None},
            {"t": 10, "source": "goal", "event": "goal_result", "details": []},
            {"t": 10, "source": ["walker"], "event": "error", "details": {}},
            {"t": 10, "source": "walker", "event": {"bad": "error"}, "details": {}},
        ]
        with tempfile.TemporaryDirectory() as directory:
            stream = run_recorder.CsvStream(Path(directory) / "events.csv", runlog.EVENT_FIELDS)
            node = SimpleNamespace(events=stream)
            try:
                for record in bad:
                    run_recorder.RunRecorder.on_event(node, SimpleNamespace(data=json.dumps(record)))
            finally:
                stream.close()
            events = run_summary.merge_events(directory)

        self.assertEqual(len(events), len(bad))
        self.assertTrue(all(math.isfinite(e["t"]) for e in events))
        self.assertTrue(all(e["event"] == "unparsed" for e in events))
        self.assertTrue(all("data" in e["details"] for e in events))
        self.assertEqual(run_summary.outcome(events)[0], "stopped")

    def test_missing_fields_keep_existing_defaults(self):
        with tempfile.TemporaryDirectory() as directory:
            stream = run_recorder.CsvStream(Path(directory) / "events.csv", runlog.EVENT_FIELDS)
            try:
                run_recorder.RunRecorder.on_event(SimpleNamespace(events=stream),
                                                  SimpleNamespace(data='{"event":"note"}'))
            finally:
                stream.close()
            events = run_summary.merge_events(directory)

        self.assertEqual(events[0]["source"], "unknown")
        self.assertEqual(events[0]["event"], "note")
        self.assertEqual(events[0]["details"], {})

    def test_old_malformed_csv_is_recoverable_and_merge_is_repeatable(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "events.csv"
            with path.open("w", newline="") as file:
                writer = csv.DictWriter(file, fieldnames=runlog.EVENT_FIELDS)
                writer.writeheader()
                writer.writerow({"t": "bad", "source": "walker", "event": "error",
                                 "details": "null"})
                writer.writerow({"t": 20, "source": "goal", "event": "goal_result",
                                 "details": '{"status":"SUCCEEDED"}'})
            first = run_summary.merge_events(directory)
            second = run_summary.merge_events(directory)

        self.assertEqual(first, second)
        self.assertEqual(run_summary.outcome(second), ("succeeded", "SUCCEEDED"))
        self.assertEqual(sum(e["event"] == "unparsed" for e in second), 1)


class AtomicRunFilesTests(unittest.TestCase):
    def test_metadata_readers_see_old_document_until_replacement(self):
        with tempfile.TemporaryDirectory() as directory:
            run = Path(directory)
            run_summary.write_meta(run, {"tag": "old", "result": None})
            old = (run / "run.yaml").read_text()
            replace = run_summary.os.replace
            seen = []

            def observe(source, target):
                self.assertEqual(Path(target).read_text(), old)
                seen.append(run_summary.read_meta(run))
                replace(source, target)

            with patch.object(run_summary.os, "replace", side_effect=observe):
                run_summary.write_meta(run, {"tag": "new", "result": {"outcome": "succeeded"}})

            self.assertEqual(seen, [{"tag": "old", "result": None}])
            self.assertEqual(run_summary.read_meta(run)["tag"], "new")
            self.assertEqual([p.name for p in run.iterdir()], ["run.yaml"])

    def test_failed_replacement_preserves_existing_file_and_removes_temporary(self):
        with tempfile.TemporaryDirectory() as directory:
            run = Path(directory)
            run_summary.write_meta(run, {"tag": "old"})
            with patch.object(run_summary.os, "replace", side_effect=OSError("disk error")):
                with self.assertRaises(OSError):
                    run_summary.write_meta(run, {"tag": "new"})

            self.assertEqual(run_summary.read_meta(run), {"tag": "old"})
            self.assertEqual([p.name for p in run.iterdir()], ["run.yaml"])

    def test_recorder_uses_same_metadata_replacement(self):
        with tempfile.TemporaryDirectory() as directory:
            node = SimpleNamespace(dir=Path(directory), meta={"tag": "new"})
            with patch.object(run_summary, "write_meta") as write:
                run_recorder.RunRecorder.write_meta(node)
            write.assert_called_once_with(node.dir, node.meta)

    def test_failed_event_merge_retains_both_original_inputs(self):
        with tempfile.TemporaryDirectory() as directory:
            run = Path(directory)
            original = "t,source,event,details\n10,walker,policy_on,{}\n"
            part = "t,source,event,details\n11,walker,stop,{}\n"
            (run / "events.csv").write_text(original)
            (run / "events-walker.csv").write_text(part)
            with patch.object(run_summary.os, "replace", side_effect=OSError("disk error")):
                with self.assertRaises(OSError):
                    run_summary.merge_events(run)

            self.assertEqual((run / "events.csv").read_text(), original)
            self.assertEqual((run / "events-walker.csv").read_text(), part)

    def test_summary_readers_keep_previous_complete_json_until_replacement(self):
        with tempfile.TemporaryDirectory() as directory:
            run = Path(directory) / "run"
            run.mkdir()
            run_summary.write_meta(run, {"kind": "sim", "tag": "new"})
            path = run / "summary.json"
            path.write_text('{"tag":"old"}')
            replace = run_summary.os.replace
            seen = []

            def observe(source, target):
                if Path(target) == path:
                    seen.append(json.loads(path.read_text()))
                replace(source, target)

            with patch.object(run_summary.os, "replace", side_effect=observe), \
                    patch.object(run_summary, "nav_tree", return_value={}), \
                    patch.object(run_summary, "criteria_for", return_value=(None, None)):
                run_summary.summarize(run)

            self.assertEqual(seen, [{"tag": "old"}])
            self.assertEqual(json.loads(path.read_text())["tag"], "new")


if __name__ == "__main__":
    unittest.main()
