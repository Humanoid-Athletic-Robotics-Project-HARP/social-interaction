"""Offline checks that resummarization leaves live recorder files attached."""

from contextlib import redirect_stdout
import io
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from harp_control import runlog
from harp_eval import run_recorder, run_summary, runs


class ResummarizeTests(unittest.TestCase):
    def make_run(self, parent, name, **meta):
        run = parent / name
        run.mkdir()
        run_summary.write_meta(run, {"kind": "sim", **meta})
        return run

    def test_explicit_unfinished_run_is_rejected_without_changing_events(self):
        with tempfile.TemporaryDirectory() as directory:
            run = self.make_run(Path(directory), "active", ended=None)
            path = run / "events.csv"
            path.write_text("t,source,event,details\n10,walker,policy_on,{}\n")
            original = path.read_bytes()
            inode = path.stat().st_ino
            with patch.object(runs, "find_run", return_value=run):
                with self.assertRaisesRegex(SystemExit, "stop its recorder"):
                    runs.cmd_summarize(SimpleNamespace(all=False, runs=["active"]))

            self.assertEqual(path.read_bytes(), original)
            self.assertEqual(path.stat().st_ino, inode)
            self.assertFalse((run / "summary.json").exists())

    def test_all_skips_live_recorder_and_summarizes_finished_runs(self):
        with tempfile.TemporaryDirectory() as directory:
            parent = Path(directory)
            active = self.make_run(parent, "active", ended=None)
            finished = self.make_run(parent, "finished", ended="2026-10-28T00:00:00Z")
            path = active / "events.csv"
            stream = run_recorder.CsvStream(path, runlog.EVENT_FIELDS)
            output = io.StringIO()
            try:
                stream.write([10, "walker", "policy_on", "{}"], flush=True)
                inode = path.stat().st_ino
                with patch.object(runlog, "runs_dir", return_value=parent), \
                        redirect_stdout(output):
                    runs.cmd_summarize(SimpleNamespace(all=True))
                stream.write([11, "walker", "stop", "{}"], flush=True)

                self.assertEqual(path.stat().st_ino, inode)
                self.assertIn("11,walker,stop", path.read_text())
                self.assertFalse((active / "summary.json").exists())
                self.assertTrue((finished / "summary.json").exists())
                self.assertIn("Skipping Unfinished run active", output.getvalue())
            finally:
                stream.close()

    def test_legacy_metadata_without_ended_can_still_be_summarized(self):
        with tempfile.TemporaryDirectory() as directory:
            run = self.make_run(Path(directory), "legacy")
            with patch.object(runs, "find_run", return_value=run), redirect_stdout(io.StringIO()):
                runs.cmd_summarize(SimpleNamespace(all=False, runs=["legacy"]))

            self.assertTrue((run / "summary.json").exists())


if __name__ == "__main__":
    unittest.main()
