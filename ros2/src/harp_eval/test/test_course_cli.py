"""Check course preflight and launch arguments without starting ROS programs."""

from contextlib import ExitStack, redirect_stderr, redirect_stdout
import io
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from harp_eval import course


class CourseCliTests(unittest.TestCase):
    def setUp(self):
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        self.root = Path(directory.name)
        self.runs = self.root / "recorded runs"

    def write_route(self, text=None):
        path = self.root / "source route.yaml"
        path.write_text(text if text is not None else
                        "frame: odom\nlaps: 2\nwaypoints:\n  - [1, 0]\n  - [0, 0, 180]\n",
                        encoding="utf-8")
        return path

    def assert_preflight_rejected(self, args, message):
        errors = io.StringIO()
        with patch.object(course.sys, "argv", ["course", *args]), \
                patch.object(course.subprocess, "call") as launch, \
                patch.object(course, "claim_domains") as domains, \
                patch.object(course.runlog, "runs_dir") as runs_dir, \
                redirect_stderr(errors):
            with self.assertRaises(SystemExit) as stopped:
                course.main()

        self.assertEqual(stopped.exception.code, 2)
        self.assertIn(message, errors.getvalue())
        launch.assert_not_called()
        domains.assert_not_called()
        runs_dir.assert_not_called()

    def invoke(self, args, summary=None, launch_code=0, recorder_started=True):
        if summary is None:
            summary = {"outcome": "succeeded", "passed": True}

        def fake_launch(command, *, env):
            if recorder_started:
                (Path(env["HARP_RUN_DIR"]) / "run.yaml").write_text("{}\n", encoding="utf-8")
            return launch_code

        with ExitStack() as stack:
            stack.enter_context(patch.object(course.sys, "argv", ["course", *args]))
            stack.enter_context(patch.object(course.runlog, "runs_dir", return_value=self.runs))
            stack.enter_context(patch.object(course, "claim_domains",
                                             return_value=(self.root / "slot.lock", 10, 11)))
            stack.enter_context(patch.object(course, "wait_while_walking"))
            stack.enter_context(patch.object(course.signal, "signal"))
            launch = stack.enter_context(patch.object(course.subprocess, "call",
                                                      side_effect=fake_launch))
            summarize = stack.enter_context(patch.object(course.run_summary, "summarize",
                                                         return_value=summary))
            stack.enter_context(patch.object(course.run_summary, "one_screen",
                                             return_value="mock summary"))
            stack.enter_context(redirect_stdout(io.StringIO()))
            result = course.main()

        launch.assert_called_once()
        return result, launch.call_args, summarize

    def test_invalid_route_yaml_is_rejected_before_launch(self):
        path = self.write_route("frame: [")
        self.assert_preflight_rejected(["open_floor", "--route", str(path)],
                                       "invalid route YAML")

    def test_goal_and_route_are_mutually_exclusive(self):
        path = self.write_route()
        self.assert_preflight_rejected(
            ["open_floor", "--route", str(path), "--goal", "1", "0"],
            "not allowed with argument")

    def test_nonfinite_or_nonpositive_timeout_is_rejected_before_launch(self):
        for value in ("nan", "inf", "-inf", "0", "-1"):
            with self.subTest(timeout=value):
                self.assert_preflight_rejected(["open_floor", f"--timeout={value}"],
                                               "--timeout must be positive and finite")

    def test_nonfinite_goal_values_are_rejected_before_launch(self):
        for goal in (("nan", "0"), ("0", "inf"), ("0", "0", "nan")):
            with self.subTest(goal=goal):
                self.assert_preflight_rejected(["open_floor", "--goal", *goal],
                                               "--goal values must be finite")

    def test_route_is_copied_and_its_snapshot_is_passed_to_launch(self):
        source = self.write_route()
        result, call, summarize = self.invoke(["open_floor", "--route", str(source),
                                               "--tag", "route-test"])

        self.assertEqual(result, 0)
        command = call.args[0]
        self.assertEqual(command[:4], ["ros2", "launch", "harp_eval", "course.launch.py"])
        values = dict(argument.split(":=", 1) for argument in command[4:])
        run = Path(call.kwargs["env"]["HARP_RUN_DIR"])
        copied = run / "route.yaml"
        self.assertEqual(copied.read_text(encoding="utf-8"), source.read_text(encoding="utf-8"))
        self.assertEqual(values["route"], str(copied))
        self.assertNotEqual(values["route"], str(source))
        self.assertNotIn("goal", values)
        self.assertEqual(values["sim_domain"], "10")
        self.assertEqual(call.kwargs["env"]["ROS_DOMAIN_ID"], "11")
        summarize.assert_called_once_with(run)

    def test_failed_outcome_or_criteria_returns_failure(self):
        for summary in ({"outcome": "failed", "passed": None},
                        {"outcome": "timed_out", "passed": False},
                        {"outcome": "succeeded", "passed": False}):
            with self.subTest(summary=summary):
                result, _, _ = self.invoke(["open_floor"], summary=summary)
                self.assertEqual(result, 1)

    def test_launch_failure_returns_failure_even_with_successful_summary(self):
        result, _, _ = self.invoke(["open_floor"], launch_code=1)
        self.assertEqual(result, 1)

    def test_missing_recorder_metadata_returns_failure(self):
        result, _, summarize = self.invoke(["open_floor"], recorder_started=False)
        self.assertEqual(result, 1)
        summarize.assert_not_called()

    def test_legacy_single_goal_launch_arguments_are_preserved(self):
        result, call, _ = self.invoke(["open_floor", "--goal", "3", "2", "90",
                                       "--timeout", "12", "--tag", "legacy", "--no-rviz"])

        self.assertEqual(result, 0)
        values = dict(argument.split(":=", 1) for argument in call.args[0][4:])
        self.assertEqual(values["world"], "open_floor")
        self.assertEqual(values["goal"], "3.0 2.0 90.0")
        self.assertEqual(values["timeout"], "12.0")
        self.assertEqual(values["tag"], "legacy")
        self.assertEqual(values["rviz"], "false")
        self.assertNotIn("route", values)
        self.assertFalse((Path(call.kwargs["env"]["HARP_RUN_DIR"]) / "route.yaml").exists())


if __name__ == "__main__":
    unittest.main()
