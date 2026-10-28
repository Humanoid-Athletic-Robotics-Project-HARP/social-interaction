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

    def test_inline_waypoints_exclude_goal_and_route(self):
        for other in (("--goal", "1", "0"), ("--route", str(self.write_route()))):
            with self.subTest(other=other):
                self.assert_preflight_rejected(
                    ["open_floor", "--waypoint", "1", "0", *other],
                    "not allowed with argument")

    def test_inline_waypoints_and_laps_are_saved_as_effective_route(self):
        result, call, _ = self.invoke([
            "open_floor", "--waypoint", "1", "-1", "90",
            "--waypoint", "0", "0", "--laps", "3"])
        run = Path(call.kwargs["env"]["HARP_RUN_DIR"])
        route = course.load_route(run / "route.yaml")
        self.assertEqual(result, 0)
        self.assertEqual(route.waypoints, ((1.0, -1.0, 90.0), (0.0, 0.0, 0.0)))
        self.assertEqual(route.lap_limit, 3)
        self.assertIsNone(route.criteria)
        values = dict(argument.split(":=", 1) for argument in call.args[0][4:])
        self.assertEqual(values["route"], str(run / "route.yaml"))
        self.assertNotIn("goal", values)

    def test_inline_waypoints_default_to_one_lap(self):
        _, call, _ = self.invoke(["open_floor", "--waypoint", "1", "0"])
        run = Path(call.kwargs["env"]["HARP_RUN_DIR"])
        self.assertEqual(course.load_route(run / "route.yaml").lap_limit, 1)

    def test_laps_override_saved_route_without_changing_source(self):
        source = self.write_route("frame: odom\nlaps: 2\nwaypoints: [[1, 0], [0, 0]]\n"
                                  "criteria: {must_succeed: true, max_contacts: 0}\n")
        original = source.read_text()
        _, call, _ = self.invoke(["open_floor", "--route", str(source), "--laps", "5"])
        run = Path(call.kwargs["env"]["HARP_RUN_DIR"])
        route = course.load_route(run / "route.yaml")
        self.assertEqual(route.lap_limit, 5)
        self.assertEqual(route.criteria, {"must_succeed": True, "max_contacts": 0})
        self.assertEqual(source.read_text(), original)

    def test_laps_without_route_are_rejected_before_launch(self):
        for args in (["open_floor", "--laps", "3"],
                     ["open_floor", "--goal", "1", "0", "--laps", "3"]):
            with self.subTest(args=args):
                self.assert_preflight_rejected(args, "--laps requires --route or --waypoint")

    def test_invalid_laps_and_repeat_are_rejected_before_launch(self):
        for option in ("--laps", "--repeat"):
            for value in ("0", "-1", "1.5", "nan"):
                with self.subTest(option=option, value=value):
                    self.assert_preflight_rejected(
                        ["open_floor", "--waypoint", "1", "0", option, value],
                        "must be a positive integer")

    def test_malformed_inline_waypoints_are_rejected_before_launch(self):
        for point in (("1",), ("1", "2", "3", "4")):
            with self.subTest(point=point):
                self.assert_preflight_rejected(["open_floor", "--waypoint", *point],
                                               "must contain X Y or X Y YAW_DEG")
        for point in (("nan", "0"), ("0", "inf"), ("1", "2", "nan")):
            with self.subTest(point=point):
                self.assert_preflight_rejected(["open_floor", "--waypoint", *point],
                                               "must be a finite number")

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

    def test_concurrent_run_directory_claim_retries_with_next_name(self):
        self.runs.mkdir()
        mkdir = Path.mkdir
        claimed = []

        def concurrent_mkdir(path, *args, **kwargs):
            if path.parent == self.runs and not claimed:
                mkdir(path, *args, **kwargs)  # another course claims this name first
                claimed.append(path)
                raise FileExistsError(str(path))
            return mkdir(path, *args, **kwargs)

        with patch.object(Path, "mkdir", new=concurrent_mkdir):
            result, call, _ = self.invoke(["open_floor", "--tag", "same-time"])

        self.assertEqual(result, 0)
        self.assertEqual(len(claimed), 1)
        run = Path(call.kwargs["env"]["HARP_RUN_DIR"])
        self.assertNotEqual(run, claimed[0])
        self.assertTrue(run.is_dir())
        self.assertEqual(len(list(self.runs.iterdir())), 2)

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
