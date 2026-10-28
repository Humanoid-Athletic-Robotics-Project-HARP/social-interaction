"""Offline run-folder checks for whole-route results and criteria."""

import csv
import json
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

import numpy as np
import yaml

from harp_control import runlog
from harp_eval import run_summary


def event(t, name, **details):
    return {"t": t, "source": "goal", "event": name, "details": details}


class RouteSummaryTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.run = Path(self.temp.name) / "2026-10-28_route"
        self.run.mkdir()
        run_summary.write_meta(self.run, {"kind": "sim", "world": "open_floor",
                                         "command": "ros2 run harp_eval course open_floor"})
        self.waypoints = [[1.0, 0.0, 0.0], [2.0, 0.0, 0.0]]
        self.world_criteria = {"must_succeed": True, "max_time_s": 1, "max_contacts": 0}

    def write_route(self, criteria=None):
        config = {"frame": "odom", "waypoints": self.waypoints, "laps": 2}
        if criteria is not None:
            config["criteria"] = criteria
        (self.run / "route.yaml").write_text(yaml.safe_dump(config))

    def route_events(self, status="SUCCEEDED"):
        events = [{"t": 5, "source": "walker", "event": "policy_on", "details": {}}]
        events.append(event(10, "goal_sent", x=2.0, y=0.0, yaw=0.0,
                            waypoints=self.waypoints, laps=2))
        times = [20, 30, 50, 70] if status == "SUCCEEDED" else [20, 30, 50]
        for i, t in enumerate(times):
            details = {"lap": i // 2 + 1, "waypoint": i % 2 + 1,
                       "x": self.waypoints[i % 2][0], "y": 0.0, "yaw": 0.0}
            events.append(event(10 if i == 0 else times[i - 1] + 1,
                                "waypoint_sent", **details))
            current_status = status if i == len(times) - 1 else "SUCCEEDED"
            events.append(event(t, "waypoint_result", status=current_status, **details))
        completed = 4 if status == "SUCCEEDED" else 2
        events.append(event(times[-1], "goal_result", status=status, recoveries=3,
                            seconds=times[-1] - 10, remaining=0.0,
                            completed_waypoints=completed, completed_laps=completed // 2))
        return events

    def write_events(self, events):
        with (self.run / "events.csv").open("w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=runlog.EVENT_FIELDS)
            writer.writeheader()
            writer.writerows({**e, "details": json.dumps(e["details"])} for e in events)

    def write_positions(self):
        t = np.arange(81, dtype=float)
        x = np.interp(t, [0, 10, 20, 30, 50, 70, 80], [0, 0, 1, 2, 1, 2, 2])
        qpos = np.zeros((len(t), 7))
        qpos[:, 0], qpos[:, 3] = x, 1.0
        np.savez(self.run / "truth.npz", t=t, qpos=qpos)
        np.savetxt(self.run / "odom.csv", np.column_stack((t, x, np.zeros_like(t),
                                                        np.zeros_like(t))),
                   delimiter=",", header="t,x,y,yaw", comments="")

    def summarize(self, world_allowed=False):
        # The test exercises real files and metrics, without ROS bags or a physics engine.
        with patch.object(run_summary, "nav_tree", return_value={}), \
                patch.object(run_summary, "clearances", return_value=None), \
                patch.object(run_summary, "criteria_for",
                             return_value=(self.world_criteria, "open_floor.yaml")) as criteria:
            summary = run_summary.summarize(self.run)
            if world_allowed:
                criteria.assert_called_once_with("open_floor")
            else:
                criteria.assert_not_called()
            return summary

    def test_route_uses_final_result_and_entire_window(self):
        self.write_route({"must_succeed": True, "max_time_s": 100})
        events = self.route_events()
        self.write_events(events)
        self.write_positions()

        summary = self.summarize()

        self.assertEqual(run_summary.window(events), (10, 70))
        self.assertEqual(summary["outcome"], "succeeded")
        self.assertEqual(summary["duration_s"], 60.0)
        self.assertEqual(summary["recoveries_feedback"], 3)
        self.assertEqual(summary["goal_world"], [2.0, 0.0])
        self.assertEqual(summary["end_error_m"], 0.0)
        self.assertEqual(summary["route"]["planned_waypoints"], 4)
        self.assertEqual(summary["route"]["completed_waypoints"], 4)
        self.assertEqual(summary["route"]["planned_laps"], 2)
        self.assertEqual(summary["route"]["completed_laps"], 2)
        results = summary["route"]["waypoint_results"]
        self.assertEqual([(r["lap"], r["waypoint"]) for r in results],
                         [(1, 1), (1, 2), (2, 1), (2, 2)])
        self.assertTrue(summary["passed"])
        self.assertEqual(summary["check"]["criteria"], "route.yaml")
        self.assertEqual(set(summary["check"]["results"]), {"must_succeed", "max_time_s"})
        self.assertIn("route 4/4 waypoints, 2/2 laps", run_summary.one_screen(summary))

    def test_failure_after_successful_waypoints_fails_the_whole_route(self):
        self.write_route({"must_succeed": True})
        self.write_events(self.route_events(status="FAILED"))

        summary = self.summarize()

        self.assertEqual(summary["outcome"], "failed")
        self.assertEqual(summary["duration_s"], 40.0)
        self.assertEqual(summary["route"]["completed_waypoints"], 2)
        self.assertEqual(summary["route"]["completed_laps"], 1)
        self.assertEqual([r["status"] for r in summary["route"]["waypoint_results"]],
                         ["SUCCEEDED", "SUCCEEDED", "FAILED"])
        self.assertFalse(summary["passed"])
        self.assertEqual(summary["check"]["failed"], ["must_succeed"])

    def test_walker_error_precedes_shutdown_route_cancellation(self):
        self.write_route({"must_succeed": True})
        events = self.route_events(status="CANCELED")
        events.extend([
            {"t": 40, "source": "walker", "event": "error",
             "details": {"message": "Robot state stopped"}},
            {"t": 40.1, "source": "walker", "event": "stop",
             "details": {"reason": "state_lost"}},
        ])
        self.write_events(events)
        self.write_positions()

        summary = self.summarize()

        self.assertEqual(run_summary.window(events), (10, 40))
        self.assertEqual(summary["outcome"], "error")
        self.assertEqual(summary["reason"], "Robot state stopped")
        self.assertEqual(summary["duration_s"], 30.0)
        self.assertEqual(summary["route"]["completed_waypoints"], 2)
        self.assertEqual(summary["route"]["completed_laps"], 1)
        self.assertFalse(summary["passed"])

    def test_remote_estop_precedes_shutdown_route_cancellation(self):
        self.write_route({"must_succeed": True})
        events = self.route_events(status="CANCELED")
        events.extend([
            {"t": 40, "source": "walker", "event": "estop",
             "details": {"by": "remote"}},
            {"t": 40.1, "source": "walker", "event": "stop",
             "details": {"reason": "remote_estop"}},
        ])
        self.write_events(events)

        summary = self.summarize()

        self.assertEqual(run_summary.window(events), (10, 40))
        self.assertEqual(summary["outcome"], "stopped")
        self.assertEqual(summary["reason"], "e-stop (remote, B)")
        self.assertEqual(summary["duration_s"], 30.0)
        self.assertEqual(summary["route"]["completed_waypoints"], 2)
        self.assertFalse(summary["passed"])

    def test_remote_b_estop_without_explicit_operator_keeps_default_reason(self):
        events = [event(10, "goal_sent", x=2, y=0, yaw=0),
                  {"t": 20, "source": "walker", "event": "estop", "details": {}},
                  {"t": 20.1, "source": "walker", "event": "stop",
                   "details": {"reason": "remote_estop"}},
                  event(21, "goal_result", status="CANCELED", recoveries=0)]

        self.assertEqual(run_summary.outcome(events), ("stopped", "e-stop (remote, B)"))
        self.assertEqual(run_summary.window(events), (10, 20))

    def test_successful_route_is_not_overridden_by_later_ordinary_stop(self):
        self.write_route({"must_succeed": True, "max_time_s": 100})
        events = self.route_events()
        events.extend([
            {"t": 72, "source": "walker", "event": "stop",
             "details": {"reason": "ctrl_c"}},
            {"t": 73, "source": "walker", "event": "damped", "details": {}},
        ])
        self.write_events(events)

        summary = self.summarize()

        self.assertEqual(run_summary.window(events), (10, 70))
        self.assertEqual(summary["outcome"], "succeeded")
        self.assertEqual(summary["reason"], "SUCCEEDED")
        self.assertEqual(summary["duration_s"], 60.0)
        self.assertEqual(summary["route"]["completed_waypoints"], 4)
        self.assertEqual(summary["route"]["completed_laps"], 2)
        self.assertTrue(summary["passed"])

    def test_route_without_criteria_does_not_inherit_world_limits(self):
        for criteria in (None, {}):
            with self.subTest(criteria=criteria):
                self.write_route(criteria)
                self.write_events(self.route_events())

                summary = self.summarize()

                self.assertIsNone(summary["check"])
                self.assertIsNone(summary["passed"])
                self.assertEqual(summary["duration_s"], 60.0)

    def test_route_events_without_saved_yaml_never_use_world_criteria(self):
        self.write_events(self.route_events())

        summary = self.summarize()

        self.assertEqual(summary["route"]["planned_waypoints"], 4)
        self.assertIsNone(summary["passed"])

    def test_terminal_counts_are_authoritative_when_waypoint_events_are_missing(self):
        self.write_route()
        events = [e for e in self.route_events()
                  if e["event"] != "waypoint_result" or e["details"]["lap"] == 1]
        self.write_events(events)

        summary = self.summarize()

        self.assertEqual(len(summary["route"]["waypoint_results"]), 2)
        self.assertEqual(summary["route"]["completed_waypoints"], 4)
        self.assertEqual(summary["route"]["completed_laps"], 2)

    def test_route_that_never_starts_keeps_plan_and_fails_its_criteria(self):
        self.write_route({"must_succeed": True})
        self.write_events([event(20, "goal_result", status="NAV2_NOT_ACTIVE", recoveries=0)])

        summary = self.summarize()

        self.assertEqual(summary["outcome"], "error")
        self.assertEqual(summary["route"]["planned_waypoints"], 4)
        self.assertEqual(summary["route"]["completed_waypoints"], 0)
        self.assertEqual(summary["route"]["completed_laps"], 0)
        self.assertIsNone(summary["duration_s"])
        self.assertFalse(summary["passed"])

    def test_single_goal_still_uses_world_criteria(self):
        self.write_events([event(10, "goal_sent", x=2, y=0, yaw=0),
                           event(70, "goal_result", status="SUCCEEDED", recoveries=0)])

        summary = self.summarize(world_allowed=True)

        self.assertNotIn("route", summary)
        self.assertEqual(summary["outcome"], "succeeded")
        self.assertFalse(summary["passed"])
        self.assertEqual(summary["check"]["criteria"], "open_floor.yaml")
        self.assertEqual(summary["check"]["failed"], ["max_time_s"])

    def test_route_plan_survives_pruning_and_resummarization(self):
        self.write_route({"must_succeed": True, "max_time_s": 100})
        self.write_events(self.route_events())
        self.write_positions()
        before = self.summarize()
        (self.run / "nav2_params.yaml").write_text("unused: true\n")

        run_summary.prune(self.run.parent, {"baseline_keep_runs": 0, "extras_keep_runs": 0,
                                           "extras_max_gb": 0})
        after = self.summarize()

        self.assertTrue((self.run / "route.yaml").exists())
        self.assertFalse((self.run / "truth.npz").exists())
        self.assertFalse((self.run / "nav2_params.yaml").exists())
        self.assertEqual(after["route"], before["route"])
        self.assertEqual(after["duration_s"], before["duration_s"])
        self.assertTrue(after["passed"])


if __name__ == "__main__":
    unittest.main()
