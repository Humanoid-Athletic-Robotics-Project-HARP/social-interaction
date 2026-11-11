"""Robot status: testground/robot/status_server.py's readings and the hub's robot source."""
import json
import os
from http.server import ThreadingHTTPServer
from pathlib import Path
import sys
import threading
import time
from types import SimpleNamespace as NS
import unittest
import unittest.mock
import urllib.request

HERE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent / "testground" / "robot"))
import status_server  # noqa: E402  (imports no SDK or ROS until it reads the robot)
from feeds import Source, robot_feeds  # noqa: E402


def low(casing=40.0, winding=50.0):
    return NS(motor_state=[NS(temperature=[casing + i, winding + i]) for i in range(35)], mode_machine=5)


class Readings(unittest.TestCase):
    def test_the_harness_reader_joins_the_harness_network_whatever_the_shell_set(self):
        class FailingInit:   # stands in for rclpy: stop right after the network is chosen
            @staticmethod
            def init(args):
                raise RuntimeError("stop here")
        shell = {"ROS_DOMAIN_ID": "0", "ROS_LOCALHOST_ONLY": "0", "RMW_IMPLEMENTATION": "rmw_cyclonedds_cpp"}
        with unittest.mock.patch.dict("os.environ", shell), \
                unittest.mock.patch.dict(sys.modules, {"rclpy": FailingInit, "std_msgs": NS(), "std_msgs.msg": NS(String=str)}):
            latest = status_server.Latest()
            status_server.read_harness(latest, threading.Event())
            self.assertEqual({k: os.environ[k] for k in shell},
                             {"ROS_DOMAIN_ID": "97", "ROS_LOCALHOST_ONLY": "1", "RMW_IMPLEMENTATION": "rmw_fastrtps_cpp"})
        self.assertIn("stop here", latest.harness_error)

    def test_summary_names_29_motors_and_keeps_the_harness_fields_shown(self):
        harness = {"armed": True, "startup_phase": "balance", "abort_reason": None, "q": [0.1] * 29}
        value = status_server.summary(low(), NS(soc=61), harness, [0.01, 0.5, 0.2], 980)
        self.assertEqual(len(value["motors"]), 29)
        self.assertEqual(value["motors"][3], {"name": "left_knee", "casing_c": 43.0, "winding_c": 53.0})
        self.assertEqual(value["motors"][-1]["name"], "right_wrist_yaw")
        self.assertEqual((value["battery"]["percent"], value["lowstate"]["hz"]), (61.0, 980))
        self.assertTrue(value["harness"]["armed"])
        self.assertNotIn("q", value["harness"])
        self.assertEqual(value["limits"], {"casing_c": 85.0, "winding_c": 120.0, "battery_percent": 10.0})

    def test_nothing_received_yet(self):
        value = status_server.Latest().status()
        self.assertEqual((value["motors"], value["battery"]["percent"], value["harness"]["age_s"]), ([], None, None))

    def test_feeds_go_stale_on_their_own_timeouts(self):
        fresh = robot_feeds({"lowstate": {"age_s": 0.01}, "battery": {"age_s": 1.0}, "harness": {"age_s": 0.3}})
        self.assertTrue(all(f["online"] for f in fresh.values()))
        stale = robot_feeds({"lowstate": {"age_s": 2.0}, "battery": {"age_s": None},
                             "harness": {"age_s": None, "error": "ModuleNotFoundError: rclpy"}})
        self.assertEqual(stale["motors"]["reason"], "no reading for 2 s")
        self.assertEqual(stale["battery"]["reason"], "no rt/lf/bmsstate")
        self.assertEqual(stale["harness"]["reason"], "ModuleNotFoundError: rclpy")


class Serving(unittest.TestCase):
    def test_hub_reads_the_status_server(self):
        latest = status_server.Latest()
        latest.on_low(low(casing=80.0))
        latest.on_bms(NS(soc=55))
        latest.on_harness(json.dumps({"armed": False, "startup_phase": "wait"}))
        latest.on_harness("not json")   # ignored
        server = ThreadingHTTPServer(("127.0.0.1", 0), status_server.handler(latest))
        threading.Thread(target=server.serve_forever, daemon=True).start()
        url = "http://127.0.0.1:%d" % server.server_port
        try:
            with self.assertRaises(urllib.error.HTTPError):
                urllib.request.urlopen(url + "/other", timeout=2)
            source = Source({"id": "robot", "kind": "robot", "url": url})
            source.start()
            deadline = time.monotonic() + 5
            while not source.status()["reachable"] and time.monotonic() < deadline:
                time.sleep(0.05)
            state = source.status()
            source.close()
            self.assertTrue(state["reachable"])
            self.assertEqual(set(state["feeds"]), {"motors", "battery", "harness"})
            self.assertEqual(state["detail"]["motors"][0]["casing_c"], 80.0)
            self.assertEqual(state["detail"]["harness"]["startup_phase"], "wait")
        finally:
            server.shutdown()
            server.server_close()


if __name__ == "__main__":
    unittest.main()
