"""submission/host_health.py's verdict, with this computer's readings replaced."""
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

sys.path.append(str(Path(__file__).resolve().parents[2] / "submission"))
import host_health  # noqa: E402


class Verdict(unittest.TestCase):
    def health(self, free, compressed_mb, viewers=(), level="normal", platform="darwin"):
        with patch.object(host_health.sys, "platform", platform), \
                patch.object(host_health, "macos_pressure", return_value=(free, level)), \
                patch.object(host_health, "viewers", return_value=list(viewers)), \
                patch.object(host_health, "host_memory", return_value={"free_mb": 900, "compressed_mb": compressed_mb}):
            return host_health.health()

    def test_a_mac_with_room_is_ready(self):
        h = self.health(80, 1800)
        self.assertEqual((h["ready"], h["reasons"]), (True, []))

    def test_each_shortfall_is_a_reason(self):
        h = self.health(65, 3300, viewers=["123"], level="warning")
        self.assertFalse(h["ready"])
        self.assertEqual(len(h["reasons"]), 4)
        self.assertIn("65% memory free (aim for 73% or more)", h["reasons"])
        self.assertIn("3.3 GB compressed (aim for 2.5 GB or less)", h["reasons"])

    def test_unmeasured_elsewhere_unless_a_viewer_is_open(self):
        self.assertIsNone(self.health(None, None, platform="linux")["ready"])
        self.assertFalse(self.health(None, None, viewers=["1"], platform="linux")["ready"])

    def test_a_mac_without_a_free_reading_is_unmeasured_unless_something_else_is_short(self):
        self.assertEqual((self.health(None, 1800)["ready"], self.health(None, 1800)["reasons"]), (None, []))
        self.assertFalse(self.health(None, 3300)["ready"])


if __name__ == "__main__":
    unittest.main()
