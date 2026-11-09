"""What harp/view_feed.py adds to a frame from the predictive filter's tracks."""
import base64
import unittest
import zlib

import numpy as np

from harp.obstacle_tracker import STRUCTURE_ID
from harp.view_feed import compact_tracks


def track(ident, x, y, credit=None):
    return {"id": ident, "x": x, "y": y, "vx": 0.5, "vy": -0.25, "radius": 0.3,
            "uncertainty": 0.123, "last_seen": 1.0, "hits": 4, "credit": credit}


class CompactTracks(unittest.TestCase):
    def test_objects_lost_and_walls_are_kept_apart(self):
        record = {"frame": "odom", "stamp": 1.0,
                  "tracks": [track(3, 1.234, -2.0, 0.4), track(STRUCTURE_ID + 1, 5.0, 0.5)],
                  "unresolved": [track(7, 0.5, 0.5)]}
        out = compact_tracks(record)
        self.assertEqual(out["objects"], [[3, 1.23, -2.0, 0.5, -0.25, 0.3, 0.12, 0.4]])
        self.assertEqual(out["lost"], [[7, 0.5, 0.5, 0.3]])
        walls = np.frombuffer(zlib.decompress(base64.b64decode(out["walls"])), np.int16)
        self.assertEqual(walls.tolist(), [500, 50])

    def test_no_tracks_is_empty_not_missing(self):
        out = compact_tracks({"tracks": [], "unresolved": []})
        self.assertEqual((out["objects"], out["lost"]), ([], []))
        self.assertEqual(len(zlib.decompress(base64.b64decode(out["walls"]))), 0)


if __name__ == "__main__":
    unittest.main()
