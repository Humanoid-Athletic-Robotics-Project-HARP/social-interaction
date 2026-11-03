"""The turn rules (listen.Turns) on made-up Silero probabilities, so no model is needed:
  python3 -m unittest discover -s tests   (from speech/, in its venv)"""

import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import listen  # noqa: E402

FRAME_S = listen.FRAME / listen.RATE


def frames(count, prob, level):
    return [(np.full(listen.FRAME, level, np.float32), prob)] * count


def turns(*parts):
    """Every turn Turns finds in a run of (frame, probability) parts."""
    found, turns_ = [], listen.Turns()
    for part in parts:
        for frame, prob in part:
            turn = turns_.push(frame, prob)
            if turn:
                found.append(turn)
    return found


QUIET = 0.001   # the room's level
LOUD = 0.1      # a voice's


class TestTurns(unittest.TestCase):
    def test_one_turn(self):
        (start, end, audio), = turns(frames(10, 0.0, QUIET), frames(30, 0.9, LOUD),
                                     frames(30, 0.0, QUIET))
        self.assertAlmostEqual(start, 10 * FRAME_S)
        self.assertAlmostEqual(end, 40 * FRAME_S)
        # the turn's audio starts PREROLL before the speech was confirmed
        self.assertGreater(len(audio), 30 * listen.FRAME)

    def test_blip_is_not_a_turn(self):
        self.assertEqual(turns(frames(10, 0.0, QUIET), frames(5, 0.9, LOUD),
                               frames(40, 0.0, QUIET)), [])

    def test_pause_under_silence_stays_one_turn(self):
        pause = int(listen.SILENCE / FRAME_S) - 2
        found = turns(frames(10, 0.0, QUIET), frames(20, 0.9, LOUD), frames(pause, 0.0, QUIET),
                      frames(20, 0.9, LOUD), frames(30, 0.0, QUIET))
        self.assertEqual(len(found), 1)

    def test_pause_over_silence_splits(self):
        pause = int(listen.SILENCE / FRAME_S) + 10
        found = turns(frames(10, 0.0, QUIET), frames(20, 0.9, LOUD), frames(pause, 0.0, QUIET),
                      frames(20, 0.9, LOUD), frames(30, 0.0, QUIET))
        self.assertEqual(len(found), 2)

    def test_voice_no_louder_than_the_room_is_ignored(self):
        self.assertEqual(turns(frames(20, 0.0, LOUD), frames(30, 0.9, LOUD),
                               frames(30, 0.0, LOUD)), [])

    def test_long_turn_is_cut(self):
        long = int(listen.MAX_SPEECH / FRAME_S) + 50
        found = turns(frames(10, 0.0, QUIET), frames(long, 0.9, LOUD))
        self.assertEqual(len(found), 1)
        start, end, _ = found[0]
        self.assertLessEqual(end - start, listen.MAX_SPEECH + listen.PREROLL)


if __name__ == "__main__":
    unittest.main()
