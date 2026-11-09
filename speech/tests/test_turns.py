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
    def test_cut_ends_the_turn_at_once(self):
        t = listen.Turns()
        for frame, prob in frames(10, 0.0, QUIET) + frames(30, 0.9, LOUD) + frames(3, 0.0, QUIET):
            self.assertIsNone(t.push(frame, prob))
        start, end, audio = t.cut()
        self.assertAlmostEqual(start, 10 * FRAME_S)
        self.assertAlmostEqual(end, 40 * FRAME_S)   # where the voice stopped, not the cut
        self.assertFalse(t.speaking)
        self.assertIsNone(t.cut())                  # nothing being heard now

    def test_cut_before_speech_is_nothing(self):
        t = listen.Turns()
        for frame, prob in frames(10, 0.0, QUIET):
            t.push(frame, prob)
        self.assertIsNone(t.cut())

    def test_ending_lines(self):
        import io
        now = listen.ending_lines(io.StringIO("\n"))
        self.assertTrue(now.wait(1))

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

    def test_gap_while_starting_keeps_the_start(self):
        # voice, one frame without, voice: the start is where the voice began
        start_frames = int(listen.START / FRAME_S)
        (start, _, _), = turns(frames(10, 0.0, QUIET), frames(start_frames - 2, 0.9, LOUD),
                               frames(1, 0.3, LOUD), frames(30, 0.9, LOUD),
                               frames(30, 0.0, QUIET))
        self.assertAlmostEqual(start, 10 * FRAME_S)

    def test_goodbye_and_thanks_are_kept(self):
        for said in ("Bye.", "Thank you.", "You"):
            self.assertIsNone(listen.dropped(said, 0.0, 1.0))
        self.assertEqual(listen.dropped("Thank you for watching.", 0.0, 1.0), "made_up")

    def test_voice_no_louder_than_the_room_is_ignored(self):
        self.assertEqual(turns(frames(20, 0.0, LOUD), frames(30, 0.9, LOUD),
                               frames(30, 0.0, LOUD)), [])

    def test_long_turn_is_cut(self):
        long = int(listen.MAX_SPEECH / FRAME_S) + 50
        found = turns(frames(10, 0.0, QUIET), frames(long, 0.9, LOUD))
        self.assertEqual(len(found), 1)
        start, end, _ = found[0]
        self.assertLessEqual(end - start, listen.MAX_SPEECH + listen.PREROLL)


class TestSmartTurn(unittest.TestCase):
    """The turn's end with a made-up Smart Turn (listen.TurnEnd) that always answers p."""

    def silence_before_end(self, speech_frames, p, parts=()):
        """Seconds of silence after which the turn ends, and how often Smart Turn was asked."""
        asked = []
        turns_ = listen.Turns(lambda audio: asked.append(len(audio)) or p)
        for frame, prob in frames(10, 0.0, QUIET) + frames(speech_frames, 0.9, LOUD) + list(parts):
            self.assertIsNone(turns_.push(frame, prob))
        for quiet in range(1, int(5 / FRAME_S)):
            if turns_.push(np.full(listen.FRAME, QUIET, np.float32), 0.0):
                return quiet * FRAME_S, len(asked)
        self.fail("the turn never ended")

    def test_finished_ends_soon(self):
        waited, asked = self.silence_before_end(50, 0.9)
        self.assertAlmostEqual(waited, listen.CHECK, delta=FRAME_S)
        self.assertEqual(asked, 1)

    def test_unfinished_waits(self):
        waited, _ = self.silence_before_end(50, 0.1)
        self.assertAlmostEqual(waited, listen.WAIT, delta=FRAME_S)

    def test_one_word_isnt_judged(self):
        waited, asked = self.silence_before_end(int(0.5 / FRAME_S), 0.9)
        self.assertAlmostEqual(waited, listen.SILENCE, delta=FRAME_S)
        self.assertEqual(asked, 0)

    def test_going_on_is_judged_again(self):
        # judged unfinished in a pause, then they go on: one turn, asked again at its end
        pause = frames(int(listen.WAIT / 2 / FRAME_S), 0.0, QUIET) + frames(30, 0.9, LOUD)
        _, asked = self.silence_before_end(50, 0.1, pause)
        self.assertEqual(asked, 2)



if __name__ == "__main__":
    unittest.main()
