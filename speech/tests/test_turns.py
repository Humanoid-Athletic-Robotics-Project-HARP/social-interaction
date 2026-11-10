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



def run(t, *parts):
    """Push parts through one Turns; the turns found."""
    found = []
    for part in parts:
        for frame, prob in part:
            turn = t.push(frame, prob)
            if turn:
                found.append(turn)
    return found


NEAR = 0.1     # the person, -20 dBFS
OFF = NEAR * 10 ** (-(max(listen.LOCK_DB, listen.REL_DB) + 3) / 20)   # someone further off: past both
SOFTER = NEAR * 10 ** (-(listen.LOCK_DB - 3) / 20)   # the person, quieter this turn: inside the lock
FAR = 10 ** ((listen.MIN_DB - 6) / 20)               # under MIN_DB


class TestLock(unittest.TestCase):
    def setUp(self):
        self.t = listen.Turns()
        # the person's first turn: over SHORT_TURN of speech, so it sets their level
        self.assertEqual(len(run(self.t, frames(10, 0.0, QUIET), frames(40, 0.9, NEAR),
                                 frames(30, 0.0, QUIET))), 1)

    def test_the_persons_level_is_kept(self):
        self.assertAlmostEqual(self.t.person, -20, delta=1)
        self.assertAlmostEqual(self.t.start_db(), -20 - listen.LOCK_DB, delta=1)

    def test_a_quieter_voice_doesnt_start_a_turn(self):
        self.assertEqual(run(self.t, frames(60, 0.9, OFF), frames(30, 0.0, QUIET)), [])

    def test_the_person_again_does(self):
        self.assertEqual(len(run(self.t, frames(40, 0.9, NEAR * 0.7), frames(30, 0.0, QUIET))), 1)

    def test_the_person_quieter_still_does(self):
        self.assertEqual(len(run(self.t, frames(40, 0.9, SOFTER), frames(30, 0.0, QUIET))), 1)

    def test_a_quieter_voice_doesnt_hold_the_turn_open(self):
        # the person stops; someone further off keeps talking: the turn still ends on time
        found = run(self.t, frames(40, 0.9, NEAR), frames(40, 0.9, OFF))
        self.assertEqual(len(found), 1)
        start, end, audio = found[0]
        self.assertAlmostEqual(end - start, 40 * FRAME_S, delta=2 * FRAME_S)

    def test_quiet_frames_are_turned_down(self):
        duck, listen.DUCK_DB = listen.DUCK_DB, 20.0   # off by default
        self.addCleanup(setattr, listen, "DUCK_DB", duck)
        (start, end, audio), = run(self.t, frames(30, 0.9, NEAR), frames(6, 0.9, OFF),
                                   frames(30, 0.9, NEAR), frames(30, 0.0, QUIET))
        level = np.sqrt(np.mean(audio.reshape(-1, listen.FRAME) ** 2, axis=1))
        ducked = OFF * 10 ** (-listen.DUCK_DB / 20)
        self.assertGreaterEqual(np.sum(np.abs(level - ducked) < ducked * 0.2), 3)   # the middle ones
        self.assertFalse(np.any(np.abs(level - OFF) < OFF * 0.1))                   # none left as it was
        self.assertGreater(np.sum(np.abs(level - NEAR) < 0.001), 50)                # the person's kept

    def test_the_lock_is_forgotten_after_lock_s(self):
        gone = int(listen.LOCK_S / FRAME_S) + 1
        self.assertEqual(len(run(self.t, frames(gone, 0.0, QUIET), frames(40, 0.9, OFF),
                                 frames(30, 0.0, QUIET))), 1)

    def test_a_cough_doesnt_set_the_level(self):
        run(self.t, frames(12, 0.9, 0.5), frames(30, 0.0, QUIET))   # short and loud
        self.assertAlmostEqual(self.t.person, -20, delta=1)

    def test_too_quiet_never_starts_one(self):
        t = listen.Turns()
        self.assertEqual(run(t, frames(10, 0.0, QUIET * 0.1), frames(40, 0.9, FAR),
                             frames(30, 0.0, QUIET * 0.1)), [])


if __name__ == "__main__":
    unittest.main()
