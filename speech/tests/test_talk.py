"""talk.py's text handling, which needs no models:
  python3 -m unittest discover -s tests   (from speech/, in its venv)"""

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import talk  # noqa: E402


class TestNumbers(unittest.TestCase):
    def test_words(self):
        for n, words in ((0, "zero"), (7, "seven"), (12, "twelve"), (40, "forty"), (42, "forty-two"),
                         (100, "one hundred"), (305, "three hundred five"),
                         (2026, "two thousand twenty-six"),
                         (1_000_000, "one million")):
            self.assertEqual(talk.number_words(n), words)

    def test_speakable(self):
        self.assertEqual(talk.speakable("I walked 12 meters in 2026."),
                         "I walked twelve meters in two thousand twenty-six.")
        self.assertEqual(talk.speakable("It's 3.5 km, about 12,000 steps."),
                         "It's three point five km, about twelve thousand steps.")
        self.assertEqual(talk.speakable("Have a great day! <bye>"), "Have a great day!")
        self.assertEqual(talk.speakable("I'm a Unitree G1 humanoid."), "I'm a Unitree gee one humanoid.")
        self.assertEqual(talk.speakable("Room B12."), "Room B twelve.")


class TestSForms(unittest.TestCase):
    def test_endings_follow_the_last_sound(self):
        lines = ["cat k ˈ æ t", "dog || dˈ ɑː ɡ", "church ʧ ˈ ɜ ɹ ʧ", "city s ˈ ɪ ɾ i",
                 "cats k ˈ æ t s"]
        forms = {line.split(" ", 1)[0]: line for line in talk.with_s_forms(lines)}
        self.assertEqual(forms["cat's"], "cat's k ˈ æ t s")
        self.assertEqual(forms["dogs"], "dogs || dˈ ɑː ɡ z")
        self.assertEqual(forms["churches"], "churches ʧ ˈ ɜ ɹ ʧ ᵻ z")
        self.assertEqual(forms["cities"], "cities s ˈ ɪ ɾ i z")
        self.assertEqual(forms["cats"], "cats k ˈ æ t s")   # its own entry is kept


class TestPieces(unittest.TestCase):
    def test_sentences_as_they_stream(self):
        stream = ["Oh, ", "nice", "! What ", "was the class", " about?"]
        self.assertEqual(list(talk.pieces(stream)), ["Oh, nice!", "What was the class about?"])

    def test_long_clause_is_cut_at_a_comma(self):
        stream = ["That sounds like a really long and tiring day for anyone, ",
                  "so I hope you rest tonight."]
        self.assertEqual(list(talk.pieces(stream)),
                         ["That sounds like a really long and tiring day for anyone,",
                          "so I hope you rest tonight."])

    def test_short_clause_waits_for_the_sentence(self):
        self.assertEqual(list(talk.pieces(["Oh, ", "nice to meet you."])), ["Oh, nice to meet you."])

    def test_first_piece_is_cut_sooner(self):
        stream = ["I don't know it from experience, ", "like humans do. I follow rules, ",
                  "and maths is one of them."]
        self.assertEqual(list(talk.pieces(stream)),
                         ["I don't know it from experience,", "like humans do.",
                          "I follow rules, and maths is one of them."])

    def test_a_comma_too_early_is_not_a_cut(self):
        stream = ["Oh, that sounds like a really long and tiring day for anyone, ", "so rest up."]
        self.assertEqual(list(talk.pieces(stream)),
                         ["Oh, that sounds like a really long and tiring day for anyone,", "so rest up."])

    def test_end_tag_is_its_own_piece(self):
        self.assertEqual(list(talk.pieces(["Bye now! ", "<bye>"])), ["Bye now!", "<bye>"])


class TestFarewell(unittest.TestCase):
    def test_replies_that_end_a_conversation(self):
        for reply in ("No problem, it was lovely meeting you. Have a great day!", "Okay, bye for now!",
                      "Enjoy your lunch!", "Take care!"):
            self.assertTrue(talk.FAREWELL.search(reply), reply)

    def test_replies_that_dont(self):
        # the ones the grammar pushed into a <bye> when asked for more sentences
        for reply in ("I know that because I was taught math facts by the team that built me. "
                      "It's like how you learn basic rules in school.",
                      "I'm HARP! A robot from Stanford."):
            self.assertFalse(talk.FAREWELL.search(reply), reply)


class TestEcho(unittest.TestCase):
    def setUp(self):
        self.talk = talk.Conversation.__new__(talk.Conversation)   # no models needed
        self.talk.said = ["Hi, I'm HARP, a robot from Stanford. How's your day going?",
                          "Oh, nice! What was the lecture about?"]

    def test_own_voice_is_an_echo(self):
        for heard in ("Hi I'm Harp, robot from Stanford. How's your day going?",
                      "what was the lecture about"):
            self.assertTrue(self.talk.echo(heard), heard)

    def test_the_person_is_not(self):
        # under three words is too short to tell: a person may well say "Oh nice." too
        for heard in ("It was about machine learning.", "Pretty good, thanks!", "Nice.", "Oh nice."):
            self.assertFalse(self.talk.echo(heard), heard)


class TestGoodbye(unittest.TestCase):
    def test_goodbyes(self):
        for said in ("Okay, bye!", "I gotta go", "See you later", "I have to run now"):
            self.assertTrue(talk.GOODBYE.search(said), said)
        for said in ("Nice to meet you", "Hi there", "I'm going to the library later"):
            self.assertFalse(talk.GOODBYE.search(said), said)


if __name__ == "__main__":
    unittest.main()
