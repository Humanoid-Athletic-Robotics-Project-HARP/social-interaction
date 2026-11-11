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
        self.assertEqual(talk.speakable("CS106B at 1:30, or 9:05."),
                         "CS one hundred six B at one thirty, or nine oh five.")
        self.assertEqual(talk.speakable("At 3:00."), "At three.")


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

    def test_a_dash_cuts_like_a_comma(self):
        stream = ["Mine's pretty good too", "\u2014just buzzing ", "around campus, chatting."]
        self.assertEqual([talk.clean(p) for p in talk.pieces(stream)],
                         ["Mine's pretty good too,", "just buzzing around campus, chatting."])

    def test_a_decimal_point_is_not_a_sentence_end(self):
        stream = ["I'm about 1", ".", "3 metres tall. ", "How about you?"]
        self.assertEqual(list(talk.pieces(stream)), ["I'm about 1.3 metres tall.", "How about you?"])

    def test_end_tag_is_its_own_piece(self):
        self.assertEqual(list(talk.pieces(["Bye now! ", "<bye>"])), ["Bye now!", "<bye>"])


class TestFarewell(unittest.TestCase):
    def test_replies_that_end_a_conversation(self):
        for reply in ("No problem, it was lovely meeting you. Have a great day!", "Okay, bye for now!",
                      "Enjoy your lunch!", "Take care!",
                      "You too! Have a smooth day, and maybe don't spill coffee on your laptop again.",
                      # live, 2026-11-09: neither ended the conversation
                      "Good night, Sam! Sleep well, and maybe dream of robots and students having fun.",
                      "Oh right! Good night, Sam!", "Goodnight!", "Sleep well!", "Sweet dreams!",
                      "Bye-bye!", "Later!", "So long, and good luck with your exam!", "Be well!", "Have fun!", "Talk to you soon!",
                      "It was a pleasure chatting with you.", "Safe travels!", "Come back anytime!",
                      "Glad we got to chat. Have fun at the game!", "All the best with the project.",
                      "Until next time!", "Take it easy!"):
            self.assertTrue(talk.FAREWELL.search(reply), reply)

    def test_replies_that_dont(self):
        # the ones the grammar pushed into a <bye> when asked for more sentences
        for reply in ("I know that because I was taught math facts by the team that built me. "
                      "It's like how you learn basic rules in school.",
                      "I'm HARP! A robot from Stanford.",
                      "That's sweet. It's always hard when you say goodbye to people you care about.",
                      "That sounds like a good night out!", "Did you sleep well last night?",
                      "Later, I'll walk over to the quad.", "I could talk about robots all day!",
                      "Was it fun?", "Did you have fun?", "It might be well worth it.",
                      "I wish we had all the best parts.", "That was a treat, wasn't it?"):
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

    def test_the_tail_of_the_last_line_is_an_echo(self):
        for heard in ("Lecture about?", "about"):
            self.assertTrue(self.talk.echo(heard), heard)

    def test_the_person_is_not(self):
        # under three words is too short to tell, unless it ends what was just said: a
        # person may well say "Oh nice." too
        for heard in ("It was about machine learning.", "Pretty good, thanks!", "Nice.", "Oh nice.",
                      "Day going?", "Lecture"):
            self.assertFalse(self.talk.echo(heard), heard)


class TestEnding(unittest.TestCase):
    def test_a_goodbye_reply_ends_it(self):
        self.assertEqual(talk.ending("Enjoy your lunch! <bye>", "I'm getting lunch now."), "bye")
        self.assertEqual(talk.ending("Okay, bye for now!", "Okay, bye!"), "goodbye")
        self.assertEqual(talk.ending("Good night, Sam! Sleep well.",
                                     "I need to go to sleep now, so I'm gonna say bye."), "goodbye")

    def test_the_persons_words_alone_dont(self):
        self.assertIsNone(talk.ending("Oh, that sounds stressful! Have you been before?",
                                      "I have to go to the dentist tomorrow."))
        # the grammar's forced tag on a reply that isn't a goodbye
        self.assertIsNone(talk.ending("That sounds hard. Do you miss them? <bye>",
                                      "I said goodbye to my parents."))


class TestSounds(unittest.TestCase):
    """Words to sounds, with a made-up lexicon and a stand-in for the G2P model."""
    LEXICON = {"i": "ˈI", "said": "sˈɛd", "chat": "ʧˈæt", "torch": "tˈɔɹʧ", "harp": "hˈɑɹp",
               "g": "ʤˈi", "p": "pˈi", "t": "tˈi", "it's": "ɪts"}

    def guess(self, word):
        return "<" + word + ">"

    def test_known_and_guessed(self):
        sounds, guessed, unknown = talk.text_sounds("I said bijective.", self.LEXICON, self.guess)
        self.assertEqual(sounds, "ˈI sˈɛd <bijective>.")
        self.assertEqual((guessed, unknown), ({"bijective": "<bijective>"}, []))

    def test_capitals_and_joined_names(self):
        self.assertEqual(talk.word_sounds("HARP", self.LEXICON, self.guess), ("hˈɑɹp", "known"))
        self.assertEqual(talk.word_sounds("GPT", self.LEXICON, self.guess), ("ʤˈi pˈi tˈi", "spelled"))
        self.assertEqual(talk.word_sounds("ChatGPT", self.LEXICON, self.guess)[0], "ʧˈæt ʤˈi pˈi tˈi")
        self.assertEqual(talk.word_sounds("PyTorch", self.LEXICON, self.guess), ("<py> tˈɔɹʧ", "guessed"))
        self.assertEqual(talk.word_sounds("GPA", {"a": "ə"}, None, {"g": "ʤˈi", "p": "pˈi", "a": "ˈA"}),
                         ("ʤˈi pˈi ˈA", "spelled"))   # letters keep their names
        self.assertEqual(talk.parts("TikTok"), ["Tik", "Tok"])
        self.assertEqual(talk.parts("Qwen's"), ["Qwen's"])

    def test_without_a_model(self):
        sounds, guessed, unknown = talk.text_sounds("It's Kubernetes, café!", self.LEXICON, None)
        self.assertEqual((sounds, guessed, unknown), ("ɪts!", {}, ["Kubernetes", "cafe"]))

    def test_piper_symbols(self):
        self.assertEqual(talk.piper_sounds("bIʤˈɛktɪv"), "baɪdʒˈɛktɪv")
        self.assertEqual(talk.piper_sounds("bˈɜɹd wˈɔɾəɹ"), "bˈɜːd wˈɔːɾɚ")


class TestGoodbye(unittest.TestCase):
    def test_goodbyes(self):
        for said in ("Okay, bye!", "I gotta go", "See you later", "I have to run now",
                     "Anyway, I should get going, nice meeting you.", "Okay, okay, I'm heading out",
                     "Great talking to you", "Good night!", "Okay, I'm off to bed", "Bye-bye",
                     "I should let you go", "I better get going", "I have to leave now",
                     "Talk to you later", "It was a pleasure", "Peace out", "Alright, I'm leaving"):
            self.assertTrue(talk.GOODBYE.search(said), said)
        for said in ("Nice to meet you", "Hi there", "I'm going to the library later",
                     "I'll talk about it later", "I left my bag at home", "It was fun!",
                     "It was great, we went hiking", "You should go see it", "Should I run for office?"):
            self.assertFalse(talk.GOODBYE.search(said), said)



class TestClean(unittest.TestCase):
    def test_what_the_voice_shouldnt_read(self):
        self.assertEqual(talk.clean("*waves* Hi there!"), "Hi there!")
        self.assertEqual(talk.clean("Ha (laughs) that's good."), "Ha that's good.")
        self.assertEqual(talk.clean("Pizza \U0001F355 for sure."), "Pizza for sure.")
        self.assertEqual(talk.clean("Coffee \u2014 or tea - either works."), "Coffee, or tea, either works.")
        self.assertEqual(talk.clean("Salt & 10% sugar."), "Salt and 10 percent sugar.")
        self.assertEqual(talk.clean("Two:\n1. sleep\n2. eat"), "Two: sleep eat")

    def test_curly_quotes_and_stars(self):
        self.assertEqual(talk.clean("I\u2019d pick \u201cUp\u201d."), 'I\'d pick "Up".')
        self.assertEqual(talk.clean("Something like *WALL-E*. It's fun."), "Something like WALL-E. It's fun.")
        self.assertEqual(talk.clean("Sure, happy to! *smiles*"), "Sure, happy to!")
        self.assertEqual(talk.clean("Hmm, I don't eat much."), "I don't eat much.")
        self.assertEqual(talk.clean("Mm, good point. Hmmm."), "Good point.")

    def test_first_sentences(self):
        self.assertEqual(talk.first_sentences("One. Two! Three? Four.", 3), "One. Two! Three?")
        self.assertEqual(talk.first_sentences("I'm 1.3 metres. Hi. Yes. No.", 3), "I'm 1.3 metres. Hi. Yes.")
        self.assertEqual(talk.first_sentences("Just one", 3), "Just one")

    def test_what_it_keeps(self):
        self.assertEqual(talk.clean("A well-known spot (the big oak tree by the quad)."),
                         "A well-known spot (the big oak tree by the quad).")
        self.assertEqual(talk.clean("Bye! <bye>"), "Bye! <bye>")


class TestFillers(unittest.TestCase):
    def test_kind_fits_what_they_said(self):
        for heard, kind in (("What do you think about AI?", "think"), ("What's your favourite food?", "think"),
                            ("Why are you here", "think"), ("Who built you?", "question"),
                            ("are you a real robot", "question"), ("I just got out of class.", "news"),
                            ("Fine.", "news"), ("Hi, I'm Jordan.", None), ("Hello!", None),
                            ("Okay, bye!", None), ("What?", None), ("Huh?", None),
                            ("Sorry, what did you say?", None)):
            self.assertEqual(talk.filler_kind(heard), kind, heard)

    def test_reply_doesnt_repeat_the_filler(self):
        self.assertEqual(talk.after_filler("Oh, that's great!", "Oh."), "That's great!")
        self.assertEqual(talk.after_filler("Hmm, I'd say pizza.", "Hmm."), "I'd say pizza.")
        self.assertEqual(talk.after_filler("Great question! I'd say pizza.", "Oh, good question."), "I'd say pizza.")
        self.assertEqual(talk.after_filler("Good question.", "Oh, good question."), "Good question.")
        self.assertEqual(talk.after_filler("My day was fine.", "Mm."), "My day was fine.")
        self.assertEqual(talk.after_filler("Oh.", "Oh."), "Oh.")

    def test_every_filler_is_rendered(self):
        lines = [f for kind in talk.FILLERS.values() for f in kind]
        self.assertEqual(len(lines), len(set(lines)))


class SpeakerIdle(unittest.TestCase):
    def test_idle_only_once_every_piece_said_has_played(self):
        import threading
        import numpy as np
        speaker = talk.Speaker(16000, play=False, save=None)
        playing, release = threading.Event(), threading.Event()
        def first_starts(t):
            playing.set()
            release.wait(2)
        speaker.say(np.zeros(160, np.float32), started=first_starts)
        self.assertTrue(playing.wait(2))
        speaker.say(np.zeros(160, np.float32))   # the next piece, synthesized while one plays
        self.assertFalse(speaker.idle.is_set())
        release.set()
        self.assertTrue(speaker.idle.wait(2))
        self.assertEqual((speaker.pending, speaker.queue.empty()), (0, True))

    def test_a_piece_said_as_the_last_one_ends_keeps_it_busy(self):
        # A say() landing just as a piece finishes playing, staged where the speaker once
        # checked for an empty queue before setting idle.
        import threading
        import numpy as np
        speaker = talk.Speaker(16000, play=False, save=None)
        second_plays, release = threading.Event(), threading.Event()
        def second_starts(t):
            second_plays.set()
            release.wait(2)
        def say_second():
            speaker.say(np.zeros(160, np.float32), started=second_starts)
        empty = speaker.queue.empty
        def empty_with_a_say_racing_in():
            was = empty()
            if not second_plays.is_set() and not speaker.queue.qsize() and not said:
                said.append(True)
                say_second()
            return was
        said = []
        speaker.queue.empty = empty_with_a_say_racing_in
        speaker.say(np.zeros(160, np.float32))
        self.assertTrue(speaker.idle.wait(2) or second_plays.wait(2))
        if not said:   # the speaker never asked whether its queue was empty: say it now
            said.append(True)
            say_second()
        self.assertTrue(second_plays.wait(2))
        self.assertFalse(speaker.idle.is_set())   # listen.py must stay muted while it plays
        release.set()
        self.assertTrue(speaker.idle.wait(2))


if __name__ == "__main__":
    unittest.main()
