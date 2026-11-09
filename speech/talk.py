"""The talking half of a conversation (context/talking.md): what the person said in, the
robot's spoken reply out.

  python3 talk.py                      type what the person says, one line a turn
  python3 listen.py --mic | python3 talk.py --heard   listening's transcripts, out loud

While it speaks, and while it makes a reply, it tells listen.py to stop listening (UDP to
MUTE_PORT, listen.py's docstring), so it doesn't hear itself. A transcript that is still
its own voice (close to something it just said, ECHO) is dropped as an echo event.

A conversation: a fixed greeting at once, then for each turn the reply from a local LLM
(llama.cpp's llama-server, which must already be running: README.md), streamed. The reply
is cut into sentences, each spoken as soon as it is ready while the LLM goes on.
  - The reply's text: numbers spelled out, then each word as sounds: from our lexicon, the
    voice's, or misaki's G2P model for any other word (reported, so a wrong guess can be
    added to our lexicon); a short word in capitals in no lexicon is said letter by letter.
  - A filler plays if the LLM has given no text, and nothing is playing, FILLER s after
    the person's turn ended.
  - A fixed line instead on an LLM error, or if it hasn't answered in GIVE_UP s.
  - The end: a reply that sounds like a goodbye (FAREWELL), with the LLM's <bye> tag or
    after the person said goodbye; MAX_TURNS turns or MAX_S s; or MISSES empty turns in a row.

Events, one JSON line each on stdout: said (text, and from the turn's end: llm_s to the
first sentence, ready_s to its audio, first_sound_s to its sound, which also waits for
anything still playing; sent once the whole line has played), speaking (each piece's
text and seconds as its sound starts, first for a line's first piece), guessed_words
({word: sounds} from the G2P model), unknown_words (skipped: no G2P model), ignored_bye
(a <bye> on a reply that wasn't a goodbye), echo, and done (reason). --save DIR keeps
everything said as WAVs; --no-play stays silent; --output-device picks the speaker.

Models in models/ beside the main checkout, like listen.py's. Plain Python that also runs
on 3.8 (PC2's).
"""

import argparse
import json
from pathlib import Path
import queue
from difflib import SequenceMatcher
import re
import socket
import sys
import tempfile
import threading
import time
import unicodedata
import urllib.request
import wave

import numpy as np

from listen import MUTE_PORT, log, models_dir

HERE = Path(__file__).resolve().parent
FILLER = 1.2       # s after the turn ends with nothing to say yet: a filler
GIVE_UP = 5.0      # s without a first sentence: a fixed line instead
MAX_TURNS = 8      # turns before it says goodbye itself
MAX_S = 90.0       # s before it says goodbye itself
MISSES = 2         # empty turns in a row before it says goodbye
MAX_TOKENS = 60    # the LLM's reply, at most (two short sentences)
HEARTBEAT = 0.5    # s between speaking messages to listen.py (it unmutes 1 s after the last)
ECHO = 0.75        # a transcript this similar to something just said is the robot's own voice
FIRST_WORDS = 4    # a comma ends a reply's first piece once this many words come before it
CLAUSE_WORDS = 10  # ...and any later piece
VOICES = {         # model folder, speaker, our own lexicon
    "kokoro": ("kokoro-multi-lang-v1_0", 3, "kokoro-en.txt"),            # af_heart
    "piper": ("vits-piper-en_US-ljspeech-medium", 0, "piper-en.txt"),
}

GREETING = "Hi, I'm HARP, a robot from Stanford. How's your day going?"
FILLERS = ["Let me think.", "Let me see."]   # words both voices have ("Hmm" came out as "Hammel")
SPOKEN = {"G1": "gee one"}     # said as letters, not read as a word
SORRY = "Sorry, I didn't catch that."
LOST = "Sorry, I lost my train of thought. What were you saying?"
CLOSE = "I should let you go. It was lovely talking with you, have a great day!"
GOODBYE = re.compile(r"\b(bye|goodbye|see you|see ya|gotta go|got to go|have to go|need to go|"
                     r"have to run|gotta run|i'm off)\b", re.I)
END_TAG = "<bye>"
# The reply must sound like a goodbye for its END_TAG to count: when the grammar has used up
# its two sentences, END_TAG is the only way on, so a model that wanted to say more takes it.
FAREWELL = re.compile(r"\b(bye|goodbye|see you|take care|have a (good|great|nice|lovely|wonderful)|"
                      r"(nice|lovely|great|good) (meeting|talking|chatting|to meet)|enjoy (your|the))\b",
                      re.I)

SYSTEM = f"""You are HARP, a friendly humanoid robot built by students on Stanford's HARP \
robotics team. You walk around campus and stop to chat with people. You are talking out \
loud with someone standing about one metre in front of you, and you have just said: \
"{GREETING}"

How you talk:
- One or two short sentences each time, in plain spoken words, even if they ask for \
more. No lists, emoji, symbols or stage directions.
- Speak only English. If they use another language, say in English that you only speak \
English.
- Answer what they actually said. A short reaction ("Oh, nice!", "Ha, fair enough.") \
fits when they tell you something; a greeting gets a greeting, and a question gets an \
answer.
- Then at most one simple question about them, their day or what they're up to. Never \
ask a question you've already asked; ask something new.
- If they seem confused ("What?", "Huh?"), say what you meant in other, simpler words, \
never the same words as before.
- What they say comes from speech-to-text, which often mishears your name: "heart", \
"hard", "Hart" or "harp" usually means HARP, you. Answer as if they had said HARP, \
without mentioning it.
- Be warm and curious. Talk about them, not about yourself.
- You only know what they tell you. Don't claim to see, remember or have done things. If \
they ask what you are: a Unitree G1 humanoid robot, about 1.3 metres tall, running \
software the team wrote; you can walk and talk but can't use your arms yet. HARP stands \
for Humanoid Athletic Robotics Project.
- Never ask for personal details like their full name, phone number or address. If they \
give you one, say kindly that you don't need it and keep chatting.
- Don't offer to help with things or go places: you can only chat.
- If they are rude, stay kind, don't argue, and say goodbye.
- When they want to leave or have nothing more to say, say a short warm goodbye and end \
with {END_TAG}

Examples:
Person: Hello!
HARP: Hi there! What are you up to today?
Person: What?
HARP: Sorry, I was just asking what you're doing today.
Person: Pretty good, just got out of class.
HARP: Oh, nice! What was the class about?
Person: Organic chemistry, it was rough.
HARP: Ha, that sounds tough. Do you have more classes today?
Person: Are you a real robot?
HARP: I am! I'm a Unitree G1 humanoid running software the team wrote.
Person: Okay, I have to go now.
HARP: No problem, it was lovely meeting you. Have a great day! {END_TAG}"""

GRAMMAR = r'''root ::= sentence (" " sentence)? (" <bye>")?
sentence ::= ([A-Za-z0-9 ,;:'"-] | [0-9] "." [0-9])+ [.!?]'''

ONES = ("zero one two three four five six seven eight nine ten eleven twelve thirteen "
        "fourteen fifteen sixteen seventeen eighteen nineteen").split()
TENS = "_ _ twenty thirty forty fifty sixty seventy eighty ninety".split()


def number_words(n):
    """A whole number in words: 12 -> twelve, 2026 -> two thousand twenty-six."""
    if n < 20:
        return ONES[n]
    if n < 100:
        return TENS[n // 10] + ("-" + ONES[n % 10] if n % 10 else "")
    for size, name in ((10 ** 9, "billion"), (10 ** 6, "million"), (1000, "thousand"), (100, "hundred")):
        if n >= size:
            rest = n % size
            return number_words(n // size) + " " + name + (" " + number_words(rest) if rest else "")
    return str(n)


def clock(hour, minute):
    """A time in words: 1:30 -> one thirty, 9:05 -> nine oh five, 3:00 -> three."""
    if minute == 0:
        return number_words(hour)
    return number_words(hour) + (" oh " if minute < 10 else " ") + number_words(minute)


def speakable(text):
    """Text as the voice should read it: tags out, numbers in words."""
    text = text.replace(END_TAG, "")
    for written, said in SPOKEN.items():
        text = re.sub(rf"\b{written}\b", said, text)
    text = re.sub(r"([A-Za-z])(\d)", r"\1 \2", text)   # "A4" is "A four", not "Afour"
    text = re.sub(r"(\d)(?!(?:st|nd|rd|th)\b)([A-Za-z])", r"\1 \2", text)   # "106B": not "sixB"
    text = re.sub(r"\b(\d{1,2}):(\d\d)\b", lambda m: clock(int(m.group(1)), int(m.group(2))), text)
    text = re.sub(r"(\d+)\.(\d+)", lambda m: number_words(int(m.group(1))) + " point " +
                  " ".join(ONES[int(d)] for d in m.group(2)), text)
    text = re.sub(r"\d+", lambda m: number_words(int(m.group())), text.replace(",000", "000"))
    return re.sub(r"\s+", " ", text).strip()


VOICELESS = set("ptkfθ")   # a plural after these ends in s; after sibilants in ᵻz; else z
SIBILANT = set("szʃʒʧʤ")


def with_s_forms(lines):
    """Lexicon lines plus each word's -s and -'s forms where they're missing (Kokoro's list
    has "student" but not "students", "what" but not "what's"). Both voices write a line as
    the word, a space, then its sounds, ending in the last sound."""
    words = {line.split(" ", 1)[0] for line in lines}
    added = []
    for line in lines:
        word, _, sounds = line.partition(" ")
        if not word.isalpha() or not sounds.strip():
            continue
        last = sounds.strip()[-1]
        ending = " ᵻ z" if last in SIBILANT else " s" if last in VOICELESS else " z"
        if re.search(r"(s|x|z|ch|sh)$", word):
            plural = word + "es"
        elif re.search(r"[^aeiou]y$", word):
            plural = word[:-1] + "ies"
        else:
            plural = word + "s"
        for form in (plural, word + "'s"):
            if form not in words:
                words.add(form)
                added.append(form + " " + sounds.rstrip() + ending)
    return lines + added


G2P = "misaki-en-us-g2p"   # models/ folder: misaki's US English G2P model and its tables
# misaki's symbols (Kokoro's) as espeak-style ones (Piper's): checked against Piper's own list
PIPER_SOUNDS = [("ɜɹ", "ɜː"), ("əɹ", "ɚ"), ("A", "eɪ"), ("I", "aɪ"), ("O", "oʊ"), ("W", "aʊ"),
                ("Y", "ɔɪ"), ("ʤ", "dʒ"), ("ʧ", "tʃ"), ("ᵊ", "ə"), ("ɑ", "ɑː"), ("ɔ", "ɔː"), ("u", "uː")]
SPELL_UP_TO = 5      # letters: a word in capitals this short, in no lexicon, is said letter by letter
WORD = re.compile(r"[A-Za-z]+(?:'[A-Za-z]+)*|[.,!?;:]")


def piper_sounds(sounds):
    for misaki, espeak in PIPER_SOUNDS:
        sounds = sounds.replace(misaki, espeak)
    return sounds


def parts(word):
    """A joined-up name in the pieces it's said in: ChatGPT -> Chat GPT, PyTorch -> Py Torch."""
    return re.findall(r"[A-Z]+(?![a-z])|[A-Z]?[a-z]+(?:'[a-z]+)*", word) or [word]


def word_sounds(word, lexicon, guess, letters=None):
    """A word's sounds, and how they were found: known (a lexicon), spelled (letter by letter,
    with letters' names: {letter: sounds}, else the lexicon's), guessed (the G2P model), or
    None and unknown when there's no way to say it."""
    letters = letters or lexicon
    if word.lower() in lexicon:
        return lexicon[word.lower()], "known"
    pieces = parts(word)
    if len(pieces) > 1:
        found = [word_sounds(piece, lexicon, guess, letters) for piece in pieces]
        if all(sounds for sounds, _ in found):
            how = "guessed" if any(h == "guessed" for _, h in found) else "spelled" if any(
                h == "spelled" for _, h in found) else "known"
            return " ".join(sounds for sounds, _ in found), how
    if word.isupper() and len(word) <= SPELL_UP_TO and all(c.lower() in letters for c in word):
        return " ".join(letters[c.lower()] for c in word), "spelled"
    if guess:
        sounds = guess(word.lower())
        if sounds:
            return sounds, "guessed"
    return None, "unknown"


def text_sounds(text, lexicon, guess, letters=None):
    """Text as the voice's sounds, words apart and punctuation kept, and the words that had to
    be guessed ({word: sounds}) or couldn't be said at all."""
    out, guessed, unknown = [], {}, []
    text = unicodedata.normalize("NFKD", text).encode("ascii", "ignore").decode()   # café -> cafe
    for token in WORD.findall(text):
        if not token[0].isalpha():
            if out:   # one mark at most, also when the words between were skipped
                out[-1] = out[-1].rstrip(".,!?;:") + token
            continue
        sounds, how = word_sounds(token, lexicon, guess, letters)
        if how == "guessed":
            guessed[token] = sounds
        elif how == "unknown":
            unknown.append(token)
        if sounds:
            out.append(sounds)
    return " ".join(out), guessed, unknown


class Guess:
    """misaki's US English G2P (Apache-2.0): a word's sounds from its spelling, in misaki's
    symbols, which are Kokoro's. Its small BART model, decoded greedily, one step at a time."""

    def __init__(self, folder):
        import onnxruntime
        tables = json.loads((folder / "misaki-en_us-g2p.json").read_text(encoding="utf-8"))
        self.graphemes, self.phonemes = tables["graphemes"], tables["phonemes"]
        self.bos, self.eos, self.start = tables["bos_id"], tables["eos_id"], tables["start_id"]
        self.unknown, self.steps = tables["unknown_id"], tables["max_steps"]
        options = onnxruntime.SessionOptions()
        options.intra_op_num_threads = options.inter_op_num_threads = 1
        self.session = onnxruntime.InferenceSession(str(folder / "misaki-en_us-g2p.onnx"), options,
                                                    providers=["CPUExecutionProvider"])
        self.cache = {}

    def __call__(self, word):
        if word not in self.cache:
            ids = [self.bos] + [self.graphemes.index(c) if c in self.graphemes[4:] else self.unknown
                                for c in word] + [self.eos]
            out = [self.start]
            for _ in range(self.steps):
                logits = self.session.run(None, {"input_ids": np.array([ids], np.int64),
                                                 "decoder_input_ids": np.array([out], np.int64)})[0]
                step = int(logits[0, -1].argmax())
                if step == self.eos:
                    break
                out.append(step)
            self.cache[word] = "".join(self.phonemes[i] for i in out[1:] if i >= 4)   # 0-3: special ids
        return self.cache[word]


class Voice:
    """A voice through sherpa-onnx, given sounds rather than text (no espeak-ng): each word's
    sounds come from our lexicon, then the voice's own, then misaki's G2P model (text_sounds)."""

    def __init__(self, name, provider):
        import sherpa_onnx
        self.sherpa = sherpa_onnx
        folder, self.speaker, ours = VOICES[name]
        folder = models_dir() / folder
        if not folder.exists():
            sys.exit(f"{folder} is missing: README.md says how to get it")
        theirs = sorted(folder.glob("lexicon*en*.txt"))[-1]   # Kokoro: lexicon-us-en.txt
        # One file with our words first: sherpa-onnx wants a lexicon even when given sounds, and
        # reads a Piper voice's "word || phonemes" format only from a single file.
        lexicon = Path(tempfile.gettempdir()) / f"harp-{name}-lexicon.txt"
        lines = (HERE / "lexicon" / ours).read_text(encoding="utf-8").splitlines()
        lines = with_s_forms(lines + theirs.read_text(encoding="utf-8").splitlines())
        lexicon.write_text("\n".join(lines) + "\n", encoding="utf-8")
        self.lexicon = {}
        for line in lines:
            word, _, sounds = line.partition(" || ") if " || " in line else line.partition(" ")
            if sounds.strip():
                self.lexicon.setdefault(word.strip().lower(), sounds.replace(" ", ""))
        # Letters' names from the voice's own list: ours says "a" as in "a day", not "ay".
        self.letters = {}
        for line in theirs.read_text(encoding="utf-8").splitlines():
            word, _, sounds = line.partition(" || ") if " || " in line else line.partition(" ")
            if len(word.strip()) == 1 and word.strip().isalpha():
                self.letters.setdefault(word.strip().lower(), sounds.replace(" ", ""))
        self.guess = None
        if (models_dir() / G2P).exists():
            misaki = Guess(models_dir() / G2P)
            self.guess = misaki if name == "kokoro" else (lambda word: piper_sounds(misaki(word)))
        else:
            log(f"{models_dir() / G2P} is missing (README.md): words in no lexicon will be skipped")
        if name == "kokoro":
            model = sherpa_onnx.OfflineTtsKokoroModelConfig(
                model=str(folder / "model.onnx"), voices=str(folder / "voices.bin"),
                tokens=str(folder / "tokens.txt"), lexicon=str(lexicon))
            config = sherpa_onnx.OfflineTtsModelConfig(kokoro=model, num_threads=2, provider=provider)
        else:
            model = sherpa_onnx.OfflineTtsVitsModelConfig(
                model=str(next(folder.glob("*.onnx"))), tokens=str(folder / "tokens.txt"),
                lexicon=str(lexicon))
            config = sherpa_onnx.OfflineTtsModelConfig(vits=model, num_threads=2, provider=provider)
        self.tts = sherpa_onnx.OfflineTts(sherpa_onnx.OfflineTtsConfig(model=config))
        self.rate = self.tts.sample_rate

    def sounds(self, text):
        """(sounds, {guessed word: sounds}, [words it can't say])"""
        return text_sounds(text, self.lexicon, self.guess, self.letters)

    def __call__(self, text, sounds=None):
        if sounds is None:
            sounds = self.sounds(text)[0]
        if not sounds:
            return np.zeros(0, np.float32)
        config = self.sherpa.GenerationConfig()
        config.sid = self.speaker
        config.phoneme_codepoints = [[ord(c) for c in sounds]]   # one list per sentence: one here
        return np.array(self.tts.generate(text, config).samples, np.float32)


class Mute:
    """Tells listen.py not to listen, in its own thread: from the start while the voice and
    the LLM load (loading, so it doesn't hear people before it can answer them), and while
    the robot plays audio (playing) or makes a reply (busy). {"speaking": true, "loading":
    ...} as UDP to MUTE_PORT every HEARTBEAT s, then {"speaking": false} once when it stops."""

    def __init__(self, port=MUTE_PORT):
        self.loading, self.busy = True, False
        self.playing = lambda: False
        self.sock, self.port = socket.socket(socket.AF_INET, socket.SOCK_DGRAM), port
        threading.Thread(target=self.run, daemon=True).start()

    def run(self):
        speaking, last = False, 0.0
        while True:
            now = self.loading or self.busy or self.playing()
            if now and (not speaking or time.monotonic() - last >= HEARTBEAT):
                self.send({"speaking": True, "loading": self.loading})
                last = time.monotonic()
            elif speaking and not now:
                self.send({"speaking": False})
            speaking = now
            time.sleep(0.05)

    def send(self, message):
        try:
            self.sock.sendto(json.dumps(message).encode(), ("127.0.0.1", self.port))
        except OSError:
            pass   # nobody listening is fine


class Speaker:
    """Plays audio in order on this computer, in its own thread, and keeps what it played.
    listen.py stays muted while it plays (mute)."""

    def __init__(self, rate, play, save, device=None, mute=None):
        self.rate, self.save, self.count = rate, save, 0
        self.queue = queue.Queue()
        self.stream = None
        if play:
            import sounddevice
            self.stream = sounddevice.OutputStream(samplerate=rate, channels=1, dtype="float32",
                                                   device=device)
            self.stream.start()
        if save:
            save.mkdir(parents=True, exist_ok=True)
        self.idle = threading.Event()
        self.idle.set()
        if mute:
            mute.playing = lambda: not self.idle.is_set()
        threading.Thread(target=self.run, daemon=True).start()

    def say(self, audio, started=None):
        """Queue audio; started(t) is called with the time its sound begins."""
        self.idle.clear()
        self.queue.put((audio, started))

    def run(self):
        while True:
            audio, started = self.queue.get()
            if started:
                started(time.perf_counter())
            if self.stream:
                self.stream.write(audio.reshape(-1, 1))
            if self.save:
                self.count += 1
                with wave.open(str(self.save / f"{self.count:03d}.wav"), "wb") as f:
                    f.setnchannels(1)
                    f.setsampwidth(2)
                    f.setframerate(self.rate)
                    f.writeframes((np.clip(audio, -1, 1) * 32767).astype(np.int16).tobytes())
            if self.queue.empty():
                self.idle.set()

    def wait(self):
        self.idle.wait()


def ending(reply, heard):
    """Why this reply ends the conversation, or None. Only a reply that sounds like a goodbye
    (FAREWELL) ends it, with the model's END_TAG or after the person's goodbye: the person's
    words alone end too much ("I have to go to the dentist tomorrow")."""
    if not FAREWELL.search(reply.replace(END_TAG, "")):
        return None
    if END_TAG in reply:
        return "bye"
    return "goodbye" if GOODBYE.search(heard) else None


def stream_reply(url, messages, grammar):
    """The LLM's reply as it comes, piece by piece."""
    body = {"messages": messages, "stream": True, "max_tokens": MAX_TOKENS, "temperature": 0.7,
            "cache_prompt": True}
    if grammar:
        body["grammar"] = GRAMMAR
    request = urllib.request.Request(url.rstrip("/") + "/v1/chat/completions",
                                     json.dumps(body).encode(), {"Content-Type": "application/json"})
    with urllib.request.urlopen(request, timeout=GIVE_UP) as response:   # each read, so a stall too
        for raw in response:
            line = raw.decode().strip()
            if line.startswith("data:") and line != "data: [DONE]":
                choices = json.loads(line[5:]).get("choices") or [{}]
                piece = (choices[0].get("delta") or {}).get("content")
                if piece:
                    yield piece


def pieces(stream):
    """Text to speak, cut at sentence ends, and at the first comma with enough words before
    it: FIRST_WORDS for a reply's first piece, so its sound starts sooner, CLAUSE_WORDS
    after that (shorter pieces sound flat)."""
    text, words = "", FIRST_WORDS
    for piece in stream:
        text += piece
        while True:
            # a "." just after a digit may be a decimal point still streaming ("1." then "3")
            end = re.search(r"[.!?]\s|[!?]$|(?<!\d)\.$", text)
            for comma in re.finditer(r"[,;:]\s", text):
                if end and comma.start() > end.start():
                    break
                if len(text[:comma.start()].split()) >= words:
                    end = comma
                    break
            if not end:
                break
            yield text[:end.end()].strip()
            text, words = text[end.end():], CLAUSE_WORDS
    if text.strip():
        yield text.strip()


def timing(said):
    """A reply's times in words, from a said event."""
    think, sound = said["llm_s"], said["first_sound_s"]
    if "heard_s" in said:
        return (f"first sound {said['heard_s'] + sound:.1f} s after you stopped: "
                f"{said['heard_s']:.1f} s to hear you, {think:.1f} s to think, "
                f"{sound - think:.1f} s to speak")
    return f"first sound {sound:.1f} s after your line: {think:.1f} s to think, {sound - think:.1f} s to speak"


class Conversation:
    def __init__(self, args, mute):
        began = time.perf_counter()
        self.mute, self.json, self.printing = mute, args.json, threading.Lock()
        self.voice = Voice(args.voice, args.provider)
        self.speaker = Speaker(self.voice.rate, not args.no_play, args.save,
                               args.output_device, mute)
        self.said = []   # what it has said, newest last (for the echo check)
        self.canned = {line: self.voice(speakable(line))
                       for line in [GREETING, SORRY, LOST, CLOSE] + FILLERS}
        log(f"voice {args.voice} ready in {time.perf_counter() - began:.1f} s")
        self.url, self.grammar = args.llm, not args.no_grammar
        self.messages = [{"role": "system", "content": SYSTEM},
                         {"role": "assistant", "content": GREETING}]
        self.turns, self.misses, self.fillers = 0, 0, 0

    def show(self, line):
        """A line of the readable conversation (stdout is a terminal and not --json)."""
        if not self.json:
            with self.printing:
                print(line, flush=True)

    def event(self, name, **values):
        if name == "said":
            self.said.append(values["text"])
        if self.json:
            with self.printing:   # the speaker's thread sends speaking events too
                print(json.dumps({"event": name, **values}), flush=True)
        elif name == "said" and "first_sound_s" in values:
            self.show("      (" + timing(values) + ")")
        elif name == "echo":
            self.show(f'      (ignored "{values["text"]}": it sounded like my own voice)')
        elif name == "unknown_words":
            self.show("      (the voice can't say: " + ", ".join(values["words"]) + ")")
        elif name == "guessed_words":
            self.show("      (guessed how to say: " + ", ".join(values["words"]) + ")")
        elif name == "done":
            self.show(f"(conversation over: {values['reason']}, {values['turns']} turns, "
                      f"{values['seconds']} s)")

    def say(self, text, audio, first=True, started=None):
        """Queue audio; the readable conversation shows its text when its sound starts, and
        started(t) is called then too."""
        def begin(t):
            if started:
                started(t)
            if self.json:
                self.event("speaking", text=text.replace(END_TAG, "").strip(), first=first,
                           seconds=round(len(audio) / self.voice.rate, 2))
            self.show(("HARP: " if first else "      ") + text.replace(END_TAG, "").strip())
        self.speaker.say(audio, begin)

    def echo(self, heard):
        """True if what was heard is the robot's own voice, leaked past listening's mute:
        close to something it said lately, a piece of it, or the last word or two it said."""
        heard = " ".join(re.findall(r"[a-z']+", heard.lower()))
        last = " ".join(re.findall(r"[a-z']+", self.said[-1].lower())) if self.said else ""
        if heard and len(heard.split()) <= 2 and (" " + last).endswith(" " + heard):
            return True   # the tail of what it just said, heard as the mute ended ("day going")
        for said in self.said[-3:]:
            said = " ".join(re.findall(r"[a-z']+", said.lower()))
            if heard and (SequenceMatcher(None, heard, said).ratio() >= ECHO
                          or (len(heard.split()) >= 3 and heard in said)):
                return True
        return False

    def warm(self):
        """Process the fixed start of the prompt once, so every turn reuses it from the cache
        (this first one may take several seconds, so no GIVE_UP here)."""
        body = {"messages": self.messages + [{"role": "user", "content": "Hi."}],
                "max_tokens": 1, "cache_prompt": True}
        request = urllib.request.Request(self.url.rstrip("/") + "/v1/chat/completions",
                                         json.dumps(body).encode(),
                                         {"Content-Type": "application/json"})
        try:
            urllib.request.urlopen(request, timeout=120).read()
        except OSError as error:
            sys.exit(f"no LLM at {self.url} ({error}): start llama-server first (README.md)")

    def start(self):
        self.began = time.perf_counter()
        self.say(GREETING, self.canned[GREETING])
        self.mute.loading = False   # after the greeting is queued, so listening stays muted
        self.event("said", text=GREETING, canned=True)

    def turn(self, heard, heard_s=None):
        """One turn; returns why the conversation ended, or None. heard_s: listening's time
        from the person going quiet to the text, for the readable timing."""
        ended = time.perf_counter()
        if not heard.strip():
            self.misses += 1
            if self.misses >= MISSES:
                return self.close("misses")
            self.say(SORRY, self.canned[SORRY])
            self.event("said", text=SORRY, canned=True)
            return None
        self.misses, self.turns = 0, self.turns + 1
        self.mute.busy = True   # listening stays muted until the whole reply has played
        try:
            return self.reply(heard, ended, heard_s)
        finally:
            self.mute.busy = False

    def reply(self, heard, ended, heard_s=None):
        self.messages.append({"role": "user", "content": heard.strip()})
        first, reply, spoken = {}, "", []
        filler = threading.Timer(FILLER, self.filler, args=(first,))
        filler.start()
        try:
            llm = stream_reply(self.url, self.messages, self.grammar)
            for text in pieces(llm):
                reply = (reply + " " + text).strip()
                say = speakable(text)
                if not say:
                    continue
                sounds, guessed, unknown = self.voice.sounds(say)
                if guessed:
                    self.event("guessed_words", words=guessed)
                if unknown:
                    self.event("unknown_words", words=unknown)
                if "llm" not in first:
                    first["llm"] = time.perf_counter() - ended
                audio = self.voice(say, sounds)
                first.setdefault("ready", time.perf_counter() - ended)
                self.say(text, audio, not spoken, None if spoken else
                         (lambda t: first.setdefault("sound", t - ended)))
                spoken.append(say)
        except (OSError, TimeoutError, ValueError) as error:
            log(f"LLM failed: {error}")
            reply = LOST
            self.say(LOST, self.canned[LOST], not spoken)
        finally:
            filler.cancel()
        text = reply.replace(END_TAG, "").strip()
        end = ending(reply, heard)
        if END_TAG in reply and not end:
            self.event("ignored_bye", text=text)
        # The history keeps only a goodbye that counts, so the model doesn't think it left.
        self.messages.append({"role": "assistant", "content": text + (" " + END_TAG if end else "")})
        self.speaker.wait()
        times = {"heard_s": heard_s} if heard_s is not None else {}
        self.event("said", text=text, llm_s=round(first.get("llm", 0), 3),
                   ready_s=round(first.get("ready", 0), 3),
                   first_sound_s=round(first.get("sound", 0), 3), **times)
        if end:
            return end
        if self.turns >= MAX_TURNS or time.perf_counter() - self.began >= MAX_S:
            return self.close("time")
        return None

    def filler(self, first):
        """Fill the pause, but only while the LLM has said nothing yet and nothing is still
        playing (once there's text, its audio follows within the voice's synthesis time)."""
        if "llm" not in first and self.speaker.idle.is_set():
            line = FILLERS[self.fillers % len(FILLERS)]
            self.fillers += 1
            self.say(line, self.canned[line])
            self.event("said", text=line, canned=True, filler=True)

    def close(self, reason):
        self.say(CLOSE, self.canned[CLOSE])
        self.event("said", text=CLOSE, canned=True)
        return reason


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--heard", action="store_true",
                        help="read listen.py's JSON lines instead of typed lines")
    parser.add_argument("--voice", default="kokoro", choices=sorted(VOICES))
    parser.add_argument("--provider", default="cpu", help="sherpa-onnx's device: cpu or cuda")
    parser.add_argument("--llm", default="http://127.0.0.1:28731", help="llama-server's address")
    parser.add_argument("--no-grammar", action="store_true", help="let the LLM write freely")
    parser.add_argument("--no-play", action="store_true", help="don't play the audio")
    parser.add_argument("--save", type=Path, help="keep everything said as WAVs here")
    parser.add_argument("--output-device", help="the speaker's number or name (listen.py --devices)")
    parser.add_argument("--mute-port", type=int, default=MUTE_PORT, help="where listen.py hears it's speaking")
    parser.add_argument("--json", action="store_true",
                        help="JSON events on stdout even in a terminal (always when piped)")
    args = parser.parse_args()
    if args.output_device and args.output_device.isdigit():
        args.output_device = int(args.output_device)
    args.json = args.json or not sys.stdout.isatty()

    mute = Mute(args.mute_port)   # first, so listen.py doesn't hear anyone while this loads
    talk = Conversation(args, mute)
    talk.warm()
    log("ready" + (": say something after the greeting" if args.heard else
                   ": type what the person says, one line a turn (Ctrl-D to end)"))
    talk.start()
    reason = "input_ended"
    for line in sys.stdin:
        heard_s = None
        if args.heard:
            turn = json.loads(line)
            if turn.get("dropped") in ("short", "made_up"):   # coughs and noise, not turns
                talk.show("      (ignored a short sound)" if turn["dropped"] == "short" else
                          f'      (ignored "{turn.get("text", "")}": noise heard as words)')
                continue
            heard, heard_s = turn.get("text", ""), turn.get("after_end_s")
            if talk.echo(heard):
                talk.event("echo", text=heard)
                continue
            talk.show("you:  " + (heard or "(I couldn't make out the words)"))
        else:
            heard = line
        reason = talk.turn(heard, heard_s) or reason
        if reason != "input_ended":
            break
    talk.speaker.wait()
    talk.event("done", reason=reason, turns=talk.turns,
               seconds=round(time.perf_counter() - talk.began, 1))


if __name__ == "__main__":
    main()
