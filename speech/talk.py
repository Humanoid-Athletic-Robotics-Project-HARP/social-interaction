"""The talking half of a conversation (context/talking.md): what the person said in, the
robot's spoken reply out.

  python3 talk.py                      type what the person says, one line a turn
  python3 listen.py --mic | python3 talk.py --heard   listening's transcripts (use
                                       headphones: the robot's own voice isn't muted yet)

A conversation: a fixed greeting at once, then for each turn the reply from a local LLM
(llama.cpp's llama-server, which must already be running: README.md), streamed. The reply
is cut into sentences, each spoken as soon as it is ready while the LLM goes on.
  - The reply's text: numbers spelled out, and any word the voice has no entry for
    reported (sherpa-onnx skips those).
  - A filler plays if nothing is ready or playing FILLER s after the person's turn ended.
  - A fixed line instead on an LLM error, or if it hasn't answered in GIVE_UP s.
  - The end: the LLM's <bye> tag on a reply that sounds like a goodbye (FAREWELL), the
    person saying goodbye, MAX_TURNS turns or MAX_S s, or MISSES empty turns in a row.

Events, one JSON line each on stdout: said (text, and from the turn's end: llm_s to the
first sentence, ready_s to its audio, first_sound_s to its sound, which also waits for
anything still playing), unknown_words, ignored_bye (a <bye> on a reply that wasn't a
goodbye), and done (reason). --save DIR keeps everything said as WAVs; --no-play stays
silent.

Models in models/ beside the main checkout, like listen.py's. Plain Python that also runs
on 3.8 (PC2's).
"""

import argparse
import json
from pathlib import Path
import queue
import re
import sys
import tempfile
import threading
import time
import urllib.request
import wave

import numpy as np

from listen import log, models_dir

HERE = Path(__file__).resolve().parent
FILLER = 1.2       # s after the turn ends with nothing to say yet: a filler
GIVE_UP = 5.0      # s without a first sentence: a fixed line instead
MAX_TURNS = 8      # turns before it says goodbye itself
MAX_S = 90.0       # s before it says goodbye itself
MISSES = 2         # empty turns in a row before it says goodbye
MAX_TOKENS = 60    # the LLM's reply, at most (two short sentences)
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
- Answer what they actually said. A short reaction ("Oh, nice!", "Ha, fair enough.") \
fits when they tell you something; a greeting gets a greeting, and a question gets an \
answer.
- Then at most one simple question about them, their day or what they're up to. Never \
ask a question you've already asked; ask something new.
- If they seem confused or say "What?", say what you meant in other, simpler words.
- Be warm and curious. Talk about them, not about yourself.
- You only know what they tell you. Don't claim to see, remember or have done things. If \
they ask what you are: a Unitree G1 humanoid robot running software the team wrote; you \
can walk and talk but can't use your arms yet.
- Never ask for personal details like their full name, phone number or address.
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
sentence ::= [A-Za-z0-9 ,;:'"-]+ [.!?]'''

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


def speakable(text):
    """Text as the voice should read it: tags out, numbers in words."""
    text = text.replace(END_TAG, "")
    for written, said in SPOKEN.items():
        text = re.sub(rf"\b{written}\b", said, text)
    text = re.sub(r"([A-Za-z])(\d)", r"\1 \2", text)   # "A4" is "A four", not "Afour"
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


class Voice:
    """A voice through sherpa-onnx (no espeak-ng: words come only from the lexicons)."""

    def __init__(self, name, provider):
        import sherpa_onnx
        folder, self.speaker, ours = VOICES[name]
        folder = models_dir() / folder
        if not folder.exists():
            sys.exit(f"{folder} is missing: README.md says how to get it")
        theirs = sorted(folder.glob("lexicon*en*.txt"))[-1]   # Kokoro: lexicon-us-en.txt
        # One file with our words first: sherpa-onnx reads a Piper voice's "word || phonemes"
        # format only from a single lexicon, not a comma-separated list of them.
        lexicon = Path(tempfile.gettempdir()) / f"harp-{name}-lexicon.txt"
        lines = (HERE / "lexicon" / ours).read_text(encoding="utf-8").splitlines()
        lines = with_s_forms(lines + theirs.read_text(encoding="utf-8").splitlines())
        lexicon.write_text("\n".join(lines) + "\n", encoding="utf-8")
        self.words = {line.split(" ", 1)[0].lower() for line in lines}
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

    def unknown(self, text):
        return [w for w in re.findall(r"[A-Za-z][A-Za-z']*", text) if w.lower() not in self.words]

    def __call__(self, text):
        return np.array(self.tts.generate(text, sid=self.speaker).samples, np.float32)


class Speaker:
    """Plays audio in order on this computer, in its own thread, and keeps what it played."""

    def __init__(self, rate, play, save):
        self.rate, self.save, self.count = rate, save, 0
        self.queue = queue.Queue()
        self.stream = None
        if play:
            import sounddevice
            self.stream = sounddevice.OutputStream(samplerate=rate, channels=1, dtype="float32")
            self.stream.start()
        if save:
            save.mkdir(parents=True, exist_ok=True)
        self.idle = threading.Event()
        self.idle.set()
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
            end = re.search(r"[.!?](\s|$)", text)
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


class Conversation:
    def __init__(self, args):
        began = time.perf_counter()
        self.voice = Voice(args.voice, args.provider)
        self.speaker = Speaker(self.voice.rate, not args.no_play, args.save)
        self.canned = {line: self.voice(speakable(line))
                       for line in [GREETING, SORRY, LOST, CLOSE] + FILLERS}
        log(f"voice {args.voice} ready in {time.perf_counter() - began:.1f} s")
        self.url, self.grammar = args.llm, not args.no_grammar
        self.messages = [{"role": "system", "content": SYSTEM},
                         {"role": "assistant", "content": GREETING}]
        self.turns, self.misses, self.fillers = 0, 0, 0

    def event(self, name, **values):
        print(json.dumps({"event": name, **values}), flush=True)

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
        self.speaker.say(self.canned[GREETING])
        self.event("said", text=GREETING, canned=True)

    def turn(self, heard):
        """One turn; returns why the conversation ended, or None."""
        ended = time.perf_counter()
        if not heard.strip():
            self.misses += 1
            if self.misses >= MISSES:
                return self.close("misses")
            self.speaker.say(self.canned[SORRY])
            self.event("said", text=SORRY, canned=True)
            return None
        self.misses, self.turns = 0, self.turns + 1
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
                unknown = self.voice.unknown(say)
                if unknown:
                    self.event("unknown_words", words=unknown)
                if "llm" not in first:
                    first["llm"] = time.perf_counter() - ended
                audio = self.voice(say)
                first.setdefault("ready", time.perf_counter() - ended)
                self.speaker.say(audio, None if spoken else
                                 (lambda t: first.setdefault("sound", t - ended)))
                spoken.append(say)
        except (OSError, TimeoutError, ValueError) as error:
            log(f"LLM failed: {error}")
            reply = LOST
            self.speaker.say(self.canned[LOST])
        finally:
            filler.cancel()
        text = reply.replace(END_TAG, "").strip()
        bye = END_TAG in reply and bool(FAREWELL.search(text) or GOODBYE.search(heard))
        if END_TAG in reply and not bye:
            self.event("ignored_bye", text=text)
        # The history keeps only a goodbye that counts, so the model doesn't think it left.
        self.messages.append({"role": "assistant", "content": text + (" " + END_TAG if bye else "")})
        self.speaker.wait()
        self.event("said", text=text, llm_s=round(first.get("llm", 0), 3),
                   ready_s=round(first.get("ready", 0), 3),
                   first_sound_s=round(first.get("sound", 0), 3))
        if bye:
            return "bye"
        if GOODBYE.search(heard):
            return "goodbye"
        if self.turns >= MAX_TURNS or time.perf_counter() - self.began >= MAX_S:
            return self.close("time")
        return None

    def filler(self, first):
        """Fill the pause, but only while the LLM has said nothing yet and nothing is still
        playing (once there's text, its audio follows within the voice's synthesis time)."""
        if "llm" not in first and self.speaker.idle.is_set():
            line = FILLERS[self.fillers % len(FILLERS)]
            self.fillers += 1
            self.speaker.say(self.canned[line])
            self.event("said", text=line, canned=True, filler=True)

    def close(self, reason):
        self.speaker.say(self.canned[CLOSE])
        self.event("said", text=CLOSE, canned=True)
        return reason


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--heard", action="store_true",
                        help="read listen.py's JSON lines instead of typed lines")
    parser.add_argument("--voice", default="kokoro", choices=sorted(VOICES))
    parser.add_argument("--provider", default="cpu", help="sherpa-onnx's device: cpu or cuda")
    parser.add_argument("--llm", default="http://127.0.0.1:8080", help="llama-server's address")
    parser.add_argument("--no-grammar", action="store_true", help="let the LLM write freely")
    parser.add_argument("--no-play", action="store_true", help="don't play the audio")
    parser.add_argument("--save", type=Path, help="keep everything said as WAVs here")
    args = parser.parse_args()

    talk = Conversation(args)
    talk.warm()
    talk.start()
    reason = "input_ended"
    for line in sys.stdin:
        if args.heard:
            turn = json.loads(line)
            if turn.get("dropped") in ("short", "made_up"):   # coughs and noise, not turns
                continue
            heard = turn.get("text", "")
        else:
            heard = line
        reason = talk.turn(heard) or reason
        if reason != "input_ended":
            break
    talk.speaker.wait()
    talk.event("done", reason=reason, turns=talk.turns,
               seconds=round(time.perf_counter() - talk.began, 1))


if __name__ == "__main__":
    main()
