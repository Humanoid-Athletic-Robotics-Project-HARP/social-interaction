"""Made-up test clips for listen.py (context/speech-tuning.md): macOS `say` voices with
noise, room echo, a rough robot mic, quiet speakers and pauses, so listening can be tuned
without a mic or a person; and Kokoro's voices for when a turn ends (the end_ groups), since
Smart Turn judges `say`'s flat voices unfinished. macOS only (it needs `say`).

  python3 bench/clips.py      (from speech/, in its venv; about 2 minutes)

Writes out/clips/NAME.wav (16 kHz mono int16) and out/clips/manifest.json, which gives each
clip its group, the text of each turn it should give ("expect") and where its speech is
("span", s). The same clips come out on every run.
"""

import hashlib
import json
import random
import subprocess
import sys
import wave
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import talk  # noqa: E402

OUT = Path(__file__).resolve().parent / "out"
CLIPS, CACHE = OUT / "clips", OUT / "say"
RATE = 16000
PERSON_DB = -22   # dBFS: a person speaking from about a metre away, in these clips
rng = np.random.default_rng(7)
random.seed(7)

VOICES = ["Samantha", "Daniel", "Karen", "Moira", "Rishi", "Tessa", "Kathy", "Ralph",
          "Flo (English (US))", "Reed (English (UK))", "Shelley (English (UK))",
          "Grandpa (English (US))"]
LINES = [
    "Hi there, are you the robot from Stanford?",
    "Pretty good, I just got out of my chemistry class.",
    "What's your name?",
    "I'm studying computer science, mostly robotics and machine learning.",
    "Can you walk all the way to the engineering quad?",
    "Not much, I'm just waiting for my friend Priya to get here.",
    "Do you know who built you?",
    "Okay, I have to go now, it was nice meeting you.",
    "My name is Jordan and this is my first year here.",
    "Is HARP short for something?",
]
SHORT = ["Yes.", "No.", "Hi.", "Yeah.", "Okay.", "Nope.", "Sure.", "Hey!", "Bye.", "What?"]
HESITANT = [   # one turn each, said with a pause between the pieces
    ["I went to the library", "and then I got lunch with some friends."],
    ["Well, I think", "maybe I'll go to the gym later."],
    ["My favorite class is", "um", "probably the robotics one."],
    ["I'm not sure,", "let me think about it."],
    ["So yesterday I was walking past the Oval", "and I saw a bunch of people flying drones."],
]
KOKORO = [3, 0, 11, 14, 20, 25]   # Kokoro speakers: American and British, women and men
WHOLE = ["I just got out of my chemistry class.", "Pretty good, thanks for asking.",
         "Yeah, I'm a computer science major.", "What's your name?", "Okay, see you later.",
         "I'm waiting for my friend to get here.", "Is it hard to walk on two legs?",
         "We're going to the game this weekend.", "Not much, just heading home.",
         "I think robots are really cool."]
ONE_WORD = ["Yeah.", "No.", "Sure.", "Okay.", "Bye!", "Hi!"]
PAUSED = [("I went to the library,", "and then I got lunch with some friends."),
          ("Well, I think", "maybe I'll go to the gym later."),
          ("My favorite class is, um,", "probably the robotics one."),
          ("So yesterday I was walking past the Oval,", "and I saw people flying drones."),
          ("I'm studying, uh,", "mechanical engineering."),
          ("The thing is,", "I'm not really sure yet.")]
OTHERS = ["Did you finish the problem set yet?", "I think the dining hall closes at eight.",
          "We should get coffee after the lecture.", "No way, that's hilarious.",
          "Have you seen the new building on campus?", "I'm so tired, I barely slept.",
          "The midterm was way harder than I expected.", "Let's meet at the bookstore."]


def say(text, voice="Samantha", rate=180):
    """A line from macOS's voice, trimmed of its own silence (cached in out/say/)."""
    CACHE.mkdir(parents=True, exist_ok=True)
    path = CACHE / (hashlib.sha1(f"{voice}|{rate}|{text}".encode()).hexdigest()[:16] + ".wav")
    if not path.exists():
        subprocess.run(["say", "-v", voice, "-r", str(rate), "-o", str(path), "--file-format=WAVE",
                        "--data-format=LEI16@16000", text], check=True)
    with wave.open(str(path)) as f:
        audio = np.frombuffer(f.readframes(f.getnframes()), np.int16).astype(np.float32) / 32768
    sound = np.flatnonzero(np.abs(audio) > 1e-3)
    return audio[sound[0]:sound[-1] + 1] if len(sound) else audio


KOKORO_VOICE = []   # talk.Voice, loaded on first use


def kokoro(text, speaker):
    """A line in one of Kokoro's voices at 16 kHz (cached in out/say/)."""
    CACHE.mkdir(parents=True, exist_ok=True)
    path = CACHE / (hashlib.sha1(f"kokoro|{speaker}|{text}".encode()).hexdigest()[:16] + ".npy")
    if not path.exists():
        if not KOKORO_VOICE:
            KOKORO_VOICE.append(talk.Voice("kokoro", "cpu"))
        voice = KOKORO_VOICE[0]
        audio = np.array(voice.tts.generate(text, sid=speaker).samples, np.float32)
        rate = voice.rate
        np.save(path, np.interp(np.arange(0, len(audio) / rate, 1 / RATE), np.arange(len(audio)) / rate,
                                audio).astype(np.float32))
    audio = np.load(path)
    sound = np.flatnonzero(np.abs(audio) > 1e-3)
    return audio[sound[0]:sound[-1] + 1]


def rms(x):
    return float(np.sqrt(np.mean(x ** 2)) + 1e-12)


def at_db(x, db):
    """x scaled so its voiced part is db dBFS."""
    voiced = x[np.abs(x) > 0.01 * np.abs(x).max()]
    return x * (10 ** (db / 20) / rms(voiced))


def silence(s):
    return np.zeros(int(s * RATE), np.float32)


def colored(n, power):
    """Noise of RMS 1: power 0 white, 1 pink, 2 brown."""
    spectrum = rng.standard_normal(n // 2 + 1) + 1j * rng.standard_normal(n // 2 + 1)
    f = np.arange(n // 2 + 1)
    f[0] = 1
    x = np.fft.irfft(spectrum / f ** (power / 2), n).astype(np.float32)
    return x / rms(x)


def fan(n):
    """A fan's hum and rush, like the robot's own."""
    t = np.arange(n) / RATE
    hum = sum(np.sin(2 * np.pi * f * t + rng.uniform(0, 6)) / (i + 1)
              for i, f in enumerate([118, 236, 354, 472, 1900]))
    x = hum / rms(hum) * 0.5 + colored(n, 1.5)
    return (x / rms(x)).astype(np.float32)


def others_talking(n, talkers=5):
    """Other people talking nearby, all at once."""
    x = np.zeros(n, np.float32)
    for _ in range(talkers):
        at = int(rng.uniform(0, 1) * RATE)
        while at < n:
            line = say(random.choice(OTHERS), random.choice(VOICES[:8]), random.choice([170, 190, 210]))
            line = line[:n - at]
            x[at:at + len(line)] += line / rms(line)
            at += len(line) + int(rng.uniform(0.2, 1.0) * RATE)
    return x / rms(x)


def footsteps(n, period=0.55):
    x = 0.02 * colored(n, 1)
    t = 0
    while t < n:
        k = min(n - t, int(0.08 * RATE))
        x[t:t + k] += colored(k, 2) * np.exp(-np.arange(k) / (0.015 * RATE)) * 8
        t += int(period * RATE * rng.uniform(0.9, 1.1))
    return x / rms(x)


def clicks(n):
    x = 0.01 * colored(n, 0)
    for t in rng.integers(0, n - 2000, 25):
        k = int(rng.uniform(0.005, 0.05) * RATE)
        x[t:t + k] += colored(k, 0) * np.exp(-np.arange(k) / (0.005 * RATE)) * 30
    return x / rms(x)


def music(n):
    t = np.arange(n) / RATE
    x = np.zeros(n, np.float32)
    for i in range(0, n, RATE // 4):
        f = random.choice([262, 294, 330, 392, 440, 523])
        note = slice(i, min(n, i + RATE // 4))
        decay = np.exp(-np.arange(note.stop - note.start) / (0.1 * RATE))
        x[note] += (np.sin(2 * np.pi * f * t[note]) + 0.5 * np.sin(4 * np.pi * f * t[note])) * decay
    return x / rms(x)


NOISES = {"white": lambda n: colored(n, 0), "pink": lambda n: colored(n, 1),
          "brown": lambda n: colored(n, 2), "fan": fan, "babble": others_talking,
          "steps": footsteps, "clicks": clicks, "music": music}


def reverb(x, rt60):
    """A room's echo: decaying noise as its impulse response."""
    n = int(rt60 * RATE)
    response = colored(n, 0) * np.exp(-6.9 * np.arange(n) / n)
    response = response / np.sqrt(np.sum(response ** 2)) * 0.6
    response[0] = 1.0
    m = len(x) + n
    y = np.fft.irfft(np.fft.rfft(x, m) * np.fft.rfft(response, m), m)[:len(x) + n // 2]
    return y.astype(np.float32) * (rms(x) / rms(y))


def robot_mic(x):
    """A rough onboard mic: 150 Hz to 6 kHz, some saturation."""
    spectrum = np.fft.rfft(x)
    f = np.fft.rfftfreq(len(x), 1 / RATE)
    spectrum *= ((f > 150) & (f < 6000)).astype(float)
    return np.tanh(np.fft.irfft(spectrum, len(x)).astype(np.float32) * 3) / 3


def main():
    clips = {}

    def add(name, audio, group, expect, span=None, **params):
        """Write one clip. Without span, the audio is all speech and gets 1 s of silence
        before and 2 s after."""
        if span is None:
            span = [1.0, round(1.0 + len(audio) / RATE, 3)]
            audio = np.concatenate([silence(1.0), audio, silence(2.0)])
        audio = audio / max(1.0, np.abs(audio).max() / 0.99)
        with wave.open(str(CLIPS / f"{name}.wav"), "wb") as f:
            f.setnchannels(1)
            f.setsampwidth(2)
            f.setframerate(RATE)
            f.writeframes((audio * 32767).astype(np.int16).tobytes())
        clips[name] = {"group": group, "expect": expect, "span": span if expect else None, **params}

    def in_noise(speech, noise, before=1.5, after=5.0):
        """Speech with noise all through it (noise(n) gives n samples at the right level)."""
        audio = np.concatenate([silence(before), speech, silence(after)])
        return audio + noise(len(audio)), [before, round(before + len(speech) / RATE, 3)]

    CLIPS.mkdir(parents=True, exist_ok=True)
    for i, line in enumerate(LINES):   # every line, at three speeds
        for j, rate in enumerate([140, 185, 240]):
            voice = VOICES[(i * 3 + j) % len(VOICES)]
            add(f"clean_{i}_{rate}", at_db(say(line, voice, rate), PERSON_DB), "clean", [line],
                voice=voice, rate=rate)

    for i in [0, 3, 5, 9]:   # in noise, 20 to 0 dB below the person
        speech = at_db(say(LINES[i], VOICES[i % 6], 185), PERSON_DB)
        for kind in ["pink", "fan", "babble", "steps", "music"]:
            for snr in [20, 10, 5, 0]:
                audio, span = in_noise(speech, lambda n: NOISES[kind](n) * 10 ** ((PERSON_DB - snr) / 20))
                add(f"noise_{kind}_{snr}_{i}", audio, "noisy", [LINES[i]], span, noise=kind, snr=snr)

    for i in [1, 4, 6]:   # loud to very quiet, over a quiet room
        speech = say(LINES[i], VOICES[(i + 2) % 8], 185)
        for db in [-12, -30, -40, -50]:
            audio, span = in_noise(at_db(speech, db), lambda n: colored(n, 1) * 10 ** (-60 / 20), after=2.5)
            add(f"level_{-db}_{i}", audio, "level", [LINES[i]], span, db=db)

    for i in [2, 3, 7]:   # room echo, and echo through the robot's mic beside its fan
        speech = say(LINES[i], VOICES[(i + 4) % 8], 185)
        for rt60 in [0.3, 0.6, 1.0]:
            add(f"reverb_{rt60}_{i}", at_db(reverb(speech, rt60), PERSON_DB), "reverb", [LINES[i]], rt60=rt60)
        audio, span = in_noise(at_db(robot_mic(reverb(speech, 0.5)), PERSON_DB),
                               lambda n: fan(n) * 10 ** ((PERSON_DB - 12) / 20))
        add(f"robotmic_{i}", audio, "robotmic", [LINES[i]], span, snr=12)

    for i, pieces in enumerate(HESITANT):   # one turn with a pause inside
        voice = VOICES[i % 8]
        for pause in [0.3, 0.5, 0.7, 0.9, 1.2, 1.6]:
            parts = []
            for k, piece in enumerate(pieces):
                parts += [at_db(say(piece, voice, 175), PERSON_DB)] + ([silence(pause)] if k < len(pieces) - 1 else [])
            audio = np.concatenate(parts)
            add(f"pause_{pause}_{i}", audio + colored(len(audio), 1) * 10 ** (-55 / 20), "pause",
                [" ".join(pieces)], pause=pause, voice=voice)

    for i, word in enumerate(SHORT):
        for voice in VOICES[:3]:
            add(f"short_{i}_{voice[:4]}", at_db(say(word, voice, 185), PERSON_DB), "short", [word], voice=voice)

    for kind in NOISES:   # noise alone, 20 s each, 20 to 0 dB below the person
        for below in [20, 10, 0]:
            add(f"only_{kind}_{below}", NOISES[kind](20 * RATE) * 10 ** ((PERSON_DB - below) / 20),
                "noise_only", [], noise=kind, below_db=below, span=[])
    bang = colored(int(0.3 * RATE), 1) * np.exp(-np.arange(int(0.3 * RATE)) / (0.04 * RATE)) * 0.5
    cough = colored(int(0.25 * RATE), 0.5) * np.hanning(int(0.25 * RATE)) * 0.15
    add("only_bangs", np.concatenate([silence(2), bang, silence(3), cough, silence(2), cough,
                                      silence(3), bang, silence(2)]), "noise_only", [], noise="bangs", span=[])

    monologue = " ".join(LINES[1:8])   # longer than listen.py's MAX_SPEECH
    add("long_0", at_db(say(monologue, "Samantha", 175), PERSON_DB), "long", [monologue])

    for gap in [1.0, 1.5]:   # two turns
        first, second = (at_db(say(LINES[k], "Daniel", 185), PERSON_DB) for k in (0, 2))
        add(f"two_{gap}", np.concatenate([first, silence(gap), second]), "two", [LINES[0], LINES[2]], gap=gap)

    # Someone further off, talking in the pause between the person's two lines, or straight
    # after their only one: listen.py's lock should hear only the person (below_db: how much
    # quieter the other voice is; at 1 m against 2, 3, 4 m, about 6, 10, 12 dB)
    quiet = lambda n: colored(n, 1) * 10 ** (-60 / 20)
    for below in [6, 10, 15, 20]:
        for i, (k, voice, other) in enumerate([(0, "Daniel", "Karen"), (5, "Samantha", "Ralph")]):
            first, second = (at_db(say(LINES[j], voice, 185), PERSON_DB) for j in (k, k + 2))
            them = at_db(say(OTHERS[i], other, 190), PERSON_DB - below)
            for kind, parts, expect in [("gap", [first, silence(0.4), them, silence(0.4), second],
                                         [LINES[k], LINES[k + 2]]),
                                        ("after", [first, silence(0.4), them], [LINES[k]])]:
                speech = np.concatenate([silence(1.0)] + parts)
                last = parts[-1] if kind == "gap" else first
                ends = len(speech) / RATE if kind == "gap" else 1.0 + len(first) / RATE
                audio = np.concatenate([speech, silence(2.5)])
                add(f"others_{kind}_{below}_{i}", audio + quiet(len(audio)), "others", expect,
                    [1.0, round(ends, 3)], below_db=below, kind=kind)

    # The person alone, their second line quieter than their first (a voice varies by 5 to 10
    # dB between turns): the lock must still hear them
    for softer in [4, 8, 12]:
        for i, (k, voice) in enumerate([(1, "Moira"), (6, "Rishi")]):
            first = at_db(say(LINES[k], voice, 185), PERSON_DB)
            second = at_db(say(LINES[k + 1], voice, 185), PERSON_DB - softer)
            speech = np.concatenate([silence(1.0), first, silence(1.2), second])
            audio = np.concatenate([speech, silence(2.5)])
            add(f"softer_{softer}_{i}", audio + quiet(len(audio)), "softer", [LINES[k], LINES[k + 1]],
                [1.0, round(len(speech) / RATE, 3)], softer_db=softer)

    room = lambda n: colored(n, 1) * 10 ** (-60 / 20)   # a quiet room under the end_ clips
    for i, line in enumerate(WHOLE):   # when a turn ends, in Kokoro's voices
        for speaker in KOKORO:
            audio, span = in_noise(at_db(kokoro(line, speaker), PERSON_DB), room, after=4.0)
            add(f"end_whole_{i}_{speaker}", audio, "end_whole", [line], span, speaker=speaker)
    for i, word in enumerate(ONE_WORD):
        for speaker in KOKORO:
            audio, span = in_noise(at_db(kokoro(word, speaker), PERSON_DB), room, after=4.0)
            add(f"end_word_{i}_{speaker}", audio, "end_word", [word], span, speaker=speaker)
    for i, (first, rest) in enumerate(PAUSED):
        for speaker in KOKORO[:2]:
            for pause in [0.5, 0.8, 1.2, 1.6]:
                speech = np.concatenate([at_db(kokoro(first, speaker), PERSON_DB), silence(pause),
                                         at_db(kokoro(rest, speaker), PERSON_DB)])
                audio, span = in_noise(speech, room, after=4.0)
                add(f"end_pause_{pause}_{i}_{speaker}", audio, "end_pause", [first + " " + rest], span,
                    speaker=speaker, pause=pause)

    (CLIPS / "manifest.json").write_text(json.dumps(clips, indent=1))
    print(len(clips), "clips in", CLIPS)


if __name__ == "__main__":
    main()
