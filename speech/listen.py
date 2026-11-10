"""The listening half of a conversation (context/listening.md): sound in, one transcript
per thing a person says out, as JSON lines on stdout.

  python3 listen.py --wav FILE [FILE ...]   recordings, as fast as they decode (--realtime:
                                            at their own speed, as if live)
  python3 listen.py --mic [--device N]      this computer's microphone (--devices lists them)
  python3 listen.py --robot                 the G1's mic stream, on PC2 (or on a laptop
                                            on the robot's network, --iface its address)

Stages:
  source   16 kHz mono, cut into 32 ms frames (what Silero takes). The robot's mic comes
           as UDP multicast from PC1 (context/g1-builtin-apis.md), only while Unitree's
           voice assistant is in wake-up mode; other programs can read it too.
  voice    Silero VAD on each frame, plus a loudness check against the room's noise floor.
  lock     only the person close by: other voices are quieter. A turn starts only on voice
           of at least MIN_DB; once a turn has been heard, only on voice within LOCK_DB of
           that person's level (until LOCK_S s without a turn: they've gone). During a turn,
           a frame REL_DB quieter than the turn's own level is silence, so someone talking
           further off doesn't hold the turn open, and those frames are turned down DUCK_DB
           before speech-to-text.
  turn     speech starts after START s of voice (a frame without voice takes one frame
           off, so a quiet syllable doesn't start the count over). After CHECK s without
           voice, Smart Turn judges from the sound of the turn so far whether the person
           has finished: if so the turn ends there, otherwise after WAIT s without voice.
           Turns with under SHORT_TURN s of speech, which it misjudges ("Yeah."), and every
           turn with --fixed-turn, end after SILENCE s. Turns under MIN_SPEECH s are dropped,
           ones over MAX_SPEECH s are cut. The PREROLL s before the start are kept, so
           first syllables survive.
  text     the whole turn through Parakeet TDT 0.6B v2 (sherpa-onnx).
  filter   empty text and phrases Whisper-style models make up from noise are dropped
           (Parakeet made none up on noise alone).
  mute     with the mic or the robot's, nothing is heard while talk.py says the robot is
           speaking, and for TAIL s after, so it doesn't hear itself: talk.py sends
           {"speaking": true} every 0.5 s while it plays and {"speaking": false} when it
           stops, as UDP to MUTE_PORT on this computer. A mute ends HOLD s after the last
           message anyway, so it can't stay deaf if talk.py stops. A turn cut off by it is
           dropped. talk.py also sends them (with "loading": true) from its start until its
           voice and LLM are ready, so nobody is heard before it can answer; listening
           waits STARTUP s for its first message before it hears anyone.

Each line: text, start and end (s into the stream), silence_s (the wait for the turn
to end), stt_s (decoding time) and after_end_s (their sum: from the person going quiet
to the text being ready), and where Smart Turn judged, finished (its probability) and
turn_s (its time, included in after_end_s), level_db (the turn's loudness, which the lock
follows); dropped turns carry "dropped" and the reason.

Each line on stdin (Enter, in a terminal; web.py's "Done talking" button) ends the turn
being heard at once, without waiting for the silence or Smart Turn; its line carries
"ended_early": true. A line while nobody is being heard does nothing.

--events adds what listening is doing, for the web page (web.py), as lines with an
"event" key among the turns: state (waiting for talk.py to load, listening, hearing,
transcribing, or muted while the robot speaks) when it changes, and every LEVEL_S s a level (t s into the stream, the loudest frame's db and
Silero's highest prob in that time, the noise floor's floor_db, and during a turn its
speech_s so far and the quiet_s at its end), and judged when Smart Turn has judged a
silence (finished, quiet_s). talk.py --heard reads only turns, so web.py passes it only
those.

Models are found in models/ beside the main checkout, like HIM's recovery policy (or
HARP_MODELS; README.md says how to get them). Plain Python that also runs on 3.8 (PC2's).
"""

import argparse
import collections
import json
import os
from pathlib import Path
import queue
import socket
import subprocess
import sys
import threading
import time
import wave

os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")   # PC2's BLAS pool otherwise spins 7 threads
import numpy as np

RATE = 16000        # Hz
FRAME = 512         # samples per Silero step (32 ms)
CONTEXT = 64        # samples of the previous frame Silero wants in front
START_PROB = 0.6    # Silero's probability that starts speech (0.5 is its default; noise needs more)
END_PROB = 0.45     # ...and below which a frame is silence
START = 0.25        # s of voice before speech counts as started
SILENCE = 0.7       # s of silence that ends the turn (--fixed-turn)
CHECK = 0.2         # s of silence before Smart Turn judges whether the turn is finished
FINISHED = 0.5      # Smart Turn's probability from which it is
WAIT = 1.5          # s of silence that ends a turn it judged unfinished (longer leaves people waiting)
JUDGED_S = 8.0      # s of the turn's end that Smart Turn hears
SHORT_TURN = 1.0    # s of speech under which Smart Turn isn't asked: it calls one word unfinished
MIN_SPEECH = 0.3    # s: shorter turns are dropped (coughs, clicks)
MAX_SPEECH = 15.0   # s: longer turns are cut and sent as they are
PREROLL = 0.8       # s kept before speech starts (a short "Not much," before it is kept)
FLOOR_MARGIN = 1.0  # a voiced frame is louder than the noise floor (more lost quiet syllables in noise)
# The lock on the person's voice (set MIN_DB from the robot mic's recordings of someone at 1 m)
MIN_DB = -60.0      # dBFS: quieter voice never starts a turn (-50 lost a speaker at -50)
LOCK_DB = 15.0      # dB: after a turn, voice this much quieter than the person's doesn't start one
                    # (one person's turns vary by about 9 dB; 10 dropped their 12 dB softer lines)
LOCK_S = 60.0       # s without a turn before the lock is forgotten (the person has gone)
REL_DB = 20.0       # dB: during a turn, a frame this much quieter than the turn is silence
                    # (15 split lines in noise 15 dB below the voice)
DUCK_DB = 0.0       # dB those frames are turned down before speech-to-text (0: not at all;
                    # 20 turned down quiet speakers' own soft sounds: more word errors)
LEVEL_FRAMES = 8    # voiced frames before a turn's own level is trusted
LEVEL_PCT = 70      # a turn's level: this percentile of its voiced frames' loudness
MADE_UP = {"thank you for watching.", "thanks for watching."}   # Whisper-style; not "Bye." or "Thank you."
MUTE_PORT = 28732   # UDP on this computer: talk.py's speaking messages (its LLM server is on 28731;
                    # both below Linux's temporary-port range, and unused by common software)
TAIL = 0.5          # s still muted after the robot stops speaking (the room's echo)
HOLD = 1.0          # s a speaking message mutes for, unless another comes
STARTUP = 1.5       # s muted from the start, for talk.py's first message if it's there
PAD = WAIT + 0.5    # s of silence after each recording, so its last turn ends (even unfinished)
LEVEL_S = 0.096     # s between --events level lines (3 frames: smooth enough for a meter)

GROUP, PORT = "239.168.123.161", 5555   # the G1's mic stream, from PC1
PC2 = "192.168.123.164"                  # PC2's address on the robot network
PACKET = 5120                            # bytes per mic packet (160 ms)
SILERO = "silero_vad.onnx"
SMART_TURN = "smart-turn-v3.2-cpu.onnx"
PARAKEET = "sherpa-onnx-nemo-parakeet-tdt-0.6b-v2-int8"


def models_dir():
    """models/ beside the main checkout (submission/checkout.py finds HIM's the same way)."""
    if os.environ.get("HARP_MODELS"):
        return Path(os.environ["HARP_MODELS"])
    here = Path(__file__).resolve().parent
    common = subprocess.run(["git", "-C", str(here), "rev-parse", "--path-format=absolute",
                             "--git-common-dir"], capture_output=True, text=True).stdout.strip()
    main = Path(common).parent if common else here.parent
    return main.parent / "models"


def ending_lines(stream=sys.stdin):
    """An event set by each line on stdin, read in its own thread: end the turn now."""
    now = threading.Event()

    def read():
        for _ in stream:
            now.set()
    threading.Thread(target=read, daemon=True).start()
    return now


def log(*parts):
    print(*parts, file=sys.stderr, flush=True)


# Sources: each yields float32 frames of FRAME samples at RATE.

def wav_frames(paths, realtime=False):
    """Recordings back to back, each followed by PAD s of silence; realtime: no faster than
    they play."""
    began, sent = time.monotonic(), 0
    for path in paths:
        with wave.open(str(path)) as f:
            rate, channels, width = f.getframerate(), f.getnchannels(), f.getsampwidth()
            data = f.readframes(f.getnframes())
        if width != 2:
            sys.exit(f"{path}: only 16-bit WAVs")
        audio = np.frombuffer(data, np.int16).astype(np.float32).reshape(-1, channels).mean(axis=1)
        audio /= 32768
        if rate != RATE:   # linear resampling is enough for speech models at 16 kHz
            audio = np.interp(np.arange(0, len(audio) / rate, 1 / RATE),
                              np.arange(len(audio)) / rate, audio).astype(np.float32)
        audio = np.concatenate([audio, np.zeros(int(PAD * RATE), np.float32)])
        for start in range(0, len(audio) - FRAME + 1, FRAME):
            if realtime:
                time.sleep(max(0.0, began + sent * FRAME / RATE - time.monotonic()))
            sent += 1
            yield audio[start:start + FRAME]


def mic_frames(device):
    import sounddevice
    frames = queue.Queue()
    silent = 0

    def callback(data, count, when, status):
        frames.put(data[:, 0].astype(np.float32) / 32768)

    with sounddevice.InputStream(samplerate=RATE, channels=1, dtype="int16", blocksize=FRAME,
                                 device=device, callback=callback):
        log("listening on", sounddevice.query_devices(device, "input")["name"])
        while True:
            frame = frames.get()
            silent = silent + 1 if not frame.any() else 0
            if silent == int(2 * RATE / FRAME):   # macOS gives zeros, not an error, without permission
                log("the mic has sent only zeros for 2 s: on macOS, allow this terminal "
                    "app in System Settings > Privacy & Security > Microphone")
            yield frame


def robot_frames(iface):
    """The G1's mic: 16 kHz mono int16 in 160 ms UDP multicast packets from PC1."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)   # others may listen too
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1 << 18)
    sock.bind(("", PORT))
    sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP,
                    socket.inet_aton(GROUP) + socket.inet_aton(iface))
    sock.settimeout(3.0)
    log(f"listening to {GROUP}:{PORT} on {iface}")
    buffered = np.zeros(0, np.float32)
    zeros = 0
    while True:
        try:
            packet = sock.recv(8192)
        except socket.timeout:
            log("no mic packets for 3 s: is Unitree's voice assistant in wake-up mode?")
            continue
        zeros = zeros + 1 if not any(packet) else 0
        if zeros == 20:   # some firmware sends only zeros (sdk2_python issue #143)
            log("the robot's mic has sent only zeros for 3 s")
        samples = np.frombuffer(packet[:len(packet) // 2 * 2], np.int16).astype(np.float32) / 32768
        buffered = np.concatenate([buffered, samples])
        while len(buffered) >= FRAME:
            yield buffered[:FRAME]
            buffered = buffered[FRAME:]


class Voice:
    """Silero VAD: the probability that a frame is speech."""

    def __init__(self, model):
        import onnxruntime
        options = onnxruntime.SessionOptions()
        options.intra_op_num_threads = options.inter_op_num_threads = 1
        self.session = onnxruntime.InferenceSession(str(model), options,
                                                    providers=["CPUExecutionProvider"])
        self.state = np.zeros((2, 1, 128), np.float32)
        self.context = np.zeros(CONTEXT, np.float32)
        self.rate = np.array(RATE, dtype=np.int64)

    def __call__(self, frame):
        prob, self.state = self.session.run(
            None, {"input": np.concatenate([self.context, frame])[None], "state": self.state,
                   "sr": self.rate})
        self.context = frame[-CONTEXT:]
        return float(prob[0][0])

    def reset(self):
        """Forget the audio so far (after a mute, so the robot's own voice doesn't linger)."""
        self.state[:] = 0
        self.context[:] = 0


class Mute:
    """Whether the robot is speaking, from talk.py's messages on MUTE_PORT."""

    def __init__(self, port):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("127.0.0.1", port))
        self.sock.setblocking(False)
        self.until = time.monotonic() + STARTUP
        self.heard = False     # a message has come from talk.py
        self.loading = False   # talk.py is still loading its voice and LLM

    def __call__(self):
        """True while muted; reads every message waiting."""
        while True:
            try:
                message = json.loads(self.sock.recv(1024))
            except (BlockingIOError, ValueError):
                break
            now = time.monotonic()
            self.until = now + (HOLD if message.get("speaking") else TAIL)
            self.heard, self.loading = True, bool(message.get("loading"))
        return time.monotonic() < self.until


class Turns:
    """Frames and their speech probabilities in, finished turns out. Kept apart from Silero
    and Smart Turn so its timing rules can be tested with made-up probabilities
    (tests/test_turns.py). finished(audio) gives the probability that a turn is finished
    (TurnEnd); without it, SILENCE s of silence end every turn."""

    def __init__(self, finished=None):
        self.finished = finished
        self.judged = None        # finished()'s answer for this stretch of silence
        self.frame_s = FRAME / RATE
        self.preroll = collections.deque(maxlen=int(PREROLL / self.frame_s))
        self.frames = []          # the turn so far, from its preroll
        self.speaking = False
        self.voiced = 0           # frames of voice (before the start), less one per frame without
        self.first = 0            # the frame where that run of voice began
        self.quiet = 0            # frames of silence in a row (during speech)
        self.floor = None         # the room's loudness without speech (RMS)
        self.count = 0            # frames seen
        self.levels = []          # the turn's voiced frames' loudness (RMS)
        self.person = None        # the person's level (dBFS), from their turns, for the lock
        self.person_at = 0.0      # s into the stream of their last turn
        self.level = None         # the last turn's level (dBFS)

    def start_db(self):
        """The quietest voice that starts a turn now (dBFS): MIN_DB, or near the person's."""
        if self.person is None or self.count * self.frame_s - self.person_at > LOCK_S:
            return MIN_DB
        return max(MIN_DB, self.person - LOCK_DB)

    def turn_level(self):
        """The turn's loudness so far (RMS of its louder voiced frames), or None too early."""
        return float(np.percentile(self.levels, LEVEL_PCT)) if len(self.levels) >= LEVEL_FRAMES else None

    def push(self, frame, prob):
        """One frame; returns a finished turn as (start s, end s, audio) or None."""
        self.count += 1
        rms = float(np.sqrt(np.mean(frame ** 2)))
        loud = self.floor is None or rms > FLOOR_MARGIN * self.floor
        if prob < END_PROB:       # the floor follows the room while nobody speaks
            self.floor = rms if self.floor is None else 0.95 * self.floor + 0.05 * rms
        if not self.speaking and decibels(rms) < self.start_db():
            prob = min(prob, START_PROB - 1e-3)   # someone further off: not the start of a turn
        if self.speaking:
            if prob >= START_PROB:
                self.levels.append(rms)
            level = self.turn_level()
            if level is not None and rms < level * 10 ** (-REL_DB / 20):
                prob = 0.0                         # much quieter than the person: silence
        voice = prob >= START_PROB and loud
        if not self.speaking:
            self.preroll.append(frame)
            # One frame without voice takes one off rather than starting over: in noise, a
            # quiet syllable otherwise delays the start past the first words.
            if voice and not self.voiced:
                self.first = self.count
            self.voiced = self.voiced + 1 if voice else max(0, self.voiced - 1)
            if self.voiced * self.frame_s >= START:
                self.speaking, self.quiet, self.judged = True, 0, None
                self.frames = list(self.preroll)
                self.start = (self.first - 1) * self.frame_s
            return None
        self.frames.append(frame)
        self.quiet = self.quiet + 1 if prob < END_PROB else 0
        if not self.quiet:
            self.judged = None    # they went on: judge the next silence afresh
        quiet_s = self.quiet * self.frame_s
        if self.finished is None or (self.count - self.quiet) * self.frame_s - self.start < SHORT_TURN:
            wait = SILENCE
        else:
            if self.judged is None and quiet_s >= CHECK:
                self.judged = self.finished(np.concatenate(self.frames))
            wait = CHECK if self.judged is not None and self.judged >= FINISHED else WAIT
        speech_s = len(self.frames) * self.frame_s - PREROLL
        if quiet_s >= wait or speech_s >= MAX_SPEECH:
            return self.close()
        return None

    def close(self):
        """The turn being heard, as (start s, end s, audio), and ready for the next. Its
        quiet frames (other voices) are turned down, and its level sets the person's."""
        end = (self.count - self.quiet) * self.frame_s
        audio = np.concatenate(self.frames)
        level = self.turn_level()
        if level is not None and DUCK_DB:
            loudness = np.sqrt(np.mean(np.stack(self.frames) ** 2, axis=1))
            gains = np.where(loudness < level * 10 ** (-REL_DB / 20), 10 ** (-DUCK_DB / 20), 1.0)
            centres = (np.arange(len(gains)) + 0.5) * FRAME   # ramps between frames, no clicks
            audio = (audio * np.interp(np.arange(len(audio)), centres, gains)).astype(np.float32)
        self.level = decibels(level) if level is not None else None
        if self.level is not None and end - self.start >= SHORT_TURN:   # not a cough or a bang
            self.person = self.level if self.person is None else (self.person + self.level) / 2
            self.person_at = self.count * self.frame_s
        self.speaking, self.voiced, self.frames, self.levels = False, 0, [], []
        self.preroll.clear()
        return self.start, end, audio

    def cut(self):
        """End the turn being heard now (they said they're done), or None if there's none."""
        return self.close() if self.speaking else None

    def skip(self):
        """A frame not heard (muted): drop any turn in progress, keep the clock and the
        room's noise floor."""
        self.count += 1
        self.speaking, self.voiced, self.quiet, self.frames, self.levels = False, 0, 0, [], []
        self.judged = None
        self.preroll.clear()


N_FFT, HOP, MELS = 400, 160, 80   # Whisper's log-mel features, which Smart Turn takes


def mel_filters():
    """Triangular filters on Slaney's mel scale, area-normalised, 0 to 8 kHz: (MELS, bins)."""
    def mel(hz):
        return np.where(hz < 1000, 3 * hz / 200, 15 + np.log(np.maximum(hz, 1e-9) / 1000) * 27 / np.log(6.4))

    def hz(m):
        return np.where(m < 15, 200 * m / 3, 1000 * np.exp((m - 15) * np.log(6.4) / 27))

    edges = hz(np.linspace(mel(0.0), mel(RATE / 2), MELS + 2))
    bins = np.linspace(0, RATE / 2, N_FFT // 2 + 1)
    rising = (bins[None, :] - edges[:-2, None]) / (edges[1:-1] - edges[:-2])[:, None]
    falling = (edges[2:, None] - bins[None, :]) / (edges[2:] - edges[1:-1])[:, None]
    return np.maximum(0, np.minimum(rising, falling)) * (2 / (edges[2:] - edges[:-2]))[:, None]


class TurnEnd:
    """Pipecat's Smart Turn v3.2 (BSD-2): the probability that a turn is finished, from how
    the last JUDGED_S s of it sound (zeros in front when shorter)."""

    def __init__(self, model):
        import onnxruntime
        options = onnxruntime.SessionOptions()
        options.intra_op_num_threads = options.inter_op_num_threads = 1
        self.session = onnxruntime.InferenceSession(str(model), options,
                                                    providers=["CPUExecutionProvider"])
        self.window = np.hanning(N_FFT + 1)[:-1]   # periodic, as Whisper's
        self.filters = mel_filters()
        self.took = 0.0   # s the last judgement took

    def features(self, audio):
        """Whisper's log-mel: (MELS, JUDGED_S / 10 ms) from normalised audio."""
        n = int(JUDGED_S * RATE)
        audio = np.concatenate([np.zeros(max(0, n - len(audio))), audio[-n:]])
        audio = (audio - audio.mean()) / np.sqrt(audio.var() + 1e-7)
        frames = np.lib.stride_tricks.sliding_window_view(np.pad(audio, N_FFT // 2, mode="reflect"),
                                                          N_FFT)[::HOP][:-1]
        power = np.abs(np.fft.rfft(frames * self.window, axis=1)) ** 2
        logs = np.log10(np.maximum(power @ self.filters.T, 1e-10))
        logs = np.maximum(logs, logs.max() - 8.0)
        return ((logs + 4) / 4).T.astype(np.float32)

    def __call__(self, audio):
        began = time.perf_counter()
        finished = float(self.session.run(None, {"input_features": self.features(audio)[None]})[0][0][0])
        self.took = time.perf_counter() - began
        return finished


class Text:
    """Parakeet TDT through sherpa-onnx: a whole turn in, its text out."""

    def __init__(self, folder, provider):
        import sherpa_onnx
        folder = Path(folder)
        self.recognizer = sherpa_onnx.OfflineRecognizer.from_transducer(
            encoder=str(folder / "encoder.int8.onnx"), decoder=str(folder / "decoder.int8.onnx"),
            joiner=str(folder / "joiner.int8.onnx"), tokens=str(folder / "tokens.txt"),
            model_type="nemo_transducer", provider=provider, num_threads=2)

    def __call__(self, audio):
        stream = self.recognizer.create_stream()
        stream.accept_waveform(RATE, audio)
        self.recognizer.decode_stream(stream)
        return stream.result.text.strip()


def decibels(rms):
    return round(20 * float(np.log10(max(rms, 1e-5))), 1)   # dBFS, -100 for silence


class Monitor:
    """--events: state lines when listening's state changes, level lines every LEVEL_S s."""

    def __init__(self, turns):
        self.turns, self.state, self.judged = turns, None, None
        self.frames, self.rms, self.prob = 0, 0.0, 0.0

    def event(self, name, **values):
        print(json.dumps({"event": name, **values}), flush=True)

    def set(self, state):
        if state != self.state:
            self.state = state
            self.event("state", state=state)

    def frame(self, frame, prob, state):
        self.set(state)
        turns = self.turns
        if turns.judged is not None and self.judged is None:   # Smart Turn has just judged
            self.event("judged", finished=round(turns.judged, 2),
                       quiet_s=round(turns.quiet * turns.frame_s, 2))
        self.judged = turns.judged
        self.rms = max(self.rms, float(np.sqrt(np.mean(frame ** 2))))
        self.prob = max(self.prob, prob)
        self.frames += 1
        if self.frames * self.turns.frame_s >= LEVEL_S - 1e-6:
            floor, during = turns.floor, {}
            if turns.speaking:   # the turn so far: its speech, and the silence at its end
                during = {"speech_s": round((turns.count - turns.quiet) * turns.frame_s - turns.start, 2),
                          "quiet_s": round(turns.quiet * turns.frame_s, 2)}
            self.event("level", t=round(turns.count * turns.frame_s, 2),
                       db=decibels(self.rms), prob=round(self.prob, 2),
                       floor_db=None if floor is None else decibels(floor), **during)
            self.frames, self.rms, self.prob = 0, 0.0, 0.0


def dropped(text, start, end):
    """Why a turn's text isn't passed on, or None."""
    if end - start < MIN_SPEECH:
        return "short"
    if not text:
        return "empty"
    if text.lower() in MADE_UP:
        return "made_up"
    return None


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--wav", nargs="+", type=Path, help="16-bit WAV recordings")
    source.add_argument("--mic", action="store_true", help="this computer's microphone")
    source.add_argument("--robot", action="store_true", help="the G1's mic stream")
    source.add_argument("--devices", action="store_true", help="list the microphones and stop")
    parser.add_argument("--device", type=int, help="the microphone's number (--devices)")
    parser.add_argument("--iface", default=PC2, help="this computer's address on the robot network")
    parser.add_argument("--models", type=Path, default=None, help="where the models are")
    parser.add_argument("--provider", default="cpu", help="sherpa-onnx's device: cpu or cuda")
    parser.add_argument("--no-mute", action="store_true", help="hear the robot speaking too")
    parser.add_argument("--fixed-turn", action="store_true",
                        help=f"end every turn after {SILENCE} s of silence, without Smart Turn")
    parser.add_argument("--mute-port", type=int, default=MUTE_PORT, help="talk.py's speaking messages")
    parser.add_argument("--realtime", action="store_true", help="play --wav at its own speed")
    parser.add_argument("--events", action="store_true",
                        help="also print state and level lines (for web.py)")
    args = parser.parse_args()
    if args.devices:
        import sounddevice
        print(sounddevice.query_devices())
        return

    # Bound first, so talk.py's messages while it loads are heard while these models load.
    mute = None if args.wav or args.no_mute else Mute(args.mute_port)
    models = args.models or models_dir()
    needed = [models / SILERO, models / PARAKEET] + ([] if args.fixed_turn else [models / SMART_TURN])
    for path in needed:
        if not path.exists():
            sys.exit(f"{path} is missing: README.md says how to get it")
    voice = Voice(models / SILERO)
    turn_end = None if args.fixed_turn else TurnEnd(models / SMART_TURN)
    turns = Turns(turn_end)
    began = time.perf_counter()
    text = Text(models / PARAKEET, args.provider)
    log(f"models loaded in {time.perf_counter() - began:.1f} s")
    frames = (wav_frames(args.wav, args.realtime) if args.wav else mic_frames(args.device)
              if args.mic else robot_frames(args.iface))
    live = not args.wav   # say on stderr what it's doing, for a person talking to it
    state = None          # the last frame's: listening, or why it was muted
    states = {"loading": "(waiting for talk.py's voice and LLM to load)",
              "talking": "(muted while HARP talks)", "listening": "(listening)"}
    monitor = Monitor(turns) if args.events else None
    end_now = ending_lines()
    try:
        for frame in frames:
            now = "listening"
            if mute and mute():
                now = "loading" if mute.loading else "talking" if mute.heard else "starting"
            if now != state:
                if live and now in states:
                    log(states[now])
                if state not in (None, "listening") and now == "listening":
                    voice.reset()
                state = now
            if now != "listening":
                turns.skip()
                if monitor:
                    monitor.frame(frame, 0.0, "muted" if now == "talking" else "waiting")
                continue
            hearing = turns.speaking
            prob = voice(frame)
            turn = turns.push(frame, prob)
            early = turn is None and end_now.is_set() and turns.speaking
            if early:
                turn = turns.cut()
            end_now.clear()   # a press while nobody is heard doesn't end the next turn
            if live and turns.speaking and not hearing:
                log("(hearing you)")
            if monitor:
                monitor.frame(frame, prob, "transcribing" if turn else
                              "hearing" if turns.speaking else "listening")
            if turn is None:
                continue
            start, end, audio = turn
            waited = turns.count * turns.frame_s - end
            judging = turn_end.took if turn_end and turns.judged is not None else 0.0
            began = time.perf_counter()
            said = text(audio) if end - start >= MIN_SPEECH else ""
            stt = time.perf_counter() - began
            line = {"text": said, "start": round(start, 2), "end": round(end, 2),
                    "silence_s": round(waited, 2), "stt_s": round(stt, 3),
                    "after_end_s": round(waited + judging + stt, 3)}
            if turns.judged is not None:
                line["finished"] = round(turns.judged, 2)
                line["turn_s"] = round(judging, 3)
            if turns.level is not None:
                line["level_db"] = turns.level
            if early:
                line["ended_early"] = True
            reason = dropped(said, start, end)
            if reason:
                line["dropped"] = reason
            print(json.dumps(line), flush=True)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
