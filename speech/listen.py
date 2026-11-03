"""The listening half of a conversation (context/listening.md): sound in, one transcript
per thing a person says out, as JSON lines on stdout.

  python3 listen.py --wav FILE [FILE ...]   recordings, as fast as they decode
  python3 listen.py --mic [--device N]      this computer's microphone (--devices lists them)
  python3 listen.py --robot                 the G1's mic stream, on PC2 (or on a laptop
                                            on the robot's network, --iface its address)

Stages:
  source   16 kHz mono, cut into 32 ms frames (what Silero takes). The robot's mic comes
           as UDP multicast from PC1 (context/g1-builtin-apis.md), only while Unitree's
           voice assistant is in wake-up mode; other programs can read it too.
  voice    Silero VAD on each frame, plus a loudness check against the room's noise floor.
  turn     speech starts after START s of voice; the turn ends after SILENCE s without
           it. Turns under MIN_SPEECH s are dropped, ones over MAX_SPEECH s are cut. The
           PREROLL s before the start are kept, so first syllables survive.
  text     the whole turn through Parakeet TDT 0.6B v2 (sherpa-onnx).
  filter   empty text and phrases speech models make up from noise are dropped.

Each line: text, start and end (s into the stream), silence_s (the wait for the turn
to end), stt_s (decoding time) and after_end_s (their sum: from the person going quiet
to the text being ready); dropped turns carry "dropped" and the reason.

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
SILENCE = 0.7       # s of silence that ends the turn
MIN_SPEECH = 0.3    # s: shorter turns are dropped (coughs, clicks)
MAX_SPEECH = 15.0   # s: longer turns are cut and sent as they are
PREROLL = 0.5       # s kept before speech starts
FLOOR_MARGIN = 2.0  # a voiced frame is this many times louder than the noise floor (6 dB)
MADE_UP = {"thank you.", "thank you for watching.", "thanks for watching.", "you", "bye."}

GROUP, PORT = "239.168.123.161", 5555   # the G1's mic stream, from PC1
PC2 = "192.168.123.164"                  # PC2's address on the robot network
PACKET = 5120                            # bytes per mic packet (160 ms)
SILERO = "silero_vad.onnx"
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


def log(*parts):
    print(*parts, file=sys.stderr, flush=True)


# Sources: each yields float32 frames of FRAME samples at RATE.

def wav_frames(paths):
    """Recordings back to back, each followed by a second of silence so its last turn ends."""
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
        audio = np.concatenate([audio, np.zeros(RATE, np.float32)])
        for start in range(0, len(audio) - FRAME + 1, FRAME):
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


class Turns:
    """Frames and their speech probabilities in, finished turns out. Kept apart from Silero
    so its timing rules can be tested with made-up probabilities (tests/test_turns.py)."""

    def __init__(self):
        self.frame_s = FRAME / RATE
        self.preroll = collections.deque(maxlen=int(PREROLL / self.frame_s))
        self.frames = []          # the turn so far, from its preroll
        self.speaking = False
        self.voiced = 0           # frames of voice in a row (before the start)
        self.quiet = 0            # frames of silence in a row (during speech)
        self.floor = None         # the room's loudness without speech (RMS)
        self.count = 0            # frames seen

    def push(self, frame, prob):
        """One frame; returns a finished turn as (start s, end s, audio) or None."""
        self.count += 1
        rms = float(np.sqrt(np.mean(frame ** 2)))
        loud = self.floor is None or rms > FLOOR_MARGIN * self.floor
        if prob < END_PROB:       # the floor follows the room while nobody speaks
            self.floor = rms if self.floor is None else 0.95 * self.floor + 0.05 * rms
        voice = prob >= START_PROB and loud
        if not self.speaking:
            self.preroll.append(frame)
            self.voiced = self.voiced + 1 if voice else 0
            if self.voiced * self.frame_s >= START:
                self.speaking, self.quiet = True, 0
                self.frames = list(self.preroll)
                self.start = (self.count - self.voiced) * self.frame_s
            return None
        self.frames.append(frame)
        self.quiet = self.quiet + 1 if prob < END_PROB else 0
        speech_s = len(self.frames) * self.frame_s - PREROLL
        if self.quiet * self.frame_s >= SILENCE or speech_s >= MAX_SPEECH:
            end = (self.count - self.quiet) * self.frame_s
            audio = np.concatenate(self.frames)
            self.speaking, self.voiced, self.frames = False, 0, []
            self.preroll.clear()
            return self.start, end, audio
        return None


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
    args = parser.parse_args()
    if args.devices:
        import sounddevice
        print(sounddevice.query_devices())
        return

    models = args.models or models_dir()
    for path in (models / SILERO, models / PARAKEET):
        if not path.exists():
            sys.exit(f"{path} is missing: README.md says how to get it")
    voice, turns = Voice(models / SILERO), Turns()
    began = time.perf_counter()
    text = Text(models / PARAKEET, args.provider)
    log(f"models loaded in {time.perf_counter() - began:.1f} s")
    frames = (wav_frames(args.wav) if args.wav else mic_frames(args.device) if args.mic
              else robot_frames(args.iface))
    try:
        for frame in frames:
            turn = turns.push(frame, voice(frame))
            if turn is None:
                continue
            start, end, audio = turn
            began = time.perf_counter()
            said = text(audio) if end - start >= MIN_SPEECH else ""
            stt = time.perf_counter() - began
            line = {"text": said, "start": round(start, 2), "end": round(end, 2),
                    "silence_s": SILENCE, "stt_s": round(stt, 3),
                    "after_end_s": round(SILENCE + stt, 3)}
            reason = dropped(said, start, end)
            if reason:
                line["dropped"] = reason
            print(json.dumps(line), flush=True)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
