"""Time the local speech models on PC2 (or a laptop), to replace our estimates with
measurements (context/speech-research.md). Read-only on the robot: it makes no sound,
uses no robot service and changes no setting; it only loads models and times them.

Each stage runs only when its model is given:
  vad  Silero VAD (silero_vad.onnx) on onnxruntime, CPU, one thread: ms per 32 ms chunk.
  stt  A NeMo transducer such as Parakeet TDT 0.6B through sherpa-onnx (--stt-provider
       cuda or cpu): seconds from the end of each clip to its text.
  llm  Any OpenAI-compatible server (llama.cpp's llama-server, Ollama), streamed: time
       to the first token and the first sentence, with the prompt fresh (a new first
       line forces it all to be processed) and then cached (one more turn appended).
  tts  A voice through sherpa-onnx (Kokoro or a Piper/VITS voice folder; --tts-provider
       cuda or cpu): seconds to synthesize each sentence (the robot speaks a reply
       sentence by sentence, so this is the time to first sound).
STT runs on --wavs, or on the TTS stage's sentences when there are none. Each stage
skips its first run (loading, warm-up) in the summary.

  python3 bench.py NAME [--note "MAXN, HARP's harness running"] [--repeat 5]
      [--vad silero_vad.onnx] [--stt PARAKEET_DIR] [--stt-provider cuda]
      [--llm http://127.0.0.1:8080] [--llm-model NAME]
      [--tts VOICE_DIR [--tts-speaker 3]] [--wavs DIR]

writes captures/NAME/ beside this file:
  meta.json        the system: Jetson release, power mode (nvpmodel -q), whether
                   HARP's harness is running, load, Python and package versions
  results.jsonl    one line per timed run (stage, run, the times, the text)
  summary.json     per stage: median and 90th percentile of each time
  tegrastats.log   clocks, load, memory and temperatures every 0.5 s (Jetson only)
  tts/             the sentences the voice made

Python 3.8 (PC2's); each stage imports only what it needs. Runs at low priority beside
HARP's harness. Setup: README.md here.
"""

import argparse
import datetime
import importlib.util
import json
import os
from pathlib import Path
import platform
import random
import re
import shutil
import statistics
import subprocess
import sys
import time
import wave

os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")   # PC2's BLAS pool otherwise spins 7 threads
import numpy as np

HERE = Path(__file__).resolve().parent
RATE = 16000        # Hz, what the robot's mic sends and Silero takes
CHUNK = 512         # samples per Silero step at 16 kHz (32 ms)
CONTEXT = 64        # samples of the previous chunk Silero v5+ wants in front
HARNESS = "him_safety_nodes"   # HARP's harness processes on PC2 (context/robot-pc2.md)
SENTENCE_END = re.compile(r"[.!?](\s|$)")

SYSTEM = """You are HARP, a humanoid robot built by the Human Interaction team of \
Stanford's HARP robotics project. You walk around, notice people, walk up to them and \
talk with them. You are talking out loud with someone standing about one metre in \
front of you, so everything you say is spoken: answer in one or two short sentences, \
in plain words, with no lists, no emoji and no stage directions. Be warm and curious; \
ask simple questions about them, their day or what brings them here. You can't move \
your arms or walk while you talk, and you don't know anything about the person except \
what they tell you, so never claim to see or remember things you don't. Don't ask for \
personal details such as their full name, address or phone number. If they ask what \
you are: a Unitree G1 humanoid running software the team wrote. If they want to stop, \
say goodbye or have nothing more to say, say a short goodbye and end your reply with \
<END>."""
HISTORY = [
    ("assistant", "Hi, I'm HARP, Stanford's humanoid. Nice to meet you!"),
    ("user", "Oh wow, hi! I didn't know robots walked around here."),
    ("assistant", "I'm still learning to! What brings you here today?"),
    ("user", "I'm just heading to a lecture, but I had a few minutes."),
    ("assistant", "Then I'm glad you stopped. What's the lecture about?"),
]
TURNS = ["It's about machine learning, actually. Do you use any of that?",
         "That's cool. How do you know where people are?",
         "Do you ever bump into things?",
         "Well, I should probably go now."]
LINES = ["Hi, I'm HARP, Stanford's humanoid.",
         "Nice to meet you!",
         "What brings you here today?",
         "I find people with my lidar and then check with my camera.",
         "It was lovely talking with you, have a great lecture!"]


def run(command):
    """A command's output, or what went wrong, for meta.json."""
    try:
        result = subprocess.run(command, capture_output=True, text=True, timeout=10)
        return (result.stdout or result.stderr).strip()
    except (OSError, subprocess.SubprocessError) as error:
        return f"unavailable: {error}"


def system_info(args):
    info = {"time": datetime.datetime.now(datetime.timezone.utc).isoformat(),
            "note": args.note, "host": platform.node(), "machine": platform.machine(),
            "python": platform.python_version(), "cpus": os.cpu_count(),
            "load": os.getloadavg(), "args": vars(args)}
    release = Path("/etc/nv_tegra_release")
    if release.exists():
        info["jetson"] = release.read_text().splitlines()[0]
        info["power_mode"] = run(["nvpmodel", "-q"])
    try:
        info["harness_running"] = subprocess.run(["pgrep", "-f", HARNESS],
                                                 capture_output=True).returncode == 0
    except OSError:
        info["harness_running"] = None
    for name in ("onnxruntime", "sherpa_onnx", "numpy"):
        try:
            info[name] = __import__(name).__version__
        except Exception as error:   # not installed is fine: that stage can't run
            info[name] = f"not importable: {error}"
    return info


def read_wav(path):
    """A WAV as float32 mono and its rate."""
    with wave.open(str(path)) as f:
        rate, channels, width = f.getframerate(), f.getnchannels(), f.getsampwidth()
        frames = f.readframes(f.getnframes())
    if width != 2:
        sys.exit(f"{path}: only 16-bit WAVs")
    samples = np.frombuffer(frames, np.int16).astype(np.float32) / 32768
    return samples.reshape(-1, channels).mean(axis=1), rate


def to_16k(samples, rate):
    if rate == RATE:
        return samples
    times = np.arange(0, len(samples) / rate, 1 / RATE)
    return np.interp(times, np.arange(len(samples)) / rate, samples).astype(np.float32)


def bench_vad(model, clips, record):
    import onnxruntime as ort
    options = ort.SessionOptions()
    options.intra_op_num_threads = options.inter_op_num_threads = 1
    session = ort.InferenceSession(model, options, providers=["CPUExecutionProvider"])
    rate = np.array(RATE, dtype=np.int64)
    for i, (path, samples, clip_rate) in enumerate(clips):
        audio = to_16k(samples, clip_rate)
        state = np.zeros((2, 1, 128), np.float32)
        context = np.zeros(CONTEXT, np.float32)
        times, speech = [], 0
        for start in range(0, len(audio) - CHUNK + 1, CHUNK):
            chunk = audio[start:start + CHUNK]
            began = time.perf_counter()
            prob, state = session.run(None, {"input": np.concatenate([context, chunk])[None],
                                             "state": state, "sr": rate})
            times.append(time.perf_counter() - began)
            context = chunk[-CONTEXT:]
            speech += float(prob[0][0]) >= 0.5
        record("vad", i, clip=path.name, chunks=len(times),
               chunk_ms=1000 * statistics.median(times), speech_chunks=speech)


def find(folder, part, int8):
    """The model file in a sherpa-onnx folder, preferring the int8 one if asked."""
    files = sorted(folder.glob(f"*{part}*.onnx"))
    picked = [f for f in files if ("int8" in f.name) == int8] or files
    if not picked:
        sys.exit(f"no *{part}*.onnx in {folder}")
    return str(picked[0])


def bench_stt(folder, provider, int8, clips, repeat, record):
    import sherpa_onnx
    folder = Path(folder)
    began = time.perf_counter()
    recognizer = sherpa_onnx.OfflineRecognizer.from_transducer(
        encoder=find(folder, "encoder", int8), decoder=find(folder, "decoder", int8),
        joiner=find(folder, "joiner", int8), tokens=str(folder / "tokens.txt"),
        model_type="nemo_transducer", provider=provider, num_threads=2)
    record("stt_load", 0, seconds=time.perf_counter() - began)
    run_number = 0
    for _ in range(repeat):
        for path, samples, rate in clips:
            began = time.perf_counter()
            stream = recognizer.create_stream()
            stream.accept_waveform(rate, samples)
            recognizer.decode_stream(stream)
            seconds = time.perf_counter() - began
            audio = len(samples) / rate
            record("stt", run_number, clip=path.name, audio_s=audio, seconds=seconds,
                   rtf=seconds / audio, text=stream.result.text.strip())
            run_number += 1


def stream_chat(url, model, messages):
    """One streamed chat completion: time to first token, to first sentence, in all."""
    import urllib.request
    body = {"messages": messages, "stream": True, "max_tokens": 60, "temperature": 0.7,
            "cache_prompt": True}   # llama-server's prompt cache (its default; Ollama ignores it)
    if model:
        body["model"] = model
    request = urllib.request.Request(url.rstrip("/") + "/v1/chat/completions",
                                     json.dumps(body).encode(),
                                     {"Content-Type": "application/json"})
    began = time.perf_counter()
    first = sentence = None
    text, pieces, timings = "", 0, None
    with urllib.request.urlopen(request, timeout=120) as response:
        for raw in response:
            line = raw.decode().strip()
            if not line.startswith("data:") or line == "data: [DONE]":
                continue
            data = json.loads(line[5:])
            timings = data.get("timings", timings)   # llama-server's own counts, if sent
            choices = data.get("choices") or [{}]
            piece = (choices[0].get("delta") or {}).get("content") or ""
            if not piece:
                continue
            now = time.perf_counter() - began
            pieces += 1
            text += piece
            first = first if first is not None else now
            if sentence is None and SENTENCE_END.search(text):
                sentence = now
    total = time.perf_counter() - began
    return {"first_token_s": first, "first_sentence_s": sentence, "total_s": total,
            "pieces": pieces, "pieces_per_s": pieces / (total - first) if first and total > first else None,
            "text": text.strip(), "server_timings": timings}


def bench_llm(url, model, repeat, record):
    for i in range(repeat):
        # A fresh first line makes the server process the whole prompt; the turn after
        # it only appends, so it should reuse what the server cached.
        fresh = f"Session {random.randrange(10**9)}. {SYSTEM}"
        messages = [{"role": "system", "content": fresh}]
        messages += [{"role": role, "content": text} for role, text in HISTORY]
        messages.append({"role": "user", "content": TURNS[i % len(TURNS)]})
        reply = stream_chat(url, model, messages)
        record("llm_fresh", i, prompt_chars=sum(len(m["content"]) for m in messages), **reply)
        messages.append({"role": "assistant", "content": reply["text"]})
        messages.append({"role": "user", "content": TURNS[(i + 1) % len(TURNS)]})
        reply = stream_chat(url, model, messages)
        record("llm_cached", i, prompt_chars=sum(len(m["content"]) for m in messages), **reply)


def voice_config(folder, provider):
    """A sherpa-onnx voice folder: Kokoro (it has voices.bin) or a Piper/VITS voice. Words
    come from the folder's lexicon; sherpa-onnx 2.x has no espeak-ng."""
    import sherpa_onnx
    folder = Path(folder)
    lexicons = sorted(folder.glob("lexicon*en*.txt"))
    if not lexicons:
        sys.exit(f"{folder}: no English lexicon (lexicon*en*.txt)")
    if (folder / "voices.bin").exists():
        model = sherpa_onnx.OfflineTtsModelConfig(kokoro=sherpa_onnx.OfflineTtsKokoroModelConfig(
            model=str(folder / "model.onnx"), voices=str(folder / "voices.bin"),
            tokens=str(folder / "tokens.txt"), lexicon=str(lexicons[-1])),
            num_threads=2, provider=provider)
    else:
        onnx = [f for f in sorted(folder.glob("*.onnx"))]
        model = sherpa_onnx.OfflineTtsModelConfig(vits=sherpa_onnx.OfflineTtsVitsModelConfig(
            model=str(onnx[0]), tokens=str(folder / "tokens.txt"), lexicon=str(lexicons[0])),
            num_threads=2, provider=provider)
    return sherpa_onnx.OfflineTtsConfig(model=model)


def bench_tts(folder, speaker, provider, out, repeat, record):
    """Seconds to synthesize each sentence through sherpa-onnx, saving each as a WAV."""
    import sherpa_onnx
    out.mkdir(parents=True, exist_ok=True)
    began = time.perf_counter()
    tts = sherpa_onnx.OfflineTts(voice_config(folder, provider))
    record("tts_load", 0, seconds=time.perf_counter() - began)
    made, run_number = [], 0
    for _ in range(repeat):
        for i, line in enumerate(LINES):
            began = time.perf_counter()
            audio = tts.generate(line, sid=speaker)
            seconds = time.perf_counter() - began
            samples = np.array(audio.samples, np.float32)
            path = out / f"{run_number:03d}-{i}.wav"
            with wave.open(str(path), "wb") as f:
                f.setnchannels(1)
                f.setsampwidth(2)
                f.setframerate(audio.sample_rate)
                f.writeframes((np.clip(samples, -1, 1) * 32767).astype(np.int16).tobytes())
            length = len(samples) / audio.sample_rate
            record("tts", run_number, text=line, seconds=seconds, audio_s=length,
                   rtf=seconds / length, wav=path.name)
            made.append(path)
            run_number += 1
    return made[:len(LINES)]


def summarize(results):
    """Median and 90th percentile of each time, per stage, leaving out run 0."""
    summary = {}
    for stage in sorted({r["stage"] for r in results}):
        rows = [r for r in results if r["stage"] == stage and (r["run"] > 0 or stage == "stt_load")]
        keys = [k for k in rows[0] if k.endswith(("_s", "_ms", "seconds", "rtf", "per_s"))] if rows else []
        summary[stage] = {"runs": len(rows)}
        for key in keys:
            values = sorted(r[key] for r in rows if isinstance(r.get(key), (int, float)))
            if values:
                summary[stage][key] = {"median": round(statistics.median(values), 4),
                                       "p90": round(values[-(-9 * len(values) // 10) - 1], 4)}
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("name", help="the capture's name, under captures/")
    parser.add_argument("--note", default="", help="what this run is (power mode, what else ran)")
    parser.add_argument("--repeat", type=int, default=5, help="runs per stage (the first is warm-up)")
    parser.add_argument("--wavs", type=Path, help="a folder of 16-bit WAVs for VAD and STT")
    parser.add_argument("--vad", help="silero_vad.onnx")
    parser.add_argument("--stt", help="a sherpa-onnx NeMo transducer folder (Parakeet)")
    parser.add_argument("--stt-provider", default="cuda", choices=["cuda", "cpu"])
    parser.add_argument("--stt-int8", action="store_true", help="use the model's int8 files")
    parser.add_argument("--llm", help="an OpenAI-compatible server, e.g. http://127.0.0.1:8080")
    parser.add_argument("--llm-model", default="", help="the model's name (Ollama needs it)")
    parser.add_argument("--tts", help="a sherpa-onnx voice folder (Kokoro or Piper/VITS)")
    parser.add_argument("--tts-speaker", type=int, default=0, help="the voice's speaker id (Kokoro af_heart: 3)")
    parser.add_argument("--tts-provider", default="cpu", choices=["cuda", "cpu"])
    args = parser.parse_args()
    if not any((args.vad, args.stt, args.llm, args.tts)):
        parser.error("give at least one of --vad, --stt, --llm, --tts")
    needs = [("--vad", args.vad, "onnxruntime"), ("--stt", args.stt, "sherpa_onnx"),
             ("--tts", args.tts, "sherpa_onnx")]
    for flag, path, module in needs:
        if path and not Path(path).exists():
            parser.error(f"{flag} {path}: not found")
        if path and module and importlib.util.find_spec(module) is None:
            parser.error(f"{flag} needs the {module} package (README.md: Setup)")

    out = HERE / "captures" / args.name
    if out.exists():
        sys.exit(f"{out} exists: pick another name")
    out.mkdir(parents=True)
    os.nice(10)
    (out / "meta.json").write_text(json.dumps(system_info(args), indent=2) + "\n")
    monitor = None
    if shutil.which("tegrastats"):
        monitor = subprocess.Popen(["tegrastats", "--interval", "500"],
                                   stdout=open(out / "tegrastats.log", "w"),
                                   stderr=subprocess.DEVNULL)

    results = []
    log = open(out / "results.jsonl", "w")

    def record(stage, run_number, **values):
        row = {"stage": stage, "run": run_number, "t": time.time(), **values}
        results.append(row)
        log.write(json.dumps(row) + "\n")
        log.flush()
        shown = {k: (round(v, 3) if isinstance(v, float) else v) for k, v in values.items()
                 if k not in ("server_timings",)}
        print(stage, run_number, shown, flush=True)

    try:
        made = []
        if args.tts:
            made = bench_tts(args.tts, args.tts_speaker, args.tts_provider, out / "tts",
                             args.repeat, record)
        clips = sorted(args.wavs.glob("*.wav")) if args.wavs else made
        clips = [(path, *read_wav(path)) for path in clips]
        if (args.vad or args.stt) and not clips:
            sys.exit("VAD and STT need --wavs, or --tts to make their clips")
        if args.vad:
            bench_vad(args.vad, clips, record)
        if args.stt:
            bench_stt(args.stt, args.stt_provider, args.stt_int8, clips, args.repeat, record)
        if args.llm:
            bench_llm(args.llm, args.llm_model, args.repeat, record)
    finally:
        log.close()
        if monitor:
            monitor.terminate()
        summary = summarize(results)
        (out / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
        print(json.dumps(summary, indent=2))
        print("wrote", out)


if __name__ == "__main__":
    main()
