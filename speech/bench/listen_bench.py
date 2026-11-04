"""Scores listen.py on bench/clips.py's clips (context/speech-tuning.md): turns, word
errors, false starts and how long after the person stops each turn ends.

  python3 bench/listen_bench.py [--only GROUP,...] [--sweep] [--configs JSON]

Silero runs once per clip, then each config runs listen.py's turn rules on its
probabilities, so a sweep takes minutes. A config is {NAME: value} for listen.py's settings
(SILENCE, FLOOR_MARGIN, ...), plus two options still being tried:
  MIN_DB    frames quieter than this (dBFS) are never voice: other people further away
  REL_END   during a turn, a frame this many dB quieter than the person counts as silence
Per group: word errors, clips with the right number of turns, L mean lag from the end of
speech to the turn's end, and s split, m missed, h never ended (only the clip's end did).
Noise-only clips: false starts (turns passed on) per minutes of noise. Writes out/listen.json.
"""

import argparse
import collections
import json
import re
import sys
import time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
import listen  # noqa: E402
from talk import number_words  # noqa: E402

CLIPS = HERE / "out" / "clips"
SETTINGS = {k: getattr(listen, k) for k in
            ["START_PROB", "END_PROB", "START", "SILENCE", "MIN_SPEECH", "MAX_SPEECH", "PREROLL",
             "FLOOR_MARGIN"]}
SWEEP = {"SILENCE": [0.5, 0.6, 0.8, 0.9, 1.1], "START_PROB": [0.5, 0.7, 0.8],
         "END_PROB": [0.35, 0.55], "START": [0.15, 0.2, 0.35], "FLOOR_MARGIN": [1.5, 2.0, 3.0],
         "PREROLL": [0.5, 1.0]}
SAME = {"ok": "okay", "yep": "yeah"}


def words(text):
    text = re.sub(r"\d+", lambda m: number_words(int(m.group())), text.lower())
    return [SAME.get(w, w) for w in re.findall(r"[a-z']+", text.replace("-", " "))]


def errors(ref, hyp):
    """Word errors (substitutions, insertions, deletions) and the reference's length."""
    r, h = words(ref), words(hyp)
    row = list(range(len(h) + 1))
    for i in range(1, len(r) + 1):
        diagonal, row[0] = row[0], i
        for j in range(1, len(h) + 1):
            diagonal, row[j] = row[j], min(row[j] + 1, row[j - 1] + 1, diagonal + (r[i - 1] != h[j - 1]))
    return row[len(h)], max(len(r), 1)


class Trial(listen.Turns):
    """listen.Turns with MIN_DB and REL_END (see the docstring)."""

    def __init__(self, min_db=None, rel_end=None):
        super().__init__()
        self.min_db, self.rel_end, self.levels = min_db, rel_end, []

    def push(self, frame, prob):
        rms = float(np.sqrt(np.mean(frame ** 2)))
        if not self.speaking and self.min_db is not None and 20 * np.log10(rms + 1e-12) < self.min_db:
            prob = min(prob, listen.START_PROB - 1e-3)   # not voice, but the floor still follows it
        if self.speaking and self.rel_end is not None:
            if prob >= listen.START_PROB:
                self.levels.append(rms)
            if len(self.levels) >= 8 and rms < np.percentile(self.levels, 70) * 10 ** (-self.rel_end / 20):
                prob = 0.0
        elif not self.speaking:
            self.levels = []
        return super().push(frame, prob)


def find_turns(frames, probs, config):
    for name, value in SETTINGS.items():
        setattr(listen, name, config.get(name, value))
    turns = Trial(config.get("MIN_DB"), config.get("REL_END"))
    found = []
    for i, (frame, prob) in enumerate(zip(frames, probs)):
        turn = turns.push(frame, prob)
        if turn:
            found.append(turn + ((i + 1) * turns.frame_s,))   # start, end, audio, when it ended
    return found


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--only", help="groups, comma-separated")
    parser.add_argument("--sweep", action="store_true", help="each setting in SWEEP alone")
    parser.add_argument("--configs", help='JSON {"name": {"SILENCE": 0.9, ...}, ...}')
    args = parser.parse_args()
    if not (CLIPS / "manifest.json").exists():
        sys.exit("no clips: run bench/clips.py first")
    manifest = json.loads((CLIPS / "manifest.json").read_text())
    if args.only:
        manifest = {k: v for k, v in manifest.items() if v["group"] in args.only.split(",")}
    models = listen.models_dir()
    voice, text = listen.Voice(models / listen.SILERO), listen.Text(models / listen.PARAKEET, "cpu")

    data = {}
    for name in manifest:
        voice.reset()
        frames = list(listen.wav_frames([CLIPS / f"{name}.wav"]))
        data[name] = frames, [voice(f) for f in frames]

    configs = {"listen.py": {}}
    if args.sweep:
        configs.update({f"{k}={v}": {k: v} for k, values in SWEEP.items() for v in values})
    if args.configs:
        configs.update(json.loads(args.configs))

    heard, stt = {}, []   # transcripts by turn, and (turn s, decoding s)
    results = {}
    for config_name, config in configs.items():
        groups = collections.defaultdict(lambda: collections.defaultdict(float))
        detail = {}
        for name, clip in manifest.items():
            frames, probs = data[name]
            passed = []
            for start, end, audio, ended in find_turns(frames, probs, config):
                key = (name, round(start, 3), round(end, 3), len(audio))
                if key not in heard:
                    began = time.perf_counter()
                    heard[key] = text(audio) if end - start >= listen.MIN_SPEECH else ""
                    stt.append((len(audio) / listen.RATE, time.perf_counter() - began))
                if not listen.dropped(heard[key], start, end):
                    passed.append({"text": heard[key], "start": round(start, 2), "ended": round(ended, 2)})
            g = groups[clip["group"]]
            g["clips"] += 1
            expect = clip["expect"]
            if expect:
                e, n = errors(" ".join(expect), " ".join(p["text"] for p in passed))
                g["errors"] += e
                g["words"] += n
                g["right"] += len(passed) == len(expect)
                g["split"] += len(passed) > len(expect)
                g["missed"] += not passed
                if passed:
                    clip_end = len(frames) * listen.FRAME / listen.RATE - 1.0   # wav_frames adds 1 s of zeros
                    if passed[-1]["ended"] > clip_end + 0.05:
                        g["hung"] += 1
                    else:
                        g["lag"] += passed[-1]["ended"] - clip["span"][1]
                        g["lags"] += 1
            else:
                g["false"] += len(passed)
                g["seconds"] += len(frames) * listen.FRAME / listen.RATE
            detail[name] = passed
        results[config_name] = {"groups": {k: dict(v) for k, v in groups.items()}, "turns": detail}

    (HERE / "out" / "listen.json").write_text(json.dumps(results, indent=1))
    names = sorted({c["group"] for c in manifest.values()})
    print(f"{'config':24}" + "".join(f"{g:>22}" for g in names))
    for config_name, result in results.items():
        cells = []
        for group in names:
            g = result["groups"].get(group, {})
            if group == "noise_only":
                cells.append(f"{g.get('false', 0):.0f} in {g.get('seconds', 0) / 60:.0f} min")
                continue
            cell = f"{100 * g['errors'] / g['words']:.1f}% {g['right']:.0f}/{g['clips']:.0f}"
            cell += f" L{g['lag'] / g['lags']:.2f}" if g.get("lags") else ""
            for mark in ("split", "missed", "hung"):
                cell += f" {mark[0]}{g[mark]:.0f}" if g.get(mark) else ""
            cells.append(cell)
        print(f"{config_name:24}" + "".join(f"{c:>22}" for c in cells))
    times = np.array(stt)
    for low, high in [(0, 3), (3, 6), (6, 20)]:
        within = (times[:, 0] >= low) & (times[:, 0] < high)
        if within.any():
            print(f"STT for {low}-{high} s turns: median {np.median(times[within, 1]):.3f} s "
                  f"(n={within.sum()})")


if __name__ == "__main__":
    main()
