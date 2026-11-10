"""Scores listen.py on bench/clips.py's clips (context/speech-tuning.md): turns, word
errors, false starts and how long after the person stops each turn ends.

  python3 bench/listen_bench.py [--only GROUP,...] [--sweep] [--configs JSON]

Silero runs once per clip, then each config runs listen.py's turn rules on its
probabilities (Smart Turn's answers are kept per clip too), so a sweep takes minutes. A
config is {NAME: value} for listen.py's settings (SILENCE, WAIT, FLOOR_MARGIN, its lock's
MIN_DB, LOCK_DB, REL_DB and DUCK_DB, ...), plus:
  FIXED     true: end every turn after SILENCE s, as listen.py --fixed-turn
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
             "FLOOR_MARGIN", "CHECK", "FINISHED", "WAIT", "SHORT_TURN", "MIN_DB", "LOCK_DB",
             "LOCK_S", "REL_DB", "DUCK_DB", "LEVEL_PCT"]}
SWEEP = {"SILENCE": [0.5, 0.9], "WAIT": [1.0, 1.5, 2.0], "FINISHED": [0.3, 0.7],
         "CHECK": [0.1, 0.3], "SHORT_TURN": [0.0, 1.5], "START_PROB": [0.5, 0.7],
         "END_PROB": [0.35, 0.55], "START": [0.15, 0.35], "FLOOR_MARGIN": [1.5, 2.0],
         "PREROLL": [0.5, 1.0], "FIXED": [True], "LOCK_DB": [10, 12, 20, 100], "REL_DB": [15, 25, 100],
         "DUCK_DB": [20], "MIN_DB": [-50, -100]}
SAME = {"ok": "okay", "yep": "yeah"}
FRAME_KEY = 64   # samples that, with the length, tell turns of one clip apart


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


def find_turns(frames, probs, config, finished):
    for name, value in SETTINGS.items():
        setattr(listen, name, config.get(name, value))
    turns = listen.Turns(None if config.get("FIXED") else finished)
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
    turn_end, judged = listen.TurnEnd(models / listen.SMART_TURN), {}

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

            def finished(audio, name=name):   # the same turn so far gets the same answer
                key = (name, len(audio), float(audio[:FRAME_KEY].sum()))
                if key not in judged:
                    judged[key] = turn_end(audio)
                return judged[key]

            for start, end, audio, ended in find_turns(frames, probs, config, finished):
                key = (name, round(start, 3), round(end, 3), len(audio), float(np.abs(audio).sum()))
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
                    clip_end = len(frames) * listen.FRAME / listen.RATE - listen.PAD   # wav_frames adds PAD s of zeros
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
