---
name: talking
description: Read before working on what the robot says and how (the README's "Speaker" and LLM milestones), or on where the speech processes run and how they meet behavior.py
metadata:
  type: project
  updated: 2026-11-03T02:50:21Z
  covers: [testground/speech/*]
---

Design for the talking half of a conversation, and what both halves share. Nothing is built; every choice is **proposed** unless marked **decided**, and decisions are recorded here as they are made. The listening half: [[listening]]. Options, licenses and research behind the choices: [[talking-research]], speeds on PC2: [[speech-research]]. The robot's audio calls: [[g1-builtin-apis]].

**Its job (decided, 2026-11-02; the split: [[listening]]):** own the conversation, the LLM included. It opens one when `behavior.py` has walked up to someone, decides each reply with an LLM, speaks it, and ends the conversation. Its input is listening's transcripts (or typed lines while testing).

**Shared by both halves**
- Each runs as its own process, beside our image, not in it: on a laptop while developing, then on PC2. The image can't host them: it is Python 3.8, has no network in the sim and no `/dev/snd`. HIM's launcher starts it with host networking, so a PC2 process reaches the same ROS 2 graph (domain 97, localhost) and Unitree's DDS.
- **Decided:** local models wherever latency allows, cloud only as a fallback (a teammate's direction, 2026-11-02). Estimated all-local time to first sound on PC2: about 1.1–1.6 s ([[speech-research]]).
- Keep work off PC2's CPU (HIM's 0.2 s deadlines, [[safety]]) and on its GPU. The GPU is free, since HIM's walker runs on the CPU (`CPUExecutionProvider`), and HIM's launcher adds `--runtime nvidia` with `G1_NVIDIA=1`. PC2 is an Orin NX 16GB on JetPack 5.1.1 ([[robot-pc2]]). Any cloud API keys live in ignored files, never in the image or git.
- Every speed is an estimate until measured on PC2 with `testground/speech/bench.py` (its README has the setup). It times each model on recordings and a built-in conversation of about 300 tokens, logging `nvpmodel -q` and `tegrastats`. Run it with HIM's harness off and on. Installing its models in our folder on PC2 needs the team's OK.
- Measured so far, on an M1 MacBook Air's CPU (2026-11-03; llama.cpp's Metal GPU code fails on the M1: [[talking-research]]):
  - Qwen3-4B: first sentence 1.17 s with the prompt cached, 7.2 s fresh; 15 tokens a second
  - Kokoro: real-time factor 0.55, so 0.65–1.6 s a sentence
  - Piper `ljspeech`: real-time factor 0.05
  - Parakeet: 0.08 s per sentence on Kokoro's audio
  - Silero: 0.12 ms a chunk
- The core is plain Python. A thin ROS 2 adapter connects it on the robot; on a laptop (no ROS 2) it uses the terminal.
- The handshake with `behavior.py` ([[behavior-loop]]): on reaching someone, it sends `{"start": person, "distance": m}` on `/harp/talk` and waits for a `talk_done` event, which replaces its fixed wait. If nothing answers (the sim has no speech) it keeps the fixed wait, so the courses still pass. It sends `{"stop": person}` when the person leaves.

**Pipeline**
1. Opening: a fixed greeting, pre-rendered as audio, plays the moment the robot arrives, with no LLM wait.
2. Reply: each `/harp/heard` text, plus the conversation so far, goes to a local LLM through llama.cpp's `llama-server` on PC2's GPU (Ollama's `jetpack5` build if it won't build), streamed.
   - Model (**decided**, a teammate's call, 2026-11-03): Qwen3-4B-Instruct-2507 Q4_K_M (Apache-2.0; about 0.75 s to the first sentence, cached), or Qwen3-1.7B (Apache-2.0, about 0.4 s) if that's too slow. Not Llama 3.2: its license restricts use ([[talking-research]]).
   - Cloud fallback: Claude Haiku 5.5 (`claude-haiku-5-5`), thinking off.
   - Keep the prompt append-only so llama.cpp reuses its cache. Changing robot state goes after the history, never in the system prompt. Cap the context at 4,096 tokens.
   - Topics (**decided**, 2026-11-03): small talk about the person: a greeting, one easy question about them or their day, a short friendly back-and-forth, a warm goodbye. Few facts needed, so less to make up.
   - The system prompt covers:
   - Who HARP is: Stanford's humanoid, from the Human Interaction team.
   - Spoken style: one or two short sentences, simple questions, no lists or emoji.
   - Limits: don't ask for personal details, and don't promise actions it can't do.
   - Current state: the conversation's length and turn count.
3. Waiting: a short pre-rendered filler plays if no reply sound has come about 1.2 s after the person's turn ends. A canned reply is used on an LLM error or after 5 s.
4. Ending: a closing line, then `talk_done` with the reason. The reason is one of:
   - the model ends its reply with an `<END>` tag, checked in code (more reliable than tool calls for small models)
   - the person says goodbye
   - listening reports about 8 s of silence
   - about 90 s in all
   - `behavior.py`'s stop
5. Speech: sherpa-onnx's TTS, the package listening already uses, with no espeak-ng (GPL) anywhere. Never Piper's C++ binary or the `piper-tts` package: both carry GPL code ([[talking-research]]).
   - Voice (**decided**, 2026-11-03: compare both by ear and speed, then pick one): Kokoro v1.0 `af_heart` (Apache-2.0; on PC2's GPU about 0.35 s to first audio) and Piper `ljspeech` through sherpa-onnx (public domain; light on the CPU).
   - Text is split at sentence ends, and at commas once 8–12 words have built up, then synthesized while the previous piece plays.
   - Numbers are spelled out first (inflect), and our own lexicon adds names, since sherpa-onnx drops words it has no entry for.
   - Fixed phrases (the greeting, fillers, goodbyes) are rendered ahead as WAV.
6. Output: laptop speakers, or the robot's PlayStream (resampled to 16 kHz with scipy).
   - One worker thread makes every PlayStream call and checks its code.
   - Audio goes in 100–200 ms chunks, paced by a clock, with about 0.3–0.5 s queued ahead. Each call blocks about 60 ms, and gaps are audible.
   - Each reply gets its own `stream_id` (nanoseconds plus a counter) and one fixed `app_name`. To cut a reply off, empty our queue and call PlayStop directly: the SDK's wrapper hides its return code.
   - Speaking ends at `play_state` 0 on `rt/audio_msg`, or else at the audio's length plus a margin.
   - Unitree's TtsMaker (`speaker_id` 1) needs nothing installed, for the very first speaker test.
7. LED (1010): one colour each for listening, thinking and speaking.

**Publishes** (events on `/harp/events`, source `talk`):
- `said`: text, time to first sound, each stage's time
- `speaking`: on or off, for listening's gate
- `talk_done`: the reason

**Measuring:** time from listening's turn end to first sound; target 1–1.5 s ([[speech-research]]). Keep each whole conversation's text in the run's log.

**First steps**
1. Laptop: type a line → llama.cpp (the same model) → sherpa-onnx's voice → speakers; then join it to listening.
2. Robot: a WAV through PlayStream, and a TtsMaker line; check the volume. Ask first: both make sound and use the voice service.
3. The handshake in `behavior.py`, with the sim keeping its fixed wait.

**Open decisions:**
- which of the two voices, after comparing them
- Qwen3-1.7B instead, if the 4B is too slow on PC2
- PC2's power mode (MAXN is fastest; changing it needs sudo)
- whether to run on PC2 or a laptop over Wi-Fi during robot runs
- whether it remembers people by name (the README's "remembering people")
