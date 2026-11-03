---
name: talking
description: Read before working on what the robot says and how (the README's "Speaker" and LLM milestones), or on where the speech processes run and how they meet behavior.py
metadata:
  type: project
  updated: 2026-11-03T01:44:13Z
  covers: [testground/speech/*]
---

Design for the talking half of a conversation, and what both halves share. Nothing is built; every choice is **proposed** unless marked **decided**, and decisions are recorded here as they are made. The listening half: [[listening]]. Options and numbers behind the choices: [[speech-research]]. The robot's audio calls: [[g1-builtin-apis]].

**Its job (decided, 2026-11-02; the split: [[listening]]):** own the conversation, the LLM included. It opens one when `behavior.py` has walked up to someone, decides each reply with an LLM, speaks it, and ends the conversation. Its input is listening's transcripts (or typed lines while testing).

**Shared by both halves**
- Each runs as its own process, beside our image, not in it: on a laptop while developing, then on PC2. The image can't host them: it is Python 3.8, has no network in the sim and no `/dev/snd`. HIM's launcher starts it with host networking, so a PC2 process reaches the same ROS 2 graph (domain 97, localhost) and Unitree's DDS.
- **Decided:** local models wherever latency allows, cloud only as a fallback (a teammate's direction, 2026-11-02). Estimated all-local time to first sound on PC2: about 1.1–1.6 s ([[speech-research]]).
- Keep work off PC2's CPU (HIM's 0.2 s deadlines, [[safety]]) and on its GPU. The GPU is free, since HIM's walker runs on the CPU (`CPUExecutionProvider`), and HIM's launcher adds `--runtime nvidia` with `G1_NVIDIA=1`. PC2 is an Orin NX 16GB on JetPack 5.1.1 ([[robot-pc2]]). Any cloud API keys live in ignored files, never in the image or git.
- Every speed is an estimate until measured on PC2 with `testground/speech/bench.py` (its README has the setup). It times each model on recordings and a built-in conversation of about 300 tokens, logging `nvpmodel -q` and `tegrastats`. Run it with HIM's harness off and on. It has only been run against stand-ins on a laptop (2026-11-03). Installing its models in our folder on PC2 needs the team's OK.
- The core is plain Python. A thin ROS 2 adapter connects it on the robot; on a laptop (no ROS 2) it uses the terminal.
- The handshake with `behavior.py` ([[behavior-loop]]): on reaching someone, it sends `{"start": person, "distance": m}` on `/harp/talk` and waits for a `talk_done` event, which replaces its fixed wait. If nothing answers (the sim has no speech) it keeps the fixed wait, so the courses still pass. It sends `{"stop": person}` when the person leaves.

**Pipeline**
1. Opening: a fixed greeting, pre-rendered as audio, plays the moment the robot arrives, with no LLM wait.
2. Reply: each `/harp/heard` text, plus the conversation so far, goes to a local LLM through llama.cpp's `llama-server` on PC2's GPU (Ollama's `jetpack5` build if it won't build), streamed.
   - Model: Llama 3.2 3B Instruct Q4_K_M (about 0.6 s to the first sentence); Qwen3-1.7B if that's too slow, Qwen3-4B-Instruct-2507 if it's too weak.
   - Cloud fallback: Claude Haiku 5.5 (`claude-haiku-5-5`), thinking off.
   - Keep the prompt append-only so llama.cpp reuses its cache. Changing robot state goes after the history, never in the system prompt. Cap the context at 4,096 tokens.
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
5. Speech: sentences are split off the stream as they arrive, synthesized and queued. Piper's archived C++ binary (MIT; not the GPL `piper-tts` package) on CPU first, about 0.2 s to first audio. Kokoro-82M through sherpa-onnx on the GPU (about 0.35 s) if Piper sounds too robotic. Fixed phrases are cached as WAV.
6. Output: laptop speakers, or the robot's PlayStream (resampled to 16 kHz). Each reply gets one `stream_id`, so its chunks queue; PlayStop cuts it off. Unitree's TtsMaker (`speaker_id` 1) is a fallback with no dependencies for the first speaker test. The robot doesn't report when playback ends, so speaking ends at the audio's length after the start, plus a margin.
7. LED (1010): one colour each for listening, thinking and speaking.

**Publishes** (events on `/harp/events`, source `talk`):
- `said`: text, time to first sound, each stage's time
- `speaking`: on or off, for listening's gate
- `talk_done`: the reason

**Measuring:** time from listening's turn end to first sound; target 1–1.5 s ([[speech-research]]). Keep each whole conversation's text in the run's log.

**First steps**
1. Laptop: type a line → llama.cpp (the same model) → Piper → speakers; then join it to listening.
2. Robot: a WAV through PlayStream, and a TtsMaker line; check the volume. Ask first: both make sound and use the voice service.
3. The handshake in `behavior.py`, with the sim keeping its fixed wait.

**Open decisions:**
- the TTS voice (Piper or Kokoro)
- the LLM (by measured speed and replies on PC2)
- PC2's power mode (MAXN is fastest; changing it needs sudo)
- what HARP talks about
- whether to run on PC2 or a laptop over Wi-Fi during robot runs
- whether it remembers people by name (the README's "remembering people")
