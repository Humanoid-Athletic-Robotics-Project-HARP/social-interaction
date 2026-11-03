---
name: talking
description: Read before working on what the robot says and how (the README's "Speaker" and LLM milestones), or on where the speech processes run and how they meet behavior.py
metadata:
  type: project
  updated: 2026-11-03T03:26:47Z
  covers: [testground/speech/*, speech/talk.py, speech/tests/test_talk.py, speech/lexicon/*]
---

Design for the talking half of a conversation, and what both halves share, and what of it is built (**Built**, below). Every choice is **proposed** unless marked **decided**, and decisions are recorded here as they are made. The listening half: [[listening]]. Options, licenses and research behind the choices: [[talking-research]], speeds on PC2: [[speech-research]]. The robot's audio calls: [[g1-builtin-apis]].

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
   - the model ends its reply with a `<bye>` tag, checked in code (more reliable than tool calls for small models)
   - the person says goodbye
   - listening reports about 8 s of silence
   - about 90 s in all
   - `behavior.py`'s stop
5. Speech: sherpa-onnx's TTS, the package listening already uses, with no espeak-ng (GPL) anywhere. Never Piper's C++ binary or the `piper-tts` package: both carry GPL code ([[talking-research]]).
   - Voice (**decided**, 2026-11-03: compare both by ear and speed, then pick one): Kokoro v1.0 `af_heart` (Apache-2.0; on PC2's GPU about 0.35 s to first audio) and Piper `ljspeech` through sherpa-onnx (public domain; light on the CPU).
   - Text is split at sentence ends, and at commas once 8–12 words have built up, then synthesized while the previous piece plays.
   - Numbers are spelled out first, and our own lexicon adds names, since sherpa-onnx drops words it has no entry for.
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
- `speaking`: on or off, for listening's mute. On a laptop it goes as UDP to port 28732 instead ([[listening]] has the protocol).
- `talk_done`: the reason

**Measuring:** time from listening's turn end to first sound; target 1–1.5 s ([[speech-research]]). Keep each whole conversation's text in the run's log.

**Built** (`speech/talk.py`, 2026-11-03; setup and use: `speech/README.md`)
- What it does: typed lines or `listen.py`'s JSON lines in, through `llama-server` (Qwen3-4B, streamed), to a sherpa-onnx voice (`--voice kokoro` or `piper`), out through this computer's speakers (`--no-play`, `--save DIR`).
  - The greeting, fillers, "Sorry, I didn't catch that" and the closing line are rendered at start.
  - The fillers are "Let me think." and "Let me see." ("Hmm" came out as "Hammel"). One plays only if the LLM has given no text 1.2 s after the turn ends, and nothing is still playing. Keying it on the audio being ready instead made one play before every Kokoro reply on a CPU.
  - A reply's first piece is cut at the first comma with 4 words before it (later pieces: 10), so its sound starts sooner.
  - The prompt's fixed start is processed once at start, so each turn is cached.
  - Its prompt rules out offering help or going places; without that, it offered to help find food.
  - Events are JSON lines on stdout.
  - Text tests in `speech/tests/test_talk.py`.
- Words: sherpa-onnx skips any word missing from a voice's lexicon. So `talk.py` writes one combined lexicon at start:
  - our `speech/lexicon/` entries first, then the voice's own list
  - plus each word's -s and -'s forms where missing (Kokoro's list lacks "students", "robots", "what's", "how's")
  - Piper's list lacked "or" and "her" (added). Of 210 very common words, both lists now have all.
  - one file, because a Piper voice reads its `word || sounds` format only from a single lexicon; a comma-separated list made it skip every word
  - numbers are spelled out, and "G1" is said as "gee one"; left as it was, the spelled-out digit turned it into "Gone"
- A GBNF grammar (on by default; `--no-grammar` turns it off) allows only two plain sentences and an optional `<bye>`. It stops emoji and lists.
  - After two sentences, `<bye>` is the grammar's only way on, so a model that wants to say more takes it. Asked for "multiple sentences", it did so 3 times out of 3, and the conversation ended.
  - So `<bye>` counts only if the reply sounds like a goodbye (`FAREWELL`) or the person said one. Otherwise it's an `ignored_bye` event and is kept out of the history.
- The prompt has the model answer what was said, not always open with a reaction: "Hello" was getting "Oh, nice!". It never repeats a question, and rephrases when they're confused: "What" was getting the same reply again. Its examples for those cases are now often copied word for word.
- Joined to listening (2026-11-03): `listen.py --mic | talk.py --heard`.
  - While it plays audio, and while a reply is being made (`busy`, so the pause between two synthesized sentences doesn't unmute), it sends listening the speaking signal. Measured: on from each reply's start to its last sound, heartbeats at most 0.54 s apart.
  - A transcript within 0.75 similarity of one of its last three lines, or a piece of one with three or more words, is dropped as an `echo` event.
  - Ports (a teammate's ask: no common ones; 8080–8082 are taken on PC2 by our lidar, camera and hybrid viewers): `llama-server` on 28731, the speaking signal on 28732. Both are below Linux's temporary-port range (32768–60999).
  - People can't interrupt it: it doesn't listen while it speaks.
- Not built yet:
  - the robot's PlayStream player and the LED
  - ROS 2 and the `behavior.py` handshake
  - the silence ending
  - a guard model
- Measured on an M1 MacBook Air's CPU (2026-11-03), on a scripted conversation of five turns:
  - Every reply was one or two short sentences, in character, ending on `<bye>` when the person was leaving.
  - Time from the end of a line to the first sound: Piper 0.68–1.57 s; Kokoro 1.5–3.0 s, since Kokoro takes 0.9–1.7 s per first sentence on a CPU.
  - Run back through `listen.py`, Kokoro's speech came back nearly word for word; Piper's was rougher ("Harr, A. Robart").

**First steps**
1. Laptop: a live conversation through speakers and the mic; then about 50 scripted conversations to count false or missed goodbyes, repeated questions and made-up facts, with and without the grammar.
2. Robot: a WAV through PlayStream, and a TtsMaker line; check the volume. Ask first: both make sound and use the voice service.
3. The handshake in `behavior.py`, with the sim keeping its fixed wait.

**Open decisions:**
- which of the two voices, after comparing them
- Qwen3-1.7B instead, if the 4B is too slow on PC2
- PC2's power mode (MAXN is fastest; changing it needs sudo)
- whether to run on PC2 or a laptop over Wi-Fi during robot runs
- whether it remembers people by name (the README's "remembering people")
