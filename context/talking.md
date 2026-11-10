---
name: talking
description: Read before working on what the robot says and how (the README's "Speaker" and LLM milestones), or on where the speech processes run and how they meet behavior.py
metadata:
  type: project
  updated: 2026-11-10T06:31:07Z
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
   - Topics (**decided**, 2026-11-03): small talk about the person: a greeting, one easy question about them or their day, a short friendly back-and-forth, a warm goodbye. Few facts needed, so less to make up. It goes along with whatever they bring up, with light opinions and humour of its own, and mentions being a robot only when it fits (a teammate's ask, 2026-11-09: talk more freely and naturally).
   - The system prompt covers:
   - Who HARP is: Stanford's humanoid, from the social interaction team.
   - Spoken style: usually one sentence, two at most, about as long as what they said; a question only when curious; no lists or emoji.
   - Limits: don't ask for personal details, and don't promise actions it can't do.
   - Current state: the conversation's length and turn count.
3. Waiting: a short pre-rendered filler that fits what they said plays 1.2 s after the person's turn ends, unless the reply's sound is expected within 0.6 s. A canned reply is used on an LLM error or after 5 s.
4. Ending: a closing line, then `talk_done` with the reason. The reason is one of:
   - the model ends a goodbye reply with a `<bye>` tag, checked in code (more reliable than tool calls for small models)
   - the person says goodbye and the reply is a goodbye too (the person's words alone end too much: "I have to go to the dentist tomorrow")
   - listening reports about 8 s of silence
   - after 10 turns, or the first reply past 180 s (a teammate's ask, 2026-11-09)
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
  - Fillers fit what was said (`filler_kind()`): "Oh, good question." or "Ooh, let me think." before a question that needs thought, "Um." or "Well." before another question, "Oh." after news, and none after a greeting, a goodbye or "What?". A reply that starts with the filler's sound doesn't say it again (`after_filler()`). Only sounds Kokoro says clearly: its "hmm" and "mm" came back from speech-to-text as "Hemley", "Come" and "Math".
  - At 1.2 s (`FILLER`) one plays unless the reply's sound is expected within 0.6 s (`expected()`: what's left of the first piece's words and synthesis, from the voice's measured seconds a word), or something is playing. A `filler` event says which, and why not (unfit, soon, playing). Keyed on no first piece yet, a filler played before 58% of replies, the answer right behind it ("Let me think." then "Sure! Why did the robot…"). Keyed on the LLM's text having started, none played before a 5.5 s silence while Kokoro synthesised on an M1 (live test, 2026-11-09).
  - A reply's first piece is cut at the first comma or dash with 4 words before it (later pieces: 10), so its sound starts sooner.
  - The prompt's fixed start is processed once at start, so each turn is cached.
  - Its prompt rules out offering help or going places; without that, it offered to help find food.
  - Events are JSON lines on stdout when it is piped (`talk_bench.py` reads them) or with `--json`. A `speaking` event gives each reply piece as its sound starts (the page shows replies live, [[speech-web]]); `said` comes once the whole line has played. In a terminal it prints the conversation instead: `you:` with each transcript, each reply piece as its sound starts, and the reply's times.
  - Text tests in `speech/tests/test_talk.py`.
- Words: `talk.py` gives the voice sounds, not text (sherpa-onnx's `phoneme_codepoints`), so no word is skipped. Each word comes from our `speech/lexicon/`, then the voice's list, then misaki's G2P model (`Guess`, findings: [[talking-research]]), reported as `guessed_words`. Joined-up names are split first (ChatGPT, PyTorch), and words in capitals of up to five letters in no list are spelled with the voice's letter names. Our lexicon says "a" as "ə" (both voices' lists have only the letter). Without the G2P files, words in no list are skipped as `unknown_words`.
- The lexicon `talk.py` builds at start:
  - our `speech/lexicon/` entries first, then the voice's own list
  - plus each word's -s and -'s forms where missing (Kokoro's list lacks "students", "robots", "what's", "how's")
  - Piper's list lacked "or" and "her" (added). Of 210 very common words, both lists now have all.
  - one file, because a Piper voice reads its `word || sounds` format only from a single lexicon; a comma-separated list made it skip every word
  - numbers are spelled out, and "G1" is said as "gee one"; left as it was, the spelled-out digit turned it into "Gone". A digit and a letter are split ("106B", or "sixB", a skipped word), and times read as "one thirty"
- No grammar: the model writes freely (a teammate's ask, 2026-11-09). The two-sentence GBNF grammar it replaced cut a joke before its punchline, pushed 24 of 153 replies into a `<bye>` and made it dodge open questions ([[speech-tuning]]).
  - `clean()` takes out what the voice shouldn't read: stage directions at a piece's start or end (`*waves*`, `(laughs)`), emoji, list marks, and "hmm" and "mm" (Kokoro can't say them); dashes become pauses, and curly quotes straight ones, since the voice drops anything not ASCII ("I’d" was "Id").
  - Only `MAX_SENTENCES` (2) sentences are spoken, then the request is closed, so llama-server stops writing. Left open until the reply had played, llama-server went on writing, and the next turn waited about 15 s for the whole prompt to be read again on another slot (an M1's CPU reads about 47 tokens a second), so it timed out to "Sorry, I lost my train of thought".
  - `llama-server` runs with `-np 1` (README, web.py's Start): with 4 slots, a turn sent while a cut reply was still being cancelled went to an empty slot and read the whole prompt.
  - The history keeps what the model wrote, as written and cut where the reply was cut, so llama.cpp reuses it all from its cache; cleaned or prefixed text made it read the last reply again.
  - A conversation ends only on a reply that sounds like a goodbye (`FAREWELL`, `ending()`), with `<bye>` or after the person's goodbye (`GOODBYE`, which also has "get going", "nice meeting you", "good night" and "off to bed"). Words that also come up mid-conversation ("bye", "good night", "sleep well", "later", "have fun", "be well", "good luck") count only at a sentence's start or after a comma: "it's hard to say goodbye to people" ended one. In a live test (2026-11-09) "Good night! Sleep well…" twice didn't end it, since neither was in `FAREWELL`; both lists now also have farewell, so long, see ya, talk to you soon, until next time, take it easy, safe travels, come back anytime, glad we got to talk and it was a pleasure. A `<bye>` on any other reply is an `ignored_bye` event and is kept out of the history.
- The prompt has the model answer what was said, react to its particulars in its own words and join in on any topic, with tastes and opinions of its own when asked. It asks a question only when curious, never one it has asked, and says what it meant in other words when they're confused. After three or more short answers in a row it says goodbye. Its three examples are marked as examples only.
- The prompt also gives its height (about 1.3 m) and what HARP stands for (Humanoid Athletic Robotics Project, as the GitHub organisation spells it), keeps it to English (it answered Spanish in Spanish, with words the voice can't say), has it answer "heart" or "hard" as its own name without remarking on it, and has it turn down personal details kindly and keep chatting (it said goodbye when given an address).
- Joined to listening (2026-11-03): `listen.py --mic | talk.py --heard`.
  - While it loads, plays audio, and makes a reply (`busy`, so the pause between two synthesized sentences doesn't unmute), it sends listening the speaking signal (`Mute`; the protocol: [[listening]]). Measured: on from each reply's start to its last sound, heartbeats at most 0.54 s apart.
  - A transcript within 0.75 similarity of one of its last three lines, a piece of one with three or more words, or the last one or two words of its newest line, is dropped as an `echo` event.
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
1. Laptop: a live conversation through speakers and the mic (done 2026-11-09, three conversations; findings in [[speech-tuning]]). Scripted ones run with `speech/bench/talk_bench.py` (results: [[speech-tuning]]).
2. Robot: a WAV through PlayStream, and a TtsMaker line; check the volume. Ask first: both make sound and use the voice service.
3. The handshake in `behavior.py`, with the sim keeping its fixed wait.

**Voice (decided, 2026-11-09, the user's call after a live test):** Kokoro. Piper is quicker but sounded too unclear live; it stays only as `--voice piper`. Kokoro's speed on PC2's GPU is still to be measured.

**Open decisions:**
- Qwen3-1.7B instead, if the 4B is too slow on PC2
- PC2's power mode (MAXN is fastest; changing it needs sudo)
- whether to run on PC2 or a laptop over Wi-Fi during robot runs
- whether it remembers people by name (the README's "remembering people")
