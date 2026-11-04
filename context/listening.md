---
name: listening
description: Read before working on how the robot hears people (the README's "Microphone" milestone): the mic, telling when someone has finished, speech-to-text, and the transcript it hands to talking
metadata:
  type: project
  updated: 2026-11-04T22:44:59Z
  covers: [speech/listen.py, speech/requirements.txt, speech/tests/test_turns.py]
---

Design for the listening half of a conversation, and what of it is built (**Built**, below). Every choice is **proposed** unless marked **decided**, and decisions are recorded here as they are made. The other half and what both share (where they run, the handshake with `behavior.py`): [[talking]]. Options, numbers, other projects and licenses: [[speech-research]]. The robot's audio calls: [[g1-builtin-apis]].

**Its job (decided, a teammate's call, 2026-11-02; ask before changing the split):** turn sound into one final transcript per thing a person says during a conversation. It doesn't decide what the words mean or what to answer ([[talking]] does). It ends where text begins, so each half is tested alone: listening prints transcripts; talking takes typed lines.

**Shape:** one process, a chain of stages, each with one job and a swappable backend. Every stage also runs on a WAV file, so it is tuned and tested on recordings without a robot or a person. Plain Python that runs on 3.8. Dependencies (all permissively licensed, imported and never copied, [[external-dependencies]]):
   - Laptops: `speech/requirements.txt`, which needs Python 3.12 (numpy 2.5.4, onnxruntime 1.19.2, sherpa-onnx, sounddevice).
   - PC2: NVIDIA's onnxruntime-gpu 1.16.0 (the last for CUDA 11.4) and a GPU build of sherpa-onnx.
   - A cloud backend would add requests.

1. **Source**, re-cut into 32 ms frames (512 samples at 16 kHz mono int16, what Silero takes):
   - Laptop: a sounddevice callback into a queue. On macOS the terminal app owns the mic permission, and a denied grant gives silent zeros with no error, so say so on all-zero input.
   - Robot: our own stdlib multicast reader (the stream: [[g1-builtin-apis]]), with SO_REUSEADDR so other programs can listen too. It warns on all-zero packets (a firmware fault) and on no packets (the assistant isn't in wake-up mode).
   - A USB mic on PC2, if recordings show the onboard one is too noisy: ALSA through `arecord`, since PortAudio may be missing and PC2 has no sudo.
   - A WAV file, for tests and tuning.
2. **Gate:** listen only during a conversation, which [[talking]] opens once the robot has walked up to someone (on a laptop, from the start). The walk up is the opener, so there's no wake word or button. Mute while the robot speaks and for 0.5 s after (talking's `speaking` events). A 0.8 s ring buffer keeps the audio just before speech starts, so first syllables survive (0.5 s lost a short "Not much," in noise: [[speech-tuning]]).
3. **Voice detection:** Silero VAD v6 (`silero_vad.onnx`, MIT; fetched at setup from a pinned release and checked against its hash, not committed), on onnxruntime with one thread. Speech starts after 0.25 s of frames whose probability is at or above 0.6 and whose RMS is above the measured noise floor; a frame without voice takes one frame off the count rather than resetting it ([[speech-tuning]] has why). It ends when the probability falls below 0.45. Utterances under 0.3 s are dropped; ones over 15 s are cut.
4. **Turn end:**
   - Pipecat's Smart Turn v3.2 (a BSD-2 int8 ONNX model, used on its own without Pipecat). After 0.2 s of silence it judges the last 8 s of the turn. If it says finished (0.5 or more), the turn ends; otherwise after 1.5 s of silence (2.5 s left people waiting when it misjudged a finished line).
   - Turns with under 1 s of speech end after 0.7 s of silence without asking it: it calls one-word answers ("Yeah.", "Bye!") unfinished. `--fixed-turn` does that for every turn.
   - Its log-mel features are our own numpy code (`TurnEnd.features`), matching transformers' `WhisperFeatureExtractor` to 1e-5 (checked 2026-11-04), so no transformers dependency.
5. **Speech-to-text, local first** ([[talking]]'s shared direction): each turn goes whole, as one clip, to a model on PC2's GPU.
   - First choice: NVIDIA's Parakeet TDT 0.6B v2 through sherpa-onnx with onnxruntime-gpu 1.16.0, the build sherpa-onnx lists for Orin NX on CUDA 11.4. Estimated 0.15–0.35 s for a 4 s clip.
   - CPU alternative: Moonshine streaming (`moonshine-voice`, MIT, Python 3.8 wheels for aarch64 and macOS), 0.25–0.5 s, but it uses the CPU all the while someone speaks.
   - Cloud fallback, over plain HTTP since current vendor SDKs need Python 3.10+: OpenAI `gpt-transcribe`, Deepgram Nova-3 or Groq `whisper-large-v3-turbo`.
   - Chosen by a bake-off of latency and errors on our own recordings. Parakeet's CC-BY-4.0 credit is in the README's Licenses.
   - Vocabulary hints (HARP, Stanford, names): sherpa-onnx's hotwords where the model supports them, otherwise the cloud's keyword lists.
6. **Filter:** drop a transcript if it is:
   - empty
   - a known Whisper hallucination ("Thank you for watching"; never "Bye." or "Thank you.", which people say, and Parakeet made none up on noise)
   - low-confidence where the backend reports it (Whisper's `no_speech_prob` above 0.6 with `avg_logprob` below −1)
   - a repeat of what the robot just said (echo that leaked past the gate)

**Publishes** (JSON on `std_msgs/String`, like `/harp/people`, [[feedback-minimal-setup]])
- `/harp/heard`, one message per utterance: conversation id, text, start and end times, confidence when given, and each stage's time (turn end, STT) for the latency budget.
- Events on `/harp/events` (source `listen`):
  - `state`: idle, listening, hearing or transcribing (for talking's LED)
  - `speech_start`: for a nod
  - `silence`: nothing said for N s in a conversation (talking uses it to end the conversation)

**Not chosen**
- Unitree's own ASR: another G1 team found it wrong too often, and its final result late or missing. Logged at most, for its angle.
- A wake word: false triggers and memory use; the walk up opens the session instead.
- Push-to-talk: it needs an operator. Kept as a fallback for noisy demos.
- Noise suppression: it hurts Whisper-style STT.
- Echo cancellation and interrupting: not until muting has been tried.
- Pipecat and LiveKit Agents: they need Python 3.10+, so they serve as design references only.
- TEN VAD and LiveKit's turn detector: their licenses restrict use.

**On PC2:** a venv inside our testground folder, since `pip --user` would write outside it ([[robot-pc2]]). Run at nice 10.

**Measuring:**
- False starts per minute on noise-only recordings (fans, walking, a talking room).
- Word errors on test phrases said from 1 m.
- Time from turn end to transcript; target about 0.4 s locally.

While testing, keep each utterance's WAV and transcript with the run. Record strangers only with their OK, and never commit recordings (they go to `robot-captures/`).

**Built** (`speech/listen.py`, 2026-11-03; setup and use: `speech/README.md`).
- Decided (a teammate's call, 2026-11-03): the code lives in `speech/`, outside the image; the models go in `harp/models/` beside the main checkout (`HARP_MODELS` to change), fetched by pinned URL and hash.
- What it does:
  - sources: WAV files, the laptop mic, the robot's multicast mic
  - Silero plus the loudness floor
  - Smart Turn's turn end (0.7 s of silence for one-word turns)
  - Parakeet on CPU (`--provider cuda` on PC2)
  - the filter's short, empty and made-up checks
  - one JSON line per turn on stdout
- The turn rules (`Turns`) are tested in `speech/tests/` without models.
- The mute (built 2026-11-03, with the mic or the robot's mic):
  - `talk.py` sends `{"speaking": true}` every 0.5 s while it plays or makes a reply, and `false` when done, as UDP to port 28732 on the same computer.
  - `listen.py` hears nothing until 0.5 s after `false`, or 1 s after the last `true` if `talk.py` dies.
  - A turn cut off by it is dropped, and Silero's state is reset after it.
  - On the robot this becomes the `speaking` ROS 2 event.
  - The echo check is in `talk.py` (it knows what it said).
- Tested on a laptop (2026-11-03), on the stand-in robot mic stream: person speech, then HARP's own greeting, then person speech. Without the mute the greeting was transcribed; with it, only the person's two lines, the second caught whole 1 s after HARP stopped. Not yet tried with a live person through real speakers and mic.
- Not built yet: ROS 2 output, a USB mic, and installing it on PC2. How `speech/` gets to PC2 is open: `sync.sh` pushes only `testground/`.
- Measured on an M1 MacBook Air's CPU (2026-11-03), with macOS `say` voices, which are easier than a person at 1 m:
  - two sentences 1.5 s apart gave two turns; a 0.4 s pause stayed inside one turn
  - the text came out word for word, also with noise 15 dB below the speech
  - 5 s of noise alone gave nothing
  - Parakeet took 0.14–0.19 s for a 2–3 s turn: 0.84–0.89 s from going quiet to text
  - the same clips gave the same turns from a stand-in robot mic stream over loopback multicast

**First steps**
1. Laptop: real voices through `--mic`, and recordings of people and rooms for tuning.
2. Robot, listening only (`testground/audio/`): read the voice mode (1007), record the multicast mic standing and walking, note the firmware. Switching the mode changes the robot for everyone: ask first.
3. From those recordings, choose the mic and the STT provider.

**Open decisions:**
- the STT model (by the bake-off on PC2)
- the onboard or a USB mic
- whether to listen outside a conversation
