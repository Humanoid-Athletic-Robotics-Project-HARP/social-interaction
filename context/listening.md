---
name: listening
description: Read before working on how the robot hears people (the README's "Microphone" milestone): the mic, telling when someone has finished, speech-to-text, and the transcript it hands to talking
metadata:
  type: project
  updated: 2026-11-02T22:28:58Z
  covers: []
---

Design for the listening half of a conversation. Nothing is built; every choice is **proposed** unless marked **decided**, and decisions are recorded here as they are made. The other half and what both share (where they run, the handshake with `behavior.py`): [[talking]]. Options and numbers behind the choices: [[speech-research]]. The robot's audio calls: [[g1-builtin-apis]].

**Its job (decided, a teammate's call, 2026-11-02; ask before changing the split):** turn sound into one final transcript per thing a person says during a conversation. It doesn't decide what the words mean or what to answer ([[talking]] does). It ends where text begins, so each half is tested alone: listening prints transcripts; talking takes typed lines.

**Pipeline** (each stage a swappable backend; plain Python that also runs on 3.8)
1. Source, as 16 kHz mono 16-bit frames: a laptop mic first; on the robot, Unitree's mic multicast, or a USB mic on PC2 if that is too noisy (chosen from recordings).
2. Gate: frames pass only during a conversation and while the robot isn't speaking, plus about 1 s after (talking's `speaking` events). This replaces echo cancellation, so people can't interrupt the robot at first, and nothing is heard outside a conversation.
3. Turn end: Silero VAD (its ONNX model on onnxruntime) on 32 ms frames. Speech starts after about 0.25 s of voice and ends after about 0.6 s of silence. Utterances under 0.3 s are dropped; ones over 15 s are cut.
4. Speech-to-text: each finished utterance goes whole to a cloud model (OpenAI `gpt-4o-mini-transcribe` or Deepgram); streaming only if latency needs it. Local faster-whisper `base.en` as the offline fallback. Unitree's own ASR (`rt/audio_msg`) is a third backend to compare; it also gives the speaker's angle.

**Publishes** (JSON on `std_msgs/String`, like `/harp/people`, [[feedback-minimal-setup]])
- `/harp/heard`, one message per utterance: conversation id, text, start and end times, STT confidence and angle when given, and each stage's time (turn end, STT) for the latency budget.
- Events on `/harp/events` (source `listen`): `speech_start` (for talking's LED or a nod) and `silence` (nothing said for N s in a conversation), which talking uses to end it.

**Measuring:** while testing, keep each utterance's WAV and transcript with the run. That lets us tune the VAD settings and compare STT backends on the same recordings. Check word error rate on a few test phrases said from 1 m, with the robot standing and walking. Record strangers only with their OK, and never commit recordings.

**First steps**
1. Laptop: mic → VAD → STT, printing transcripts and stage times.
2. Robot, listening only (`testground/audio/`): record the mic multicast to WAV, standing and walking; log `rt/audio_msg`; note the voice firmware version. Switching the voice assistant's mode changes the robot for everyone: ask first.
3. Choose the onboard or a USB mic from those recordings.

**Open decisions:** the cloud STT provider; the onboard or a USB mic; whether to listen outside a conversation (to notice someone saying hello first).
