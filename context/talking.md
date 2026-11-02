---
name: talking
description: Read before working on what the robot says and how (the README's "Speaker" and LLM milestones), or on where the speech processes run and how they meet behavior.py
metadata:
  type: project
  updated: 2026-11-02T22:28:58Z
  covers: []
---

Design for the talking half of a conversation, and what both halves share. Nothing is built; every choice is **proposed** unless marked **decided**, and decisions are recorded here as they are made. The listening half: [[listening]]. Options and numbers behind the choices: [[speech-research]]. The robot's audio calls: [[g1-builtin-apis]].

**Its job (decided, 2026-11-02; the split: [[listening]]):** own the conversation, the LLM included. It opens one when `behavior.py` has walked up to someone, decides each reply with an LLM, speaks it, and ends the conversation. Its input is listening's transcripts (or typed lines while testing).

**Shared by both halves**
- Each runs as its own process, beside our image, not in it: on a laptop while developing, then on PC2. The image can't host them: it is Python 3.8, has no network in the sim and no `/dev/snd`. HIM's launcher starts it with host networking, so a PC2 process reaches the same ROS 2 graph (domain 97, localhost) and Unitree's DDS.
- Keep heavy compute off PC2 (HIM's 0.2 s deadlines, [[safety]]): cloud LLM and STT, over PC2's Wi-Fi. API keys live in ignored files, never in the image or git.
- The core is plain Python. A thin ROS 2 adapter connects it on the robot; on a laptop (no ROS 2) it uses the terminal.
- The handshake with `behavior.py` ([[behavior-loop]]): on reaching someone, it sends `{"start": person, "distance": m}` on `/harp/talk` and waits for a `talk_done` event, which replaces its fixed wait. If nothing answers (the sim has no speech) it keeps the fixed wait, so the courses still pass. It sends `{"stop": person}` when the person leaves.

**Pipeline**
1. Opening: a fixed greeting, pre-rendered as audio, plays the moment the robot arrives, with no LLM wait.
2. Reply: each `/harp/heard` text, plus the conversation so far, goes to Claude Haiku 5.5 (`claude-haiku-5-5`), streamed, with thinking off. The system prompt covers:
   - Who HARP is: Stanford's humanoid, from the Human Interaction team.
   - Spoken style: one or two short sentences, simple questions, no lists or emoji.
   - Limits: don't ask for personal details, and don't promise actions it can't do.
   - Current state: the conversation's length and turn count.
3. Waiting: a short pre-rendered filler plays if no reply sound has come about 1.2 s after the person's turn ends. A canned reply is used on an LLM error or after 5 s.
4. Ending: a closing line, then `talk_done` with the reason. The reason is one of:
   - the model marks a goodbye (a tag or a tool call; to decide)
   - the person says goodbye
   - listening reports about 8 s of silence
   - about 90 s in all
   - `behavior.py`'s stop
5. Speech: sentences are split off the stream as they arrive, synthesized and queued. Piper on CPU first (local, free, about 0.2 s to first audio); a cloud voice if Piper sounds too robotic. Fixed phrases are cached as WAV.
6. Output: laptop speakers, or the robot's PlayStream (resampled to 16 kHz). Each reply gets one `stream_id`, so its chunks queue; PlayStop cuts it off. Unitree's TtsMaker (`speaker_id` 1) is a fallback with no dependencies for the first speaker test. The robot doesn't report when playback ends, so speaking ends at the audio's length after the start, plus a margin.
7. LED (1010): one colour each for listening, thinking and speaking.

**Publishes** (events on `/harp/events`, source `talk`):
- `said`: text, time to first sound, each stage's time
- `speaking`: on or off, for listening's gate
- `talk_done`: the reason

**Measuring:** time from listening's turn end to first sound; target 1–1.5 s ([[speech-research]]). Keep each whole conversation's text in the run's log.

**First steps**
1. Laptop: type a line → Claude → Piper → speakers; then join it to listening.
2. Robot: a WAV through PlayStream, and a TtsMaker line; check the volume. Ask first: both make sound and use the voice service.
3. The handshake in `behavior.py`, with the sim keeping its fixed wait.

**Open decisions:**
- the TTS voice (Piper or cloud)
- the LLM
- how the model signals the end
- what HARP talks about
- whether to run on PC2 or a laptop over Wi-Fi during robot runs
- whether it remembers people by name (the README's "remembering people")
