---
name: speech-research
description: Read when choosing or revisiting a piece of the speech stack (mic, turn detection, STT, LLM, TTS, frameworks): the options, their published speeds, and what other G1 projects found
metadata:
  type: reference
  updated: 2026-11-02T22:23:54Z
  covers: []
---

Research from 2026-11-02, nothing tried by us. Speeds are published figures, mostly measured on an Orin Nano (our PC2 is an Orin NX, a bit faster, and shared with HIM's harness) or claimed by vendors. Our designs: [[listening]], [[talking]]. The robot's audio calls: [[g1-builtin-apis]].

**What other G1 teams found**
- The onboard mic was too noisy (fans, motors) for one team, who moved to a USB mic ([Marvik](https://www.marvik.ai/blog/turning-a-unitree-g1-edu-into-a-conversational-humanoid); their whole stack took 3–5 s end to end).
- G1 projects mute the mic while the robot speaks, plus about 1 s after; none use echo cancellation. AEC3 (through `livekit.rtc.AudioProcessingModule`) would allow interrupting, but PlayStream's delay varies, so it may not lock on.
- Unitree's TTS sounds poor; its offline ASR gives text and the speaker's angle.

**Options by stage**
- Turn end: Silero VAD, under 1 ms a 32 ms chunk on one CPU thread. Pipecat's open Smart Turn model judges whether a pause ends the turn, about 65 ms on CPU. LiveKit's turn detector is the other.
- STT, local: faster-whisper `base.en` 0.4 s an utterance, `tiny.en` 0.3 s (Orin Nano GPU); Parakeet TDT 0.6B 0.3 s. JetPack 5 (ours) blocks Riva, whisper_trt and current jetson-containers.
- STT, cloud: Deepgram Flux (turn detection built in), AssemblyAI Universal-Streaming (about 0.15–0.3 s claimed), OpenAI `gpt-4o-mini-transcribe`.
- LLM: Claude Haiku-class and Gemini Flash about 0.6–0.7 s to first token (third-party tests); Claude Haiku 5.5's own latency unmeasured. Local Llama 3.2 3B: about 28 tokens/s on an Orin Nano, but it shares the GPU and memory with HIM's harness.
- TTS: Piper on CPU 0.18 s to first audio, 12× real time (Orin Nano Super, still 10× beside an LLM); Kokoro on GPU 0.34 s. Cloud: ElevenLabs about 0.19 s, Cartesia about 0.29 s (independent tests, 2026-10).
- Speech-to-speech APIs (OpenAI Realtime, Gemini Live): fewest parts and built-in interrupting, but less control over echo, robot state and what is said.
- Frameworks: Pipecat and LiveKit Agents need Python 3.10+ (PC2 has 3.8), so a container or a laptop; neither speaks ROS 2.

**Latency and manner:** people prefer about 1 s before a robot answers; a filler makes 2 s fine (Shiwa et al. 2009), and over 4 s hurts. So aim for first sound 1–1.5 s after they stop. Face them first, keep turns short, ask simple questions, show listening (LED), and end clearly before leaving.

**Other G1 projects:**
- SaxionMechatronics/unitree_converse: all on PC2 with faster-whisper, Ollama Llama 3.2 3B and Piper, push-to-talk. A fork with Whisper on the GPU took 0.37 s for 5.5 s of speech.
- namanbansal013/unitree-openai-voice: OpenAI Realtime, with interrupting. Its issue #1 lists Python 3.8 and CycloneDDS pitfalls on PC2.
- experientialtech/g1-audio-driver: the mic and speaker as PulseAudio devices.
- ytphi/g1-sb01: Claude, run from a laptop.
