---
name: speech-research
description: Read when choosing or revisiting a piece of the speech stack (mic, turn detection, STT, LLM, TTS, frameworks): the options, their published speeds, and what other G1 projects found
metadata:
  type: reference
  updated: 2026-11-03T02:13:38Z
  covers: []
---

Research from 2026-11-02, nothing tried by us. Speeds are published figures, mostly measured on an Orin Nano (our PC2 is an Orin NX, a bit faster, and shared with HIM's harness) or claimed by vendors. Our designs: [[listening]], [[talking]]. The robot's audio calls: [[g1-builtin-apis]].

**What other G1 teams found**
- The onboard mic was too noisy (fans, motors) for one team, who moved to a USB mic ([Marvik](https://www.marvik.ai/blog/turning-a-unitree-g1-edu-into-a-conversational-humanoid); their whole stack took 3–5 s end to end).
- G1 projects mute the mic while the robot speaks, plus about 1 s after; none use echo cancellation. AEC3 (through `livekit.rtc.AudioProcessingModule`) would allow interrupting, but PlayStream's delay varies, so it may not lock on.
- Unitree's TTS sounds poor; its offline ASR gives text and the speaker's angle.

**Options by stage**
- Voice detection: Silero VAD v6 (2.3 MB ONNX; 512-sample chunks at 16 kHz, a 64-sample context and a `[2,1,128]` state), under 1 ms a chunk on one CPU thread. In noise, raise the threshold to 0.6–0.7 and add an RMS floor.
- Turn end: Pipecat's Smart Turn v3.2 (8 MB int8 ONNX, a Whisper-tiny encoder on the last 8 s) judges whether a pause ends the turn: 13–60 ms on cloud CPUs. Pipecat runs it after 0.2 s of Silero silence, with a 3 s fallback. Skantze's review: fixed-silence endpointing is the main cause of robots interrupting and answering late.
- STT, local: faster-whisper `base.en` 0.4 s an utterance, `tiny.en` 0.3 s (Orin Nano GPU); Parakeet TDT 0.6B 0.3 s. On CPU, Moonshine Tiny/Small Streaming: 237/527 ms after speech ends on a Raspberry Pi 5 (Whisper tiny 5.9 s), 12%/7.8% WER. JetPack 5 (ours) blocks Riva, whisper_trt and current jetson-containers.
- STT, cloud (prices 2026-11): OpenAI `gpt-transcribe` $0.0045/min, with `keywords` (the `gpt-4o-*-transcribe` models shut down 2027-02-26); Deepgram Nova-3 $0.0043/min, about 1.0 s median in one independent test, with `keyterm`; Groq `whisper-large-v3-turbo` $0.04/hour, billed per 10 s minimum, with `no_speech_prob`; Deepgram Flux streams and sends end-of-turn events itself ($0.0077/min).
- Whisper-style STT hallucinates on noise and silence ("Thank you for watching"): filter by `no_speech_prob`/`avg_logprob`, a phrase list and a voice-detection gate. Denoising first makes its errors worse (arXiv 2512.17562).
- Python 3.8 (PC2): the last versions are onnxruntime 1.19.2, faster-whisper 1.1.0 (ctranslate2 4.5.0, CPU only on aarch64), openai SDK 2.7.1 and deepgram-sdk 6.1.1. `moonshine-voice` and sherpa-onnx 2.0.1 have cp38 aarch64 wheels. sounddevice bundles PortAudio on macOS and Windows only.
- LLM: Claude Haiku-class and Gemini Flash about 0.6–0.7 s to first token (third-party tests); Claude Haiku 5.5's own latency unmeasured. Local Llama 3.2 3B: about 28 tokens/s on an Orin Nano, but it shares the GPU and memory with HIM's harness.
- TTS: Piper on CPU 0.18 s to first audio, 12× real time (Orin Nano Super, still 10× beside an LLM); Kokoro on GPU 0.34 s. Cloud: ElevenLabs about 0.19 s, Cartesia about 0.29 s (independent tests, 2026-10).
- Speech-to-speech APIs (OpenAI Realtime, Gemini Live): fewest parts and built-in interrupting, but less control over echo, robot state and what is said.
- Frameworks: Pipecat and LiveKit Agents need Python 3.10+ (PC2 has 3.8), so a container or a laptop; neither speaks ROS 2.

**Local on PC2** (Orin NX 16GB, JetPack 5.1.1: CUDA 11.4, TensorRT 8.5; estimates scaled from other boards, none measured on ours). The Orin Nano Super at 25 W has the same GPU clock (918 MHz) and memory bandwidth (102 GB/s) as the NX in MAXN, so its numbers carry over about 1:1. LLM decoding scales with bandwidth and prompt processing with GPU clock.
- Power mode decides the speed: MAXN runs the GPU at 918 MHz; the default 15 W mode at 612 MHz (GPU work about 1.5× slower); 25 W mode at 408 MHz. Read it with `nvpmodel -q`; changing it needs sudo (the team's call).
- STT: sherpa-onnx (Apache-2.0, Python 3.7+) lists Orin NX with CUDA 11.4 and onnxruntime-gpu 1.16.0 as a GPU build. It runs Parakeet TDT 0.6B v2 (CC-BY-4.0), estimated at 0.15–0.35 s for a 4 s clip. onnxruntime-gpu 1.16.0 is the last for CUDA 11.4. Moonshine streaming on CPU takes 0.25–0.5 s, but uses the CPU all the while someone speaks. faster-whisper/CTranslate2 don't build with CUDA 11.4. Riva's JetPack 5 models are gone, and whisper_trt needs JetPack 6.
- LLM: llama.cpp `llama-server` (MIT), built from source for `sm_87` in an `l4t-jetpack:r35` container (unverified on CUDA 11.4); Ollama ships a `jetpack5` build (v0.40.13) as the fallback. jetson-containers' r35 images are too old for current models.
  - Measured on Orin Nano at 25 W, Q4_K_M, prompt/decode tokens per second: Qwen3-1.7B 1408/39, Llama 3.2 3B 814/25, Qwen3-4B 610/20.
  - Estimated first sentence (12 tokens, 1,200-token prompt, cached prefix / not): Qwen3-1.7B 0.4/1.2 s, Llama 3.2 3B 0.6/2.0 s, Qwen3-4B-Instruct-2507 0.75/2.6 s.
  - Caching re-processes only the new turn if the prompt only grows. Gemma 3/4 and Qwen3.5 break llama.cpp's cache (open bugs).
  - Each fits in about 2–3 GB beside the speech models.
- TTS: Piper's archived C++ binary (rhasspy/piper, MIT) on CPU, about 0.15–0.2 s to first audio. The newer `piper-tts` 1.3+ package is GPL-3.0, so avoid it. Kokoro-82M (Apache-2.0) through sherpa-onnx on GPU, about 0.35–0.4 s; `kokoro-onnx` needs Python 3.10+.
- End of speech to first sound, all local, MAXN, cached: about 0.3 s turn end (Smart Turn) + 0.15–0.35 s STT + 0.4–0.6 s LLM + 0.15–0.2 s TTS + PlayStream's buffer (unmeasured, about 0.1–0.2 s). That is about 1.1–1.6 s, or 1.5–2.1 s with a fixed 0.7 s silence; add about 0.3–0.5 s in 15 W mode. Cloud paths came out similar (about 1.0 s for STT alone in one test).
- Others on the G1: unitree_converse runs Llama 3.2 3B on Ollama on PC2 (no latency published; one fork ran out of memory mid-reply, so cap the context). SensePC found local over 4 s, with throttling (unquantified). Throttling starts near 99 °C; the NX drew about 26 W while processing a prompt.
- Reflashing to JetPack 6.2 or 7.2 gives newer CUDA, prebuilt images and Super mode (up to about +25%). But HIM's harness on PC2 sources the host's ROS 2 Foxy (`setup.sh`), which needs Ubuntu 20.04 (JetPack 5). Plan on JetPack 5.

**Latency and manner:** people prefer about 1 s before a robot answers; a filler makes 2 s fine (Shiwa et al. 2009), and over 4 s hurts. So aim for first sound 1–1.5 s after they stop. Face them first, keep turns short, ask simple questions, show listening (LED), and end clearly before leaving.

**Other G1 projects** (what recurs: Silero with pre-roll, a minimum length and a cap; the mic muted while speaking; hallucination filters; a vocabulary prompt; listening state on the LED; a session opened by a button, wake word or seeing a person, closed on silence):
- SaxionMechatronics/unitree_converse: all on PC2 with faster-whisper, Ollama Llama 3.2 3B and Piper, push-to-talk. A fork with Whisper on the GPU took 0.37 s for 5.5 s of speech.
- namanbansal013/unitree-openai-voice: OpenAI Realtime, with interrupting. Its semantic VAD (eagerness low) ignores the robot's own voice better than energy VAD. Its issue #1 lists Python 3.8 and CycloneDDS pitfalls on PC2.
- experientialtech/g1-audio-driver: the mic and speaker as PulseAudio devices.
- ytphi/g1-sb01: Claude, run from a laptop. It dropped Unitree's ASR (too many mistakes, `is_final` late or missing) for Silero and faster-whisper.
- SensePC "Babis": a session opened by seeing a person (YOLO) or a wake word, at most 6 questions, ended by 12 s of silence; fully local Whisper and LLM were too slow (over 4 s) and throttled.
- Marvik: a wake word gave 11 false triggers an hour; audio took them a month to get right.

**Licenses** (from each LICENSE file, 2026-11-02; the rule: [[external-dependencies]])
- Permissive: Silero VAD code and model (MIT); smart-turn code and weights (BSD-2); onnxruntime, faster-whisper, CTranslate2, Whisper code and weights, whisper.cpp, sounddevice, PortAudio, webrtcvad (MIT); Moonshine code and its English and streaming weights (MIT); sherpa-onnx (Apache-2.0, each model its own); Pipecat (BSD-2); LiveKit Agents (Apache-2.0); unitree_sdk2 and unitree_sdk2_python (BSD-3); unitree-openai-voice (MIT); whisper_ros (MIT).
- Attribution: Parakeet weights (CC-BY-4.0).
- Avoid: TEN VAD (Apache-2.0 plus a no-compete clause); LiveKit's turn-detector weights (only within LiveKit Agents); some Vosk models (AGPL, LGPL); Moonshine's older non-English models (non-commercial).
- No license, so all rights reserved (read only, never copy): unitree_converse, g1-audio-driver, g1-sb01, SensePC's Babis, losslesswow-tech's assistant.
