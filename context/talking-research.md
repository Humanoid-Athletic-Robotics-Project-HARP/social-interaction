---
name: talking-research
description: Read when choosing or revisiting the talking half's pieces (the voice, the LLM and its license, the robot's speaker, how a conversation with a stranger should go): options, licenses and what others found
metadata:
  type: reference
  updated: 2026-11-09T01:16:05Z
  covers: []
---

Research from 2026-11-03, nothing tried by us. Our design: [[talking]]. Speeds on PC2 and the listening side: [[speech-research]]. The speaker's API calls: [[g1-builtin-apis]]. The license rule: [[external-dependencies]].

**Voices (local TTS)**
- **sherpa-onnx 2.0.0 removed espeak-ng (GPL-3.0)** on 2026-10-20 (PR #4055), so the version we use holds no GPL code. Its English voices now read words from lexicon files. Words not in a lexicon, digits included, are silently dropped (issue #4139), so:
  - spell numbers out first (inflect, MIT; not num2words, which is LGPL)
  - add our own lexicon for names ("HARP", "Unitree")
- **Words missing from the lexicons: a G2P model (spelling to sounds) for them.** Tried 2026-11-08 on an M1 CPU with 46 words the LLM could say (maths, tech, campus places, names, foods) that a voice's lexicon lacks, Parakeet as the listener ("I said X."):
  - Kokoro, words heard within 0.7 letter similarity: misaki's G2P 35 of 46, g2p_en 35, none 11 (those 11 were only missing from Piper's list). Piper: misaki 21, g2p_en 13, none 4.
  - misaki's US G2P (word lists plus a small BART model, `PeterReid/graphemes_to_phonemes_en_us`, Apache-2.0): its symbols are Kokoro's (Kokoro's lexicon is misaki's lists, 174,330 of 174,335 entries identical), 2.2 ms a word, and onnxruntime is all it needs. Used as `nekwo/misaki-en-us-g2p-onnx` (a third party's ONNX export, Apache-2.0), pinned by commit and hash; exporting the official weights ourselves would make each teammate install PyTorch once.
  - g2p_en (Apache-2.0, CMUdict plus a numpy net): as good on Kokoro, worse on Piper, and it adds nltk, inflect and distance plus nltk data downloads.
  - Piper needs misaki's symbols turned into espeak-style ones (A to eɪ, ɜɹ to ɜː, and so on): a fixed table matched Piper's own entries with median similarity 1.00.
  - sherpa-onnx 2.0.1 takes sounds directly: `GenerationConfig.phoneme_codepoints`, one list of code points per sentence, for both voices. Piper got the same token ids either way. On 14 of HARP's usual lines Kokoro's word errors were 1.5% this way against 3.8% from text; Piper's varied between runs (8 to 15%), as its VITS model adds noise.
  - Still wrong with either model: names from other languages (Nguyen, Saoirse, Siobhan, tteokbokki) and brands spelled as letters (ChatGPT came out "chat"). Those need our lexicon, or a rule that spells capital runs as letters.
- **Kokoro-82M v1.0** (code, weights and voices Apache-2.0; best English voice `af_heart`): runs as is through sherpa-onnx, since its lexicon comes from misaki's dictionaries (Apache-2.0).
  - Real-time factor (synthesis time over audio length) 3.19 on a Raspberry Pi 4, so likely too heavy for PC2's CPU, but about 0.34 s to first audio on an Orin GPU.
- **Piper voices through sherpa-onnx:** RTF 0.36 on a Pi 4, light enough for PC2's CPU. They need sherpa-onnx's `lexicon-en-us.txt`, whose origin is unverified (if it was built from espeak-ng's dictionaries, it's a grey area).
  - Dataset licenses: `ljspeech` (public domain) and `libritts_r` (CC-BY-4.0) are fine.
  - Avoid `lessac` (research-only; the voice unitree_converse uses), `ryan` and `hfc_*` (CC-BY-NC-SA), and `glados`.
- **Piper's own C++ binary** (rhasspy/piper, MIT) bundles libespeak-ng (GPL), and the `piper-tts` package is GPL-3.0: avoid both. KittenTTS's Python package needs `phonemizer` (GPL); its model through sherpa-onnx is fine.
- **Others:**
  - Pocket TTS (Kyutai): MIT code, CC-BY-4.0 weights; some voices are non-commercial.
  - Avoid:
    - use-restricted: Supertonic (OpenRAIL-M), Orpheus (Llama's license), F5/E2 (non-commercial)
    - XTTS-v2 (CPML)
    - need CUDA 12 PyTorch, so not on JetPack 5: Chatterbox, Dia, Kyutai TTS, Qwen3-TTS
- **How to stream it:** split the LLM's text at `.!?`, and at `,;:` once 8–12 words have built up. Pieces under about 4 words sound flat. Synthesize the next piece while one plays. Resample to 16 kHz with scipy's `resample_poly` (BSD); python-soxr is LGPL.
- **Voice character:** a natural, warm, clear voice, slightly slow (140–160 words a minute). More human-like synthetic voices were rated more likeable, with no "uncanny valley" of the voice (Kühne 2020). Don't clone a real person.

**LLM licenses**
- Apache-2.0:
  - Qwen3 (0.6B/1.7B/4B, and Qwen3-4B-Instruct-2507, which never thinks)
  - Qwen3.5 small: it thinks by default; turn that off with `enable_thinking: false`
  - Gemma 4 E2B/E4B: the first Gemma under Apache
  - SmolLM3-3B, Granite 4 Nano and 4.2 3B, Ministral 3 3B
- MIT: Phi-4-mini (weak on facts, says its card).
- **Use-restricted:**
  - Llama 3.2: an acceptable-use policy, plus a "Built with Llama" credit, a NOTICE and the license if we distribute it
  - Gemma 3 and ShieldGemma (Gemma Terms)
  - LFM2/3 (a revenue cap)
  - Nemotron (NVIDIA's model license)
- **Prompt caching in llama-server breaks for hybrid models** (Qwen3.5, probably Granite "H", LFM2) and Gemma's sliding-window attention: every turn is re-processed. Qwen3 is a plain transformer, so caching works.
- **Running it on a Mac:** Homebrew's llama.cpp 0.8.0 (build 11960) fails to start on an M1 (macOS 15.6): its Metal library uses bfloat16, which the M1's Metal compiler rejects ("failed to build 'unary' library"). Run it CPU-only with `--device none -ngl 0`. Newer Apple chips are untested.
- **Qwen3-4B-Instruct-2507 as GGUF:** Qwen publishes no GGUF of it. We use lmstudio-community's Q4_K_M, pinned by commit and hash (`speech/README.md`); ggml-org has only Q8_0.
- **Guard models:** Qwen3Guard-Gen-0.6B (Apache-2.0, about 480 MB at Q4); Llama Guard 3 1B (Llama license); Granite Guardian (Apache, 8B only).
- **Quality:** no benchmark measures short spoken persona replies at this size. IFEval (from model cards): Qwen3.5-4B 89.8, Qwen3-4B-Instruct-2507 83.4, SmolLM3 about 77. Whether a model reliably emits an end tag needs our own test, about 50 dialogues.

**The robot's speaker** (from the SDK's code and others' players)
- PlayStream blocks until the service replies (about 60 ms; default timeout 1 s), not until the audio has played. The service queues audio sent faster than real time; gaps in what we send are audible.
- Others send 50–200 ms chunks. Gaps came from sleeping a whole chunk's time after each blocking call; pace by a clock instead.
- The end of playback: `{"play_state": 0}` on `rt/audio_msg`, probably only while the voice mode is on. Fall back on the audio's length.
- There is no measurement anywhere of the time from PlayStream to sound, or of loudness at 1 m. One project still doubled its PCM's gain at volume 100.

**Conversations with strangers** (HRI research and deployed robots)
- **Approaching:** walk up from the front and signal before speaking (Satake 2009). People who stop out of curiosity are often quiet, so open with a fixed greeting and one easy question (Koike 2025, a GPT-4o robot with a scripted greeting).
- **Ending:** forecast it from distance, stillness and attention, and hesitate while unsure (Bohus & Horvitz 2014). So combine the robot's "walking away" signal with the LLM's end tag. A close in steps (a summary or goal, a positive comment, a wish, thanks) was rated more polite than an abrupt one.
- **Hard cases:**
  - Never argue: give one polite deflection, then close.
  - Children crowd a robot that adults are using, and unsupervised groups block or hit it; leave early when that starts (Brščić 2015).
- **Recording people:**
  - Say it's a robot. Don't ask for names or personal details.
  - Talk in public places is likely outside California's eavesdropping law (Penal Code 632). This is not legal advice; check Stanford's IRB if it becomes research.
- **Remembering people** (a later goal):
  - Kanda's mall robot recognised only visitors who opted in.
  - Users want to see and delete what is stored.
  - So: spoken opt-in, a nickname, an embedding rather than images, a retention limit, and "forget me".
- **Speed tricks:** pre-rendered greeting and goodbye. Few-shot replies that open with a 2–4 word reaction ("Oh, nice!"), whose audio can start at the first comma. A filler or nod if the LLM takes over about 0.7 s.

**Other projects**
- With licenses we may use (as packages, or to learn from):
  - bob_llm (Apache-2.0): conversation history, OpenAI-compatible backends; used by unitree_converse
  - namanbansal013/unitree-openai-voice (MIT): a persona file; on interruption it flushes its queue and calls PlayStop
  - jsk's g1_ros (BSD-3): a speaker player with 100 ms chunks on a real-time deadline
- No license (read only): unitree_converse, ytphi/g1-sb01 (waits for `play_state` 0; profiles per face), g1-audio-driver, PiotrG1996's fork, C3PO (a live probe of the voice API).
