# Speech

How the robot talks with people. Both halves run beside our robot image, not in it: on
a laptop, or on PC2.
- `listen.py`: sound in, one transcript per thing a person says out. Design:
  `context/listening.md`.
- `talk.py`: what they said in, a spoken reply out, from a local LLM and voice. Design:
  `context/talking.md`.

## Setup

The models go beside this repo, in `harp/models/` next to HARP's recovery policy, so
every checkout and worktree finds them (`HARP_MODELS` points elsewhere). Run the
commands from inside `harp/`. Every download is pinned and checked by its hash.

**Listening.** Silero VAD (MIT):

```bash
mkdir -p models && curl -L -o models/silero_vad.onnx https://raw.githubusercontent.com/snakers4/silero-vad/v6.2.3/src/silero_vad/data/silero_vad.onnx
```

```bash
echo "1a153a22f4509e292a94e67d6f9b85e8deb25b4988682b7e174c65279d8788e3  models/silero_vad.onnx" | shasum -a 256 -c
```

NVIDIA's Parakeet TDT 0.6B v2 for sherpa-onnx (CC-BY-4.0, credited in the README's
Licenses), a 482 MB archive:

```bash
curl -L -o models/parakeet.tar.bz2 https://github.com/k2-fsa/sherpa-onnx/releases/download/asr-models/sherpa-onnx-nemo-parakeet-tdt-0.6b-v2-int8.tar.bz2
```

```bash
echo "157c157bc51155e03e37d2466522a3a737dd9c72bb25f36eb18912964161e1ad  models/parakeet.tar.bz2" | shasum -a 256 -c && tar -xjf models/parakeet.tar.bz2 -C models && rm models/parakeet.tar.bz2
```

**Talking.** The LLM, Qwen3-4B-Instruct-2507 (Apache-2.0) at Q4_K_M, 2.5 GB. Qwen
publishes no GGUF of it, so this is LM Studio's community build, pinned to one commit:

```bash
curl -L -o models/Qwen3-4B-Instruct-2507-Q4_K_M.gguf https://huggingface.co/lmstudio-community/Qwen3-4B-Instruct-2507-GGUF/resolve/4edb920b6f14e3b9284d4502a6485103d72cde05/Qwen3-4B-Instruct-2507-Q4_K_M.gguf
```

```bash
echo "8cdb57cbb880d313736a9bc4e3d3d2485f145b5e19cf33783746e753e82641fc  models/Qwen3-4B-Instruct-2507-Q4_K_M.gguf" | shasum -a 256 -c
```

The two voices we're comparing, from sherpa-onnx's releases. Unpack them without their
`espeak-ng-data/`: it is GPL, and sherpa-onnx 2.x no longer uses it. Kokoro v1.0
(Apache-2.0), 350 MB:

```bash
curl -L -o models/kokoro.tar.bz2 https://github.com/k2-fsa/sherpa-onnx/releases/download/tts-models/kokoro-multi-lang-v1_0.tar.bz2
```

```bash
echo "60703707bc05ccd48d60e1eb723f38b07d6e474795a0b6a588d7220400e0f7a4  models/kokoro.tar.bz2" | shasum -a 256 -c && tar -xjf models/kokoro.tar.bz2 -C models --exclude '*/espeak-ng-data' && rm models/kokoro.tar.bz2
```

Piper's `ljspeech` voice (a public-domain recording), 67 MB, and the word list it reads
from:

```bash
curl -L -o models/piper.tar.bz2 https://github.com/k2-fsa/sherpa-onnx/releases/download/tts-models/vits-piper-en_US-ljspeech-medium.tar.bz2
```

```bash
echo "3dfb4b759d8be032a4903a9538d128b0fda2a06ab1de6cbc2d93a97e2dd83dba  models/piper.tar.bz2" | shasum -a 256 -c && tar -xjf models/piper.tar.bz2 -C models --exclude '*/espeak-ng-data' && rm models/piper.tar.bz2
```

```bash
curl -L -o models/vits-piper-en_US-ljspeech-medium/lexicon-en-us.txt https://github.com/k2-fsa/sherpa-onnx/releases/download/tts-models/lexicon-en-us.txt
```

```bash
echo "8b67ae549a465eee78f92ecb74644ad90c3c58ce65f19c68baf85878a10ef73a  models/vits-piper-en_US-ljspeech-medium/lexicon-en-us.txt" | shasum -a 256 -c
```

llama.cpp (MIT) serves the LLM. On a Mac, with Homebrew:

```bash
brew install llama.cpp
```

Then, in `speech/`, on a laptop with Python 3.12:

```bash
python3.12 -m venv .venv && .venv/bin/pip install -r requirements.txt
```

PC2 has Python 3.8 and needs NVIDIA's onnxruntime-gpu, a GPU build of sherpa-onnx and a
CUDA build of llama.cpp instead ([../testground/speech/README.md](../testground/speech/README.md));
not yet installed there.

## Listen

From recordings (16-bit WAV, any rate):

```bash
.venv/bin/python listen.py --wav recording.wav
```

From this computer's microphone (Ctrl-C to stop). `--devices` lists the microphones and
`--device N` picks one. On macOS, allow your terminal app under System Settings >
Privacy & Security > Microphone; without it the mic sends silence.

```bash
.venv/bin/python listen.py --mic
```

On PC2, from the robot's mic. This needs Unitree's voice assistant in wake-up mode;
switching its mode changes the robot for everyone, so ask first.

```bash
.venv/bin/python listen.py --robot
```

Each turn is one JSON line on stdout:

```json
{"text": "Hi there, are you the robot from Stanford?", "start": 0.54, "end": 2.5, "silence_s": 0.7, "stt_s": 0.142, "after_end_s": 0.842}
```

## Talk

Start the LLM server in its own terminal and leave it running (about 3 GB of memory). On
an M1 Mac its GPU code fails, so run it on the CPU as here; on newer Macs, try `-ngl 99`
without `--device none` for the GPU.

```bash
llama-server -m ../../models/Qwen3-4B-Instruct-2507-Q4_K_M.gguf --device none -ngl 0 -c 4096 -t 4 --port 8080
```

That path is from this repo's main checkout; from a worktree, give the full path to
`harp/models/`. Then talk to HARP by typing what the person says, one line a turn
(Ctrl-D to end):

```bash
.venv/bin/python talk.py
```

`--voice piper` picks the other voice. `--no-play --save DIR` stays silent and keeps
what it said as WAVs. Each thing it says is one JSON line on stdout, with the times
from the end of your line to the LLM's first sentence, to its audio, and to its first
sound (which also waits for anything still playing, such as the greeting):

```json
{"event": "said", "text": "Oh, nice! What was the lecture about?", "llm_s": 0.603, "ready_s": 0.675, "first_sound_s": 0.676}
```

To talk to it out loud, give it listening's transcripts. Use headphones: listening
isn't muted yet while HARP speaks, so it would hear itself.

```bash
.venv/bin/python listen.py --mic | .venv/bin/python talk.py --heard
```

A word a voice can't say is skipped, and reported as an `unknown_words` event. Add it
to that voice's file in `lexicon/`, in the same notation as its neighbours there.

Tests of the turn rules and the text handling (no models needed):

```bash
.venv/bin/python -m unittest discover -s tests
```

## Licenses

Each package and model, its license, and the credit Parakeet's license asks for, is in
the [README's Licenses](../README.md#licenses).
