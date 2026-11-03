# Speech models

How fast the local speech models run on PC2, the G1's Jetson Orin NX 16GB, measured
rather than estimated. The plan and the estimates: `context/speech-research.md`,
`context/listening.md`, `context/talking.md`.

`bench.py` times each stage it's given a model for:
- voice detection: Silero VAD
- speech-to-text: Parakeet through sherpa-onnx
- the LLM: llama.cpp's `llama-server` or Ollama
- text-to-speech: a Kokoro or Piper voice through sherpa-onnx

It writes `captures/NAME/`: the times, a summary, the power mode, and `tegrastats`
every 0.5 s. It makes no sound and uses no robot service. It needs no sudo.

## Setup on PC2

Ask the team first: this installs packages and downloads models (about 4 GB) into our
folder on PC2. Everything goes inside `speech/`. `models/` is ignored by git and by
`sync.sh push`, so download the models on PC2 itself.

**Python**
```bash
python3 -m venv --system-site-packages .venv
```
`--system-site-packages` keeps PC2's numpy 1.24.4.

**Voice detection:** Silero VAD's `silero_vad.onnx` (MIT), from
[snakers4/silero-vad](https://github.com/snakers4/silero-vad) at a release tag, into
`models/`. It runs on onnxruntime:
- on PC2, NVIDIA's onnxruntime-gpu 1.16.0 wheel for JetPack 5 and Python 3.8, from the
  [Jetson Zoo](https://elinux.org/Jetson_Zoo#ONNX_Runtime);
- on a laptop, `pip install onnxruntime==1.19.2`.

**Speech-to-text:** sherpa-onnx.
- `pip install sherpa-onnx` is CPU only (`--stt-provider cpu`).
- For the GPU, build it with `SHERPA_ONNX_ENABLE_GPU=ON`, as
  [its Jetson instructions](https://k2-fsa.github.io/sherpa/onnx/install/linux.html)
  say for Orin NX with CUDA 11.4 (untested by us).
- The model: `sherpa-onnx-nemo-parakeet-tdt-0.6b-v2-int8` from sherpa-onnx's
  [asr-models release](https://github.com/k2-fsa/sherpa-onnx/releases/tag/asr-models),
  unpacked into `models/`. It is CC-BY-4.0: credit NVIDIA if we use it.

**LLM:** llama.cpp's `llama-server` (MIT), built for the GPU (`-DGGML_CUDA=ON
-DCMAKE_CUDA_ARCHITECTURES=87`), for example in an `l4t-jetpack:r35` container. If it
won't build on CUDA 11.4, use Ollama's `jetpack5` build. Then the model we chose,
Qwen3-4B-Instruct-2507 at Q4_K_M (Apache-2.0; `../../speech/README.md` has the pinned
download), or Qwen3-1.7B to compare. Serve it on the GPU with two CPU threads:
```bash
llama-server -m models/MODEL.gguf -ngl 99 -c 4096 -t 2 --port 28731
```

**Text-to-speech:** the same sherpa-onnx, with a voice folder from its
[tts-models release](https://github.com/k2-fsa/sherpa-onnx/releases/tag/tts-models):
Kokoro (`kokoro-multi-lang-v1_0`, speaker 3 is `af_heart`) or Piper `ljspeech`
(`vits-piper-en_US-ljspeech-medium`, plus sherpa-onnx's `lexicon-en-us.txt` in its
folder). `../../speech/README.md` has the pinned downloads; leave out their
`espeak-ng-data/` (GPL, and unused since sherpa-onnx 2.0). Don't use Piper's own C++
binary or the `piper-tts` package: both carry GPL code. Other Piper voices: check the
dataset's license first, since several (`lessac`, `ryan`, `hfc_*`) are non-commercial.

## Running it

In `speech/` on PC2, with the venv:
```bash
nice -n 10 .venv/bin/python bench.py maxn-idle --note "MAXN, harness off" --vad models/silero_vad.onnx --stt models/sherpa-onnx-nemo-parakeet-tdt-0.6b-v2-int8 --stt-int8 --llm http://127.0.0.1:28731 --tts models/kokoro-multi-lang-v1_0 --tts-speaker 3 --tts-provider cuda
```

Run it twice: with HARP's harness off, then running. The capture's `meta.json` records
which, and the power mode (`nvpmodel -q`), since the default 15 W mode runs the GPU
at two thirds of MAXN's clock.

STT times the TTS stage's sentences unless `--wavs` gives recordings. Use recordings
for word errors, since a synthetic voice is easier than a person 1 m away. From
the laptop, bring the captures back:
```bash
./sync.sh pull speech
```
