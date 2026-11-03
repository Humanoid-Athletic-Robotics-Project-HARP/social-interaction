# Speech

How the robot hears people: sound in, one transcript per thing a person says out
(`listen.py`). The design is in `context/listening.md`. It runs beside our robot image,
not in it: on a laptop, or on PC2.

## Setup

The models go beside this repo, in `harp/models/` next to HARP's recovery policy, so
every checkout and worktree finds them. From inside `harp/`, get Silero VAD (MIT):

```bash
mkdir -p models && curl -L -o models/silero_vad.onnx https://raw.githubusercontent.com/snakers4/silero-vad/v6.2.3/src/silero_vad/data/silero_vad.onnx
```

```bash
echo "1a153a22f4509e292a94e67d6f9b85e8deb25b4988682b7e174c65279d8788e3  models/silero_vad.onnx" | shasum -a 256 -c
```

Then NVIDIA's Parakeet TDT 0.6B v2 for sherpa-onnx (CC-BY-4.0, below). The archive is
482 MB:

```bash
curl -L -o models/parakeet.tar.bz2 https://github.com/k2-fsa/sherpa-onnx/releases/download/asr-models/sherpa-onnx-nemo-parakeet-tdt-0.6b-v2-int8.tar.bz2
```

```bash
echo "157c157bc51155e03e37d2466522a3a737dd9c72bb25f36eb18912964161e1ad  models/parakeet.tar.bz2" | shasum -a 256 -c && tar -xjf models/parakeet.tar.bz2 -C models && rm models/parakeet.tar.bz2
```

`HARP_MODELS` points elsewhere if you keep them somewhere else. Then, in `speech/`, on a
laptop with Python 3.12:

```bash
python3.12 -m venv .venv && .venv/bin/pip install -r requirements.txt
```

PC2 has Python 3.8 and needs NVIDIA's onnxruntime-gpu and a GPU build of sherpa-onnx
instead ([../testground/speech/README.md](../testground/speech/README.md)); not yet
installed there.

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

Tests of the turn rules (no models needed):

```bash
.venv/bin/python -m unittest discover -s tests
```

## Licenses

We use these as packages and models, never copying their code:
- [Silero VAD](https://github.com/snakers4/silero-vad) (MIT)
- [sherpa-onnx](https://github.com/k2-fsa/sherpa-onnx) (Apache-2.0)
- [onnxruntime](https://github.com/microsoft/onnxruntime) (MIT)
- [python-sounddevice](https://github.com/spatialaudio/python-sounddevice) (MIT)
- Speech-to-text: [Parakeet TDT 0.6B v2](https://huggingface.co/nvidia/parakeet-tdt-0.6b-v2) by NVIDIA, licensed under [CC-BY-4.0](https://creativecommons.org/licenses/by/4.0/), in sherpa-onnx's int8 export.
