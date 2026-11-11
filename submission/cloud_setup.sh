#!/usr/bin/env bash
# Get a Claude Code cloud session (x86 Ubuntu) ready to build our image and run HIM's sim:
# HIM's checkout and Python environment beside this repo, the Holosoma checkpoint in models/,
# and Docker running. Run it at the start of a cloud session, or as the cloud environment's
# setup script (bash submission/cloud_setup.sh); each step is skipped once done. Then:
#   HARP_PLATFORM=linux/amd64 submission/build.sh
#   python3 submission/course.py NAME
set -uo pipefail
[[ "$(uname -s)" == Linux ]] || { echo "cloud_setup.sh is for Linux cloud sessions" >&2; exit 0; }

here="$(cd -- "$(dirname -- "$0")" && pwd -P)"
eval "$(python3 "$here/checkout.py")"
him="$HARP_HIM"
note() { echo "cloud_setup: $*"; }

if ! docker info >/dev/null 2>&1; then
  (dockerd >/tmp/dockerd.log 2>&1 &)
  for _ in $(seq 60); do docker info >/dev/null 2>&1 && break; sleep 1; done
  docker info >/dev/null 2>&1 || note "Docker didn't start (/tmp/dockerd.log)"
fi

# HIM's repo is private: the session needs access to it (attach it to the session).
if [[ ! -d "$him/.git" ]]; then
  git clone -q https://github.com/HIMRobotics/him-sim2real-management.git "$him" \
    || { note "can't clone HIM's repo into $him; give the session access to it"; exit 0; }
fi

venv="$him/.venv-policy-safety"
if [[ ! -x "$venv/bin/python" ]]; then
  { python3 -m venv "$venv" 2>/dev/null || uv venv -q --python 3.12 "$venv"; } \
    && "$venv/bin/python" -m pip install -q "$him/packages/policy_safety[test]" 2>/dev/null \
    || uv pip install -q --python "$venv/bin/python" "$him/packages/policy_safety[test]" \
    || note "can't make HIM's Python environment ($venv)"
fi

# The README's pinned Holosoma recovery checkpoint.
onnx="$HARP_MODELS/fastsac_g1_29dof.onnx"
sha=8346fd90778439395922a8c7256f24125ae84b8dea949128bac9e23c02bc7717
if [[ ! -f "$onnx" ]]; then
  mkdir -p "$HARP_MODELS"
  curl -fsSL --retry 3 -o "$onnx.part" https://raw.githubusercontent.com/amazon-far/holosoma/d1e8499c10670f44c49d161b9876a4c17ac305f7/src/holosoma_inference/holosoma_inference/models/loco/g1_29dof/fastsac_g1_29dof.onnx \
    && echo "$sha  $onnx.part" | sha256sum -c --quiet && mv "$onnx.part" "$onnx" \
    || { rm -f "$onnx.part"; note "can't download the Holosoma checkpoint"; }
fi
exit 0
