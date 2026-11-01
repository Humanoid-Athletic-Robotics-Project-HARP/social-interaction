#!/usr/bin/env bash
# Native diagnostic runner; find the retained environment from any git worktree.
set -euo pipefail
here="$(cd -- "$(dirname -- "$0")" && pwd -P)"
common="$(git -C "$here" rev-parse --path-format=absolute --git-common-dir)"
main="$(dirname -- "$common")"
native_python="${HARP_NATIVE_PYTHON:-$main/ros2/.pixi/envs/default/bin/python}"
if [[ ! -x "$native_python" ]]; then
  echo "Missing native Jazzy Python: $native_python. Set HARP_NATIVE_PYTHON to an installed native environment." >&2
  exit 2
fi
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
export PYTHONPATH="$here${PYTHONPATH:+:$PYTHONPATH}"
exec "$native_python" "$here/native_g1_sim.py" "$@"
