#!/usr/bin/env bash
# Run HARP's image in him-sim2real-management's simulator, with the walker's joint profile.
# Usage: submission/sim.sh [--world NAME] [--no-view] [its flags, e.g. --seconds 20 --output DIR]
# It opens HIM's browser viewer (--view) unless --no-view: press Start there to run, and
# Ctrl-C closes the page afterwards. --world NAME loads submission/worlds/NAME.xml: its
# scene into HIM's sim, and the world with its movers into an image layer
# (harp-submission:NAME, or harp-submission-WORKTREE:NAME in a git worktree) for our lidar.
# Build first with submission/build.sh. Uses desktop_sim.sh on macOS and sim.sh on Linux.
# One run at a time on this machine: it waits for any build or sim running first
# (submission/checkout.py's lock), and holds the lock until HIM's viewer is closed.
set -euo pipefail

here="$(cd -- "$(dirname -- "$0")" && pwd -P)"
eval "$(python3 "$here/checkout.py")"
him="$HARP_HIM"
model="$here/.build/him/environment"
profile="$here/.build/him/container/project/joint_profile.json"
if [[ ! -f "$profile" ]]; then
  echo "Missing $profile. Run submission/build.sh first." >&2
  exit 2
fi

world=""
view=(--view)
out=""
scene=""
given=("$@")
args=()
while (($#)); do
  case "$1" in
    --world) world="${2:?--world needs a name}"; shift 2 ;;
    --no-view) view=(); shift ;;
    --view) shift ;;
    --output) out="${2:?--output needs a folder}"; args+=("$1" "$2"); shift 2 ;;
    --scene) scene="${2:?--scene needs a file}"; args+=("$1" "$2"); shift 2 ;;
    *) args+=("$1"); shift ;;
  esac
done
args+=(${view[@]+"${view[@]}"})

# HIM's launcher starts by stopping whatever sim is running, so wait for it to finish.
mkdir -p "$HARP_STAGE"
if [[ -z "${HARP_SIM_LOCK_HELD:-}" ]]; then
  exec 9>>"$HARP_LOCK"
  python3 "$here/checkout.py" lock 9 sim.sh
  export HARP_SIM_LOCK_HELD=1
fi
# Compose mounts only the HIM checkout, so the profile and scenes go in its ignored .cache.
cp "$profile" "$HARP_STAGE/joint_profile.json"
export G1_JOINT_PROFILE="$HARP_STAGE/joint_profile.json"
# HIM's sim needs the Holosoma recovery checkpoint; by default it sits in models/ beside HIM.
holosoma="$HARP_MODELS/fastsac_g1_29dof.onnx"
[[ -z "${HIM_HOLOSOMA_ONNX:-}" && -f "$holosoma" ]] && export HIM_HOLOSOMA_ONNX="$holosoma"
image="${HARP_IMAGE:-$HARP_IMAGE_REPO:local}"
if [[ -n "$world" ]]; then
  scenes="$HARP_STAGE/scenes"
  python3 "$here/stage_world.py" "$world" "$model" "$scenes" >/dev/null
  docker build -q -f "$here/Dockerfile.world" --build-arg BASE="$image" \
    -t "$HARP_IMAGE_REPO:$world" "$scenes/image/$world" >/dev/null
  image="$HARP_IMAGE_REPO:$world"
  scene="$scenes/$world.xml"
  args+=(--scene "$scene")
fi
# HIM's viewer takes the first free port from 8765. Right after a run, macOS holds 8765
# for up to a minute after the browser's connections, so wait for it to stay on 8765.
port="${G1_SIM_VIEW_PORT:-8765}"
if ((${#view[@]})); then
  for _ in $(seq 60); do
    python3 -c "import socket, sys; socket.socket().bind(('127.0.0.1', int(sys.argv[1])))" "$port" \
      2>/dev/null && break
    sleep 1
  done
fi
launcher="sim.sh"
[[ "$(uname)" == Darwin ]] && launcher="desktop_sim.sh"
# With --output, the run folder keeps how the run was made (run.json, and its scene in
# scene/, so a stripped trace can be replayed with its obstacles: course.py --replay), and
# is pruned once HIM's sim and viewer have finished (course.py --prune).
if [[ -n "$out" ]]; then
  python3 "$here/course.py" --record "$out" --scene "$scene" --image "$image" \
    --sim-args ${given[@]+"${given[@]}"}
fi
# Ctrl-C closes HIM's viewer; a no-op trap (not an ignored signal, which HIM's launcher would
# inherit) keeps this script alive to prune after it. The launcher doesn't get the lock's
# file (9>&-), so nothing it leaves running can hold the lock after this script.
trap ':' INT
status=0
"$him/$launcher" --image "$image" ${args[@]+"${args[@]}"} 9>&- || status=$?
trap - INT
[[ -z "$out" ]] || python3 "$here/course.py" --prune "$out" || true
exit "$status"
