#!/usr/bin/env bash
# Run HARP's image in him-sim2real-management's simulator, with the walker's joint profile.
# Usage: submission/sim.sh [--world NAME] [--no-view] [its flags, e.g. --seconds 20 --output DIR]
# It opens HIM's browser viewer (--view) unless --no-view: press Start there to run, and
# Ctrl-C closes the page afterwards. --world NAME loads submission/worlds/NAME.xml: its
# scene into HIM's sim, and the world with its movers into an image layer
# (harp-submission:NAME) for our lidar.
# Build first with submission/build.sh. Uses desktop_sim.sh on macOS and sim.sh on Linux.
set -euo pipefail

here="$(cd -- "$(dirname -- "$0")" && pwd -P)"
him="${HIM_REPO:-$(cd -- "$here/../.." && pwd -P)/him-sim2real-management}"
model="$here/.build/him/environment"
profile="$here/.build/him/container/project/joint_profile.json"
if [[ ! -f "$profile" ]]; then
  echo "Missing $profile. Run submission/build.sh first." >&2
  exit 2
fi

world=""
view=(--view)
args=()
while (($#)); do
  case "$1" in
    --world) world="${2:?--world needs a name}"; shift 2 ;;
    --no-view) view=(); shift ;;
    --view) shift ;;
    *) args+=("$1"); shift ;;
  esac
done
args+=(${view[@]+"${view[@]}"})

# Compose mounts only the HIM checkout, so the profile and scenes go in its ignored .cache.
mkdir -p "$him/.cache/harp"
cp "$profile" "$him/.cache/harp/joint_profile.json"
export G1_JOINT_PROFILE="$him/.cache/harp/joint_profile.json"
image="${HARP_IMAGE:-harp-submission:local}"
if [[ -n "$world" ]]; then
  scenes="$him/.cache/harp/scenes"
  python3 "$here/stage_world.py" "$world" "$model" "$scenes" >/dev/null
  docker build -q -f "$here/Dockerfile.world" --build-arg BASE="$image" \
    -t "harp-submission:$world" "$scenes/image/$world" >/dev/null
  image="harp-submission:$world"
  args+=(--scene "$scenes/$world.xml")
fi
launcher="sim.sh"
[[ "$(uname)" == Darwin ]] && launcher="desktop_sim.sh"
exec "$him/$launcher" --image "$image" ${args[@]+"${args[@]}"}
