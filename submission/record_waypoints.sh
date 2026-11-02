#!/usr/bin/env bash
# Record poses in a running submission container, then copy the route to the host.
# Usage: submission/record_waypoints.sh CONTAINER OUTPUT.yaml
# Build the image first; run this in another terminal while its sim is running.
set -euo pipefail
if [[ $# != 2 ]]; then
  echo "Usage: $0 CONTAINER OUTPUT.yaml" >&2
  exit 2
fi
container="$1"
output="$2"
if [[ -e "$output" || -L "$output" ]]; then
  echo "Output already exists: $output" >&2
  exit 2
fi
parent="$(dirname -- "$output")"
[[ -d "$parent" ]] || { echo "Missing output directory: $parent" >&2; exit 2; }
host_tmp="$(mktemp -d "$parent/.harp-route.XXXXXX")"
container_tmp=""
cleanup() {
  [[ -z "$container_tmp" ]] || docker exec "$container" rm -rf -- "$container_tmp" >/dev/null 2>&1 || true
  rm -rf -- "$host_tmp"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
container_tmp="$(docker exec "$container" mktemp -d /tmp/harp-route.XXXXXX)"
input=(-i)
[[ ! -t 0 || ! -t 1 ]] || input+=(-t)
docker exec "${input[@]}" "$container" bash -c '
  set -e
  source /opt/ros/foxy/setup.bash
  if [[ -f /tmp/harp_fastdds.xml ]]; then
    export FASTRTPS_DEFAULT_PROFILES_FILE=/tmp/harp_fastdds.xml
  fi
  export PYTHONPATH="/harp:${PYTHONPATH:-}"
  exec python3 -m harp.record_waypoints --output "$1"
' bash "$container_tmp/route.yaml"
docker cp "$container:$container_tmp/route.yaml" "$host_tmp/route.yaml"
# Refuse a destination created while recording, rather than replace it.
python3 -c 'import os, sys; os.link(sys.argv[1], sys.argv[2])' "$host_tmp/route.yaml" "$output"
echo "Saved route on host: $output"
