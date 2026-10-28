#!/usr/bin/env bash
# Build HARP's submission image (linux/arm64) and save it as submission/submission.tar.
# Usage: submission/build.sh [TAG]        (default tag: harp-submission:local)
# HIM_REPO is the him-sim2real-management checkout (default: beside this repo). It is
# only read: its walker example is staged into submission/.build, never into git.
set -euo pipefail

here="$(cd -- "$(dirname -- "$0")" && pwd -P)"
him="${HIM_REPO:-$(cd -- "$here/../.." && pwd -P)/him-sim2real-management}"
python="$him/.venv-policy-safety/bin/python"
tag="${1:-harp-submission:local}"
if [[ ! -x "$python" ]]; then
  echo "Missing $python. Set HIM_REPO and create its .venv-policy-safety (its README)." >&2
  exit 2
fi

stage="$here/.build"
rm -rf "$stage"
mkdir -p "$stage"
# The walker example's code, models, joint profile and flat scene, exactly as HIM builds it.
"$python" -c 'import sys; sys.path.insert(0, sys.argv[1]); from prepare import prepare; prepare("walker", sys.argv[2])' \
  "$him/containers/examples" "$stage/him" >/dev/null
cp "$here/Dockerfile" "$here/entrypoint.sh" "$here/harp_node.py" "$stage/"
cp -R "$here/harp" "$here/nav2" "$here/nav2_graceful_controller" "$stage/"

docker build --platform linux/arm64 -t "$tag" "$stage"
docker save "$tag" -o "$here/submission.tar"
# The walker's joint profile, which HIM's setup.sh needs on the robot (--profile).
cp "$stage/him/container/project/joint_profile.json" "$here/joint_profile.json"
echo "Saved $tag to $here/submission.tar, and its joint profile to $here/joint_profile.json"
