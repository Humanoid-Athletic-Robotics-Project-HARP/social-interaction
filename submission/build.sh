#!/usr/bin/env bash
# Build HARP's submission image (linux/arm64) and save it as submission/submission.tar.
# Usage: submission/build.sh [TAG]   (default: harp-submission:local, or in a git worktree
#                                     harp-submission-NAME:local: submission/checkout.py)
# HIM_REPO is the him-sim2real-management checkout (default: beside the main checkout). It
# is only read: its walker example is staged into submission/.build, never into git.
# It waits for any build or sim running on this machine first (checkout.py's lock).
set -euo pipefail

here="$(cd -- "$(dirname -- "$0")" && pwd -P)"
eval "$(python3 "$here/checkout.py")"
him="$HARP_HIM"
python="$him/.venv-policy-safety/bin/python"
tag="${1:-$HARP_IMAGE_REPO:local}"
if [[ ! -x "$python" ]]; then
  echo "Missing $python. Set HIM_REPO and create its .venv-policy-safety (its README)." >&2
  exit 2
fi
# A build during someone's sim run takes the memory and CPU its deadlines need.
if [[ -z "${HARP_SIM_LOCK_HELD:-}" ]]; then
  mkdir -p "$(dirname -- "$HARP_LOCK")"
  exec 9>>"$HARP_LOCK"
  python3 "$here/checkout.py" lock 9 build.sh
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
