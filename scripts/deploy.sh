#!/usr/bin/env bash
# Copy this repo to the robot's Jetson over Ethernet (no GitHub round trip).
#   scripts/deploy.sh                    # default host
#   ROBOT=unitree@10.0.0.5 scripts/deploy.sh
set -euo pipefail

ROBOT="${ROBOT:-unitree@192.168.123.164}"
DEST="${DEST:-~/harp-social-interaction}"
REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"

rsync -av --delete \
  --exclude .git --exclude build --exclude install --exclude log --exclude deps \
  --exclude .DS_Store --exclude __pycache__ \
  "$REPO_ROOT/" "$ROBOT:$DEST/"

echo "Synced to $ROBOT:$DEST. On the robot: cd $DEST && colcon build && source install/setup.bash"
