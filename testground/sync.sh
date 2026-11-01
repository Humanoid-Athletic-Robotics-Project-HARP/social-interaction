#!/usr/bin/env bash
# Between this laptop and the testground on PC2, over SSH:
#   ./sync.sh pull TEST   copy TEST/captures/ on PC2 (e.g. lidar) into TEST/captures/ here
#   ./sync.sh push        copy this folder's scripts to PC2: a quick fix without a commit
#                         (the clone's `git pull` is the usual way: README)
#   ./sync.sh ssh         a shell in the testground on PC2
# G1_HOST: user@address of PC2 (required). G1_DIR: this folder's copy on PC2, relative
# to the login's home (default: in the repo's clone in the robot's testground folder).
# Never deletes anything on either side.
set -euo pipefail
here=$(cd "$(dirname "$0")" && pwd)
host=${G1_HOST:?set G1_HOST=user@address of PC2}
dir=${G1_DIR:-workspaces/testground-social-interaction/social-interaction/testground}

case ${1:-} in
  pull)
    test=${2:?usage: $0 pull TEST (a folder here, e.g. lidar)}
    mkdir -p "$here/$test/captures"
    rsync -av --partial "$host:$dir/$test/captures/" "$here/$test/captures/" ;;
  push)
    rsync -av --exclude .venv --exclude __pycache__ --exclude captures "$here/" "$host:$dir/" ;;
  ssh)
    ssh -t "$host" "cd $dir && exec \$SHELL -l" ;;
  *)
    echo "usage: G1_HOST=user@address $0 pull TEST | push | ssh" >&2; exit 2 ;;
esac
