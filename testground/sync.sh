#!/usr/bin/env bash
# Between this laptop and the testground folder on PC2, over SSH:
#   ./sync.sh push        copy this folder's scripts to PC2 (files that are already the
#                         same there are left untouched)
#   ./sync.sh pull TEST   copy TEST/captures/ on PC2 (e.g. lidar) into TEST/captures/ here
#   ./sync.sh ssh         a shell in the folder on PC2
# G1_HOST: user@address of PC2, read from the main checkout's .env (never committed) if
# not set. G1_DIR: the folder on PC2, relative to the login's home. G1_SSH: extra ssh
# options, e.g. "-S ~/.ssh/cm-g1" to reuse a shared connection.
# Never deletes anything on either side.
set -euo pipefail
here=$(cd "$(dirname "$0")" && pwd)
main=$(cd "$here" && dirname "$(cd "$(git rev-parse --git-common-dir)" && pwd)")
if [ -z "${G1_HOST-}" ] && [ -f "$main/.env" ]; then
  G1_HOST=$(sed -n 's/^G1_HOST=//p' "$main/.env")
fi
host=${G1_HOST:?set G1_HOST=user@address of PC2, or put it in $main/.env}
dir=${G1_DIR:-Workspace/testground_social_interaction}
ssh_cmd="ssh ${G1_SSH-}"

case ${1:-} in
  push)
    rsync -rlv --checksum -e "$ssh_cmd" --exclude .venv --exclude __pycache__ \
      --exclude captures --exclude models --exclude .gitignore "$here/" "$host:$dir/" ;;
  pull)
    test=${2:?usage: $0 pull TEST (a folder here, e.g. lidar)}
    mkdir -p "$here/$test/captures"
    rsync -av --partial -e "$ssh_cmd" "$host:$dir/$test/captures/" "$here/$test/captures/" ;;
  ssh)
    $ssh_cmd -t "$host" "cd $dir && exec \$SHELL -l" ;;
  *)
    echo "usage: $0 push | pull TEST | ssh" >&2; exit 2 ;;
esac
