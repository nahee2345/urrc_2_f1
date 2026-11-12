#!/usr/bin/env bash
set -euo pipefail
REPO_URL="https://github.com/nahee2345/urrc_2_f1.git"
DEST="${1:-$HOME/urrc_2_f1}"
if [[ ! -e "$DEST" ]]; then
    git clone --branch main "$REPO_URL" "$DEST"
elif [[ -d "$DEST/.git" ]]; then
    cd "$DEST"
    if [[ -n "$(git status --porcelain)" ]]; then
        echo "STOP: local changes detected. Commit/stash before updating." >&2
        exit 2
    fi
    if [[ "$(git remote get-url origin)" != "$REPO_URL" && "$(git remote get-url origin)" != "git@github.com:nahee2345/urrc_2_f1.git" ]]; then
        echo "STOP: origin is not urrc_2_f1" >&2; exit 2
    fi
    git fetch --prune origin
    git switch main
    git pull --ff-only origin main
else
    echo "STOP: $DEST exists but is not a Git checkout." >&2
    exit 2
fi
echo "Updated: $DEST"
