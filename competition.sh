#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
TARGET="$ROOT/upstream/urrc_f1_ws/competition.sh"
if [[ ! -f "$TARGET" ]]; then
  echo "대회 운영 파일이 없습니다. git submodule update --init --recursive 를 실행하세요." >&2
  exit 2
fi
exec bash "$TARGET" "$@"
