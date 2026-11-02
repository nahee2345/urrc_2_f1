#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
TARGET="$ROOT/upstream/urrc_f1_ws/watch_v10.sh"
if [[ ! -f "$TARGET" ]]; then
  echo "관전 카메라 파일이 없습니다. git submodule update --init --recursive 를 실행하세요." >&2
  exit 2
fi
if [[ "${1:-}" == "random" ]]; then
  TRACK="$(python3 - <<'PY'
import json
import os
from pathlib import Path
state = Path(os.environ.get("XDG_STATE_HOME", str(Path.home() / ".local/state")))
path = state / "urrc_f1" / "last_run.json"
try:
    name = json.loads(path.read_text(encoding="utf-8"))["track"]
    if name not in ("monza", "spa", "monaco", "cota", "hungaroring", "baku"):
        raise ValueError("지원하지 않는 트랙")
except (OSError, KeyError, ValueError) as exc:
    raise SystemExit(f"실행 중인 랜덤 맵을 확인할 수 없습니다: {exc}")
print(name)
PY
)"
  shift
  set -- "$TRACK" "$@"
fi
exec bash "$TARGET" "$@"
