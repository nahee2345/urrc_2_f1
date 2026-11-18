#!/usr/bin/env bash
# Official-map qualifying rehearsal for TEAM08. No changes to organizer tooling.
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT"
source /opt/ros/jazzy/setup.bash
if [[ ! -f race/install/setup.bash ]]; then
  echo "[TEAM08] Package not prepared; run: bash race_v24.sh prepare" >&2
  exit 1
fi
source race/install/setup.bash
export GZ_SIM_SYSTEM_PLUGIN_PATH="$ROOT/race/install/lib:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"
TRACK="${1:-monaco}"
SECONDS="${2:-600}"
case "$TRACK" in monza|spa|monaco|cota|hungaroring|baku) ;; *)
  echo "[TEAM08] Unsupported track: $TRACK" >&2; exit 2;; esac
# Only the displayed labels are simplified. Underlying official programs and
# filenames are unchanged, and the original process exit status is preserved.
bash race_v24.sh rehearse qualifying --track "$TRACK" --team team_08 \
  --seconds "$SECONDS" --start-mode signal 2>&1 |
  sed -u -E 's/\\[v[0-9]+ ([^]]+)\\]/[\\1]/g; s/\\[v[0-9]+\\]/[RACE]/g'
