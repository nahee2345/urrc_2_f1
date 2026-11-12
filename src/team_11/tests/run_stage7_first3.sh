#!/usr/bin/env bash
set -Eeuo pipefail
# Stage 7 focused diagnostic: first three grids, onboard camera only.
TRACK="${1:-monza}"
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
export TEAM11_GRID_END=3
export TEAM11_SIGNAL_WIDE_ONLY=1
export PYTHONUNBUFFERED=1
echo "[TEAM11] Checking start lights ONLY for grids 1, 2 and 3"
exec bash "$ROOT/src/team_11/tests/run_stage7_all_grids.sh" "$TRACK"
