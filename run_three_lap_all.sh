#!/usr/bin/env bash
set -Eeuo pipefail
# Six official tracks, three laps per map, three cars. "both" also checks grid2.
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT"
TRACKS=(monza spa monaco cota hungaroring baku)
GRIDS=(1)
if [[ "${1:-}" == both ]]; then GRIDS=(1 2)
elif [[ -n "${1:-}" ]]; then
  echo "Usage: bash run_three_lap_all.sh [both]" >&2
  exit 2
fi
RUN_ID="$(date +%Y%m%d_%H%M%S)"
DEST="$ROOT/log/race3/$RUN_ID"
mkdir -p "$DEST"
CSV="$DEST/summary.csv"
FAIL_COUNT=0
for TRACK in "${TRACKS[@]}"; do
  for GRID in "${GRIDS[@]}"; do
    if [[ "$GRID" == 1 ]]; then LANE=3; else LANE=1; fi
    CASE_LOG="$DEST/${TRACK}_grid${GRID}.log"
    echo "RACE3 START track=$TRACK grid=$GRID laps=3 lane=$LANE"
    if timeout --signal=TERM --kill-after=15 "${URRC_RACE3_CASE_TIMEOUT_S:-3300}" \
         bash "$ROOT/run_battle.sh" "$TRACK" "$GRID" "$LANE" race3 >"$CASE_LOG" 2>&1; then
      RC=0
    else
      RC=$?
      FAIL_COUNT=$((FAIL_COUNT+1))
    fi
    python3 "$ROOT/kinematic/three_lap_report.py" \
      --case "$CASE_LOG" --track "$TRACK" --grid "$GRID" --lane "$LANE" \
      --exit-code "$RC" --append "$CSV"
  done
done
echo "RACE3 COMPLETE: $CSV"
echo "Rank, peak/average speeds, road-boundary crossings, car collisions and failed objectives are in summary.csv"
echo "RACE3 FAILED_CASES=$FAIL_COUNT"
if (( FAIL_COUNT>0 )); then exit 1; fi
