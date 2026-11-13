#!/usr/bin/env bash
set -Eeuo pipefail
# Stage 1: 6 official tracks x start grids 1 and 2 = 12 independent battles.
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT"
TRACKS=(monza spa monaco cota hungaroring baku)
RUN_ID="$(date +%Y%m%d_%H%M%S)"
DEST="$ROOT/log/battle_stage1/$RUN_ID"
mkdir -p "$DEST"
CSV="$DEST/summary.csv"
printf 'number,track,grid,lane,result,exit_code,log_file\n' > "$CSV"
PASS_COUNT=0
FAIL_COUNT=0
PARTIAL_COUNT=0
N=0
echo "STAGE1: 12 independent cases, 6 tracks x 2 grids"
echo "Summary: $CSV"
for TRACK in "${TRACKS[@]}"; do
  for GRID in 1 2; do
    N=$((N+1))
    if [[ "$GRID" == 1 ]]; then LANE=3; else LANE=1; fi
    CASE_LOG="$DEST/${TRACK}_grid${GRID}.log"
    echo "[$N/12] $TRACK grid=$GRID lane=$LANE"
    if timeout --signal=TERM --kill-after=15 "${URRC_STAGE1_CASE_TIMEOUT_S:-480}" \
       bash "$ROOT/run_battle.sh" "$TRACK" "$GRID" "$LANE" > "$CASE_LOG" 2>&1; then
      RC=0
      if grep -q "^PASS BATTLE: $TRACK grid $GRID lane $LANE " "$CASE_LOG"; then
        STATUS=PASS
        PASS_COUNT=$((PASS_COUNT+1))
      elif grep -q "^PARTIAL BATTLE: $TRACK grid $GRID lane $LANE " "$CASE_LOG"; then
        STATUS=PARTIAL_DEFENSE_UNVERIFIED
        PARTIAL_COUNT=$((PARTIAL_COUNT+1))
      else
        STATUS=FAIL_UNCONFIRMED
        FAIL_COUNT=$((FAIL_COUNT+1))
      fi
    else
      RC=$?
      STATUS=FAIL
      FAIL_COUNT=$((FAIL_COUNT+1))
    fi
    printf '%s,%s,%s,%s,%s,%s,%s\n' "$N" "$TRACK" "$GRID" "$LANE" "$STATUS" "$RC" "$CASE_LOG" >> "$CSV"
    echo "[$N/12] $STATUS (exit=$RC); $CASE_LOG"
    sleep 2
  done
done
echo "STAGE1 COMPLETE: PASS=$PASS_COUNT PARTIAL=$PARTIAL_COUNT FAIL=$FAIL_COUNT"
echo "Results: $CSV"
if (( FAIL_COUNT>0 )); then exit 1; fi
