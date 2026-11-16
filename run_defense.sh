#!/usr/bin/env bash
set -Eeuo pipefail
# Defense validation: 6 official tracks x two grids, rear 7m behind at 14m/s.
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT"
TRACKS=(monza spa monaco cota hungaroring baku)
RUN_ID="$(date +%Y%m%d_%H%M%S)"
DEST="$ROOT/log/battle_defense/$RUN_ID"
mkdir -p "$DEST"
CSV="$DEST/summary.csv"
printf 'number,track,grid,lane,result,exit_code,replay_result,failure_category,wall_contact,log_file\n' > "$CSV"
PASS_COUNT=0
FAIL_COUNT=0
PARTIAL_COUNT=0
N=0
echo "DEFENSE CHECK: 12 cases, rear challenger 7m behind at 14m/s"
echo "Summary: $CSV"
for TRACK in "${TRACKS[@]}"; do
  for GRID in 1 2; do
    N=$((N+1))
    if [[ "$GRID" == 1 ]]; then LANE=3; else LANE=1; fi
    CASE_LOG="$DEST/${TRACK}_grid${GRID}.log"
    echo "[$N/12] $TRACK grid=$GRID lane=$LANE"
    if timeout --signal=TERM --kill-after=15 "${URRC_STAGE1_CASE_TIMEOUT_S:-720}" \
       bash "$ROOT/run_battle.sh" "$TRACK" "$GRID" "$LANE" defense > "$CASE_LOG" 2>&1; then
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
    DIAG="$(python3 "$ROOT/kinematic/diagnose_battle.py" --case "$CASE_LOG" --exit-code "$RC")"
    REPLAY_RESULT="$(printf '%s\n' "$DIAG" | sed -n 's/^replay_result=//p')"
    FAILURE_CATEGORY="$(printf '%s\n' "$DIAG" | sed -n 's/^failure_category=//p')"
    WALL_CONTACT="$(printf '%s\n' "$DIAG" | sed -n 's/^wall_contact=//p')"
    printf '%s,%s,%s,%s,%s,%s,%s,%s,%s,%s\n' "$N" "$TRACK" "$GRID" "$LANE" "$STATUS" "$RC" "$REPLAY_RESULT" "$FAILURE_CATEGORY" "$WALL_CONTACT" "$CASE_LOG" >> "$CSV"
    echo "[$N/12] $STATUS cause=$FAILURE_CATEGORY replay=$REPLAY_RESULT wall=$WALL_CONTACT (exit=$RC); $CASE_LOG"
    sleep 2
  done
done
echo "DEFENSE CHECK COMPLETE: PASS=$PASS_COUNT PARTIAL=$PARTIAL_COUNT FAIL=$FAIL_COUNT"
python3 "$ROOT/kinematic/diagnose_battle.py" --summary "$CSV"
echo "Results: $CSV"
if (( FAIL_COUNT>0 )); then exit 1; fi
