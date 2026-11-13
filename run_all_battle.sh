#!/usr/bin/env bash
set -Eeuo pipefail
# 18 battles: lane1 x six tracks, lane2 x six tracks, lane3 x six tracks.
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT"
TRACKS=(monza spa monaco cota hungaroring baku)
RUN_ID="$(date +%Y%m%d_%H%M%S)"
OUT="$ROOT/log/battle_batch/$RUN_ID"
mkdir -p "$OUT"
REPORT="$OUT/summary.csv"
printf 'lane,track,result,exit_code,log_file\n' >"$REPORT"
PASSED=0
FAILED=0
for LANE in 1 2 3; do
  for TRACK in "${TRACKS[@]}"; do
    N=$((PASSED+FAILED+1))
    CASE_LOG="$OUT/${LANE}_${TRACK}.log"
    echo "[$N/18] BATTLE lane=$LANE track=$TRACK"
    GRID=1
    if [[ "$LANE" == 1 ]];then GRID=2;fi
    if timeout -s TERM -k 15 "${URRC_BATTLE_CASE_TIMEOUT_S:-480}" \
      bash "$ROOT/run_battle.sh" "$TRACK" "$GRID" "$LANE" >"$CASE_LOG" 2>&1; then
      CODE=0
      if grep -q "^PASS BATTLE: $TRACK grid $GRID lane $LANE " "$CASE_LOG"; then
        RESULT=PASS
        PASSED=$((PASSED+1))
      else
        RESULT=FAIL_UNCONFIRMED
        FAILED=$((FAILED+1))
      fi
    else
      CODE=$?
      RESULT=FAIL
      FAILED=$((FAILED+1))
    fi
    printf '%s,%s,%s,%s,%s\n' "$LANE" "$TRACK" "$RESULT" "$CODE" "$CASE_LOG" >>"$REPORT"
    echo "[$N/18] $RESULT (code $CODE)"
    sleep 2
  done
done
echo "BATTLE 18 COMPLETE: PASS=$PASSED FAIL=$FAILED"
echo "Summary: $REPORT"
((FAILED==0))
