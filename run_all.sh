#!/usr/bin/env bash
set -Eeuo pipefail
# Run 18 independent 1-lap tests: lane 1 six tracks, lane 2 six tracks,
# lane 3 six tracks. Each test starts from official grid 1 and closes after
# the replay reports PASS_LAP. Continue to the next track on failure.
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT"
SPEED="${1:-10}"
if ! [[ "$SPEED" =~ ^[0-9]+([.][0-9]+)?$ ]]; then
  echo "Usage: bash run_all.sh [speed_mps, default 10]" >&2
  exit 2
fi
RUN_ID=$(date +%Y%m%d_%H%M%S)
LOGDIR="$ROOT/log/batch/$RUN_ID"
mkdir -p "$LOGDIR"
SUMMARY="$LOGDIR/summary.csv"
printf 'lane,track,speed_mps,result,exit_code,log_file\n' >"$SUMMARY"
TRACKS=(monza spa monaco cota hungaroring baku)
PASSED=0
FAILED=0
echo "Starting 18 fixed-route tests (1 -> 2 -> 3, six tracks each)"
echo "Results: $SUMMARY"
for LANE in 1 2 3; do
  for TRACK in "${TRACKS[@]}"; do
    NUMBER=$((PASSED + FAILED + 1))
    LABEL="${LANE}_${TRACK}"
    CASELOG="$LOGDIR/$LABEL.log"
    echo "[$NUMBER/18] lane $LANE / $TRACK / $SPEED m/s"
    # Partition per case ensures unrelated/stale Gazebo worlds do not attach
    # to the current run. Enforce an overall wall-time limit per run.
    if URRC_BATCH=1 \
       URRC_REPLAY_TIMEOUT_S="${URRC_REPLAY_TIMEOUT_S:-300}" \
       GZ_PARTITION="urrc_batch_${UID}_${RUN_ID}_${LANE}_${TRACK}" \
       timeout -s TERM -k 15 "${URRC_CASE_TIMEOUT_S:-480}" \
         bash "$ROOT/run.sh" "$TRACK" "$LANE" "$SPEED" \
         >"$CASELOG" 2>&1; then
      RC=0
      if grep -q "^PASS: $TRACK lane $LANE " "$CASELOG"; then
        STATUS="PASS"
        PASSED=$((PASSED+1))
      else
        STATUS="FAIL_UNCONFIRMED"
        FAILED=$((FAILED+1))
      fi
    else
      RC=$?
      STATUS="FAIL"
      FAILED=$((FAILED+1))
    fi
    printf '%s,%s,%s,%s,%s,%s\n' \
      "$LANE" "$TRACK" "$SPEED" "$STATUS" "$RC" "$CASELOG" >>"$SUMMARY"
    echo "[$NUMBER/18] $STATUS (code $RC); $CASELOG"
    # Allow the previous Gazebo process to relinquish resources.
    sleep 2
  done
done
echo "Finished: PASS=$PASSED FAIL=$FAILED"
echo "Summary: $SUMMARY"
if (( FAILED>0 )); then exit 1; fi
