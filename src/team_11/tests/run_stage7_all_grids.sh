#!/usr/bin/env bash
set -Eeuo pipefail
# Sequential official Gazebo + onboard-camera start test across all 11 grids.
TRACK="${1:-monza}"
case "$TRACK" in monza|spa|monaco|cota|hungaroring|baku) ;; *) echo "Invalid track" >&2; exit 2;; esac
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
LOGROOT="$ROOT/log/team11_stage7_all_grids/$TRACK"
mkdir -p "$LOGROOT"
SUMMARY="$LOGROOT/summary.csv"
printf 'track,grid,result,detail\n' >"$SUMMARY"
passed=0
failed=0
for grid in $(seq 1 11); do
  printf '\n========== %s: grid %02d / 11 ==========\n' "$TRACK" "$grid"
  gridlog="$LOGROOT/grid_$(printf '%02d' "$grid")"
  mkdir -p "$gridlog"
  partition="team11_stage7_${TRACK}_${grid}_$$"
  status=0
  env GZ_PARTITION="$partition" \
      TEAM11_STAGE7_LOG="$gridlog" \
      TEAM11_STAGE7_AUTO_EXIT=1 \
      TEAM11_STAGE7_CAPTURE_IMAGES=0 \
      TEAM11_STAGE7_SKIP_BUILD="$([[ "$grid" == "1" ]] && echo 0 || echo 1)" \
      timeout -s INT -k 10s 160s \
      bash "$ROOT/src/team_11/tests/run_stage7.sh" "$TRACK" "$grid" \
      >"$gridlog/console.log" 2>&1 || status=$?
  if [[ "$status" -eq 0 && -f "$gridlog/result.txt" &&
        "$(cat "$gridlog/result.txt")" == "PASS" ]]; then
    printf '%s,%s,PASS,red_on_off_then_forward\n' "$TRACK" "$grid" >>"$SUMMARY"
    echo "[GRID $grid] PASS"
    passed=$((passed+1))
  else
    printf '%s,%s,FAIL,exit_%s\n' "$TRACK" "$grid" "$status" >>"$SUMMARY"
    echo "[GRID $grid] FAIL (exit $status); $gridlog"
    tail -12 "$gridlog/console.log" || true
    failed=$((failed+1))
  fi
  sleep 2
done
printf '\n[TEAM11] %s: PASS=%s FAIL=%s (total 11)\n' "$TRACK" "$passed" "$failed"
echo "[TEAM11] Summary: $SUMMARY"
cat "$SUMMARY"
if [[ "$failed" -ne 0 ]]; then exit 1; fi
