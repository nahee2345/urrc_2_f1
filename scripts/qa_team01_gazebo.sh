#!/usr/bin/env bash
# QA-only wrapper: run unmodified participant simulator, record ROS2 bag and compare real odometry.
set -Eeuo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"
source /opt/ros/jazzy/setup.bash
LOCAL=0
[[ "${1:-}" == "--local" ]] && LOCAL=1
MAP="$ROOT/upstream/urrc_f1_ws/src/urrc_track_gazebo"
REF="$MAP/tracks/processed/monza_centerline.csv"
if ((LOCAL));then
  STATE="$HOME/team01_qa/$(date +%Y%m%d_%H%M%S)"
  RUNNER="$ROOT/src/team_01/team01_race/scripts/run_local_monza.sh"
else
  STATE="$ROOT/ci_artifacts"
  RUNNER="$ROOT/scripts/ci_team01_monza.sh"
fi
mkdir -p "$STATE"
SIM='';BAG='';METRIC=''
clean(){
  set +e
  [[ -n "$BAG" ]] && kill -INT "$BAG" 2>/dev/null
  [[ -n "$METRIC" ]] && kill -INT "$METRIC" 2>/dev/null
  [[ -n "$SIM" ]] && kill -INT "$SIM" 2>/dev/null
  sleep 3
  [[ -n "$BAG" ]] && wait "$BAG" 2>/dev/null
  [[ -n "$METRIC" ]] && wait "$METRIC" 2>/dev/null
  python3 "$ROOT/scripts/qa_team01_report.py" --state "$STATE" --reference "$REF" \
    >"$STATE/qa_reporter.log" 2>&1 || true
  echo "[QA] Artifacts: $STATE"
}
trap clean EXIT
echo "[QA] running real Gazebo via $RUNNER; artifacts=$STATE"
bash "$RUNNER" >"$STATE/qa_runner.log" 2>&1 & SIM=$!
READY=0
for i in $(seq 1 180);do
  if ! kill -0 "$SIM" 2>/dev/null;then break;fi
  if [[ -f "$ROOT/install/setup.bash" ]];then
    set +u
    source "$ROOT/install/setup.bash"
    set -u
    if ros2 topic info /team01/gz_odom 2>/dev/null | grep -q 'Publisher count: 1';then
      READY=1;break
    fi
  fi
  sleep 1
done
if [[ "$READY" == 1 ]];then
  ros2 bag record --storage sqlite3 --compression-mode file --compression-format zstd \
    -o "$STATE/rosbag_core" \
    /team01/gz_odom /team01/debug/path /team01/debug/start \
    /team01/debug/adapter /team01/ready_for_lights \
    /team01/speed /team01/steering /team01/front_scan /team01/rear_scan \
    /race/start/team01 >"$STATE/rosbag.log" 2>&1 & BAG=$!
  python3 "$ROOT/scripts/qa_team01_metrics.py" --reference "$REF" --out "$STATE" \
    >"$STATE/qa_metrics.log" 2>&1 & METRIC=$!
  echo "[QA] rosbag and independent trajectory metrics started"
else
  echo "[QA] Odometry not available before runner stopped/timeout"
fi
set +e
wait "$SIM"
RESULT=$?
set -e
echo "[QA] Gazebo runner exit=$RESULT"
tail -n 25 "$STATE/qa_runner.log" || true
exit "$RESULT"
