#!/usr/bin/env bash
# QA-only wrapper: run unmodified participant simulator, record ROS2 bag and compare real odometry.
set -Eeuo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"
set +u
source /opt/ros/jazzy/setup.bash
set -u
LOCAL=0
if [[ "${1:-}" == "--local" ]]; then
  LOCAL=1
fi
MAP="$ROOT/upstream/urrc_f1_ws/src/urrc_track_gazebo"
REF="$MAP/tracks/processed/monza_centerline.csv"
if ((LOCAL));then
  STATE="$HOME/team01_qa/$(date +%Y%m%d_%H%M%S)"
  export XDG_STATE_HOME="$STATE/state"
  RUNNER="$ROOT/src/team_01/team01_race/scripts/run_local_monza.sh"
else
  STATE="$ROOT/ci_artifacts"
  RUNNER="$ROOT/scripts/ci_team01_monza.sh"
fi
mkdir -p "$STATE"
SIM='';BAG='';METRIC=''
clean(){
  set +e
  for p in "$BAG" "$METRIC" "$SIM"; do
    [[ -n "$p" ]] && kill -TERM "$p" 2>/dev/null || true
  done
  for ((i=0;i<60;i++));do
    live=0
    for p in "$BAG" "$METRIC" "$SIM"; do
      [[ -n "$p" ]] && kill -0 "$p" 2>/dev/null && live=1
    done
    [[ "$live" == 0 ]] && break
    sleep .25
  done
  for p in "$BAG" "$METRIC" "$SIM"; do
    if [[ -n "$p" ]];then
      kill -KILL "$p" 2>/dev/null || true
      wait "$p" 2>/dev/null || true
    fi
  done
  if [[ -d "$STATE/rosbag_core" && ! -f "$STATE/rosbag_core/metadata.yaml" ]];then
    timeout 35s ros2 bag reindex "$STATE/rosbag_core" --storage sqlite3 \
      >>"$STATE/rosbag.log" 2>&1 || true
  fi
  python3 "$ROOT/scripts/qa_team01_report.py" --state "$STATE" --reference "$REF" \
    >"$STATE/qa_reporter.log" 2>&1 || true
  echo "[QA] Artifacts: $STATE"
}
trap clean EXIT
echo "[QA] running real Gazebo via $RUNNER; artifacts=$STATE"
timeout --signal=TERM --kill-after=15s 1050s bash "$RUNNER" \
  >"$STATE/qa_runner.log" 2>&1 & SIM=$!
READY=0
for i in $(seq 1 180);do
  if ! kill -0 "$SIM" 2>/dev/null;then break;fi
  if [[ -f "$ROOT/install/setup.bash" ]];then
    set +u
    source "$ROOT/install/setup.bash"
    set -u
    if ros2 topic info /team01/gz_odom 2>/dev/null | grep -Eq 'Publisher count: [1-9][0-9]*';then
      READY=1;break
    fi
  fi
  sleep 1
done
if [[ "$READY" == 1 && "$LOCAL" == 1 ]];then
  local_spawn="$STATE/state/urrc_team01/qa/local_spawn_clearance.json"
  if [[ -f "$local_spawn" ]];then
    cp "$local_spawn" "$STATE/spawn_clearance.json"
  else
    echo "[QA] ERROR: local spawn report missing" >&2
    READY=0
  fi
fi
if [[ "$READY" == 1 ]];then
  ros2 bag record --storage sqlite3 --compression-mode file --compression-format zstd \
    -o "$STATE/rosbag_core" \
    /team01/gz_odom /team01/debug/path /team01/debug/start \
    /team01/debug/adapter /team01/ready_for_lights \
    /team01/speed /team01/steering /team01/front_scan /team01/rear_scan \
    /race/start/team01 >"$STATE/rosbag.log" 2>&1 & BAG=$!
  python3 "$ROOT/scripts/qa_team01_metrics.py" --reference "$REF" --out "$STATE" --spawn-report "$STATE/spawn_clearance.json" \
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
clean
trap - EXIT
if [[ "$READY" == 1 ]];then
  if [[ ! -f "$STATE/rosbag_core/metadata.yaml" ]];then
    echo "[QA] ERROR: rosbag metadata missing after real Gazebo run" >&2
    RESULT=1
  fi
  if [[ ! -f "$STATE/drive_metrics.json" ]];then
    echo "[QA] ERROR: independent trajectory metrics missing" >&2
    RESULT=1
  fi
fi
exit "$RESULT"
