#!/usr/bin/env bash
# Developer-only: world, car, sensors, then original sequential start lights.
# No changes to the competition world or signal source files.
set -euo pipefail
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$HERE/../../../.." && pwd)"
MODEL="$HERE/../models/team01_car/model.sdf"
# Use the same official submodule world as CI, with old layout as fallback.
MAP_SOURCE="${TEAM01_MAP_DIR:-$ROOT/upstream/urrc_f1_ws/src/urrc_track_gazebo}"
if [[ ! -d "$MAP_SOURCE" && -d "$ROOT/src/urrc_track_gazebo" ]]; then
  MAP_SOURCE="$ROOT/src/urrc_track_gazebo"
fi
RUNTIME_BASE="${XDG_STATE_HOME:-$HOME/.local/state}/urrc_team01"
mkdir -p "$RUNTIME_BASE"
RUNTIME_ROOT="$(mktemp -d "$RUNTIME_BASE/monza_run.XXXXXXXX")"
MAP_DIR="$RUNTIME_ROOT/src/urrc_track_gazebo"
mkdir -p "$MAP_DIR"
# Never write to the existing upstream Monza; operate on a throwaway copy.
if [[ "$MAP_SOURCE" == "$ROOT/upstream/urrc_f1_ws/src/urrc_track_gazebo" ]]; then
  # Repair only original Monza assets previously changed by the old launcher.
  bash "$ROOT/scripts/team01_restore_original_monza.sh"
  git -C "$ROOT/upstream/urrc_f1_ws" archive HEAD src/urrc_track_gazebo | tar -xf - -C "$RUNTIME_ROOT"
else
  cp -a "$MAP_SOURCE/." "$MAP_DIR/"
fi

set +u
source /opt/ros/jazzy/setup.bash
set -u
export GZ_PARTITION="${GZ_PARTITION:-urrc_practice_$(id -u)}"
export GZ_SIM_RESOURCE_PATH="$MAP_DIR/models${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
echo "[TEAM01] Gazebo partition: $GZ_PARTITION"

cd "$ROOT"
colcon build --base-paths src/team_01 --packages-select team01_race --symlink-install
set +u
source "$ROOT/install/setup.bash"
set -u

if ! gz sdf -p "$MODEL" >/dev/null; then
  echo "[TEAM01] 차량 SDF 검사 실패" >&2
  exit 2
fi
# Generate missing textured track assets on a clean checkout, exactly as CI.
if [[ ! -f "$MAP_DIR/worlds/monza_signal_active.sdf" ||
      ! -f "$MAP_DIR/models/urrc_monza/meshes/road_textured.obj" ]]; then
  python3 - "$MAP_DIR" <<'PY'
from pathlib import Path
import sys
track=Path(sys.argv[1])
sys.path.insert(0,str(track))
from urrc_tracks.v10_assets import enhance_track
enhance_track(track,'monza')
PY
fi
for file in "$MAP_DIR/worlds/monza_race.sdf" \
            "$MAP_DIR/worlds/monza_signal_active.sdf" \
            "$MAP_DIR/worlds/monza_grid.gui.config"; do
  if [[ ! -f "$file" ]]; then
    echo "[TEAM01] 지도 필수 파일 없음: $file" >&2
    echo "기존 URRC run_v10.sh를 한 번 실행해 맵 자산을 생성하십시오." >&2
    exit 2
  fi
done
if gz service -l 2>/dev/null | grep -qx '/world/monza/create'; then
  echo '[TEAM01] 같은 GZ_PARTITION에서 Monza가 이미 실행 중입니다. 기존 Gazebo를 종료하세요.' >&2
  exit 2
fi

WORLD_PID=''; CONTROL_PID=''; CLOCK_PID=''; LIGHTS_PID=''; VERIFY_PID=''
cleanup() {
  for pid in "$LIGHTS_PID" "$VERIFY_PID" "$CONTROL_PID" "$CLOCK_PID" "$WORLD_PID"; do
    if [[ -n "$pid" ]]; then kill "$pid" 2>/dev/null || true; fi
  done
  wait 2>/dev/null || true
  rm -rf "$RUNTIME_ROOT"
}
trap cleanup EXIT

# Important: do NOT use run_v10.sh here: it starts its automatic traffic-light
# sequence before this participant's camera subscriber is initialized.
gz sim -r -v 3 --gui-config "$MAP_DIR/worlds/monza_grid.gui.config" \
  "$MAP_DIR/worlds/monza_race.sdf" & WORLD_PID=$!
READY=0
for i in $(seq 1 120); do
  if gz service -l 2>/dev/null | grep -qx '/world/monza/create'; then
    READY=1;break
  fi
  if ! kill -0 "$WORLD_PID" 2>/dev/null; then break; fi
  sleep .5
done
if [[ "$READY" != 1 ]]; then
  echo '[TEAM01] Monza 생성 서비스 대기 실패' >&2;exit 2
fi

echo '[TEAM01] 1번 그리드에 노란색 team01 생성...'
REF="$MAP_DIR/tracks/processed/monza_centerline.csv"
SPAWN_REPORT="${XDG_STATE_HOME:-$HOME/.local/state}/urrc_team01/qa/local_spawn_clearance.json"
read SPX SPY SPYAW < <(python3 "$HERE/safe_grid_spawn.py" \
  --reference "$REF" --report "$SPAWN_REPORT")
echo "[TEAM01] start pose x=$SPX y=$SPY yaw=$SPYAW (inspection: $SPAWN_REPORT)"
ros2 run ros_gz_sim create --world monza --file "$MODEL" --name team01 \
  --x "$SPX" --y "$SPY" --z 1.243527 -Y "$SPYAW"
FOUND=0
for i in $(seq 1 20); do
  MODELS=$(timeout 3s gz model --list 2>&1 || true)
  if printf '%s\n' "$MODELS" | grep -Eq '(^|[[:space:]])team01([[:space:]]|$)'; then
    FOUND=1;break
  fi
  sleep .5
done
if [[ "$FOUND" != 1 ]]; then
  echo '[TEAM01] 생성 요청 후 모델 목록에서 team01 확인 실패' >&2;exit 2
fi

ros2 run ros_gz_bridge parameter_bridge \
  '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock' & CLOCK_PID=$!
ros2 launch team01_race race.launch.py race_start_mode:=camera & CONTROL_PID=$!

# QA process uses the track's reference centerline but NEVER feeds it to the
# vehicle controller. It listens only to Gazebo odometry and writes its own
# geometric lap verdict. This does not bypass the visual race start signal.
if [[ "${TEAM01_VERIFY_LAP:-1}" == '1' ]]; then
  REF="$MAP_DIR/tracks/processed/monza_centerline.csv"
  REPORT="${XDG_STATE_HOME:-$HOME/.local/state}/urrc_team01/qa/monza_lap_v10.json"
  [[ -f "$REF" ]] || { echo "[LAP QA] reference missing: $REF" >&2;exit 2; }
  echo "[LAP QA] Verifier enabled, report=$REPORT"
  python3 "$HERE/verify_monza_lap.py" \
    --reference "$REF" --report "$REPORT" --max-seconds 1800 & VERIFY_PID=$!
fi

# The two independently gated nodes must be subscribed before the signal.
CONNECTED=0
for i in $(seq 1 80); do
  COUNT=$(ros2 topic info /race/start/team01 2>/dev/null | \
    sed -n 's/.*Subscription count: //p' | tail -1 || true)
  if [[ "${COUNT:-0}" =~ ^[0-9]+$ ]] && (( COUNT >= 2 )); then
    CONNECTED=1;break
  fi
  if ! kill -0 "$CONTROL_PID" 2>/dev/null; then
    echo '[TEAM01] 컨트롤러 종료됨' >&2;exit 2
  fi
  sleep .5
done
if [[ "$CONNECTED" != 1 ]]; then
  echo '[TEAM01] 출발 신호 구독자 2개 대기 실패' >&2;exit 2
fi

# Check real sensor frames, not merely the existence of their ROS topics.
for topic in front_rgb road_rgb front_depth front_scan rear_scan; do
  if [[ "$topic" == *scan ]]; then field=angle_min; else field=height; fi
  echo "[TEAM01] 센서 데이터 확인: /team01/$topic"
  if ! timeout 15s ros2 topic echo "/team01/$topic" --once --field "$field" \
      --qos-reliability best_effort >/dev/null; then
    if [[ "$topic" == 'front_rgb' ]]; then
      echo '[TEAM01] 신호등 RGB 미수신: 스폰 7초 후 무빨간불 대체출발 허용' >&2
      continue
    fi
    echo "[TEAM01] $topic 실제 데이터 수신 실패. 출발 금지." >&2
    exit 2
  fi
done

# Both the vision background and gate intensity baseline MUST be ready.
# Starting the lamps before that calibration can make the actual reds part of
# the "dark" reference, preventing any true light-out detection.
if python3 "$HERE/wait_camera_ready.py" --timeout 6 \
  --out "${XDG_STATE_HOME:-$HOME/.local/state}/urrc_team01/qa/start_readiness.json"; then
  echo '[TEAM01] Camera calibrated; run original red-light sequence.'
  ros2 topic pub --once /race/start/team01 std_msgs/msg/Bool '{data: true}'
  python3 "$MAP_DIR/scripts/traffic_lights_v10.py" --track monza & LIGHTS_PID=$!
  wait "$LIGHTS_PID"
  LIGHTS_PID=''
else
  echo '[TEAM01] No calibrated signal camera. Controller uses no-red 7s spawn gate.' >&2
fi
echo '[TEAM01] Signal camera or 7-second no-red timer authorizes start; lidar/depth checks remain.'
wait "$CONTROL_PID"

