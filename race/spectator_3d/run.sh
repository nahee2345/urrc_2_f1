#!/usr/bin/env bash
set -euo pipefail
if ! ( : </dev/tty ) 2>/dev/null; then echo "터미널에서 직접 bash spectator/run.sh 를 실행하십시오." >&2; exit 2; fi
exec 3</dev/tty
ROOT=$(cd "$(dirname "$0")/.." && pwd); cd "$ROOT"
set +u; source /opt/ros/jazzy/setup.bash; set -u
set +u; source install/setup.bash; set -u
export URRC_SPECTATOR_STAGE="${1:-test}"
case "$URRC_SPECTATOR_STAGE" in test|qualifying|semi_a|semi_b|final) ;; *) echo '사용법: bash spectator/run.sh [test|qualifying|semi_a|semi_b|final]'; exit 2;; esac
export GZ_PARTITION="urrc_spectator_monza_$(id -u)" GZ_IP=127.0.0.1
export GZ_SIM_RESOURCE_PATH="$ROOT/src/urrc_track_gazebo/models"
export GZ_SIM_SYSTEM_PLUGIN_PATH="$ROOT/install/lib${GZ_SIM_SYSTEM_PLUGIN_PATH:+:$GZ_SIM_SYSTEM_PLUGIN_PATH}"
PKG="$ROOT/src/urrc_track_gazebo"; mkdir -p logs
if gz service -l | grep -Fxq /world/monza/create; then echo '기존 테스트 월드 종료 후 실행'; exit 2; fi
: > logs/lights.log  # clear previous race before spectator starts
PIDS=(); MDNS_PID=""; cleanup(){ if [[ -n "$MDNS_PID" ]]; then kill -TERM "$MDNS_PID" 2>/dev/null || true; wait "$MDNS_PID" 2>/dev/null || true; fi; for p in "${PIDS[@]}"; do kill -INT "$p" 2>/dev/null || true; done; }; trap cleanup EXIT; trap 'exit 130' INT TERM
/usr/bin/python3 spectator/server.py </dev/null > logs/spectator.log 2>&1 & PIDS+=("$!")
for i in $(seq 1 50); do
  if grep -q "관전 주소:" logs/spectator.log; then break; fi
  kill -0 "${PIDS[0]}" 2>/dev/null || { cat logs/spectator.log; exit 3; }
  sleep .2
done
grep "관전 주소:" logs/spectator.log || { cat logs/spectator.log; exit 3; }
# Publish a LAN-only alias for this run. Failure never blocks the race.
if command -v avahi-publish >/dev/null && systemctl is-active --quiet avahi-daemon; then
  bash spectator/fixed_address.sh </dev/null > logs/mdns.log 2>&1 & MDNS_PID=$!
  for i in $(seq 1 20); do
    if grep -qi 'established' logs/mdns.log; then break; fi
    kill -0 "$MDNS_PID" 2>/dev/null || break
    sleep .2
  done
  if kill -0 "$MDNS_PID" 2>/dev/null && grep -qi 'established' logs/mdns.log; then
    grep '고정 주소' logs/spectator.log
  else
    echo '[관전] urrc2.local 등록 미확인: 위 IP 주소를 사용하십시오. logs/mdns.log 확인.'
  fi
else
  echo '[관전] Avahi 미준비: 위 IP 주소 사용. 설치/업데이트 스크립트에서 Avahi를 준비하십시오.'
fi

gz sim -r -v 3 --gui-config "$PKG/worlds/monza_grid.gui.config" "$PKG/worlds/monza_race.sdf" > logs/gazebo.log 2>&1 & PIDS+=("$!")
for i in $(seq 1 90); do if gz service -l | grep -Fxq /world/monza/create; then break; fi; sleep 1; done
gz service -l | grep -Fxq /world/monza/create || { echo '월드 준비 실패: logs/gazebo.log'; exit 3; }
read -r X Y Z YAW < <(/usr/bin/python3 - <<'PY'
import json
j=json.load(open('src/urrc_track_gazebo/tracks/processed/monza.json'));g=next(x for x in j['grid'] if x['position']==1);print(g['x'],g['y'],g['road_z']+.04,g['yaw'])
PY
)
echo "TEAM_08 스폰 중..."
timeout 60 ros2 run ros_gz_sim create --world monza --file "$ROOT/src/team/team_08/src/team_08/models/team08_car/model.sdf" --name team_08 --x "$X" --y "$Y" --z "$Z" -Y "$YAW" </dev/null > logs/spawn.log 2>&1 || { cat logs/spawn.log; exit 4; }
cat logs/spawn.log
echo 'Gazebo에서 차량과 맵을 확인한 뒤 이 터미널을 클릭하십시오.'
while true; do
  printf 'READY 입력 후 Enter: ' >/dev/tty
  IFS= read -r READY <&3 || { echo '터미널 입력 종료'; exit 4; }
  [[ "$READY" == READY ]] && break
  echo 'READY를 정확하게 입력하십시오.'
done
echo "[SIGNAL] 원본 5등 신호 실행 (브라우저 관전 테스트: 외부 관전 카메라 준비 조건만 제외)"
/usr/bin/python3 -u "$PKG/scripts/traffic_lights_v10.py" --track monza --timeout 20 --skip-camera-check 2>&1 | tee logs/lights.log
grep -q '^LIGHTS OUT .*model_remove=single_request' logs/lights.log || { echo '소등 확인 실패: 출발 금지'; exit 5; }
# Test operator signal, same native Gazebo topic as the original TEAM08 bridge.
echo "[START] 전체 소등 확인 완료: TEAM_08 출발 신호 발행"
for i in 1 2 3; do gz topic -t /team08/replay_go_08 -m gz.msgs.Boolean -p 'data: true'; sleep .1; done
echo '관전 테스트 진행 중. Ctrl+C로 종료. 3랩은 HUD 잠정 측정값으로 확인.'
wait "${PIDS[1]}"
