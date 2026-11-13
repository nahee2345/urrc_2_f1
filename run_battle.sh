#!/usr/bin/env bash
set -Eeuo pipefail
# Run through one public command: bash run.sh battle monza
TRACK="${1:-monza}"
LANE="${2:-1}"
case "$LANE" in 1|2|3) ;; *) echo "Lane must be 1, 2 or 3" >&2;exit 2;; esac
case "$TRACK" in monza|spa|monaco|cota|hungaroring|baku) ;; *)
  echo "Valid tracks: monza spa monaco cota hungaroring baku" >&2;exit 2 ;;
esac
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT"
PKG="$ROOT/src/urrc_track_gazebo"
LOG="$ROOT/log/battle/${TRACK}_lane${LANE}/$(date +%Y%m%d_%H%M%S)"
mkdir -p "$LOG"
export GZ_PARTITION="urrc_battle_$(id -u)_$(date +%s)"
export PYTHONUNBUFFERED=1
export GZ_SIM_RESOURCE_PATH="$PKG/models${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
if ! command -v gz >/dev/null || ! command -v cmake >/dev/null;then
  echo "Gazebo Harmonic and cmake required" >&2;exit 3
fi
set +u
source /opt/ros/jazzy/setup.bash
set -u
if ! ros2 pkg prefix ros_gz_sim >/dev/null 2>&1;then
  echo "ROS2 Jazzy ros_gz_sim required" >&2;exit 3
fi
echo "[1/6] Verify six official maps and signal logic"
if ! /usr/bin/python3 import_6_tracks.py \
  --official "${URRC_OFFICIAL_WS:-$HOME/urrc_f1_ws}" --verify-only \
  >"$LOG/import.log" 2>&1;then
  /usr/bin/python3 import_6_tracks.py \
    --official "${URRC_OFFICIAL_WS:-$HOME/urrc_f1_ws}" \
    >>"$LOG/import.log" 2>&1 || { tail -60 "$LOG/import.log";exit 4; }
fi
echo "[2/6] Generate three coordinate lanes"
/usr/bin/python3 generate_routes.py >"$LOG/routes.log" 2>&1 || {
  tail -60 "$LOG/routes.log";exit 4;
}
echo "[3/6] Compile Gazebo three-car race scenario"
cmake -S "$ROOT/kinematic" -B "$ROOT/kinematic/build" \
  -DCMAKE_BUILD_TYPE=Release >"$LOG/build.log" 2>&1 || {
    tail -80 "$LOG/build.log";exit 5;
  }
cmake --build "$ROOT/kinematic/build" --parallel 2 >>"$LOG/build.log" 2>&1 || {
  tail -80 "$LOG/build.log";exit 5;
}
PLUGIN="$ROOT/kinematic/build/libteam11_battle_replay.so"
[[ -f "$PLUGIN" ]] || { echo "Missing battle plugin library" >&2;exit 5; }
export GZ_SIM_SYSTEM_PLUGIN_PATH="$(dirname "$PLUGIN")${GZ_SIM_SYSTEM_PLUGIN_PATH:+:$GZ_SIM_SYSTEM_PLUGIN_PATH}"
echo "[4/6] Prepare 0.30m walls and grid 1 / grid 10 models"
OUT="$LOG/prepared"
/usr/bin/python3 "$ROOT/kinematic/prepare_battle.py" \
  --track "$TRACK" --lane "$LANE" --plugin "$PLUGIN" --output "$OUT" >"$LOG/prepare.log" 2>&1 || {
    tail -80 "$LOG/prepare.log";exit 6;
}
export GZ_SIM_RESOURCE_PATH="$OUT/resources/urrc_track_gazebo/models:$GZ_SIM_RESOURCE_PATH"
WORLD="$OUT/resources/urrc_track_gazebo/worlds/${TRACK}_fixed.sdf"
LIGHTS="$OUT/resources/urrc_track_gazebo/scripts/traffic_lights_v10.py"
TOPIC="/urrc/$TRACK/battle/start"
WORLD_PID=""
cleanup() {
  trap - EXIT INT TERM
  if [[ -n "$WORLD_PID" ]];then
    kill -INT "$WORLD_PID" 2>/dev/null || true
    sleep .7
    kill -TERM "$WORLD_PID" 2>/dev/null || true
  fi
}
trap cleanup EXIT
trap 'exit 130' INT TERM
echo "[5/6] Gazebo start and three-vehicle spawn"
gz sim -r -v 3 --gui-config "$PKG/worlds/${TRACK}_grid.gui.config" "$WORLD" \
  >"$LOG/gazebo.log" 2>&1 &
WORLD_PID=$!
READY=0
for _ in $(seq 1 100);do
  if ! kill -0 "$WORLD_PID" 2>/dev/null;then
    tail -60 "$LOG/gazebo.log";exit 7
  fi
  if gz topic -l 2>/dev/null | grep -Fxq "$TOPIC/ready";then READY=1;break;fi
  sleep 1
done
if [[ "$READY" -ne 1 ]];then
  echo "FAIL: battle plugin not ready" >&2
  tail -70 "$LOG/gazebo.log";exit 7
fi
for VEHICLE in team_11 practice_s_09 practice_rear_11;do
  read -r MODEL X Y Z YAW < <(/usr/bin/python3 - "$OUT/battle_manifest.json" "$VEHICLE" <<'PY'
import json,sys
data=json.load(open(sys.argv[1]))
who=sys.argv[2]
model=data["vehicle_file"] if who=="team_11" else data["lead_model"] if who=="practice_s_09" else data["rear_model"]
print(model,*data["start_positions"][who])
PY
)
  ros2 run ros_gz_sim create --world "$TRACK" --file "$MODEL" --name "$VEHICLE" \
    --x "$X" --y "$Y" --z "$Z" -Y "$YAW" \
    >"$LOG/spawn_$VEHICLE.log" 2>&1 || {
      tail -70 "$LOG/spawn_$VEHICLE.log";exit 8;
    }
  echo "SPAWN: $VEHICLE"
done
echo "[6/6] Original five-lamp all-off gate"
if ! timeout -s INT -k 3 110 /usr/bin/python3 "$LIGHTS" \
   --track "$TRACK" --timeout 80 >"$LOG/lights.log" 2>&1;then
  echo "FAIL: signal sequence; battle remains disarmed" >&2
  tail -60 "$LOG/lights.log";exit 9
fi
if ! grep -q '^LIGHTS OUT .*model_remove=single_request' "$LOG/lights.log";then
  echo "FAIL: all-off signal service not confirmed" >&2;exit 9
fi
for _ in 1 2 3;do
  gz topic -t "$TOPIC" -m gz.msgs.Boolean -p 'data: true' \
    >"$LOG/start.log" 2>&1 || {
      tail -40 "$LOG/start.log";exit 10;
    }
  sleep .1
done
echo "BATTLE START: $TRACK lane $LANE team11 10m/s, lead S 9m/s/2sec, grid10 rear 11m/s"
echo "Trajectory-aligned headings; real wheel steering is disabled"
echo "Log: $LOG/prepared/battle_state.csv"
STARTED=$(date +%s)
RESULT=""
while kill -0 "$WORLD_PID" 2>/dev/null;do
  if grep -q '\[BATTLE\] RESULT ' "$LOG/gazebo.log";then
    RESULT=$(grep '\[BATTLE\] RESULT ' "$LOG/gazebo.log" | tail -1)
    echo "$RESULT"
    break
  fi
  if (( $(date +%s)-STARTED>360 ));then
    echo "FAIL: no battle result within 360 seconds" >&2
    tail -55 "$LOG/gazebo.log";exit 11
  fi
  sleep 1
done
[[ -n "$RESULT" ]] || { echo "FAIL: Gazebo closed before battle result" >&2;exit 11; }
grep '\[BATTLE\]' "$LOG/gazebo.log" | tail -35
if [[ "$RESULT" == *"RESULT PASS_BATTLE"* ]];then
  echo "PASS BATTLE: $TRACK lane $LANE results $LOG"
else
  echo "FAIL BATTLE: $TRACK lane $LANE results $LOG" >&2
  exit 12
fi
