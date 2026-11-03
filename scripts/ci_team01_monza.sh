#!/usr/bin/env bash
set -Eeuo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PKG="$ROOT/src/team_01/team01_race"
MAP="$ROOT/upstream/urrc_f1_ws/src/urrc_track_gazebo"
STATE="$ROOT/ci_artifacts"
mkdir -p "$STATE"
cd "$ROOT"
set +u
source /opt/ros/jazzy/setup.bash
set -u
export GZ_PARTITION="urrc_ci_${GITHUB_RUN_ID:-$$}"
export GZ_SIM_RESOURCE_PATH="$MAP/models${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
export LIBGL_ALWAYS_SOFTWARE=1
export GALLIUM_DRIVER=llvmpipe
export QT_QPA_PLATFORM=offscreen
export XDG_STATE_HOME="$STATE/state"
export TEAM01_VERIFY_LAP=0
[[ -f "$PKG/package.xml" ]] || { echo 'ERROR: v10 vehicle source not committed'; exit 2; }
[[ -f "$MAP/worlds/monza_race.sdf" ]] || { echo 'ERROR: submodule missing'; exit 2; }
python3 - "$MAP" <<'PY'
import pathlib,sys
p=pathlib.Path(sys.argv[1])
sys.path.insert(0,str(p))
from urrc_tracks.v10_assets import enhance_track
enhance_track(p,'monza')
PY
colcon build --base-paths "$PKG" --packages-select team01_race --symlink-install
set +u
source "$ROOT/install/setup.bash"
set -u
(cd "$PKG" && bash scripts/run_offline_tests.sh) | tee "$STATE/offline.log"
gz sdf -p "$PKG/models/team01_car/model.sdf" >/dev/null
PIDS=()
cleanup(){ for p in "${PIDS[@]}";do kill "$p" 2>/dev/null || true;done; wait 2>/dev/null || true; }
trap cleanup EXIT
# Gazebo real physics + RGB/Depth rendering, not a synthetic test.
gz sim -r -s --headless-rendering -v 3 "$MAP/worlds/monza_race.sdf" >"$STATE/gazebo.log" 2>&1 & PIDS+=("$!")
READY=0
for _ in $(seq 1 180);do
  if gz service -l 2>/dev/null | grep -qx '/world/monza/create';then READY=1;break;fi
  sleep 1
done
[[ "$READY" == 1 ]] || { echo 'ERROR: Gazebo world not ready';exit 2; }
# CI-only adjustment for the 1st grid if its wheel footprint overlaps the rail.
# Controller never reads this reference or corrected pose; record adjustment.
read SPX SPY SPYAW < <(python3 "$PKG/scripts/safe_grid_spawn.py" \
  --reference "$MAP/tracks/processed/monza_centerline.csv" --report "$STATE/spawn_clearance.json")
echo "[CI] corrected grid spawn: ($SPX,$SPY), yaw=$SPYAW" | tee "$STATE/spawn.log"
ros2 run ros_gz_sim create --world monza --file "$PKG/models/team01_car/model.sdf" --name team01 --x "$SPX" --y "$SPY" --z 1.243527 -Y "$SPYAW" | tee -a "$STATE/spawn.log"
ros2 run ros_gz_bridge parameter_bridge '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock' >"$STATE/clock.log" 2>&1 & PIDS+=("$!")
ros2 launch team01_race race.launch.py race_start_mode:=camera >"$STATE/controller.log" 2>&1 & PIDS+=("$!")
python3 "$PKG/scripts/verify_monza_lap.py" --reference "$MAP/tracks/processed/monza_centerline.csv" --report "$STATE/lap.json" --max-seconds 660 --motion-start-file "$STATE/lights_out.ready" >"$STATE/verifier.log" 2>&1 & PIDS+=("$!")
READY=0
for _ in $(seq 1 120);do
  COUNT=$(ros2 topic info /race/start/team01 2>/dev/null | sed -n 's/.*Subscription count: //p' | tail -1 || true)
  if [[ "$COUNT" =~ ^[0-9]+$ ]] && ((COUNT >= 2));then READY=1;break;fi
  sleep 1
done
[[ "$READY" == 1 ]] || { echo 'ERROR: controller signal subscribers missing';exit 2; }
for TOP in front_rgb road_rgb front_depth front_scan rear_scan;do
  if [[ "$TOP" == *scan ]];then FIELD=angle_min;else FIELD=height;fi
  echo "SENSOR $TOP"
  timeout 50s ros2 topic echo "/team01/$TOP" --once --field "$FIELD" --qos-reliability best_effort >/dev/null || { echo "ERROR: $TOP no rendered data";exit 2; }
done
sleep 2
ros2 topic pub --once /race/start/team01 std_msgs/msg/Bool '{data: true}' | tee "$STATE/permission.log"
python3 "$MAP/scripts/traffic_lights_v10.py" --track monza --skip-camera-check | tee "$STATE/lights.log"
grep -q 'LIGHTS OUT ' "$STATE/lights.log" || { echo '[CI] LIGHTS_OUT_MISSING'; exit 1; }
touch "$STATE/lights_out.ready"
# Deterministic final result: FAIL on timeout, sensor failure, offroad, no progress.
for CHECK in $(seq 1 660);do
  if ((CHECK >= 65)) && ! grep -q 'CAMERA LIGHTS OUT verified' "$STATE/controller.log";then
    echo '[CI] SIGNAL_GATE_STALL: camera gate never authorised departure';exit 1
  fi
  if [[ -f "$STATE/lap.json" ]];then
    S=$(python3 -c 'import json,sys;print(json.load(open(sys.argv[1]))["status"])' "$STATE/lap.json" || true)
    case "$S" in
      GEOMETRIC_LAP_PASS) echo '[CI] GEOMETRIC_LAP_PASS'; exit 0;;
      NO_ODOM|STALE_ODOM|DRIVE_STALL|LAP_TIMEOUT|OFF_TRACK|WRONG_DIRECTION_OR_REVERSAL|POSE_JUMP|INVALID_ODOM|START_OFF_TRACK) echo "[CI] LAP FAIL: $S"; exit 1;;
    esac
  fi
  sleep 1
done
echo '[CI] LAP TIMEOUT';exit 1
