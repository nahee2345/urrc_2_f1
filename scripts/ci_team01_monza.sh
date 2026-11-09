#!/usr/bin/env bash
set -Eeuo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PKG="$ROOT/src/team_01/team01_race"
SOURCE_MAP="$ROOT/upstream/urrc_f1_ws/src/urrc_track_gazebo"
STATE="$ROOT/ci_artifacts"
RUNTIME_ROOT="$STATE/monza_runtime"
MAP="$RUNTIME_ROOT/src/urrc_track_gazebo"
rm -rf "$RUNTIME_ROOT"
mkdir -p "$RUNTIME_ROOT"
# Read the immutable submodule tree, not its potentially dirty working copy.
git -C "$ROOT/upstream/urrc_f1_ws" archive HEAD src/urrc_track_gazebo | tar -xf - -C "$RUNTIME_ROOT"
rm -f "$STATE/start_authorized.ready" "$STATE/lights_out.ready"
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
# Augment only the disposable map copy.
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
# QA-only separation: a regression failure must not suppress independent physical
# Gazebo evidence. The dedicated QA workflow still fails overall if either stage fails.
if [[ "${TEAM01_QA_GAZEBO_ONLY:-0}" == "1" ]]; then
  echo '[QA] Offline test gate separated; see independent QA offline.log' | tee "$STATE/offline.log"
else
  (cd "$PKG" && bash scripts/run_offline_tests.sh) | tee "$STATE/offline.log"
fi
gz sdf -p "$PKG/models/team01_car/model.sdf" >/dev/null
PIDS=()
cleanup(){
  set +e
  for p in "${PIDS[@]}"; do kill -TERM "$p" 2>/dev/null || true; done
  for ((i=0;i<40;i++)); do
    still=0
    for p in "${PIDS[@]}"; do kill -0 "$p" 2>/dev/null && still=1; done
    [[ "$still" == 0 ]] && break
    sleep .25
  done
  for p in "${PIDS[@]}"; do kill -KILL "$p" 2>/dev/null || true; done
  for p in "${PIDS[@]}"; do wait "$p" 2>/dev/null || true; done
}
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
export TEAM01_SPAWN_MONOTONIC="$(python3 -c 'import time;print(time.monotonic())')"
ros2 run ros_gz_bridge parameter_bridge '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock' >"$STATE/clock.log" 2>&1 & PIDS+=("$!")
if [[ "${TEAM01_NO_SIGNAL_MODE:-0}" == 1 ]];then START_MODE=no_signal;else START_MODE=camera;fi
ros2 launch team01_race race.launch.py race_start_mode:="$START_MODE" >"$STATE/controller.log" 2>&1 & PIDS+=("$!")
# Retain independent gate and adapter evidence even if the Gazebo vehicle
# fails to move. They distinguish recognition, authorization, commanding,
# and drivetrain/physics stalls.
timeout 780s ros2 topic echo /team01/debug/start --field data >"$STATE/start_status.log" 2>&1 & PIDS+=("$!")
timeout 780s ros2 topic echo /team01/debug/adapter --field data >"$STATE/adapter_status.log" 2>&1 & PIDS+=("$!")
python3 "$PKG/scripts/verify_monza_lap.py" --reference "$MAP/tracks/processed/monza_centerline.csv" --report "$STATE/lap.json" --max-seconds 660 --motion-start-file "$STATE/start_authorized.ready" \
  --spawn-report "$STATE/spawn_clearance.json" >"$STATE/verifier.log" 2>&1 & PIDS+=("$!")
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
  if ! timeout 50s ros2 topic echo "/team01/$TOP" --once --field "$FIELD" --qos-reliability best_effort >/dev/null;then
    if [[ "$TOP" == front_rgb ]];then
      echo '[CI] Signal camera data missing; normal race remains fail-closed'
      continue
    fi
    echo "ERROR: $TOP no rendered data";exit 2
  fi
done
# Red-camera path when calibration works; otherwise a separately labelled
# no-red timeout path. Never claim a camera PASS from timeout authorization.
if [[ "$START_MODE" == no_signal ]];then
  echo '[CI] EXPLICIT_NO_SIGNAL; no traffic lights will be operated' | tee "$STATE/start_fallback.log"
  READY_FALLBACK=0
  for _ in $(seq 1 70);do
    if grep -q 'NO_RED_7S' "$STATE/start_status.log" 2>/dev/null;then
      READY_FALLBACK=1;break
    fi
    sleep 1
  done
  [[ "$READY_FALLBACK" == 1 ]] || { echo '[CI] NO_RED_7S_TIMEOUT';exit 1; }
  touch "$STATE/start_authorized.ready"
elif python3 "$PKG/scripts/wait_camera_ready.py" --timeout "${TEAM01_CAMERA_READY_TIMEOUT_S:-75}" \
  --out "$STATE/camera_ready_diag.json" | tee "$STATE/camera_ready.log";then
  python3 "$ROOT/scripts/qa_team01_permission.py" --timeout 35 \
    --out "$STATE/permission_validation.json" | tee "$STATE/permission.log"
  python3 "$MAP/scripts/traffic_lights_v10.py" --track monza --skip-camera-check | tee "$STATE/lights.log"
  grep -q 'LIGHTS OUT ' "$STATE/lights.log" || { echo '[CI] LIGHTS_OUT_MISSING';exit 1; }
  touch "$STATE/start_authorized.ready"
else
  echo '[CI] CAMERA_NOT_READY; traffic-light race remains stopped' | tee "$STATE/start_failure.txt"
  echo '[CI] Inspect controller.log, bridge topics, and /team01/debug/start; no fallback in camera mode.' | tee -a "$STATE/start_failure.txt"
  tail -n 90 "$STATE/controller.log" 2>/dev/null | tee "$STATE/controller_start_tail.log" || true
  ros2 node list >"$STATE/start_nodes.txt" 2>&1 || true
  ros2 topic info /team01/ready_for_lights >"$STATE/start_ready_topic.txt" 2>&1 || true
  exit 1
fi
# Deterministic final result: FAIL on timeout, sensor failure, offroad, no progress.
for CHECK in $(seq 1 660);do
  if ((CHECK >= 65)) && ! grep -q 'CAMERA LIGHTS OUT verified' "$STATE/controller.log" && ! grep -q 'NO_RED_7S' "$STATE/start_status.log" 2>/dev/null;then
    echo '[CI] SIGNAL_GATE_STALL: camera gate never authorised departure';exit 1
  fi
  if [[ -f "$STATE/lap.json" ]];then
    S=$(python3 -c 'import json,sys;print(json.load(open(sys.argv[1]))["status"])' "$STATE/lap.json" || true)
    case "$S" in
      GEOMETRIC_LAP_PASS) echo '[CI] GEOMETRIC_LAP_PASS'; exit 0;;
      NO_ODOM|STALE_ODOM|DRIVE_STALL|LAP_TIMEOUT|OFF_TRACK|WRONG_DIRECTION_OR_REVERSAL|POSE_JUMP|INVALID_ODOM|START_OFF_TRACK|ODOM_FRAME_UNVERIFIED) echo "[CI] LAP FAIL: $S"; exit 1;;
    esac
  fi
  sleep 1
done
echo '[CI] LAP TIMEOUT';exit 1

