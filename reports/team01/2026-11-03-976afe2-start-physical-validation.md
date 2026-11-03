# TEAM 01 검증 보고서 — 976afe2: 출발·구동 P0 실증

- 대상 차량/검증 코드 커밋: `976afe27e66092233fe2312a149302bcf2f3a357`
- GitHub Actions: [독립 QA #41582636679](https://github.com/nahee2345/urrc_2_f1/actions/runs/41582636679), [기존 Monza #41582636663](https://github.com/nahee2345/urrc_2_f1/actions/runs/41582636663)
- 독립 QA artifact: [team01-independent-qa-41582636679](https://github.com/nahee2345/urrc_2_f1/actions/runs/41582636679/artifacts/13206544334)
- 범위: **증거 수집 및 원인 분석만 수행**. 주행/ROS/Gazebo 코드 수정 없음.

## 1. 최종 결과: FAIL

| 검증 항목 | 실제 결과 | 증거 |
|---|---|---|
| ROS 2 Jazzy + Gazebo + RTAB-Map 설치 | PASS | 독립 QA job 125740739755 |
| 실제 Gazebo 서버 및 차체 생성 | PASS | `gazebo.log`, `spawn.log`, `controller.log`, `lap.json` |
| RGB·Depth·전후방 LiDAR 샘플 확인 | PASS | CI 센서 확인 단계 통과 |
| 카메라 기준 이미지 보정 | PARTIAL | `camera_calibrated=true` / `gate_baseline_ready=true` (최종 `start_debug.txt` 1회, **출발 가능 여부까지 입증하지 않음**) |
| 카메라 점등 전 준비 승인 | **FAIL** | `start_failure.txt`: `START_CALIBRATION_FAILED: camera baseline not ready` |
| 신호등 실제 순차 점등·동시 소등 | **NOT EXECUTED** | 준비 승인 전에 runner가 종료됨, `lights.log` 없음 |
| 실제 출발 허가 | FALSE | `start_debug.txt` permission false, `adapter_status.log` allowed false |
| 실제 ROS→Gazebo 속도 전달 | **0m/s** | `adapter_status.log`: sent_linear_mps 0, send_enabled false |
| 독립 Odometry 실제 이동 | **0.000m** | `lap.json`, samples=273, elapsed_wall_s=122.13 |
| Monza 1랩 | **NOT ACHIEVED** | `lap.json` `TRACKING`은 종료 전 중간 상태일 뿐 PASS 아님 |
| 전체 오프라인 회귀시험 | **FAIL: 150개 중 4개** | `offline_qa.log`, workflow job exit code 1 |
| 새 v23 카메라 준비 단위시험 | PASS | `test_v23_start_diagnostics` 2개 PASS |

**중요:** 이번 0m는 출발 명령이 0으로 유지되었기 때문이며, **물리 구동계가 고장이라는 증거가 아니다**. 이전 `0155410`의 1.122m / `OFF_TRACK` 기록도 다른 커밋의 증거로 구분한다.

## 2. 우선 분석 — 신호 보정 확인 경로

확정:
- `run_local_monza.sh`와 CI는 준비 상태 `/team01/ready_for_lights`가 true인지 확인한 뒤 신호등을 작동한다. 출발 조건을 우회하지 않도록 설계되었다.
- CI는 40회 × `timeout 4s ros2 topic echo ... --once --field data`로 준비 상태를 확인하다가 실패 처리하였다.
- `controller.log` 종료 시점에는 카메라 `calibrated=True`, `first_red_at=None`, 주요 센서 `sensor=True`, 카메라 `rgb_age` 신선 상태, `gate=WAIT_RED`, `permission=False`. 최종 단발성 `start_debug.txt`에는 `gate_baseline_ready:true`가 있다.
- `start_status.log`는 `WARNING: topic [/team01/debug/start] does not appear to be published yet\nCould not determine the type` 후 종료되어 **주기별 진단 기록이 없다**. 토픽이 존재하기 전에 CLI 수집기를 한 번 실행하고 실패하면 재시도하지 않는다.
- `start_debug.txt`는 ros2 CLI 문자열 잘림(`...'`)으로 모든 준비 필드와 `ready_for_lights` 값을 확인할 수 없다. `adapter_status.log`는 정상 수집되었으며 샘플 모두 `send_enabled:false` / `sent_linear_mps:0`이었다.

**미확정 원인 가설(증거 우선순위):**
1. ROS2 `topic echo --once`의 반복 생성/토픽 타입 발견 또는 QoS/발행 타이밍 문제로 true 메시지가 감지되지 않는다.
2. `camera_ready()`의 일부 조건이 순간적으로 false를 반복한다. 카메라 기준 보정만으로 `ready_for_lights=true`를 보장하지 않는다. `vision_ready`, `gate_baseline`, `first_bright_time`, `rgb_age`, `depth_ok`, `lidar_ok`를 **같은 시각**에 기록해야 한다.

## 3. 제작 담당자에게 수정·재검증 요청

**P0-1 준비 상태 계측**
- 준비 토픽 `/team01/ready_for_lights`의 true/false를 ROS2가 *지속 구독*해 안정적으로 캡처할 것. 한 번만 생성하는 `ros2 topic echo` CLI의 발견 실패를 전체 검증 결과로 오인하지 말 것.
- `/team01/debug/start`의 모든 JSON 필드와 `ready_for_lights`를 20Hz CSV/JSONL로 저장. `ros2 topic echo`가 토픽 생성 전 실패하면 명시적 타입으로 재구독/재시도. 문자열 잘림 없이 기록할 것.
- 각 필드 조건 충족/미충족과 준비 승인 전환 시각을 남길 것. 보정 실패가 해소되기 전 신호등 강제 점등·출발 조건 우회 금지.

**P0-2 출발→구동 실제 증거**
- `ready_for_lights=true` → 카메라 빨간 불 관찰 → `LIGHTS_OUT` → `permission=true` → `adapter.send_enabled=true` → `sent_linear_mps>0` → `gz_odom` 실측 전진의 **순서와 시간차** 검증.
- `/team01/debug/adapter`은 ROS **발행측**만 입증. Gazebo plugin 수신/차축 회전/실측 odom은 별도 확인. 앞 단계가 성공하기 전에는 뒷단계 고장으로 판단하지 말 것.
- 원래 문제인 **이전 1m 전후 정체**를 재현하려면 최소 5m 이상 실제 이동 로그를 확보해 command / Gazebo cmd / odom / 접촉을 비교할 것.

**P0-3 회귀시험**
- `test_v7_grid_and_depth_corner`의 adaptive→constant_arc 2개 및 `test_v9_startup_lap_exploration`, `test_v13_lane_hold_and_gentle_steering`의 곡선 BRAKE 2개: 기존 **4개 FAIL 지속**.
- 안전 경계/제동 검사를 삭제하거나 테스트 기대값만 완화하지 말고, 후보 탈락·BRAKE 사유를 재현. 5m/s 완주 전 8/10m/s 승인 불가.

**P1 로컬/CI 환경 동등성**
- 신규 `run_local_monza.sh`의 경로는 공식 submodule 우선이고 생성 위치는 `safe_grid_spawn.py`와 동일하다. 그러나 로컬 실행은 `monza_race.sdf`, `monza_signal_active.sdf`, `monza_grid.gui.config`를 **기존에 생성된 파일로 요구**하므로 깨끗한 clone에서 파일이 없으면 종료한다. CI는 `enhance_track(...,'monza')`로 생성한다. 제작 담당자는 assets generation 단계를 비교하고 로컬 실행 편의성을 점검할 것.
- 기존 Gazebo GUI와 동일 `GZ_PARTITION`의 중복 실행이 없는지 확인.

## 4. 최신 파일 업데이트/실행 (Ubuntu 24.04, ROS2 Jazzy)

```bash
cd ~/urrc_2_f1
git pull --ff-only
git submodule update --init --recursive

# 공식 트랙 자산이 없으면, 레이스를 시작하지 않고 자산만 생성
python3 - <<'PY'
from pathlib import Path
import sys
p=Path('upstream/urrc_f1_ws/src/urrc_track_gazebo')
sys.path.insert(0,str(p))
from urrc_tracks.v10_assets import enhance_track
enhance_track(p,'monza')
PY

source /opt/ros/jazzy/setup.bash
bash src/team_01/team01_race/scripts/run_local_monza.sh 2>&1 | tee ~/team01_976afe2.log
```

`ros2 launch team01_race race.launch.py`만으로는 신호등 점등 시퀀스/외부 출발 허가가 자동 시작되지 않으므로 반드시 통합 실행 스크립트를 사용한다.
