**v10 검증 후보: [README_V10_VALIDATION_KO.md](README_V10_VALIDATION_KO.md). 실제 Monza 1랩 미인증입니다.**

# TEAM01 최신 버전 v9

**먼저 [`README_V9_KO.md`](README_V9_KO.md)를 확인하세요.** 기존 v4~v8의 설명은 변경 이력이며 현재 v9 설정과 다를 수 있습니다.

# URRC TEAM01 F1 Depth/RGB Racing v4 (2026-11-02)

## 설치 위치와 실행

ZIP 내용의 `team01_race/` 폴더를 `~/urrc_2/src/team_01/`에 덮어씁니다. `src/urrc_track_gazebo` 트랙·신호등 소스는 변경하지 않습니다.

```bash
# 압축 파일을 Downloads에 저장한 뒤 기존 Gazebo를 Ctrl+C로 종료하세요.
mkdir -p ~/urrc_2/src/team_01
unzip -o ~/Downloads/team01_f1_rgb_depth_v4.zip -d ~/urrc_2/src/team_01
cd ~/urrc_2/src/team_01/team01_race
bash scripts/run_offline_tests.sh
bash scripts/run_local_monza.sh
```

필요한 환경은 Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic입니다. 차량 컴퓨터에 `python3-numpy`, `python3-opencv`, `ros-jazzy-ros-gz`, `python3-colcon-common-extensions`가 설치되어야 합니다. 기존 빌드 결과에 이전 모델이 남아 있을 경우 `~/urrc_2/build/team01_race`, `~/urrc_2/install/team01_race`만 확인하여 재빌드하십시오. 해당 스크립트는 `colcon` 빌드와 `source ~/urrc_2/install/setup.bash`를 실행합니다.

## v4 변경점

- **시동:** 카메라에서 빨간불 **4개 이상**이 최소 2프레임 보이고, 이후 전체 소등 상태를 3프레임 관찰하면 출발합니다. 첫 빨간불 관찰로부터 최소 3초 이후에만 허용합니다. 초기 검은 화면이나 1~3개만 켜진 영상만으로는 절대로 출발하지 않습니다. `/race/start/team01`의 `Bool true` 허가도 필요하며 `false`는 언제나 정지입니다.
- **도로 카메라:** 기존 전방 신호등 RGB(위쪽)와 별도로 Depth와 정확히 같은 포즈·해상도·시야각의 **하향 road_rgb**를 추가했습니다. 원래 노란 차체 형상과 센서 외형은 유지합니다.
- **검정 아스팔트 인식:** RGB의 어두운 중성회색과 Depth에서 실제 지면에 해당하는 픽셀만 결합합니다. 저장소의 아스팔트(42,44,47)와 녹갈색 비도로 지면(73,79,68)을 구별하도록 RGB 색채 조건을 추가했습니다. 단순히 검은 물체를 도로로 해석하지 않습니다.
- **경사:** Depth로 연속 지면을 평면 적합하여 오르막·내리막을 장애물에서 제외합니다. 전방의 평면 밖 높이 장애물은 회피 대상으로 남깁니다.
- **경로:** 추정된 도로 좌우 경계 사이에서 차량 전폭 및 여유 거리를 확보하는 Ackermann 후보 호를 평가합니다. 직선에서는 좌측·우측 편향선을 교대로 사용하고 코너에서는 해당 편향을 줄여 도로 방향을 우선합니다. 중앙만 고집하지 않습니다.
- **추월:** 전방 대상과의 연속 거리 변화와 자차 속도로 상대 속도를 추정하고, 반응거리/상대 제동거리로 추월 접근 거리를 계산합니다. 목표 추월 거리·예상 주행 거리를 로그로 확인할 수 있습니다. 양옆 공간, 후방 간격, 차단 경로를 평가하고 위험한 경우 감속·정지합니다. 추월 중에도 안전성 우선이며 **충돌 0%는 보증할 수 없습니다.**
- **속도:** 직선 목표 5.0m/s, 조향각 10° 이상 코너에서 목표값 0.3배. 실제 명령은 경계 가시거리, 횡가속도, 충돌 정지거리로 추가 제한됩니다. 좌우 조향 명령은 35° 이내입니다.

## 실제 주행 확인

두 번째 터미널에서 환경을 불러오고 확인하세요.

```bash
set +u
source /opt/ros/jazzy/setup.bash
source ~/urrc_2/install/setup.bash
export GZ_PARTITION="urrc_practice_$(id -u)"
ros2 topic hz /team01/road_rgb --qos-reliability best_effort
ros2 topic echo /team01/speed --once
ros2 topic echo /team01/steering --once
```

로그 해석:

- `gate=ARMED`, `out=False`: 빨간불 점등을 확인했지만 소등을 아직 인정하지 않았습니다.
- `gate=LIGHTS_OUT`, `out=True`: 빨간불 소등 조건 충족.
- `source=rgb_depth`, `road=True`: Depth 지면 + 검정 아스팔트 경계로 후보 경로 생성.
- `source=none`, `road=False`: 안전상 속도 0으로 유지. RGB 영상·노면 색/광량·Depth를 점검합니다.
- `lead`, `lead_v`, `pass_at`, `pass_length`: 목표차 거리, 추정속도, 추월 준비거리, 추월 예상 진행거리(m).
- `STRAIGHT / CORNER / OVERTAKE / BRAKE / STOP`: 현재 제어 상태.

## 검증 범위와 제한

포함된 39개 자동 테스트는 Python·XML·가짜 RGB/Depth·좌우 경로 평가·가상 자전거 운동학만 검증합니다. 이 환경에는 ROS 2 Jazzy/Gazebo Harmonic이 없어 실제 센서 화면, 조향 플러그인, 실주행, 트랙 완주, 물리 충돌은 실행 검증하지 못했습니다. 실제 PC에서 반드시 주행 시험이 필요합니다.

**대회 규정 주의:** 저장소 `race/README_KO.md`의 LiDAR-only 주행 규격은 RGB/Depth 기반 도로 판단을 허용하지 않습니다. 이 파일은 *독립 개발·실험용*이며 운영 규정 변경 없이는 공식 제출용이 아닙니다.

## v5: 출발 신호 인식 및 실제 영상 진단

2026-11-02 현장 로그의 `RGB=9 intensity=20544 gate=WAIT_RED`,
`road=False`는 **빨간색 고정 물체를 신호등으로 혼동하면서 실제 점멸을
인식하지 못하는 현상**과 **도로 인식 불확실성**을 나타냈습니다.

- 출발용 카메라가 신호등을 더 크게 보는 방향(상향 0.42 rad / HFOV 1.10 rad)으로 조정되었습니다.
- 영상 전체에서 빨간 물체 수를 세지 않습니다. 소등 상태의 초기 영상을 학습하고,
  신호등 위치 주변 상단의 **밝아진 빨간색 픽셀의 변화**를 검출합니다.
- 4개 이상의 점등이 확인된 뒤 화면에서 실제로 0개가 안정적으로 관찰되어야만 출발합니다.
- 조명 애니메이션이 카메라에 보이지 않으면 **절대 출발을 위조하지 않습니다.**
- 도로 RGB가 거짓으로 판정되더라도 연속된 **양쪽 LiDAR 방호벽 경계**가
  함께 인식된 경우에 한하여 최고 0.75 m/s로 저속 경계 추종을 허용합니다.
  벽이 한쪽만 보이거나 도로 경계가 모두 없으면 정지합니다.

실행:

```bash
cd ~/urrc_2/src/team_01/team01_race
bash scripts/run_offline_tests.sh
bash scripts/run_local_monza.sh
```

실제 영상 증거는 `~/.local/state/urrc_team01/camera/`에 기록됩니다.
`dark.png`(기본 상태), `armed.png`(4개 점등), `lights_out.png`(소등),
`latest.png`(마지막/주기적 카메라 화면)입니다. `armed.png`가 없으면
카메라에서 4개 점등을 검출하지 못한 것입니다.

주행 판단을 포함한 이 v5는 모의 검증용이며, 실제 Gazebo 실행 / 완주 / 무충돌
주행은 사용자 PC에서 확인해야 합니다. 맵 및 운영자 신호등은 변경하지 않습니다.

## v6 — 연석 중앙 추종 → 코너 아웃/인/아웃 → 1랩 학습

- **출발:** 검증된 v5 RGB 신호등 출발 인식 로직은 그대로 둡니다.
- **출발 그리드 (`GRID`):** Depth+RGB 또는 양측 LiDAR 연석으로 주행 가능 영역을 확인합니다. 양쪽 경계가 연속 6프레임 이상 관찰되고 약 0.65m를 저속 직진할 때까지 전방 방향을 유지합니다. 경계가 불충분하면 정지할 수 있습니다.
- **중앙 추종 (`CENTER`):** 차량 `base_link` 기준 좌우 연석 거리 및 전방 경계의 중간 경로를 실시간 계산합니다. 시뮬레이터의 월드 좌표 또는 미리 저장된 트랙 중심선은 사용하지 않습니다.
- **코너 (`CORNER_ENTRY → CORNER_APEX → CORNER_EXIT`):** 확인된 도로 폭 내에서 바깥쪽→안쪽→바깥쪽으로 목표 경로를 연속 이동합니다. 횡이동 목표는 속도 제한이 적용되며 도로 폭이 좁으면 자동 축소됩니다. 코너 목표속도는 직진 목표속도의 30%이며 실제 감속률은 차량 제동 성능 범위로 제한됩니다.
- **추월/장애물:** v5의 전방 Depth/LiDAR 위험 판정, 상대속도, 우회 경로 후보 검사를 유지합니다. 출발 그리드 탐색 중에는 추월을 새로 시작하지 않습니다. 위험하면 정지합니다.
- **1랩 기록 (`LapLearner`):** 구동 명령 속도를 적분한 *추정* 주행 거리, 조향 기반 누적 회전, 출발선처럼 보이는 흰색 가로 마킹의 재검출, 출발 구간 좌우 연석 형태가 함께 맞을 때만 랩 완료를 잠정 인정합니다. 1랩 로그는 `~/.local/state/urrc_team01/learning/lap1_*.json`에 저장됩니다. 이는 실제 차륜 주행거리 계측 또는 공식 랩타이밍을 대체하지 않습니다.
- **2랩:** 잠정 랩 완료 이후 현재 센서에 안전 여유가 있는 구간의 목표속도를 기본 10% 상향합니다. 1랩에서 제동/정지가 있었던 구간은 가속하지 않으며, 이전 랩 코너 기록은 사전 감속에만 참고합니다. 현장 데이터 검증 전 이 상향은 시험 설정입니다.

### 실행 (Ubuntu 24.04 / ROS 2 Jazzy / Gazebo Harmonic)

```bash
cd ~/urrc_2/src/team_01/team01_race
bash scripts/run_offline_tests.sh
bash scripts/run_local_monza.sh 2>&1 | tee ~/team01_v6.log
```

진단 로그: `line_phase`, `curb_left`, `curb_right`, `line_offset`, `lap`, `lap_distance_est`, `lap_yaw_est`, `lap_check`. 실행 중 센서가 불충분하거나 주행 불가 영역으로 보이는 경우에는 안전 우선으로 정지합니다.

**검증 한계:** v6는 Python 모의 센서/운동 모델의 정적·단위 테스트까지 검증되었습니다. 본 제작 환경에 ROS 2/Gazebo 실행기가 없어 실제 Monza 완주, 물리 접촉, 신호등 주행 세션, 정식 랩 검증은 수행하지 못했습니다. RGB/Depth 기반 주행은 저장소의 일부 공식 참가 규격에 기재된 LiDAR-only 제한과 충돌할 수 있습니다.

## v7 — GRID 고착 수정 + Depth 곡선 경로 추종 + 속도 측정

### 왜 v6가 느리고 코너에서 멈췄는가

2026-11-02 사용자 로그에는 `GRID v=0.65`가 누적 추정 거리 333m 이후에도 지속됐으며, `no verified road-safe candidate`에서 BRAKE로 전환됐습니다. 이는 도로가 인식됐는데도 GRID를 떠나지 못한 상태 전환 조건과, 한 번 고른 조향각을 전방 전 구간에 계속 적용한다고 가정하는 원호 충돌 검사 때문에 발생할 수 있습니다. 거리는 **구동 명령 적분 추정값**이며 실제 주행 거리는 아닙니다.

### 변경

1. **GRID 탈출:** 경계가 유효하고 1.65m 앞까지 확인되며 6번 연속 유효 프레임 + 0.65m 이동이 확인되면 CENTER로 진입합니다. 예전 3.2m 전방 확인 임계값에 의해 오랫동안 GRID에 머무르는 현상을 줄입니다. 경계가 소실되면 출발을 강행하지 않습니다.
2. **속도:** `grid_discovery_speed_mps = 1.25` (기존 0.65), `straight_speed_mps = 6.0` (기존 5.0), 코너는 30% 즉 1.8m/s **목표 상한**입니다. 실제 명령은 안전거리·횡가속·감속 한계로 낮아집니다.
3. **깊이 카메라 중심 코너:** 기존 RGB+Depth 지면/노면 인식에서 나온 `(x, right, left)` 경계를 연속 경로로 해석합니다. `curved_path.py`는 14cm씩 가상 전진하는 동안 Pure Pursuit 조향각을 *매 순간 갱신하는* 후보 경로들을 검사합니다. 일정 조향각 하나만으로 코너 전체를 예측하지 않습니다. 최종 명령은 매 주기 첫 조향만 사용합니다.
4. **좁은 도로 안전:** 장애물 및 바퀴/차체 폭을 포함하는 경계 검사와 빈틈 없는 짧은 가시거리 속도 제한을 유지합니다. 유효한 곡선 경로가 없거나 위험이 있으면 정지합니다.
5. **속도 진단:** `/team01/gz_odom`(선택적 Gazebo 차륜 모델 odometry) → ROS2 bridge를 추가해 로그의 `odom_v`로 물리 속도를 확인할 수 있습니다. `lap_distance_est`는 이 토픽이 있으면 실측 추정 속도를 적분하고, 없으면 이전처럼 명령 속도를 적분합니다. odom 사용은 **실제 운동 추정**일 뿐 공식 랩 인증은 아닙니다. 브리지가 동작하지 않아도 기존 도로 주행 명령은 계속합니다.
6. **새 로그:** `planner=adaptive_pursuit / constant_arc`, `curve`, `curb_ahead`, `curb_confirm`, `odom_v`.
7. **출발 유지:** v5 이후 검증된 RGB 신호등 4개→0개 인식 로직 및 노란색 차량 외형은 바꾸지 않았습니다. 맵·신호등·운영자 파일도 변경하지 않습니다.

### 최종 20m/s 목표 (현재 비활성)

20m/s는 `target_straight_speed_mps=20.0`으로 **미래 목표만 기록**했습니다. 현재 차체 Ackermann 모델의 `<max_velocity>12</max_velocity>` 및 약 12m 전방 감지, 도로 경계 약 7m 검증 한도로는 안전하게 20m/s를 지시할 수 없습니다. 최대제동 5m/s²·인지 지연 0.3초·여유거리 0.55m라면 20m/s에서 최소 약 **46.55m** 이상의 정지 가능 확인 거리가 요구됩니다. 20m/s 주행 전 모델·센서 거리·제동/마찰·실속도 계측·코너 궤적 검증을 별도로 마쳐야 합니다.

### 실행

```bash
# 기존 Gazebo를 Ctrl+C로 종료
unzip -o ~/Downloads/team01_f1_depth_corner_v7.zip -d ~/urrc_2/src/team_01
cd ~/urrc_2/src/team_01/team01_race
bash scripts/run_offline_tests.sh
bash scripts/run_local_monza.sh 2>&1 | tee ~/team01_v7.log
```

정상 경로 탐색이라면 출발 후 `line_phase=GRID`가 `CENTER` 또는 `CORNER_*`로 바뀌고, `planner=adaptive_pursuit`가 커브 구간에 표시됩니다. `odom_v=None`이 계속 나오면 odometry 브리지/토픽을 점검해야 합니다. `v=`는 목표/제어 명령이며 실제 속도는 `odom_v`를 확인해야 합니다.

### 참고 코드

- 현재 `urrc_2/race/pace_cars/routes.py`의 10/20m/s 차량은 지도 중심선/경로가 **미리 정해진 운영자용 키네마틱 페이스카**이며 센서 기반 4륜 자율주행 검증은 아닙니다.
- F1TENTH ROS2 예시: `https://github.com/Attiahimself/f1tenth_racing` (Follow-the-Gap, 중심선, Pure Pursuit, 주행 기록) 및 `https://github.com/dylanjian0/f1tenth-racing-stack` (벽 추종, 갭 추종).
- 정밀 주행 궤적 최적화: `https://github.com/WVU-F1Tenth/Raceline-Optimization`.

실제 Monza Gazebo 완주/물리 접촉/20m/s 주행은 이 파일로 인증되지 않았습니다. 테스트는 ROS/Gazebo 없는 Python 모의 입력, 정적 검사, 간단한 자전거 모델의 폐루프 경로 추종만 확인했습니다.

### 연석과 앞차 분리 (v7 추가)

굽은 코너의 안쪽 연석은 정면 Depth 카메라 중앙 영역으로 들어와 앞차(`lead`)로 오인되기 쉽습니다. v7에서는 도로 경계에서 차체 여유폭 이내의 점들을 앞차 **속도 추정 후보에서만 제외**합니다. 해당 점들은 장애물/충돌 검사에는 남아 있습니다. 외부 객체 식별의 정확성은 실제 영상으로 추가 검증해야 합니다.

## v8: 코너 경계 접근 사전 감속 및 영상 진단 (2026-11-02)

- 실제 Gazebo 로그에서 약 14~21m 구간 `curb_left=0.33→-0.07m`으로 낮아지며 이탈/정지하는 상황이 보고되었습니다.
- 주행 중 가까운 연석 여유를 매번 검사하고, 여유가 줄어들면 감속합니다. 검증 가능한 차폭이 사라지면 정지합니다.
- `CORNER_EXIT` 상황에서도 도로 굴곡이 심하면 바깥쪽으로 이동하지 않고, 반대 방향 새 코너가 확인되면 진입 단계를 다시 시작합니다.
- `edge=NONE|CAUTION|CRITICAL`, `edge_m`, `edge_vcap` 로그 추가.
- 도로 RGB 실측 이미지: `~/.local/state/urrc_team01/road/` (latest, edge_caution, edge_critical, first_brake).
- 기존 신호 인식은 그대로 유지합니다. **실제 Monza 완주 및 물리 충돌 검증은 아직 아닙니다.**
