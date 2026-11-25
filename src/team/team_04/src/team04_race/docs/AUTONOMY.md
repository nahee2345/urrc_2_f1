# LiDAR 전용 로컬 트랙 자율주행

4단계의 최신 모드·제동거리·속도 계획·검증된 상한은 [STAGE4.md](STAGE4.md)를 따른다. 아래 경계/중앙 경로 상세는 보존된 centerline 모드의 원래 설계를 설명한다. 새 racing_line 모드는 별도 관측 경계 연결·차체 전체 검사·후보 최적화를 사용한다.

기존 SDF·센서·차량 동역학과 `scan.cpp`, `vehicle_system.cpp`를 유지하고 C++ `autonomy_04` 노드를 추가했다. IMU·엔코더·카메라·월드 pose·중앙 계측·저장된 트랙 좌표를 주행 판단에 사용하지 않는다. 카메라 출발 게이트는 기존 제어 노드에 남는다.

## 실행과 모드

직접 호출하는 launch의 기본 모드는 `manual`이다. 대회용 race_entry.json은 `driving_mode=autonomous`, 상한4.5m/s와 racing_line 모드를 전달한다. 이 고속 기본값은 Monza에서 검증했으며 Hungaroring은2.5m/s 상한을 명시한다. manual 모드에서는 자율주행 노드/명령 발행자가 생성되지 않아 기존 수동 전후진·조향 방식을 그대로 쓴다. 자율주행을 켜기 전에 기존 `ros2 topic pub` 등 speed/steering 발행자를 종료한다.

대회 중앙이 **월드·차량·clock을 이미 제공한 상태**:

```bash
source /opt/ros/jazzy/setup.bash
source src/team_04/artifacts/stage4/install/setup.bash
ros2 launch team04_race race.launch.py driving_mode:=autonomous autonomy_max_speed:=3.0
# 수동 모드:
ros2 launch team04_race race.launch.py driving_mode:=manual
```

두 launch를 동시에 실행하지 않는다. 단일 launch에서 `driving_mode`를 선택한다. 자율 모드에서는 신규 노드와 기존 제어 노드 모두 복수 speed/steering 발행자를 감지하면 속도를 차단한다. ROS 그래프 발견 지연이 있으므로 운용 전 외부 발행자를 중단해야 한다. 내부 진단용 topic echo/hz는 발행자가 아니어서 영향이 없다.

참가 launch는 월드/차량/clock을 스폰하지 않는다. `race_start_mode`, `race_start_topic`, `speed_topic`, `steering_topic`, `lidar_topic`, `use_sim_time` 계약은 유지한다. 공식 속도/조향 토픽에 Float32를 발행하며 기존 제어 노드의 출발 허가·카메라 게이트·명령 만료·차량 watchdog을 거친다.

`autonomy_test.launch.py`는 기존 월드에 개발 차량 1대만 스폰한다. 공식 grid의 `x/y/z/yaw`를 호출자가 넘겨야 한다. 운영진이 이미 스폰했다면 이 launch를 사용하지 않는다. 트랙 좌표는 주행 코드에 내장하지 않았다.

## 소스 빌드와 테스트

저장소 루트 기준:

```bash
source /opt/ros/jazzy/setup.bash
colcon --log-base src/team_04/artifacts/stage4/log build \
  --base-paths src/team_04/team04_race \
  --build-base src/team_04/artifacts/stage4/build \
  --install-base src/team_04/artifacts/stage4/install --merge-install
source src/team_04/artifacts/stage4/install/setup.bash
colcon --log-base src/team_04/artifacts/stage4/log test \
  --base-paths src/team_04/team04_race \
  --build-base src/team_04/artifacts/stage4/build \
  --install-base src/team_04/artifacts/stage4/install --merge-install \
  --packages-select team04_race --event-handlers console_direct+
colcon --log-base src/team_04/artifacts/stage4/log test-result \
  --test-result-base src/team_04/artifacts/stage4/build/team04_race
```

이 저장소에는 이전 검증용으로 추출한 동일 패키지 복사본들이 `artifacts/`에 있다. 그러므로 `--base-paths`를 생략하면 colcon이 중복 패키지 이름을 발견한다. 해당 복사본은 개발·제출 소스가 아니며 ZIP에 넣지 않는다.

ROS 안전 시험은 실제 차량과 별도 테스트 도메인 97에서 수행한다:

```bash
PYTHONDONTWRITEBYTECODE=1 python3 -s src/team_04/team04_race/tests/autonomy_safety.py \
  --output src/team_04/artifacts/stage4/autonomy_safety_final.json
```

기존 Monza 월드를 그대로 사용하는 개발 시험:

```bash
PYTHONDONTWRITEBYTECODE=1 python3 -s src/team_04/team04_race/tests/autonomy_trial.py \
  --isolated --root . --track monza --label monza-new-test \
  --duration 400 --autonomous --max-speed 0.3
```

`--label`은 기존 결과를 덮어쓰지 않도록 새로운 이름을 사용한다. `--isolated`는 **테스트 도구만** ROS 도메인96·시험 전용 Gazebo 파티션으로 분리한다. 참가 launch/알고리즘은 환경을 상속하며 이 값을 설정하지 않는다. 기존 사용자의 Gazebo/ROS 실행을 종료하지 않는다.

시험 도구는 원본 공식 `spawn()` 함수와 시작 게이트를 호출한다. 실제 센서/제어 값은 참가 노드에서 처리한다. 별도 관측자는 pose, 엔코더 및 실제 접촉을 기록한다. 이 관측 정보가 주행 노드로 들어가는 통신 경로는 없다. 테스트 전용 모델 복사본에 `team04_trial_observer`를 추가해 접촉을 읽되 질량·형상·센서·세계·물리 파라미터·힘을 바꾸지 않는다. observer heartbeat와 정상 타이어–노면 접촉으로 계측 활성 여부를 확인한다. 원래 차량 모델에는 이 관측 플러그인이 없다.

`--start-pose <이전 poses.json>`은 코너 문제 재현을 위한 **신규 테스트 차량의 스폰 위치**만 바꾼다. 살아 있는 차량을 순간이동시키지 않으며, 그 기록은 주행 알고리즘에 제공되지 않는다. 이 방식의 부분 구간 시험을 공식 출발 위치의 1바퀴 시험과 구별한다.

`--stop-after-laps 1`은 관측 노드가 원본 centerline CSV에 투영한 누적 진행률로 종료한다. CSV는 관측자만 읽으며 주행 노드는 파일·월드 좌표를 읽지 않는다. 자체 주행 분석이며 중앙 랩 계측이나 공식 완주 인증을 대체하지 않는다.

## LiDAR 인식과 경계 추정

- 원본 `/team04/points`를 SensorDataQoS depth1로 받는다. 16×720, 20Hz와 센서 위치는 변경하지 않았다. 기존 `/team04/scan`과 높이 필터도 유지한다.
- FLOAT32 XYZ·메시지 크기·엔디언·프레임·측정 시각을 검증한다. NaN/Inf·0.05–12m 밖 점과 차량 자체 반사 영역을 제외한다.
- 가까운 하향 반사에서 RANSAC+최소제곱으로 노면 `z=ax+by+c`를 추정한다. 벽면 평면·과도한 기울기·부적절한 장착 높이·부족한 inlier는 거부한다. 노면 추정 실패 시 이동하지 않는다.
- 노면보다 0.035–0.38m 높은 점을 장애물/경계 후보로 사용한다. 고정 센서 Z 필터보다 낮은 구조물을 포함한다. 40mm 높이 장애물에 대한 정지 시험을 포함한다. 이 설정 이하 구조물이나 센서가 실제로 보지 못한 물체를 감지한다고 주장하지 않는다.
- 방위별 가장 가까운 후보를 택한 뒤 ±90° 주변에서 출발해 인접 반사의 공간 연속성으로 좌우 벽을 추적한다. Y 부호로만 벽을 나누지 않으므로 90° 코너에서도 벽의 정체성을 유지한다.
- 두 벽의 실제 측정점/그 사이 짧은 관측 구간을 대응시켜 중앙 후보를 구한다. 현재 관측 폭과 크게 다른 대응, 급격한 공간 점프, 역행은 버린다. 보이는 범위를 넘어 경계를 연장하지 않는다.
- 호 길이 기준 0.2m로 재표본화하고 짧은 구간만 평활화한다. 전체 차량 폭 .24m와 양측 안전 여유 .16m를 고려해 장애물 clearance를 검사한다. 경로가 확인되지 않으면 감속/정지한다. 장애물 우회/추월을 일반적으로 해결하는 알고리즘은 아니다.

초기 X축 단면 방식은 첫 급코너에서 경계는 보이는데 단면별 양측 정보가 끊겨 정지했다. 이 실제 점군을 분석해 현재 연속 벽 추적 방식으로 수정했다. 경계/도로 정보를 저장해 후속 프레임에 재생하지 않는다.

## Pure Pursuit와 속도

좌표는 차량 로컬 +X 전방/+Y 좌측이다. `base_link`/LiDAR가 중심에 있고 후륜 축이 X=-0.16m이므로 목표점에 대해:

```text
lookahead = clamp(1.0 + 0.6 * abs(previous_commanded_speed), 1.0, 2.0) [m]
x_rear = target.x + wheelbase/2
k_pursuit = 2 * target.y / (x_rear² + target.y²)
steering_official = -atan(0.320 * k_pursuit)
```

이는 기존 제어 노드가 기대하는 +우/-좌 부호다. 같은 변환을 두 번 하지 않는다. Ackermann 안쪽 바퀴의 35° 제한에 맞춰 가상 조향각을 제한한다. 정상 주행의 조향 변화율은 기본 .65rad/s다.

명령 속도를 lookahead의 속도 대용값으로 사용한다. 실제 엔코더·IMU를 주행 판단에 쓰지 않는다. 슬립이나 가속 지연 때문에 실제 속도와 차이가 있을 수 있으며 진단 도구에서 따로 측정한다.

속도는 여러 전방 경로점의 곡률 `k_i`에 대해 다음 제한을 함께 적용한다:

```text
v_curve_i = min(v_max/(1+1.5*abs(k_i)), sqrt(a_lat_max/max(abs(k_i),epsilon)))
v_now <= sqrt(v_curve_i² + 2*a_brake*max(0, distance_i - 0.4))
v_now <= sqrt(2*a_brake*max(0, confirmed_path_length - 0.45))
```

현재 Pure Pursuit 곡률에도 같은 횡가속도·곡률 감속 제한을 적용한다. 정상 명령 상승은 .3m/s², 하강은 .7m/s²로 제한한다. 안전 오류나 false는 즉시 0 목표 명령이다. 실제 제동은 기존 물리 구동을 거쳐 유한 거리에 걸쳐 이루어진다.

`config/autonomy.yaml`의4단계 기본 최대 속도는4.5m/s다. 원래3단계 프로필은 config/autonomy_stage3_rollback.yaml에 보존했다. `autonomy_max_speed` launch 인자로 시험 상한을 변경한다. 기존 centerline 복귀 프로필은 횡가속도 .35m/s²·제동 .7m/s²이고,4단계 racing_line 기본값은1.0m/s²·2.5m/s²다. 센서 타임아웃은 .18초로 유지한다. 계획 파라미터는 실제 물리 응답의 엄격한 상한 보장이 아니다. 현재 시험 범위와 재현 명령은 [STAGE4.md](STAGE4.md)를 확인한다.

## 안전 상태와 디버깅

시작/재시작 시 출발 미허가·속도0. Bool false는 즉시0이며 기존 카메라 조건과 watchdog을 우회하지 않는다. 측정 시각과 실제 수신 중단을 모두 검사한다. 다음 조건은 0 명령으로 전환한다:

- .18초 이상 수신 중단 또는 오래된 timestamp, 미래/역행하는 시각
- 형식/프레임 오류, 유효 노면 또는 양측 경계 부족
- 지나치게 좁은 폭, 불충분한 lookahead, 충돌 후보 때문에 잘린 경로
- 계산 완료 시점의 지연 초과, 비정상 계산/파라미터/예외, 복수 명령 발행자, 출발 false

일시적 인식 실패 후에는 새로운 유효 LiDAR 데이터로만 회복한다. 오래된 경로를 누적하지 않는다. 센서 시각이 역행하면 출발 허가를 초기화한다. 프로세스 자체가 비정상 종료되면 기존 제어 명령 만료와 차량 watchdog이 작동한다.

| 디버그 토픽 | 타입/의미 |
|---|---|
| `/team04/debug/path` | nav_msgs/Path, 현재 측정 프레임의 로컬 경로 |
| `/team04/debug/target_point` | geometry_msgs/PointStamped, Pure Pursuit 목표 |
| `/team04/debug/track_boundaries` | visualization_msgs/MarkerArray, 대응된 좌우 경계 |
| `/team04/debug/target_speed` | std_msgs/Float32, 실제 발행한 목표 속도 |
| `/team04/debug/state` | std_msgs/String JSON, 상태·지연·처리시간·수신Hz·오차·곡률·폭·성공률·정지 횟수 |

state의 arrival_hz는 벽시계 수신 Hz, stamp/age는 중앙 시뮬레이션 시각 기준이다. lateral_error는 가까운 로컬 중앙 경로와의 횡방향 차이이며 측량 기반 정답 오차가 아니다. ground-truth 비교는 별도 시험 분석에서만 수행한다.

```bash
ros2 topic echo /team04/debug/state --field data
ros2 topic echo /team04/debug/target_speed
rviz2 -d "$(ros2 pkg prefix --share team04_race)/config/autonomy.rviz"
```

RViz fixed frame은 `team04/lidar_link`다. 월드 pose→TF를 주행 노드에서 만들지 않는다. RViz GUI는 이 환경에서 자동 실행/시각 검증하지 않았으므로 GUI 결과를 성공했다고 보고하지 않는다.

## 남은 범위

지도 기반 계획, IMU/엔코더 융합, 추월, 공식 완주 판정은 포함하지 않는다. 중앙 계측 토픽 호환성 문제는 그대로 두며 수정하지 않는다. 전체 코스/반복 완주와 Hungaroring의 실제 결과는 `src/team_04/REPORT_STAGE3.md`의 실행 기록을 기준으로 확인한다. 부분 구간 성공을 전체 완주로 해석하지 않는다.
