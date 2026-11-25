# Team 04 차량·센서 및 LiDAR 자율주행

기존 물리 차량·센서를 유지하면서 LiDAR 전용 C++ 자율주행 노드를 추가했다. `driving_mode:=manual`은 기존 수동 입력, `driving_mode:=autonomous`는 로컬 경계·중앙 경로·Pure Pursuit·곡률 속도 제어를 사용한다. 직접 실행하는 launch의 기본값은 manual이고, 대회용 race_entry.json은 autonomous를 명시한다. 두 방식 모두 출발 허가 이전에는 정지한다. 4단계에서는 관측 경계 안의 로컬 레이싱 라인과 고속 속도 계획을 추가했다. 5단계에서 자기 LiDAR 객체 탐지·추적·충돌 예측과 선택적 국소 추월을 추가했다. 실제 검증 범위와 미완료 항목은 [STAGE5.md](docs/STAGE5.md) 및 외부 REPORT_STAGE5.md를 확인한다.

**최신 교통 기능은 [STAGE5.md](docs/STAGE5.md)에 있다. 기존4단계 설정·검증 범위·복귀 명령은 [STAGE4.md](docs/STAGE4.md)에 있다. 빌드·실행·안전은 [AUTONOMY.md](docs/AUTONOMY.md), 3단계 성능 시험·실측 기준·복귀는 [STAGE3.md](docs/STAGE3.md)에 있다.** 아래 내용은 1단계 차량·센서 인터페이스와 물리 정보를 포함한다.

## 분석 및 변경 범위

필수 문서 6종과 `race/scripts/{race_manager,submission,race_runtime}.py`, 기존 월드의 플러그인 구성을 읽었다. 중앙 실행기는 SDF를 변환한 임시 복사본에 PosePublisher를 넣고 팀 이름으로 스폰하며, 참가 launch에 6개 필수 인자를 전달한다. 참가 launch는 센서 Bridge·변환·제어·정적 TF와 선택한 모드의 자율주행 노드를 실행한다. 중앙 `/clock` Bridge는 재실행하지 않는다.

루트 README의 센서 자유 사용 안내와 `race/README_KO.md` v2의 LiDAR 전용 주행 규정이 다르다. 기본 설계는 v2를 따른다. 카메라 구독은 camera 출발 모드에서만 활성화하고, IMU·엔코더는 측정만 제공한다. 현재 주행 판단은 신규 LiDAR 노드에서만 수행하며, 확장 센서 융합은 운영진 허용 후 별도 노드를 추가하는 다음 단계다. GPS·월드 pose·저장된 트랙 좌표·중앙 계측을 제어 노드에 연결하지 않는다.

배포본에 유효한 Git 이력이 없었다. `src/team_04/artifacts/baseline.json`에 기존 파일 SHA-256을 기록해 최종 무변경 검사를 수행한다. 개발 소스·빌드·검증 결과·ZIP 모두 `src/team_04/` 안에 둔다. ROS/Gazebo 자체 로그와 launch 임시 파일은 실행기가 생성하는 외부 런타임 산출물이다.

## 차량 치수와 물리

| 항목 | 값 |
|---|---:|
| 직진 시 외형 길이×폭×높이 | 0.520 × 0.202 × 0.165 m |
| 양 전륜 독립 ±35° 전 구간 보수적 포락 외형 | 0.520 × 0.238958 × 0.165 m |
| 휠베이스 / 전후 트레드 | 0.320 / 0.184 m |
| 바퀴 반지름 / 폭 / 질량 | 0.035 / 0.018 m / 각각 0.060 kg |
| 전체 질량 / 정적 질량중심 높이 | 2.405 kg / 약 0.05994 m |
| base_link 질량 / 주 관성 Ixx,Iyy,Izz | 2.0 kg / 0.0074833, 0.0360833, 0.0419333 kg·m² |
| 타이어 μ, μ2 / 바퀴 조인트 감쇠 | 1.0, 1.0 / 0.0001 N·m·s/rad |
| 후륜 토크 제한 / 조향 토크 제한 | 바퀴당 0.6 / 0.8 N·m |
| 속도 제한 | 전진 5 / 후진 2 m/s (성능 보증값 아님) |
| 각 전륜 기계적 한계 | ±35° |
| 초기 스폰 노면 높이 오프셋 | 0.04 m |

차체 본체는 0.46×0.20×0.07 m 외곽을 유지하면서, 넓은 중앙부와 좁은 앞뒤 부분으로 나눠 바퀴 공간을 확보했다. 가득 찬 직육면체는 전륜 조향 공간과 간섭하기 때문이다. 범퍼 끝은 X=±0.26 m이다. 모든 visual에는 같은 형상·위치의 collision이 있다. 별도 메시나 외부 리소스는 사용하지 않는다.

base_link 원점은 차체 중앙의 지면 투영점이고 관성 원점만 Z=0.06 m이다. 후륜 회전축은 +Y이며 양수 각속도가 전진이다. 전륜 조향축은 +Z이다. 제어 플러그인은 후륜 속도 오차에 대한 토크 제어와 전륜 조향 PID를 사용한다. 속도 0에서는 후륜이 감속한 뒤 실제 조인트 위치를 유지하는 토크 제동으로 경사면 굴러내림을 억제한다. Ackermann 안쪽 바퀴가 35°를 넘지 않도록 가상 중앙 조향각을 제한한다. 따라서 공식 명령 35°에서 양 바퀴 모두 35°로 평행 회전하지 않는다. 포즈·속도 초기화 API로 이동하지 않는다.

치수 검증기는 상자·원기둥의 실제 지지함수로 경계를 계산한다. 조향 0.1° 간격의 전 구간과 연속 구간을 덮는 보수적 오차 여유를 포함한다. 전륜은 가능한 모든 독립 조향을 포함한다. 현재 primitive SDF 전용이며 메시·중첩 모델·상대 pose frame을 추가하면 검증 실패로 처리한다. 노면 기울기에 따른 월드 AABB나 물리 접촉 변형은 차량 고유 치수와 구분한다.

## 센서와 토픽

모든 좌표는 base_link 기준 지면 높이다.

| 센서 | 측정 원점 X,Y,Z (m) | 설정 | ROS 토픽 및 타입 |
|---|---|---|---|
| 3D GPU LiDAR | 0,0,0.13 | 360°×30°, 720×16, 20Hz, 0.05–12m | `/team04/points` PointCloud2 |
| 2D 변환 | LiDAR와 동일 | 높이 0.09–0.25m, 720 bins, SensorDataQoS depth 1 | `/team04/scan` LaserScan |
| IMU | 0.01,0,0.06 | 100Hz, Gaussian gyro σ=.0002, accel σ=.005 | `/team04/imu` Imu |
| 후륜 엔코더 | -0.16,±0.092,0.035 | 물리 엔진 조인트의 누적 위치·실제 각속도, 100Hz | `/team04/wheel_joint_states` JointState |
| 카메라 렌즈 | 0.21,0,0.12 | RGB 640×480, 30Hz, 수평 FOV 90° | `/team04/camera/image_raw` Image |

LiDAR 하우징은 지름 .055m, 높이 .070m로 최상단 .165m다. 카메라 하우징은 렌즈 뒤쪽 X=.186–.210m, Z=.107–.133m이고 실제 마운트도 포함한다. IMU 하우징과 후륜 안쪽 엔코더 하우징도 collision에 포함했다.

PointCloud2→LaserScan은 같은 측정 timestamp를 보존한다. 높이 필터 후 각 방위에서 가장 가까운 점을 사용하며, 차체·범퍼·카메라/마운트 표면의 자기 반사는 모델 치수에 맞춰 제외한다. 원본 3D는 변경하지 않는다. 카메라 등에 가려져 측정되지 않은 외부 거리를 복원하지 않으며, 유효 반사가 없는 bin은 infinity다. GPU 렌더링에 따른 미세한 자기 반사 경계 오차는 검증 결과와 함께 확인해야 한다. 흰 선의 색상을 거리로 해석하지 않는다. 0.2초 이상 오래된 포인트는 버리고 누적하지 않는다.

엔코더 토픽에는 후륜 2개와 전륜 조향 2개 조인트가 들어 있다. 후륜의 `position`과 `velocity`를 이름으로 선택한다. 엔코더 속도는 슬립 시 실제 차체 이동과 다를 수 있다. 헤더 frame_id가 빈 JointState여도 팀 접두어가 있는 joint name으로 식별한다.

공식 입력은 `/team04/speed` Float32(m/s), `/team04/steering` Float32(rad, +우/-좌), `/race/start/team04` Bool(reliable/volatile/depth10)이다. 내부 `/team04/drive` Twist는 linear.x=속도, angular.z=가상 조향각(+좌)이다. **내부 Twist의 angular.z는 yaw rate가 아니다.** `/team04/gz/*`는 참가팀 전용 Gazebo Transport 인터페이스다.

정적 TF는 `team04/base_link`에서 lidar_link·imu_link·camera_link, 그리고 camera_optical_frame으로 발행한다. 대회 계측에 쓰이는 실제 SDF 루트 링크 이름은 별개로 `base_link`를 유지한다. 월드 pose→TF는 참가 노드에서 발행하지 않는다.

## 안전 동작

- 노드 시작·재시작은 항상 미허가 상태다.
- signal 모드는 true를 받은 뒤 최신 속도 명령만 통과한다.
- camera 모드는 빨강 2프레임 관찰→소등 3프레임 관찰과 중앙 true가 모두 필요하다. false 대기 중 빨강 관찰은 기억한다. true를 먼저 받아도 소등 전에는 정지한다.
- false는 즉시 0 명령을 발행하고 모든 속도 명령보다 우선한다. true→false 전환에서는 카메라 출발 이력을 초기화한다.
- NaN/Inf 명령 차단, 조향 제한, 속도 제한, 명령 0.5초 만료를 적용한다. 비정상 조향은 정상 조향을 다시 받을 때까지 주행을 차단한다.
- ROS 제어/Bridge 단절 시 차량 플러그인의 실시간 0.3초 watchdog이 목표 속도를 0으로 만든다. 실제 차량은 유한 토크로 제동하므로 정지 거리는 존재한다.
- 카메라 검출은 상단 25% ROI의 단순 빨강 픽셀 조건이다. 합성 영상에서 상태 전이를 검증하되, 트랙 실제 신호등의 크기·조명·붉은 물체에 대한 인식 정확도를 보증하지 않는다. 출전 전 실제 트랙 영상으로 ROI/검출을 검증해야 한다.

## 빌드와 실행

아래 명령은 URRC 저장소 루트 기준이다. 빌드 산출물도 팀 폴더 안에 둔다.

```bash
source /opt/ros/jazzy/setup.bash
colcon --log-base src/team_04/artifacts/stage4/log build \
  --base-paths src/team_04/team04_race \
  --build-base src/team_04/artifacts/stage4/build \
  --install-base src/team_04/artifacts/stage4/install --merge-install
source src/team_04/artifacts/stage4/install/setup.bash
```

대회 중앙 시스템이 이미 차량·월드·clock을 실행한 경우:

```bash
ros2 launch team04_race race.launch.py race_start_mode:=signal
# 또는
ros2 launch team04_race race.launch.py race_start_mode:=camera
```

필수 launch 인자 race_start_mode/race_start_topic/speed_topic/steering_topic/lidar_topic/use_sim_time 모두 실제 노드 설정에 전달된다. ROS_DOMAIN_ID, RMW_IMPLEMENTATION, GZ_PARTITION은 상속하며 코드에서 덮어쓰지 않는다. install setup은 Gazebo 모델·플러그인 검색 경로만 추가한다.

독립 개발 시험은 각기 새 터미널에서 setup을 source하고 다음 순서로 실행한다. 기존 대회 월드·신호등을 수정하지 않는 별도 평면 시험장이다.

```bash
# 터미널 1
TEAM04_SHARE="$(ros2 pkg prefix --share team04_race)"
gz sim -s -r --headless-rendering "$TEAM04_SHARE/config/test_world.sdf"
# 터미널 2: 독립 시험에만 clock Bridge 1개를 제공한다
ros2 run ros_gz_bridge parameter_bridge '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'
# 터미널 3
ros2 launch team04_race test_vehicle.launch.py
```

기존 실행 중인 트랙에 개발 차량을 스폰하는 경우 test_vehicle.launch.py에 world/x/y/z/yaw를 명시한다. 중앙이 스폰한 차량에 중복 스폰하지 않는다. 대회 launch에는 스폰 코드가 없다.

**아래 출발 토픽 발행은 별도 개발 시험장에만 사용한다. 대회에서는 중앙 시스템이 출발 허가를 제공한다.**

```bash
ros2 topic pub --once /race/start/team04 std_msgs/msg/Bool '{data: true}'
# 각각 별도 터미널에서 20Hz 연속 명령. 중단하면 0.5초 이내 목표 속도 0.
ros2 topic pub -r 20 /team04/speed std_msgs/msg/Float32 '{data: 0.5}'
ros2 topic pub -r 20 /team04/steering std_msgs/msg/Float32 '{data: 0.3}'
# +0.3 우회전 / -0.3 좌회전 / 속도 -0.4 후진
ros2 topic pub --once /race/start/team04 std_msgs/msg/Bool '{data: false}'
```

센서 확인:

```bash
ros2 topic hz /team04/points
ros2 topic echo /team04/points --once --field height
ros2 topic hz /team04/scan
ros2 topic echo /team04/scan --once
ros2 topic hz /team04/imu
ros2 topic echo /team04/imu --once
ros2 topic echo /team04/wheel_joint_states --once
ros2 topic hz /team04/camera/image_raw
ros2 run rqt_image_view rqt_image_view /team04/camera/image_raw
```

표시되는 실제 수신 Hz는 실시간 배율/렌더링 부하에 영향을 받는다. 검증 보고서는 센서 timestamp로 계산한 시뮬레이션 Hz와 벽시계 Hz를 구분한다.

## LiDAR 설정 변경

기본은 반드시 20Hz다. `config/sensors.yaml`의 lidar_hz를 변경한 다음 SDF를 재생성하고 다시 빌드한다. 중앙은 제출 SDF를 직접 읽으므로 ROS 파라미터만 바꾸면 센서 주기는 바뀌지 않는다.

```bash
python3 src/team_04/team04_race/scripts/generate_model.py
# 향후 테스트 전용: --lidar-hz 30 은 YAML과 SDF를 함께 바꾼다.
python3 src/team_04/team04_race/scripts/validate_dimensions.py
```

30/40/50Hz 성능은 이번 단계에서 검증하지 않았다. 센서 설정 변경 후 기존 차량을 제거하고 새로 스폰해야 한다.

## 검증 및 제출

```bash
python3 src/team_04/team04_race/tests/check_submission.py
# 실행 중인 다른 시험장을 종료한 뒤, 격리된 개발 시험장에서만 실행:
python3 src/team_04/team04_race/tests/runtime_probe.py --all \
  --output src/team_04/artifacts/runtime_acceptance.json
python3 src/team_04/team04_race/tests/runtime_probe.py --logic \
  --output src/team_04/artifacts/control_checks_final.json
# 원본 공식 pack_entry.py를 호출한다. 기존 출력이 있으면 덮어쓰지 않는다.
python3 src/team_04/team04_race/scripts/package_submission.py \
  --output src/team_04/artifacts/team04.zip
python3 src/team_04/team04_race/tests/check_submission.py \
  --zip src/team_04/artifacts/team04.zip
```

ZIP 최상위는 race_entry.json 및 src/team04_race/이고, build/install/log/캐시/트랙/운영 시스템을 포함하지 않는다. 새 환경에서는 ZIP을 빈 워크스페이스에 풀고 `colcon build --base-paths src --merge-install`한 뒤 install/setup.bash를 source한다. 표준 Jazzy·ros_gz와 Gazebo 개발 헤더가 사전 설치돼야 한다. 의존성은 package.xml에 명시했고 pip 의존성은 없다. 경기 중 설치·다운로드하지 않는다.

운영진은 ZIP을 원래 접수 절차의 race/inbox에 넣고 `bash competition.sh prepare`로 빌드한다. 이 작업에서는 원본 race 폴더에 접수·경기 결과를 생성하지 않는다. 제출 규격과 빌드 성공은 예선 완주·실격 규정 충족 인증과 별개다.

## 런타임 호환성 유의점과 다음 단계

현 환경의 Gazebo Sim 8.11.0 PosePublisher 바이너리는 `<topic>` 사용자 설정을 무시하고 `/model/team04/pose`로 발행한다. 기존 중앙 실행기는 `/model/team04/urrc_timing`을 기대하므로, 이 설치 버전 그대로는 중앙 랩 계측 호환성을 보증할 수 없다. 테스트 전용 복사본의 PosePublisher와 기본 토픽으로 실제 움직임을 관측했으며, 이 데이터는 참가 제어에 사용하지 않았다. 참가 패키지에서 중앙 계측 토픽을 대신 발행하지 않는다. 운영진은 사용자 지정 topic을 지원하는 Gazebo 버전/빌드를 준비하거나 중앙 계측 대응을 검토해야 한다. 원본 시스템 소스는 수정하지 않았다.

또한 `gz_frame_id`는 Gazebo 확장 태그여서 SDF 검사에서 알 수 없는 자식 태그 경고가 나오지만, 실제 센서 헤더에는 지정한 팀 프레임이 출력된다. EGL 드라이버 경고가 있었으나 실제 카메라/포인트클라우드 렌더링은 출력됐다.

실제 주행 결과와 남은 범위는 STAGE3.md 및 REPORT_STAGE3.md를 확인한다. 후속 과제는 반복/다팀 주행, 충돌 회피·추월, 운영진 환경 계측 확인과 성능 최적화다. IMU/엔코더 융합·레이싱 라인 최적화는 별도 개발 단계다.
