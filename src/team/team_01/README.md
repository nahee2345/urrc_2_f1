# team_01 대회 제출 폴더

이 폴더 하나를 운영 환경의 `src/team/team_01/`에 복사합니다. 폴더명은 반드시 `team_01`을 유지합니다. 팀 소스는 `src/team_01_race`, Python 모듈과 ROS 패키지명은 `team_01_race`입니다. 차량 모델·RGB-D 카메라·LiDAR·IMU 설정·주행 알고리즘·Launch·설정·검사 자료가 모두 이 폴더 안에 있습니다. 현재 차량은 기본 기하 형상으로 만들어져 외부 메시/텍스처 다운로드가 필요 없습니다.

## 운영 시스템에서 실행

`race_entry.json`의 `algorithm_launch_file`은 `competition.launch.py`입니다. 운영 시스템이 월드, 차량 스폰, 그리드 배치를 담당합니다. 팀 진입점은 센서·액추에이터 브리지, TF 및 알고리즘만 실행하며 월드나 차량을 생성하지 않습니다. 숫자 1/2/3은 아래의 별도 연습 모드이며, 대회 진입점에서는 이 연습 실행기를 호출하지 않습니다.

운영 시스템이 이 패키지를 빌드·환경 설정한 후 실행하는 명령:

```bash
ros2 launch team_01_race competition.launch.py
```

`race_start_mode:=camera`가 기본값입니다. 차량 카메라로 빨간등 5개를 확인하고 모두 소등된 뒤에만 출발합니다. 운영자가 `race_start_mode:=signal`을 지정하면 카메라 조건과 `/race/start/team_01`의 출발 허가를 모두 요구합니다. 두 방식 모두 카메라 소등 인식을 생략하지 않습니다.

차량이 아직 없어도 노드/출발 구독을 먼저 준비하므로 운영자의 '알고리즘 시작 → 차량 스폰 → 카메라 준비 → 신호등' 순서를 지원합니다. 월드 이름/차량 위치/맵 중심선 파일은 주행 알고리즘 입력이 아닙니다. Gazebo 파티션·ROS 도메인은 운영 환경을 그대로 상속하며, 특정 PC/운영자 소스 파일/개인 맵 파일을 읽지 않습니다.

## 공통 시스템 의존성 및 빌드

Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic 환경을 사용합니다. 공통 ROS/Gazebo 라이브러리는 `package.xml`에 선언했습니다. 카메라·GPU LiDAR 사용을 위해 운영 월드의 Sensors/Ogre2 시스템이 필요하며, IMU 입력에는 운영 월드의 IMU 시스템이 필요합니다. IMU가 없거나 지연되면 기존 안전 속도 제한이 적용됩니다.

표준 의존성 설치 예:

```bash
sudo apt install ros-jazzy-ros-gz ros-jazzy-vision-msgs ros-jazzy-tf2-ros \
  ros-jazzy-ament-cmake-python python3-colcon-common-extensions \
  python3-numpy python3-opencv python3-yaml python3-rosdep
rosdep install --from-paths src --ignore-src --rosdistro jazzy -y
bash build.sh
source install/local_setup.bash
```

공통 운영 workspace의 `colcon build`로 빌드해도 됩니다. 팀 루트에 COLCON_IGNORE를 두지 않습니다. 배포본에는 build/install/log/개인 캐시/외부 심볼릭 링크를 넣지 않습니다. 다른 위치에 복사해 자체 빌드할 때는 `build.sh`가 이전 위치의 빌드 캐시를 분리하고 다시 구성합니다.

## 수동으로 연결할 경우

운영자가 월드와 차량을 이미 생성했고 알고리즘을 아직 실행하지 않았을 때만 아래 중 한 방식을 선택합니다.

```bash
# 브리지·TF·전체 알고리즘 한 번에 연결 (스폰 없음)
bash run.sh competition
```

또는 서로 다른 터미널에서:

```bash
# 운영 차량 센서/액추에이터 브리지·TF만 연결 (스폰 없음)
bash run.sh sensors
# 전체 알고리즘만 연결
bash run.sh autonomy
```

운영자가 competition.launch.py를 실행한 상태에서 위 수동 실행을 중복으로 켜지 마십시오. 숫자 1/2/3은 별도 연습 세션을 생성하므로 대회 실행과 함께 사용하지 않습니다.

## 세 터미널 연습 실행

같은 `team_01` 폴더에서 터미널 세 개를 엽니다. 매번 partition·월드·ROS domain을 입력할 필요 없이1번이 만든 연습 연결을2·3번이 공유합니다. 실행기는 자신의 폴더를 기준으로 동작하므로 다른 PC·다른 경로에서도 복사 후 빌드해 사용할 수 있습니다.

```bash
# 터미널1: 경기장만 생성 (기본 Monza)
bash run.sh 1
# 다른 맵 선택: bash run.sh 1 monaco
#                bash run.sh 1 hungaroring
#                bash run.sh 1 random
```

```bash
# 터미널2: team_01 차량 생성 + 실제 센서/액추에이터 브리지 + TF
bash run.sh 2
```

```bash
# 터미널3: 전체 자율주행 알고리즘
bash run.sh 3
```

4~9번 토픽 보기 역시 실행 중인 연습 세션의 ROS domain에 자동 연결합니다.

2·3번이 준비되면 **1번 터미널에 READY를 입력하고 Enter**를 누릅니다. 실제 카메라 영상·신호 인식 노드·제어 준비를 확인한 뒤 빨간등1~5개가 켜지고 전체 소등됩니다. 차량은 카메라로 소등을 확인한 뒤 출발합니다.

2번은 이미 존재하는 team_01 차량을 재사용하며, 같은2번/3번 중복 실행은 차단합니다. 2번이나3번을 Ctrl+C로 끝내면 경기장을 일시정지하고 바퀴 목표를0으로 보냅니다. 2번을 다시 실행하면 같은 차량의 센서를 다시 연결합니다. 3번을 새로 시작하면 카메라 출발 인식도 초기화되므로 새 출발은1번부터 새 세션으로 시작하세요. 1번 Ctrl+C는 이 연습 세션의 팀 노드와 Gazebo 서버를 함께 종료하며 다른 세션의 프로세스를 종료하지 않습니다.

연습1번은 **공통 맵 리소스**가 필요합니다. 설치된 urrc_track_gazebo 패키지 또는 현재 workspace의 src/urrc_track_gazebo를 찾습니다. 맵 폴더가 다른 곳에 있다면1번에만 지정합니다.

```bash
URRC_MAP_PACKAGE=/path/to/urrc_track_gazebo bash run.sh 1 monaco
# 임의 외부 맵: 실제 월드 이름은 파일 안에서 읽습니다.
bash run.sh 1 --map-file /path/to/map.sdf --spawn X Y ROAD_Z YAW
# 출발 위치 metadata가 있는 경우:
bash run.sh 1 --map-file /path/to/map.sdf --grid-file /path/to/grid.json
```

지도 파일과 출발 metadata는 운영 환경의 공통 리소스이며 ZIP에는 맵을 포함하지 않습니다. 출발 위치는 연습 차량 생성에만 사용하고 주행 알고리즘에 정답 경로/위치를 전달하지 않습니다. 외부 맵에 기본5개 신호등이 없다면 해당 맵 제공자의 실제 출발 신호가 필요합니다. 대회에서는 competition.launch.py가 계속 운영자의 차량/월드에만 연결합니다.

## 토픽과 시각

| 용도 | ROS 토픽 | Gazebo 원본 |
|---|---|---|
| LiDAR | /team_01/scan | /team_01/lidar |
| RGB 영상 | /team_01/camera/image_raw | /team_01/camera/image |
| 카메라 정보 | /team_01/camera/camera_info | /team_01/camera/camera_info |
| Depth | /team_01/camera/depth_image | /team_01/camera/depth_image |
| IMU | /team_01/imu | /team_01/imu |
| Odometry | /team_01/odometry | /team_01/odometry |
| 속도 | /team_01/speed | 내부 제어 어댑터 → /team_01/actuator/* |
| 조향 | /team_01/steering | 내부 제어 어댑터 → /team_01/actuator/* |
| 운영자 출발 허가 | /race/start/team_01 | ROS Bool |

센서 브리지는 SENSOR_DATA QoS, 영상·CameraInfo·LiDAR·IMU·Odom 구독은 BEST_EFFORT/VOLATILE을 사용합니다. 프레임 ID는 `team_01/base_link`, `team_01/laser_frame`, `team_01/camera_optical_frame`, `team_01/imu_link`입니다. `/tf`와 `/tf_static`은 ROS 공통 토픽이지만 프레임은 팀 이름으로 구분됩니다.

공통 `/clock`을 재발행하지 않습니다. 현재 Gazebo 월드의 `/world/<world>/clock` 하나를 `/team_01/clock`으로 연결하고 팀 노드의 `/clock`을 해당 토픽으로 remap합니다. 운영자가 이미 팀 시계를 연결한 경우 `bridge_clock:=false`로 실행합니다. 원본 센서 stamp를 유지하며 LiDAR 경로·벽 분류·장애물은 같은 스캔 단위로 동기화합니다.

속도 상한은 직선 12m/s, 코너 10m/s, 우회 2m/s입니다. 관측거리·신뢰도·조향·제동·TTC·센서 timeout 등 기존 안전 제한은 유지됩니다. 설정 상한에 항상 도달한다는 의미는 아닙니다.

직선·코너 속도 설정과 감속 원인
--------------------------------
기본 `speed_profile_standard.yaml`은 직선 12m/s, 코너 10m/s, 가속도·일반/코너 제어 감속도 3.5m/s²입니다. 정지 상태에서 12m/s까지 가속하는 데 이상적으로 약 3.43초·20.6m가 필요합니다. 제동거리 및 조향 안정성 예측에는 보수적인 2.5m/s²를 계속 사용합니다. 상한은 모든 구간에서 보장되는 속도가 아닙니다. 실제 목표는 경로 신뢰도·관측거리·장애물·조향·IMU 제한 중 가장 낮은 값입니다.

Depth의 처리 범위는 32m이고 센서 far clip은 50m입니다. 실제 관측된 연속 벽만 속도 판단에 사용하며, 설정 범위를 늘려도 관측되지 않은 공간을 채우지 않습니다. RGB 보조 preview도 고정 10m/s 대신 설정 상한과 실제 관측거리의 제동 한계를 함께 적용합니다.

* `IMU_STALE`: IMU가 없거나 0.25초보다 오래되면 5m/s. 운영 월드에 아래 표준 시스템이 필요합니다. 차량 모델에 중복 추가하지 않습니다.
* `PATH_CURVATURE` / `LATERAL_ACCELERATION`: 횡가속도 3m/s² 기준. 10m/s에는 약 33.3m 이상의 회전 반경이 필요합니다. 급코너는 10보다 낮을 수 있습니다.
* `VISIBLE_PATH_LENGTH` / `SUPPORTED_PREVIEW`: 실제 관측거리 부족. 제동 감속도 2.5m/s²에서 12m/s 정지거리는 약 28.8m이며 여유 거리도 필요합니다.
* `DEPTH_UNAVAILABLE`: Depth 유실·저신뢰 상태의 기본 제한 3.5m/s.
* `PATH_CONFIDENCE` / `PATH_SOURCE`: 저신뢰 또는 한쪽 벽 경로의 제한.
* `acceleration_limited=true`: 목표는 높지만 아직 가속 중입니다.

운영 월드의 `<world>` 아래 공통 시스템(팀 노드는 월드·차량을 생성하지 않습니다):

```xml
<plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu" />
```

현재 제한 확인:

```bash
ros2 topic echo /team_01/debug/speed_control --field data
ros2 topic hz /team_01/imu
```

`decision_target_speed`는 주행 판단 목표, `final_target_speed`는 조향 제한 적용 후 목표, `final_speed`는 가속 제한 적용 후 명령, `actual_speed`는 실제 Odom 속도입니다. `active_limiter`와 함께 비교합니다. IMU가 끊기거나 실제 장애물·센서 timeout이 발생했을 때의 감속·정지는 유지합니다.

경사로 정지 유지
----------------
IMU pitch가 5° 이상으로 3프레임 확인되고 정지 명령 및 최종 속도 명령이 0이면 경사 중력을 상쇄하는 유지 토크를 적용합니다. 실제 속도가 0이어도 출발 목표가 양수라면 브레이크를 걸지 않습니다. 출발 명령 시 보상을 해제하며, 평지 복귀는 기존 3°/3프레임 이력 조건을 따릅니다. 정지 유지 도중 IMU가 끊기면 마지막 유효 경사각의 보상은 유지합니다.

기존 Gazebo 후륜 제어기는 `use_force_commands=true`인 P 토크 제어기입니다. SDF에서 총질량·휠 반경·P gain·토크 한계를 읽고, 바퀴당 `-m*g*r*sin(pitch)/2`의 보상 토크를 P 제어기의 목표 오프셋으로 적용합니다. `/team_01/speed`와 최종 주행 명령은 0이며, 내부 휠 목표 오프셋은 정지 토크를 만들기 위한 값입니다. 별도 힘 발행자가 기존 바퀴 제어기와 충돌하거나 이상적인 속도 고정을 사용하지 않습니다.

```bash
ros2 topic echo /team_01/debug/ramp_brake --field data
```

`active`, `imu_fresh`, `holding_last_pitch`, `commanded_torque_nm_per_wheel`, `saturated`를 확인합니다. 토크는 보상 요구량이며 실제 측정 토크가 아닙니다. 보상은 현재 바퀴당 0.22N·m 한계를 넘지 않습니다. 현재 질량에서는 약 28°를 넘는 경사에서 정적 중력 보상만으로도 한계에 도달할 수 있으며, 포화 시 경고를 출력합니다. 타이어 접지와 토크 한계 때문에 임의의 급경사에서 완전 고정을 보장하지 않습니다.

Depth 처리 지연 최적화: 벽 높이/전방 범위 밖 점을 먼저 제외하고, 전방 구간 검색과 각도별 중앙값 계산을 묶어서 처리합니다. 원본 관측 점과 벽/경로 판정 기준은 유지합니다. 처리 범위32m, 기본 가속도3.5m/s², 속도12/10m/s 상한, 제동·센서 timeout, 경사 정지 유지 토크를 바꾸지 않습니다. 이 최적화는 늦은 Depth 때문에 발생하는 감속을 줄이기 위한 것이며, 관측거리 부족이나 실제 급코너의 물리적 감속 한계를 없애지 않습니다.

가감속 3.5 시험 설정: 기본 profile에서 `control.max_accel_mps2`, `control.max_decel_mps2`, `corner_control.corner_approach_decel_mps2`를 3.5로 사용합니다. 코너 접근 명령은 기존4.0에서 이번 목표3.5로 통일했습니다. `safety.assumed_max_decel_mps2`는2.5이며, Odom 유실 시 잔여 속도와 yaw 위험 예측도 이 보수적인 기준을 사용합니다. 안전 정지/확인된 위험 감속은 기존처럼 명령을 즉시 낮출 수 있습니다. 실제 가감속은 토크·접지·경사에 따라 명령보다 작을 수 있습니다.

후륜 구동·제동 토크 한계는 바퀴당 ±0.22N·m입니다(기존±0.17). P gain0.1, 질량·치수·관성·마찰·조향 한계는 유지합니다. 이 값은 토크의 상한이며 항상0.22를 적용하지 않습니다. 속도 명령은 3.5m/s² 가감속 설정과 기존 안전 제한을 계속 따릅니다. 경사 정지 유지 보상도 모델에서 새 토크 한계를 읽습니다.

다른 PC·다른 폴더·다른 맵 연결
--------------------------------
PC마다 `team_01` 폴더를 복사하고 그 위치에서 `bash build.sh`를 실행합니다. 이전 PC의 build/install/log를 복사해 재사용하지 않습니다. 실행기는 자신의 파일 위치와 ROS 패키지 share 경로를 사용합니다.

따로 실행한 맵과 연결할 때는 양쪽의 Gazebo 연결 이름을 같게 지정합니다. 이름은 PC 경로나 사용자명과 무관합니다. 예를 들어 운영 폴더의 맵 미리보기:

```bash
bash race_v24.sh preview qualifying --track monza --partition urrc_manual
```

팀 폴더의 다른 터미널:

```bash
bash run.sh sensors gz_partition:=urrc_manual
```

센서 실행기는 해당 partition에서 월드 이름을 자동으로 찾습니다. 월드 이름이 monza가 아니어도 됩니다. 같은 partition에 월드가 여러 개라면 실제 SDF의 이름을 `world_name:=이름`으로 추가합니다. 서로 다른 partition을 자동으로 섞어서 `/clock`을 연결하지 않습니다.

임의의 외부 맵 파일을 Gazebo로 직접 실행할 때도 같은 규칙입니다. 해당 맵의 모델 리소스/표준 패키지는 맵 제공 환경에서 설정합니다.

```bash
# 맵 터미널: 실제 맵 경로로 바꾸세요.
GZ_PARTITION=urrc_manual gz sim -r /path/to/your_map.sdf
# 팀 폴더 터미널
bash run.sh sensors gz_partition:=urrc_manual
```

운영자가 월드·차량을 생성한 뒤 알고리즘을 아직 실행하지 않았다면 센서와 알고리즘을 한 번에 연결할 수도 있습니다.

```bash
bash run.sh competition gz_partition:=urrc_manual
```

이 명령은 차량을 스폰하지 않습니다. 맵만 띄웠다면 차량 센서가 생기지 않으므로, 차량은 운영 시스템이 생성해야 합니다. 공식 운영 실행에서는 `gz_partition`을 생략하면 기존 운영 환경의 `GZ_PARTITION`을 상속합니다. 네트워크로 다른 PC에 접속하는 기능이 아니라 각 PC에서 독립 실행하는 연결 설정입니다.
