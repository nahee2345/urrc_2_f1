# URRC AI — 7단계 중 2단계: 센서 장착 및 검증

## 변경 범위
- 수정 및 push 허용: **nahee2345/urrc_2_f1** 한 곳만.
- urrc_f1_ws 및 urrc_2는 **읽기 전용**. 기존 F1 맵·경기 규정·신호등은 변경하지 않는다.
- 1단계 동적 4륜 물리 차량을 기반으로 센서를 달며, 월드 좌표/정답 경로를 자율주행 알고리즘의 입력으로 사용하지 않는다.
- 대회 원본 LiDAR 전용 주행 규칙과 다른 연구용 카메라/Depth 주행 허용은 **이 실험 저장소에만** 적용한다.

## 10개 주행 센서 조합
| 차량 | 구동 | 주행용 센서 |
|---|---|---|
| AI-01 | FWD | RGB camera only |
| AI-02 | RWD | 3D GPU LiDAR only |
| AI-03 | AWD | Stereo pair (two synchronized-position RGB cameras) |
| AI-04 | FWD | Depth camera only |
| AI-05 | RWD | RGB + 3D GPU LiDAR |
| AI-06 | AWD | Depth + 3D GPU LiDAR |
| AI-07 | FWD | RGB + Depth |
| AI-08 | RWD | RGB + 2D GPU LiDAR + 3D GPU LiDAR |
| AI-09 | AWD | Stereo + 3D GPU LiDAR |
| AI-10 | AWD | RGB + Depth + 3D GPU LiDAR |

**모든 차량:** 5개 빨간불 소등 감지용 start_camera를 공통 장착. 이 카메라는 AI-02 등의 주행 센서에 포함하지 않음. 신호등 인식 및 출발 차단 제어 알고리즘은 별도 단계에서 구현·검증.

## 생성 내용
- RGB: SDF camera, 160×120, 15Hz.
- Start camera: SDF camera, 160×120, 15Hz, 위쪽 방향으로 기울인 설치 자세.
- Depth: SDF depth_camera, R_FLOAT32, 160×120, 10Hz.
- Stereo: 서로 다른 물리적 설치 위치에 2개의 SDF camera, 각각 독립 Gazebo 토픽. 양안 거리 계산 알고리즘은 별도 개발.
- LiDAR 2D: gpu_lidar, 360 horizontal samples, 1 vertical beam, 10Hz.
- LiDAR 3D: gpu_lidar, 360 horizontal samples, 16 vertical beams, 10Hz.
- 센서마다 실물 모형에 해당하는 visual/collision 하우징을 추가하고 센서 질량·추가 관성 및 합성 무게중심을 base_link inertial에 포함.
- 장착 후 조향각 35도에서 전체 외접 크기가 길이 0.55m, 폭 0.25m, 높이 0.18m 이내인지 자동 검증.
- GPS, 월드 pose, 센서 정답값을 차량 주행 토픽으로 제공하지 않음.

## Ubuntu 설치된 코드 업데이트 및 생성
기존에 urrc_2_f1을 이미 clone 했을 경우:
~~~bash
cd ~/urrc_2_f1
git pull --ff-only
git submodule update --init --recursive
python3 -m unittest discover -s tests -v
python3 scripts/build_sensors.py --ai all
~~~

하나만 생성하려면:
~~~bash
python3 scripts/build_sensors.py --ai AI-05
cat generated/stage2/AI-05/manifest.json
~~~

AI-05 SDF: generated/stage2/AI-05/model.sdf

## 원본 맵을 변경하지 않는 센서 실제 출력 시험
~~~bash
cd ~/urrc_2_f1
source /opt/ros/jazzy/setup.bash
python3 scripts/stage2_sensor_smoke.py --prepare --ai AI-05
python3 scripts/stage2_sensor_smoke.py --run --ai AI-05
cat generated/stage2_sensor_result.json
~~~

- --prepare: 로컬에서 독립 평면 월드 파일만 생성. PASS로 해석하지 않음.
- --run: 실제 Gazebo Harmonic 서버에 Sensors(Ogre2)를 실행하고 start_camera, rgb, lidar3d의 토픽 목록/실제 payload 검사.
- 결과가 PASS_SENSOR_STREAMS이면 **독립 평면 센서의 송신·수신만** 통과.
- EGL/OpenGL 드라이버/헤드리스 렌더링 불가 시 FAIL_SERVER 또는 FAIL_STREAM; 로그에 원인 보존.
- 원본 Monza, 운전용 ROS bridge 실연결, 출발 신호 감지, 3랩 주행은 이 단계의 자동 테스트가 아님.

## ROS 2 Jazzy로 센서 이미지/스캔 가져오기 (별도 터미널)
Gazebo 월드가 실행되고 센서가 송신 중인 상태에서:
~~~bash
source /opt/ros/jazzy/setup.bash
python3 scripts/bridge_sensors.py \
  --manifest generated/stage2/AI-05/manifest.json \
  --output generated/stage2/AI-05/ros_bridge.yaml
ros2 run ros_gz_bridge parameter_bridge \
  --ros-args -p config_file:=$PWD/generated/stage2/AI-05/ros_bridge.yaml
~~~

생성 YAML은 Gazebo -> ROS 방향으로만 설정됩니다. 이미지: sensor_msgs/msg/Image, LiDAR 스캔: sensor_msgs/msg/LaserScan.
2D LaserScan과 달리 실제 3D 점군은 PointCloud2가 필요합니다. Gazebo `gz topic -l`에서 PointCloudPacked /points 토픽이 있는지 확인한 뒤에만 --include-points로 선택적 브리지 항목을 생성합니다. **3D PointCloud2 검증을 완료하지 않은 상태에서 검증 완료로 기록하지 않습니다.**

## GitHub Actions
https://github.com/nahee2345/urrc_2_f1/actions
- Stage 1: 기존 동역학 회귀검사
- Stage 2: 차량 10대 SDF / 정적 검사 및 Gazebo 물리·센서 오프스크린 데이터 발행 검증
- 생성 모델 및 결과 JSON은 run artifact로 저장

## 검증 승인
1. 10개 독립 SDF 생성/규격/센서 구성 정적 검증.
2. Gazebo Harmonic SDF parser에서 10개 모델 로드.
3. Gazebo headless에서 실제 카메라, 3D LiDAR 데이터 확인.
4. GPU/OS별 렌더링 편차는 별도 기록하고 GPU가 있는 실험용 Ubuntu에서도 검증.
5. 아직 AI 자율주행 PASS 또는 출발 감지 완료로 간주하지 않음.
