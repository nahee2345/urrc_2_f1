# URRC AI — 3단계 실행 설명서

**유일한 수정 대상:** nahee2345/urrc_2_f1. 원본 urrc_f1_ws와 urrc_2는 읽기 전용입니다.

## 구현 범위

- 실제 start_camera RGB 이미지에서 빨간불 5개 점등을 연속 관측하고, 이후 5개가 모두 소등되면 출발을 허가하는 판정기를 구현했습니다.
- 운영자 Bool 허가도 필요합니다. Bool만 true인 경우 출발하지 않습니다.
- 원본 몬자의 흰색 출발선 정방향 최초 통과는 0랩, 이후 유효 코스 재통과는 랩 +1입니다. 115개 체크포인트는 코스 단축 검증에 사용합니다.
- 접촉 감시, 원본 몬자 복사 월드, 운영자 전용 위치 데이터 및 ROS 2 브리지 설정을 구현했습니다.
- 실제 다섯 램프 감지 및 3바퀴 무충돌 주행은 아직 통과하지 않았습니다.

## 1. 최신 코드와 정적 검사

~~~bash
cd ~/urrc_2_f1
git pull --ff-only
git submodule update --init --recursive
python3 -m unittest discover -s tests -v
python3 scripts/check_upstream.py
python3 scripts/stage3_prepare_monza.py --ai AI-01
python3 scripts/stage3_bridge.py \
  --contacts generated/stage3/AI-01/contacts.json \
  --output generated/stage3/AI-01/bridge.yaml
~~~

생성 결과: generated/stage3/AI-01/monza_experiment.sdf, model_instrumented.sdf, contacts.json, gates_summary.json, bridge.yaml.

## 2. 실제 몬자 임시 월드 실행

ROS 2 Jazzy와 Gazebo Harmonic이 설치된 환경의 터미널 1:

~~~bash
cd ~/urrc_2_f1
source /opt/ros/jazzy/setup.bash
export GZ_PARTITION="urrc_stage3_$(id -u)"
export GZ_SIM_RESOURCE_PATH="$PWD/upstream/urrc_f1_ws/src/urrc_track_gazebo/models"
export SDF_PATH="$PWD/upstream/urrc_f1_ws/src/urrc_track_gazebo/models"
gz sim -r generated/stage3/AI-01/monza_experiment.sdf
~~~

터미널 2에서 ROS 브리지:

~~~bash
cd ~/urrc_2_f1
source /opt/ros/jazzy/setup.bash
export GZ_PARTITION="urrc_stage3_$(id -u)"
ros2 run ros_gz_bridge parameter_bridge \
  --ros-args -p config_file:=$PWD/generated/stage3/AI-01/bridge.yaml
~~~

### 출발 카메라 실제 프레임 저장

동일한 GZ_PARTITION을 적용한 별도 터미널에서 빨간불 5개를 켜고 영상 파일을 저장합니다.

~~~bash
cd ~/urrc_2_f1
export GZ_PARTITION="urrc_stage3_$(id -u)"
python3 upstream/urrc_f1_ws/race/scripts/lights.py on --track monza
~~~

다른 터미널에서:

~~~bash
cd ~/urrc_2_f1
source /opt/ros/jazzy/setup.bash
python3 scripts/stage3_capture_roi.py \
  --topic /model/urrc_ai_01/sensors/start_camera \
  --out generated/stage3/AI-01/start_camera.ppm
cp config/stage3_start_roi_template.json generated/stage3/AI-01/calibration.json
~~~

PPM 파일에서 실제 렌즈 5개의 경계를 측정하여 calibration.json의 rois 좌표를 0~1 정규화 값으로 수정합니다. 예시 좌표를 그대로 사용하면 안 됩니다. 실영상으로 검증한 뒤에만 calibrated=true를 설정합니다. 기본값 false이면 출발 노드는 안전하게 실행을 거부합니다.

## 3. ROS 안전 출발 게이트와 랩 감시

실제 ROS 브리지가 실행 중이어야 합니다.

출발 허가 게이트:

~~~bash
cd ~/urrc_2_f1
source /opt/ros/jazzy/setup.bash
python3 -m stage3.ros_gate \
  --vehicle urrc_ai_01 --entry-id urrc_ai_01 \
  --calibration generated/stage3/AI-01/calibration.json \
  --permission-topic /urrc_2_f1/operator_test_start \
  --ros-args -p use_sim_time:=true
~~~

운영자 랩 계측:

~~~bash
cd ~/urrc_2_f1
source /opt/ros/jazzy/setup.bash
python3 -m stage3.pose_monitor \
  --vehicle urrc_ai_01 --track monza \
  --contacts-manifest generated/stage3/AI-01/contacts.json \
  --output generated/stage3/AI-01/lap_events.jsonl \
  --ros-args -p use_sim_time:=true
~~~

격리 실험용 Bool 허가는 /urrc_2_f1/operator_test_start로 별도 발행하며, 공식 /race/start 토픽에 임의로 발행하지 않습니다. 원본 신호등 동작을 실험 환경에서 실행할 때는:

~~~bash
export GZ_PARTITION="urrc_stage3_$(id -u)"
python3 upstream/urrc_f1_ws/race/scripts/lights.py sequence --track monza --hold 1
~~~

차량 출발 허가 결과는 /urrc_2_f1/urrc_ai_01/start_allowed Bool로 발행합니다. 실제 모터를 구동하는 AI 알고리즘은 아직 없으므로 다음 단계에서 이 안전 인터록을 연결해야 합니다.

## 4. 검증 및 PASS 조건

- Python 모의 영상·위치 단위 시험: 44개 PASS.
- 원본 모델의 읽기 전용 복사 월드 생성: PASS.
- 실제 몬자 Gazebo 카메라 및 운영자 포즈 수신: GitHub Actions 결과 별도 확인.
- 진짜 신호등 5개 영상 판정 및 ROS2 통합·3바퀴 무충돌: 미검증.
- 모든 실제 접촉 센서 연결이 증명되지 않으면 무충돌 PASS를 내지 않습니다.

테스트 로그와 변경 이력: reports/STAGE_3.md
