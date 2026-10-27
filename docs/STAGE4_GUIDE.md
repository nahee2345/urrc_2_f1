# 4단계 AI-01 카메라 주행 안내

## 기능

- 검정 차체 SDF 및 실제 RGB 주행 카메라 기반 도로 중심 검출
- 전륜구동(FWD) + Ackermann 조향
- 3단계 5등 출발 허가 전에는 구동 속도 0
- 카메라/출발 신호 중단 또는 도로 인식 실패 시 목표 속도 0
- 원본 몬자 고저차·충돌 메시 보존
- 주행 감지기는 현재 아스팔트 색상 기반이며, 학습된 CNN은 아직 아닙니다.

## 추가 다운로드 없이 기존 저장소 업데이트

```bash
cd ~/urrc_2_f1
git pull --ff-only
git submodule update --init --recursive
python3 -m unittest discover -s tests -v
python3 scripts/stage3_prepare_monza.py --ai AI-01
python3 scripts/stage4_bridge.py --contacts generated/stage3/AI-01/contacts.json --output generated/stage4/ai01_bridge.yaml
```

**필수 전제:** 3단계에서 실제 5개 신호등의 카메라 위치를 교정한 generated/stage3/AI-01/calibration.json 파일이 있어야 합니다. calibrated=false 예시 설정은 안전상 실행을 거부합니다.

## 터미널별 실행

1. 원본 몬자+AI-01 차량:

```bash
cd ~/urrc_2_f1
source /opt/ros/jazzy/setup.bash
python3 scripts/stage3_run_monza.py --ai AI-01
```

2. ROS2↔Gazebo 카메라와 제어 브리지:

```bash
cd ~/urrc_2_f1
source /opt/ros/jazzy/setup.bash
export GZ_PARTITION="urrc_stage3_$(id -u)"
ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:=$PWD/generated/stage4/ai01_bridge.yaml
```

3. 실제 카메라 신호등 출발 안전 노드:

```bash
cd ~/urrc_2_f1
source /opt/ros/jazzy/setup.bash
python3 -m stage3.ros_gate --vehicle urrc_ai_01 --entry-id urrc_ai_01 --calibration generated/stage3/AI-01/calibration.json --permission-topic /urrc_2_f1/operator_test_start --ros-args -p use_sim_time:=true
```

4. RGB 주행 제어기(처음에는 0.55m/s 제한):

```bash
cd ~/urrc_2_f1
source /opt/ros/jazzy/setup.bash
python3 -m stage4.ros_drive --vehicle urrc_ai_01 --speed-limit 0.55 --ros-args -p use_sim_time:=true
```

5. 출발 허가 상태 관측:

```bash
source /opt/ros/jazzy/setup.bash
ros2 topic echo /urrc_2_f1/urrc_ai_01/start_allowed
```

6. 별도 터미널에서 운영자 시험용 허가:

```bash
source /opt/ros/jazzy/setup.bash
ros2 topic pub -r 10 /urrc_2_f1/operator_test_start std_msgs/msg/Bool "{data: true}"
```

7. 별도 터미널에서 실제 원본 신호등 시퀀스:

```bash
cd ~/urrc_2_f1
python3 scripts/stage3_lights.py sequence
```

**통과 기준:** 출발 전 정지, 5등 동시 소등을 카메라가 확인한 뒤에만 출발 허가, 도로가 보일 때만 조향·주행. 운영자 권한을 끄거나 카메라가 끊기면 목표 바퀴 속도 0.

**미검증:** 실제 ROS2 끝단 통합·실제 RGB 추종 주행, 실제 벽 접촉·무충돌 3랩, 학습된 CNN. 물리 구동 스모크는 실제 자율주행 완주를 대체하지 않습니다. 원본 두 GitHub 저장소는 변경하지 않았습니다.
