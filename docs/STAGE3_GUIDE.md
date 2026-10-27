# URRC 3단계 — 원본 몬자 복원 후 실제 출발 검증

## 확인된 원인

원본 urrc_f1_ws main 커밋은 기존에 고정한 a23c7eec41b98b67ec3a3a2a46f9d7ed957f8290과 동일합니다. 원본 monza_geometry_report.json에는 고저차 약 2.563m가 기록되어 있습니다.

이전 urrc_2_f1은 원본 monza_race.sdf를 단순 복사해 Gazebo에서 열었지만, **공식 run_world.py가 시작 때 수행하는 enhance_track을 거치지 않았습니다.** 따라서 도로 아스팔트 텍스처, 지면 재질, 4개 관전 카메라 및 5개 점등→단일 소등 모델이 원본 실행 화면과 달랐습니다. 기본 GUI 카메라 시점도 달랐습니다.

이제 원본 전체 맵을 임의 재설계하지 않고, 복사본 generated/stage3/resources/urrc_track_gazebo에 원본 향상 리소스를 생성합니다. 원본 충돌 OBJ와 중심선 CSV는 SHA256으로 동일함을 확인하며, 원본/복사본 OBJ의 최소·최대 Z와 정점수를 비교하여 평탄화되지 않았음을 검증합니다.

**원본 urrc_f1_ws, urrc_2 GitHub는 어떤 수정이나 push도 하지 않습니다.**

## 업데이트 방식

기존 clone이 있으므로 다운로드/ZIP 없이 변경분만 받습니다.

~~~bash
cd ~/urrc_2_f1
git pull --ff-only
git submodule update --init --recursive
source /opt/ros/jazzy/setup.bash
python3 -m unittest discover -s tests -v
python3 scripts/stage3_prepare_monza.py --ai AI-01
cat generated/stage3/AI-01/terrain_fidelity.json
~~~

PASS_ORIGINAL_MONZA_TERRAIN과 고저차가 0보다 충분히 큰 값을 확인합니다. enhanced_road의 height_range_m이 원본과 동일해야 합니다.

## 터미널 1: 공식 GUI 시점·원본 지형으로 몬자 실행

~~~bash
cd ~/urrc_2_f1
source /opt/ros/jazzy/setup.bash
python3 scripts/stage3_run_monza.py --ai AI-01
~~~

이 스크립트가 GZ_PARTITION 및 원본 복사 리소스 경로를 자동 설정하고, 원본 monza_grid.gui.config로 화면을 엽니다.

다른 Gazebo 인스턴스가 실행 중이면 닫고 다시 실행하세요. 새로운 트랙을 만드는 것이 아니라 공식 원본의 향상 리소스를 반영한 실험용 월드입니다.

## 터미널 2: ROS 2 센서 브리지

~~~bash
cd ~/urrc_2_f1
source /opt/ros/jazzy/setup.bash
export GZ_PARTITION="urrc_stage3_$(id -u)"
python3 scripts/stage3_bridge.py --contacts generated/stage3/AI-01/contacts.json --output generated/stage3/AI-01/bridge.yaml
ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:=$PWD/generated/stage3/AI-01/bridge.yaml
~~~

별도 터미널로 확인:

~~~bash
source /opt/ros/jazzy/setup.bash
ros2 topic hz /model/urrc_ai_01/sensors/start_camera
ros2 topic hz /model/urrc_ai_01/urrc_timing
~~~

## 터미널 3: 원본 v10.8 방식으로 빨간불 5개 모두 켜기

~~~bash
cd ~/urrc_2_f1
python3 scripts/stage3_lights.py on
~~~

이 명령은 5개 빨간 렌즈의 공식 v10.8 격리 모델을 켭니다. 이전 문서에 있던 race/scripts/lights.py는 레거시 방식으로, 실험에서 사용하지 않습니다.

## 터미널 4: 실제 차량 카메라 화면 검사

~~~bash
cd ~/urrc_2_f1
source /opt/ros/jazzy/setup.bash
python3 scripts/stage3_capture_roi.py --topic /model/urrc_ai_01/sensors/start_camera --out generated/stage3/AI-01/start_camera.ppm
xdg-open generated/stage3/AI-01/start_camera.ppm
~~~

신호등 렌즈 5개가 화면에 모두 보이고 정확한 영역을 식별할 수 있는지 확인해 스크린샷을 채팅에 알려 주세요. calibrated=false 샘플을 임의로 활성화하지 마세요.

## 실제 순차 점등 후 동시에 소등

~~~bash
cd ~/urrc_2_f1
python3 scripts/stage3_lights.py off
python3 scripts/stage3_lights.py sequence
~~~

원본 v10.8의 랜덤 3~6초 목표 범위로 빨간불이 하나씩 켜지고 단일 모델 제거 요청으로 한꺼번에 꺼집니다. 실제 렌더링 및 차량의 이미지 인식은 교정 이후 검증해야 합니다. GitHub Actions는 Gazebo 서비스 호출 및 카메라·위치 메시지 연결을 별도 검사합니다.

## 아직 PASS 아닌 부분

- 카메라가 **실제 픽셀**로 5개 점등을 확인하고 완전 소등 후에만 start_allowed=true를 내는지.
- ROS 2 이미지·접촉 브리지의 실제 동작 및 벽 접촉 이벤트.
- AI-01 주행 제어기: 아직 없으므로 신호 후 전진과 3바퀴 완주는 실행할 수 없습니다.

단위 검사 PASS와 실제 경기 3바퀴 PASS는 별도 판정합니다.
