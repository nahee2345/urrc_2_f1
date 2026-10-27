# 3단계 — 사용자 Ubuntu 실제 출발 검증

## 다운로드 정책
- 별도 ZIP·첨부파일·새 clone 없이 기존 ~/urrc_2_f1에서 git pull --ff-only만 실행.
- 원본 urrc_f1_ws 및 urrc_2 GitHub에는 push 금지. 변경은 urrc_2_f1 main에만 올림.

~~~bash
cd ~/urrc_2_f1
git pull --ff-only
git submodule update --init --recursive
source /opt/ros/jazzy/setup.bash
python3 -m unittest discover -s tests -v
python3 scripts/stage3_prepare_monza.py --ai AI-01
python3 scripts/stage3_bridge.py --contacts generated/stage3/AI-01/contacts.json --output generated/stage3/AI-01/bridge.yaml
~~~

## A. 몬자 화면 실행(터미널 1)

~~~bash
cd ~/urrc_2_f1
source /opt/ros/jazzy/setup.bash
export GZ_PARTITION="urrc_stage3_$(id -u)"
export GZ_SIM_RESOURCE_PATH="$PWD/upstream/urrc_f1_ws/src/urrc_track_gazebo/models"
export SDF_PATH="$PWD/upstream/urrc_f1_ws/src/urrc_track_gazebo/models"
gz sim -r generated/stage3/AI-01/monza_experiment.sdf
~~~

출발 신호등과 차량이 보이는지 확인. 보이지 않으면 화면 캡처 또는 오류 로그를 알려 주세요.

## B. ROS 2 센서 브리지(터미널 2)

~~~bash
cd ~/urrc_2_f1
source /opt/ros/jazzy/setup.bash
export GZ_PARTITION="urrc_stage3_$(id -u)"
ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:=$PWD/generated/stage3/AI-01/bridge.yaml
~~~

별도 터미널에서 영상 주파수 확인:

~~~bash
source /opt/ros/jazzy/setup.bash
ros2 topic hz /model/urrc_ai_01/sensors/start_camera
~~~

주파수가 표시되지 않으면 브리지 실행 로그를 공유해 주세요.

## C. 빨간불 5개 켜기 및 카메라 화면 확인(터미널 3과 4)

~~~bash
cd ~/urrc_2_f1
export GZ_PARTITION="urrc_stage3_$(id -u)"
python3 upstream/urrc_f1_ws/race/scripts/lights.py on --track monza
~~~

다른 터미널:

~~~bash
cd ~/urrc_2_f1
source /opt/ros/jazzy/setup.bash
python3 scripts/stage3_capture_roi.py --topic /model/urrc_ai_01/sensors/start_camera --out generated/stage3/AI-01/start_camera.ppm
xdg-open generated/stage3/AI-01/start_camera.ppm
~~~

이미지에서 빨간 렌즈 5개가 보이는지 확인하고 해당 카메라 화면 스크린샷을 채팅에 전달해 주세요. 필요하면 urrc_2_f1의 카메라 각도와 판정 영역 설정을 변경해 push합니다.

주의: config/stage3_start_roi_template.json에는 검증되지 않은 예시 영역이 들어 있고 calibrated=false입니다. 실제 화면의 렌즈 영역을 교정하기 전에는 true로 변경하지 않습니다.

## D. 출발 허가 시험(카메라 교정 후에만)

먼저 기존 5개 점등 상태를 해제합니다.

~~~bash
cd ~/urrc_2_f1
export GZ_PARTITION="urrc_stage3_$(id -u)"
python3 upstream/urrc_f1_ws/race/scripts/lights.py off --track monza
~~~

교정된 설정 파일이 생성된 뒤 다음 출발 안전 노드를 실행합니다.

~~~bash
cd ~/urrc_2_f1
source /opt/ros/jazzy/setup.bash
python3 -m stage3.ros_gate --vehicle urrc_ai_01 --entry-id urrc_ai_01 --calibration generated/stage3/AI-01/calibration.json --permission-topic /urrc_2_f1/operator_test_start --ros-args -p use_sim_time:=true
~~~

다른 터미널에서 출발 허가 결과를 관측합니다.

~~~bash
source /opt/ros/jazzy/setup.bash
ros2 topic echo /urrc_2_f1/urrc_ai_01/start_allowed
~~~

다른 터미널에서 격리 실험용 운영자 허가를 반복 발행합니다(공식 /race/start 사용 금지).

~~~bash
source /opt/ros/jazzy/setup.bash
ros2 topic pub -r 10 /urrc_2_f1/operator_test_start std_msgs/msg/Bool "{data: true}"
~~~

원본 신호등 코드는 읽기 전용으로 사용하면서 실제 점등·소등 시퀀스 실행:

~~~bash
cd ~/urrc_2_f1
export GZ_PARTITION="urrc_stage3_$(id -u)"
python3 upstream/urrc_f1_ws/race/scripts/lights.py sequence --track monza --hold 1
~~~

정상 조건: 모든 빨간등 소등 전까지 False, 5개 전부 켜졌다 동시에 꺼진 뒤 True, 운영자 허가 중지 또는 카메라 끊김 시 False.

## E. 실제 레이스 3바퀴는 지금 불가

현재 3단계는 출발 허가 신호를 발행할 뿐, 모터와 조향을 구동하는 AI-01 컨트롤러가 없습니다. 실제 무충돌 3바퀴 시험은 다음 단계에서 주행 컨트롤러와 안전 인터록을 연결한 뒤 가능해집니다.

현재 GitHub Actions는 실제 몬자 이미지 및 운영자 위치 토픽 수신까지만 검증했습니다. 출발 신호등 실제 인식, ROS 2 실시간 출발 허가, 실제 충돌/3바퀴는 아직 PASS가 아닙니다.
