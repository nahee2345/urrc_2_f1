# URRC 0·1단계 — Ubuntu 테스트 실행 방법

## 사용 환경 및 수정 금지
Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic.

GitHub 업로드는 urrc_2_f1에만 수행합니다. urrc_f1_ws 및 urrc_2는 원본 유지입니다. urrc_f1_ws 고정 커밋: a23c7eec41b98b67ec3a3a2a46f9d7ed957f8290.

## 설치 및 기준 원본 검사

~~~bash
cd ~
git clone --recurse-submodules https://github.com/nahee2345/urrc_2_f1.git
cd urrc_2_f1
git submodule update --init --recursive
python3 scripts/check_upstream.py
bash upstream/urrc_f1_ws/test_v10.sh
~~~

이미 폴더가 있다면 처음 세 명령 대신 다음을 실행하세요.

~~~bash
cd ~/urrc_2_f1
git pull --ff-only
git submodule update --init --recursive
~~~

원본 test_v10.sh는 정적 검사입니다. 실제 차량 주행을 대체하지 않습니다.

## Python 및 차량 생성

~~~bash
python3 -m unittest discover -s tests -v
python3 -m compileall -q vehicle_generator scripts tests
python3 scripts/build_vehicle.py
gz sdf -p generated/urrc_stage1_rwd/model.sdf > /tmp/urrc_stage1_expanded.sdf
~~~

## 실제 Gazebo 헤드리스 물리 검사

~~~bash
python3 scripts/stage1_smoke.py --run
cat generated/stage1_smoke_result.json
~~~

PASS_FLAT_PHYSICS는 별도 평면에서 차량이 실제 구동 바퀴의 힘으로 전진하고, 제동 명령 및 좌회전 조향에 반응한다는 의미입니다. 화면 없이 실행되므로 Gazebo 창이 나타나지 않아도 정상입니다.

## 원본 몬자에서 차량 생성 후 수동 검사

터미널 1:

~~~bash
cd ~/urrc_2_f1
source /opt/ros/jazzy/setup.bash
bash upstream/urrc_f1_ws/run_v10.sh monza
~~~

터미널 2:

~~~bash
cd ~/urrc_2_f1
source /opt/ros/jazzy/setup.bash
python3 scripts/build_vehicle.py
ros2 run ros_gz_sim create --world monza --file generated/urrc_stage1_rwd/model.sdf --name urrc_stage1_rwd --x 0.3786723464 --y -2.0560353977 --z 1.2435265615 -Y 1.4573844161
~~~

전진 명령(가제보 실제 시험으로 확인된 회전 부호는 양수):

~~~bash
gz topic -t /model/urrc_stage1_rwd/drive/rear_left -m gz.msgs.Double -p 'data: 3.0'
gz topic -t /model/urrc_stage1_rwd/drive/rear_right -m gz.msgs.Double -p 'data: 3.0'
~~~

제동 명령:

~~~bash
gz topic -t /model/urrc_stage1_rwd/drive/rear_left -m gz.msgs.Double -p 'data: 0.0'
gz topic -t /model/urrc_stage1_rwd/drive/rear_right -m gz.msgs.Double -p 'data: 0.0'
~~~

좌회전 조향 명령(가제보 조향축 기준 양수):

~~~bash
gz topic -t /model/urrc_stage1_rwd/steer/left -m gz.msgs.Double -p 'data: 0.1'
gz topic -t /model/urrc_stage1_rwd/steer/right -m gz.msgs.Double -p 'data: 0.1'
~~~

직접 확인해야 할 항목은 4개 바퀴의 노면 접촉, 충돌 형상, 전진·제동·조향, 경사로와 벽 접촉입니다. 몬자 수동 검사는 아직 PASS로 보고되지 않았습니다.

세부 결과는 reports/STAGE_0_1.md와 reports/STAGE_1_HARDENING.md를 참조하세요.
