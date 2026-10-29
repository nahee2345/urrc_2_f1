# 5단계 — Monaco 기준 AI-01 자동주행 실험

## 5단계 — 한 터미널에서 전 과정 자동 실행 (기본 권장)

**이 방법을 사용하면 터미널을 4개 열 필요가 없습니다.** 기존 Gazebo/ROS 브리지 창은 먼저 종료하세요. 커맨드 한 번으로 맵 준비 → ROS 브리지 준비 → 카메라 교정(필요 시) → 신호등 초기화 → 실험 재시작 → 5등 순차 점등·동시 소등 → RGB 주행/랩 감시 순서로 진행합니다.

~~~bash
cd ~/urrc_2_f1
git pull --ff-only
git submodule update --init --recursive
source /opt/ros/jazzy/setup.bash
python3 scripts/stage5_start.py
~~~

기본값은 AI-01, Monaco, 0.55m/s, 최대 120분입니다. 필요하면 \`--speed-limit 0.4 --minutes 30\`을 추가할 수 있습니다. **실제 교정 파일이 없으면 검증된 빨간불 5개 픽셀을 자동으로 찾아 저장**하고, 이미 교정된 Monaco 설정이 있으면 재사용합니다.

옵션:
- \`--calibrate-only\`: 맵·신호등 카메라 교정만 실행하고 안전하게 종료
- \`--recalibrate\`: 기존 ROI가 있어도 다시 카메라 영상에서 교정
- \`--dry-run\`: 실제 Gazebo·ROS를 켜지 않고 실행 순서만 확인

원본의 신호등 시퀀스를 실제 카메라가 인식하지 못하면 출발 허가가 생성되지 않습니다. 교정이 실패하면 **실제 주행이 차단**되며, \`generated/stage5/AI-01/start_camera_diagnostic.ppm\`, \`calibration_gazebo.log\`, \`calibration_bridge.log\`에 점검 자료를 남깁니다(이미지 수신이 된 경우). 이 자료의 화면과 로그를 공유하면 이후 GitHub에서 수정합니다.

실행 후 실제 3랩 판정은 \`generated/stage5/AI-01/stage5_verdict.json\`에 기록됩니다. **RGB 알고리즘의 3바퀴 완주는 아직 PASS로 검증된 상태가 아닙니다.**

아래 문서의 4개 터미널 수동 실행은 문제 원인을 파악할 때만 사용합니다.

---



## 트랙 및 정책

- 5단계의 기준 트랙은 공식 Monaco입니다.
- 원본 전체 3D 트랙 길이 약 667.16m, 고저차 약 8.41m, 최소 폭 약 1.4m입니다.
- 원본 urrc_f1_ws 및 urrc_2 GitHub는 읽기 전용입니다.
- 수정·push는 urrc_2_f1에서만 합니다.
- 기존에 clone한 폴더에서 git pull로 수정 내용만 가져옵니다. ZIP/새 파일 다운로드는 없습니다.

## 1. 소스 업데이트 및 Monaco 준비

    cd ~/urrc_2_f1
    git pull --ff-only
    git submodule update --init --recursive
    source /opt/ros/jazzy/setup.bash
    python3 -m unittest discover -s tests -v
    python3 scripts/stage5_prepare_monaco.py --ai AI-01
    python3 scripts/stage5_bridge.py

공식 Monaco의 고저차, 도로 충돌 메시, 5개 신호등, 관전 카메라 4대를 보존한 모델은 generated/stage5/AI-01/monaco_experiment.sdf에 생성됩니다.

    cat generated/stage5/AI-01/terrain_fidelity.json

PASS_MONACO_SOURCE_FIDELITY이면 원본 지형/모델 구조 검증에 성공한 것입니다. 3바퀴 완주 PASS가 아닙니다.

## 2. Monaco 노란색 AI-01 시뮬레이션 확인 (터미널 1)

    cd ~/urrc_2_f1
    source /opt/ros/jazzy/setup.bash
    python3 scripts/stage5_run_monaco.py --ai AI-01

원본 Monaco 그리드 시점, 차량 위치, 신호등을 눈으로 확인합니다.

## 3. ROS2 센서·모터 브리지 (터미널 2)

    cd ~/urrc_2_f1
    source /opt/ros/jazzy/setup.bash
    export GZ_PARTITION="urrc_stage5_$(id -u)"
    ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:=$PWD/generated/stage5/AI-01/bridge.yaml

## 4. 출발 카메라 자동 교정 (한 번만)

실제 5개 빨간불이 모두 보이는 영상으로 교정해야 합니다. 예시 좌표를 임의로 활성화하지 않습니다.

터미널 3:

    cd ~/urrc_2_f1
    python3 scripts/stage5_lights.py on

터미널 4:

    cd ~/urrc_2_f1
    source /opt/ros/jazzy/setup.bash
    python3 scripts/stage5_calibrate_lights.py --output generated/stage5/AI-01/calibration.json

정확히 다섯 붉은 렌즈가 연속 프레임에서 검출되면 교정 설정이 저장됩니다. 한 개라도 누락되거나 영상 수신이 실패하면 출발을 차단합니다. 영상 검출이 실패하면 다음 명령으로 카메라 화면을 확인해 전달할 수 있습니다.

    python3 scripts/stage3_capture_roi.py --topic /model/urrc_ai_01/sensors/start_camera --out generated/stage5/AI-01/start_camera.ppm
    xdg-open generated/stage5/AI-01/start_camera.ppm

교정 후에는 Gazebo 및 브리지 터미널을 종료합니다. 중복 Gazebo 월드를 실행하지 마세요.

## 5. 3바퀴 실제 자동주행 통합 실행

카메라 교정이 성공한 후 다음 명령을 실행합니다.

    cd ~/urrc_2_f1
    source /opt/ros/jazzy/setup.bash
    python3 scripts/stage5_race.py --calibration generated/stage5/AI-01/calibration.json --speed-limit 0.55 --minutes 120

5단계 실행기는 Gazebo 원본 Monaco 월드, ROS 브리지, 빨간불 카메라 출발 게이트, RGB 주행 제어기, 운영자 위치·체크포인트·충돌 감독, 원본 신호등을 순서대로 연결합니다.

반드시 다섯 빨간등 점등→전체 소등을 카메라가 실제로 확인한 뒤에만 바퀴 구동이 가능합니다. 주행 RGB가 도로를 찾지 못하거나 영상이 끊기면 목표 바퀴 속도를 0으로 설정합니다.

## 6. 성공 판정 및 결과 위치

    cat generated/stage5/AI-01/stage5_verdict.json
    tail -n 20 generated/stage5/AI-01/lap_events.jsonl

PASS_THREE_LAPS는 실제 위치 계측으로 유효 3랩, 체크포인트 정상 통과, 충돌 0건, 모든 접촉 센서 연결이 확인되어야 출력합니다. 이 조건 중 하나라도 증명할 수 없으면 NOT_PASSED입니다.

현재 GitHub Actions 자동 검사에서는 Monaco 실제 카메라·위치 발행과 전륜 물리 구동 기초 시험까지 확인했습니다. 실제 ROS2 카메라 인식→안전 출발→3바퀴 완주는 아직 검증되지 않았습니다.

참고: AI-01의 현재 도로 추적기는 색상 기반 기초 영상처리로, 학습된 CNN 모델을 구현한 상태는 아닙니다. Monaco 협소 구간·헤어핀 성능은 추후 보완이 필요합니다.


## 2026-10-29: 직접 실행 오류 및 신호등 미점등 수정

원인: Python scripts/stage5_race.py를 직접 실행할 때 모듈 검색 경로에 프로젝트 루트가 빠져 stage3을 불러오지 못했습니다. 실행기 내부에서 루트를 sys.path에 추가해 수정했습니다.

또한 stage5_run_monaco.py는 기본적으로 맵과 차량만 띄웁니다. 빨간불 5개 점등/소등 명령은 별도 서비스 호출이 필요합니다. 이번 수정으로 다음 명령에서 신호등 시퀀스를 직접 실행하도록 --lights 옵션을 추가했습니다.

~~~bash
cd ~/urrc_2_f1
git pull --ff-only
git submodule update --init --recursive
python3 scripts/stage5_prepare_monaco.py --ai AI-01
python3 scripts/stage5_run_monaco.py --ai AI-01 --lights
~~~

다른 Gazebo 월드가 떠 있으면 먼저 종료하세요. 한꺼번에 두 개의 Monaco 월드를 띄우면 안 됩니다. --lights를 사용하면 원본 5개 빨간불이 차례로 켜졌다가 다 같이 꺼지고 화면은 그대로 유지됩니다. 이 명령은 차량을 자동 출발시키는 레이스 통합 명령이 아닙니다.

이미 단독 실행한 Monaco 창이 있다면 별도 터미널에서 다음을 실행할 수 있습니다.

~~~bash
cd ~/urrc_2_f1
python3 scripts/stage5_lights.py sequence
~~~

서비스 점검이 필요하다면:

~~~bash
export GZ_PARTITION="urrc_stage5_$(id -u)"
gz service -l | grep '/world/monaco/visual_config'
gz topic -l | grep '/urrc/monaco/spectator/'
~~~

Gazebo GUI의 visual_config가 없거나 동일한 GZ_PARTITION이 아니면 실제 발광 서비스가 실행되지 않을 수 있습니다. 이때는 --lights를 실행한 터미널의 오류 출력을 공유하세요.

### AI-01~AI-10 차량의 고대비 원색

- AI-01 노랑
- AI-02 빨강
- AI-03 파랑
- AI-04 초록
- AI-05 주황
- AI-06 자홍
- AI-07 청록
- AI-08 보라
- AI-09 연두
- AI-10 분홍

차체 및 센서 하우징을 차량별 원색으로 바꾸고 타이어는 검정색으로 유지했습니다. 앞서 만들어둔 검정색 SDF를 계속 볼 경우 stage5_prepare_monaco.py를 다시 실행해야 새 차량 색상이 반영됩니다.

### 실제 3랩 자동주행 전제

신호등 점등 시연은 카메라 교정 없이 가능합니다. 실제 5단계 자동주행은 Monaco start_camera 영상에서 실제 빨간불 5개를 교정한 calibration.json이 필요합니다. 교정이 없을 경우 출발 안전장치가 주행을 거부해야 정상입니다. 아직 실제 무충돌 3랩 PASS는 아닙니다.

## 교정 중 빨간불 5개가 즉시 켜지고 계속 켜져 있을 때

`stage5_start.py`의 **카메라 자동 교정 모드에서는 정상 동작**입니다. 정식 레이스의 랜덤 3~6초 순차 점등과 다르게 빨간불 5개를 곧바로 모두 켜서 ROI를 측정합니다. 교정이 끝나면 다섯 등을 소등한 뒤 Gazebo/브리지를 종료하고, 정식 레이스를 다시 시작합니다.

수정된 실행기는 3초마다 **ROS 카메라 발행자 수, 실제 이미지 수신 프레임 수, 다섯 렌즈 연속 인식 횟수**를 표시합니다. `[READY] Gazebo visual service and start/overview camera topics ready` 문구는 Gazebo 자체 카메라 준비 상태로, **차량 start_camera 이미지가 ROS2에서 수신됐다는 의미가 아닙니다**.

- `ROS 카메라 이미지 미수신`: ROS2 브리지/토픽/QoS/렌더러 점검 필요
- `ROS 이미지 수신, 빨간 렌즈 5개 연속 인식 대기`: 카메라 연결은 됐지만 실제 LED 5개 검출이 실패할 수 있음
- `Monaco 출발 신호등 ROI 교정 PASS`: 실제 카메라 프레임 3회 연속 검출 성공
- 실패, 시간 만료 또는 Ctrl+C 시 교정용 신호등을 끄고 임시 월드 종료. 필요 시 로컬 start_camera_diagnostic.ppm을 확인

**기존 Gazebo를 종료한 뒤 최신 커밋으로 git pull하고 재실행해야** 새 상태 메시지가 표시됩니다.

~~~bash
cd ~/urrc_2_f1
git pull --ff-only
git submodule update --init --recursive
source /opt/ros/jazzy/setup.bash
python3 scripts/stage5_start.py
~~~



## 2026-10-29 실제 7~8개 붉은 영역 교정 문제 수정

사용자 실측 로그에서는 start_camera의 ROS2 영상 수신이 정상이었습니다(18초 동안 277프레임, 발행자 1개). 문제는 기존 교정기가 **화면 전체의 붉은 영역 수가 정확히 5개여야 한다고 가정**한 것이었습니다.

수정된 단일 터미널 실행기는 다음 순서로 동작합니다.

1. 빨간 신호등이 모두 꺼진 카메라 화면을 실제 ROS RGB 영상으로 저장
2. 교정용 빨간불 5개 전체 점등
3. 두 영상에서 새롭게 붉어진 픽셀만 비교해 신호등 후보 선정
4. 수평 배열·간격·높이·크기와 3프레임 안정성 검증
5. 성공하면 교정 파일 기록, 실패하면 실제 카메라 진단 PPM 저장 및 레이스 출발 차단
6. 교정 종료/취소 시 신호등 소등 및 Gazebo 정리

기존 빨간 물체 2~3개 때문에 7~8개 후보가 생기는 상황을 모의 회귀 시험에 추가했습니다. 실제 영상에서 5개가 보이지 않거나 후보가 모호하면 여전히 교정을 거부합니다. 사용자 차량을 강제 출발시키지 않습니다. Ctrl+C 시 ROS2 중복 shutdown도 방지했습니다.

기존 Gazebo 창을 종료한 후 터미널 하나에서 아래 명령만 실행합니다.

    cd ~/urrc_2_f1
    git pull --ff-only
    git submodule update --init --recursive
    source /opt/ros/jazzy/setup.bash
    python3 scripts/stage5_start.py

문제가 다시 발생하면 진단 이미지를 로컬에서 확인합니다.

    xdg-open generated/stage5/AI-01/start_camera_diagnostic.ppm

이 이미지와 오류 메시지를 공유하면 카메라 실제 시야를 기준으로 추가 수정합니다. 새 코드가 실제 사용자 카메라에서 5등을 인식하거나 3바퀴 주행을 성공했다는 증거는 아직 없습니다.


## 교정 성공 뒤 Gazebo 창이 사라지던 문제 수정 (2026-10-29)

현장 로그에서 출발 카메라 OFF/ON **교정 PASS**를 확인했습니다. 문제는 교정과정에서 사용한 정상 Gazebo·ROS 브리지를 종료하고 레이스가 이를 새로 기동하는 과정에 있었습니다. 그 사이 GUI 화면이 나타나지 않는 현상을 피하도록 **같은 Gazebo 프로세스/GUI와 동일 ROS 브리지를 교정부터 레이스 종료까지 유지**합니다.

작업 순서는 다음과 같습니다.

1. Monaco Gazebo GUI **한 번만 실행**하고 visual_config 서비스 확인
2. ROS 2 브리지 **한 번만 실행**해 카메라 영상 수신
3. 실제 소등/점등 카메라 차분 교정 (이미 교정되었다면 재사용)
4. 교정용 빨간불 소등, 단 Gazebo 화면과 ROS 브리지는 **종료하지 않음**
5. 같은 Gazebo 창에 **카메라 출발 게이트/AI-01 RGB 제어/운영자 계측만 추가 연결**
6. 정식 Monaco 신호등 순차 점등 및 전체 소등
7. 레이스 종료/Ctrl+C 이후 이 실행기가 시작한 ROS와 Gazebo 프로세스만 종료

기존 Gazebo가 남아 있으면 먼저 닫은 후 한 터미널에서 실행하세요.

    cd ~/urrc_2_f1
    git pull --ff-only
    git submodule update --init --recursive
    source /opt/ros/jazzy/setup.bash
    python3 scripts/stage5_start.py

이전 교정 파일이 이미 있으므로 **--recalibrate 없이** 실행해도 됩니다. 새로운 차분 교정값이 유효하면 재사용합니다.

실패하면 실행기에 다음 로그를 확인하세요.

    tail -n 60 generated/stage5/AI-01/calibration_gazebo.log
    tail -n 60 generated/stage5/AI-01/calibration_bridge.log
    tail -n 60 generated/stage5/AI-01/camera_light_gate.log
    tail -n 60 generated/stage5/AI-01/rgb_controller.log

로그는 사용자 PC에만 저장되며 자동으로 GitHub에 업로드하지 않습니다. 스크린샷과 함께 공유하면 원인을 구분할 수 있습니다.

**주의:** GUI가 실제로 떠 있는지와 카메라 안전 출발 이후 3바퀴 완주는 별도로 검증해야 합니다. 교정 PASS만으로 주행 PASS가 아닙니다.


## 2026-10-29 Monaco 차량 이동 및 종료 오류 진단

사용자가 보낸 실제 로그에서는 빨간불 전체 소등을 카메라가 감지해 출발 허가가 생성됐습니다. ROS-Gazebo 브리지의 모터 명령 변환도 확인됐습니다. 다만 신호등 소등 허가 후 약 1.1초 만에 Ctrl+C로 종료하였고, 실제 주행 카메라 도로 검출이나 위치 변위 증거는 없으므로 차량이 움직이지 않았다고 확정할 수 없습니다.

최신 코드는 단일 터미널에서 3초마다 실제 주행 RGB 프레임 수, 출발 허가, 도로 구간 검출, 좌측 전륜 목표 회전속도, Gazebo 실제 이동거리와 랩 수를 출력합니다. 출발 허가만으로는 전진하지 않으며, 도로 검출 실패 시 0속도 유지가 정상입니다.

기존 Gazebo 창을 종료하고 아래 명령을 실행하세요:

cd ~/urrc_2_f1

git pull --ff-only

git submodule update --init --recursive

source /opt/ros/jazzy/setup.bash

python3 scripts/stage5_start.py

새로 생성되는 사용자 PC의 독립 자료:

generated/stage5/AI-01/drive_status.json — 실제 RGB·출발 상태·명령 속도

generated/stage5/AI-01/pose_status.json — 운영자 전용 Gazebo 위치·변위

generated/stage5/AI-01/rgb_camera.ppm — AI-01 실제 주행용 RGB 카메라 화면

generated/stage5/AI-01/rgb_controller.log — 주행 중 차단 이유

카메라 영상 확인은 xdg-open generated/stage5/AI-01/rgb_camera.ppm 로 할 수 있습니다.

로그의 publisher context invalid 및 rcl_shutdown already called는 ROS Jazzy SIGINT 종료 처리 오류였습니다. 게이트·주행기·운영자 계측의 중복 종료를 방지했으며, 종료 직전 Gazebo 전륜 모터와 조향에 0명령을 전송합니다.

현재 실제 Monaco 3바퀴 완주 PASS는 아직 없습니다.


## 2026-10-29 Monaco 3.8m 이후 도로 경계 상실 해결 시도

실제 사용자 Monaco 레이스 로그에서 신호등 소등 인식, 출발 허가, RGB 도로 검출, 전륜 목표 11~13.75 rad/s, 차량 실제 누적 이동 약 3.8043m 확인. 그 다음 주행 카메라가 연속 도로 경계를 추적하지 못해 안전상 목표 전륜 속도 0으로 정지. **모터·신호등 오류가 아니라 RGB 도로 추종 한계**.

수정 사항: (1) 주행용 RGB 카메라만 기존 수평 0rad에서 -0.26rad 하향하여 전방/내리막 도로 관찰 개선; 출발 신호등 카메라는 변경하지 않음. (2) RGB 도로 마스크를 밝기 8~112 범위, 녹갈색 외곽과 중립색 아스팔트를 구분하도록 재설계. (3) Monaco 코너에서 이미지 하단 가까운 노면부터 먼 행으로 경계를 연결하고 앞쪽 도로 중심을 강조해 선제 조향. (4) 최대 조향 0.38rad 이내, 코너 목표속도 0.29m/s. (5) 도로 인식이 사라지면 기존처럼 전륜 목표 0 명령 유지.

주행 중 경계 상실 시 실제 RGB 화면을 generated/stage5/AI-01/rgb_camera_road_lost.ppm 에 최초 한 번 자동 저장합니다. 종전 첫 프레임은 rgb_camera.ppm 입니다. 해당 화면을 통해 실제 미검출 원인을 정확히 확인할 수 있습니다.

다시 실행 (기존 Gazebo 창 먼저 닫기):

cd ~/urrc_2_f1

git pull --ff-only

git submodule update --init --recursive

source /opt/ros/jazzy/setup.bash

python3 scripts/stage5_start.py

새 sensor pitch/추종 로직을 사용하려면 반드시 stage5_start.py가 차량 모델을 재생성해야 합니다. 이미 성공한 신호등 교정은 재사용되므로 --recalibrate가 필요하지 않습니다.

결과 파일: cat generated/stage5/AI-01/drive_status.json / cat generated/stage5/AI-01/pose_status.json / xdg-open generated/stage5/AI-01/rgb_camera_road_lost.ppm

회귀검사는 Monaco 그림자, 밝은 아스팔트, 좌우 급커브, 어두운 전체 프레임 및 도로 미검출 안전 정지를 다룹니다. **이 검사는 합성 영상이고, 실제 Monaco 주행 3랩 완주 PASS 증거는 아닙니다.**
