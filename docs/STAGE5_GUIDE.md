# 5단계 — Monaco 기준 AI-01 자동주행 실험

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

## 2. Monaco 검정 AI-01 시뮬레이션 확인 (터미널 1)

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
