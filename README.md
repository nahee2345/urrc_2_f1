# URRC 자율주행 AI 실험 프로젝트

**GitHub 수정 허용 저장소:** https://github.com/nahee2345/urrc_2_f1

원본 urrc_f1_ws와 urrc_2는 읽기 전용으로 취급합니다. 두 원본의 경기 맵·신호등·규정을 변경하지 않습니다.

## 실험 목표

물리 엔진 기반 4륜 차량 10대에 서로 다른 센서·알고리즘을 적용하고, URRC 몬자 맵에서 무충돌 3바퀴 완주를 연구합니다. 연구 목적의 RGB·Depth·스테레오·2D/3D LiDAR 주행 센서는 공식 원본 대회 규정과 구분하여 허용합니다.

차량별 공통 출발 카메라는 빨간불 5개가 차례대로 켜진 후 모두 꺼지는 것을 확인하도록 계획했습니다. 출발 신호 카메라 판정 로직은 3단계에 구현했습니다. 실제 신호등 인식과 3바퀴 완주 검증은 아직 별도 진행입니다.

## 단계별 진행

| 단계 | 구현한 내용 | 시험 결과 |
|---|---|---|
| 0단계 | urrc_f1_ws v10.8 소스를 고정 커밋 서브모듈로 연결 | 원본 파일 및 기준 확인 |
| 1단계 | 4륜 동적 차량, 질량·관성, 조향 및 구동 | Python 및 Gazebo 평면 전진·제동·좌회전 PASS |
| 2단계 | 10개 차량별 RGB·Depth·스테레오·LiDAR 센서 장착 | 정적 테스트 25개 PASS, 10대 실제 Gazebo 센서 수신 PASS |
| 2단계 확대 검증 | AI-01~AI-10 센서 실제 수신 일괄 검증 | 10대 모두 실제 센서 메시지 수신 PASS (실행 #40455462925) |
| 3단계 | 원본 v10.8 텍스처·고저차·관전 카메라·5등 신호등 자산 격리 복원 및 출발·랩·충돌 감시 | Python 47개 PASS, 고저차·충돌 메시 동일성 PASS; 실제 신호등 시뮬레이션 결과는 Actions에서 확인 |
| 후속 단계 | 실제 신호등 RGB 인식·ROS2 통합·AI-01 자율주행 3바퀴 | 미검증 |

## 내려받기

최초 1회 실행:

~~~bash
cd ~
git clone --recurse-submodules https://github.com/nahee2345/urrc_2_f1.git
cd urrc_2_f1
git submodule update --init --recursive
~~~

이미 다운로드한 경우:

~~~bash
cd ~/urrc_2_f1
git pull --ff-only
git submodule update --init --recursive
~~~

**주의:** GitHub 전체 ZIP에는 서브모듈 안의 원본 경기 맵 파일이 포함되지 않습니다. 원본까지 포함하려면 위 clone 명령을 사용하세요.

## 1단계 실제 물리 테스트

~~~bash
cd ~/urrc_2_f1
python3 -m unittest discover -s tests -v
python3 scripts/build_vehicle.py
python3 scripts/stage1_smoke.py --run
cat generated/stage1_smoke_result.json
~~~

PASS_FLAT_PHYSICS는 독립 평면 시험 월드에서 물리적 전진·제동·조향이 확인되었다는 뜻이며, 몬자 완주를 의미하지 않습니다.

## 2단계 센서 생성 및 검증

~~~bash
cd ~/urrc_2_f1
python3 scripts/build_sensors.py --ai all
python3 scripts/stage2_sensor_smoke.py --run --ai AI-05
cat generated/stage2_sensor_result.json
~~~

10개 차량의 SDF와 센서 정보는 generated/stage2/AI-XX 폴더에 생성합니다. PASS_SENSOR_STREAMS는 해당 차량의 실제 Gazebo 센서 메시지가 수신되었다는 뜻입니다.

ROS 2 브리지 설정 파일 생성:

~~~bash
python3 scripts/bridge_sensors.py --manifest generated/stage2/AI-05/manifest.json --output generated/stage2/AI-05/ros_bridge.yaml
~~~

ROS 2의 실제 메시지 수신 및 PointCloud2 변환은 별도 실험이 필요합니다.

## 문서·시험 결과

- [0·1단계 테스트 절차](docs/TEST_PROCEDURE.md)
- [1단계 안내](docs/STAGE1_GUIDE.md)
- [2단계 안내](docs/STAGE2_GUIDE.md)
- [0·1단계 초기 결과](reports/STAGE_0_1.md)
- [1단계 최종 보강 결과](reports/STAGE_1_HARDENING.md)
- [2단계 결과](reports/STAGE_2.md)
- [5개 차량 실제 센서 메시지 검사 결과](reports/stage2_ci_5cases_20261027.json)
- [자동 시험 실행 내역](https://github.com/nahee2345/urrc_2_f1/actions)

설명과 결과 보고서는 한국어로 작성합니다. 실행 필수 명령·코드 식별자·토픽 및 파일 경로는 원래 표기를 유지합니다.


## 3단계 — 카메라 신호등 및 랩 계측

실제 RGB 시작 카메라에 5개 빨간불이 모두 켜졌다 꺼지는 것을 확인하고 운영자 Bool이 허가일 때만 출발을 허용하는 안전 상태기계를 추가했습니다. 원본 몬자 흰색 출발선으로 랩을 세고 체크포인트 및 모든 접촉 센서로 유효성을 검사합니다.

Python 단위 검사 47개 PASS, 원본 몬자 도로 OBJ 높이 범위·충돌 메시 동일성 PASS, 원본 v10.8 관전 카메라·텍스처·5등 모델 생성 PASS. 이전 카메라·운영자 위치 센서 통합 결과: PASS_MONZA_INFRA_ONLY(https://github.com/nahee2345/urrc_2_f1/actions/runs/40458697552). 실제 5등 신호등 인식·ROS2 연동·원본 몬자 3바퀴 주행은 아직 PASS가 아닙니다.

~~~bash
cd ~/urrc_2_f1
git pull --ff-only
git submodule update --init --recursive
python3 -m unittest discover -s tests -v
python3 scripts/stage3_prepare_monza.py --ai AI-01
~~~

[3단계 실행 안내](docs/STAGE3_GUIDE.md) · [3단계 검증 보고서](reports/STAGE_3.md)


## GitHub 변경분 업데이트 및 실제 출발 시험

이미 clone한 Ubuntu에서는 전체 ZIP이나 별도 파일을 다시 다운로드하지 않습니다. 저장소에서 변경된 커밋만 갱신합니다.

~~~bash
cd ~/urrc_2_f1
git pull --ff-only
git submodule update --init --recursive
~~~

실제 몬자 화면에서 신호등 5개가 출발 카메라에 보이는지 먼저 확인해야 합니다. 해당 확인 결과에 따라 카메라 교정과 실제 5등 소등 후 출발 허가를 검증합니다.

**[실제 출발 시험을 위한 사용자 Ubuntu 실행 안내](docs/STAGE3_LOCAL_INTEGRATION.md)**

현재는 자동 주행 제어기가 연결되지 않아 무충돌 3바퀴 완주를 검증할 수 없습니다.


### 3단계 변경: 공식 몬자 맵과 동일하게 보기

이전 단계에서 원본 run_world.py의 enhance_track 과정을 빠뜨려 텍스처·관전 카메라·지면 재질과 신호등이 원본 실행 화면과 달랐습니다. 3단계부터는 **원본 GitHub를 수정하지 않고** generated/stage3/resources에 공식 향상 리소스를 재현합니다. 원본 도로 고저차는 약 2.563m입니다.

~~~bash
cd ~/urrc_2_f1
git pull --ff-only
git submodule update --init --recursive
python3 scripts/stage3_prepare_monza.py --ai AI-01
cat generated/stage3/AI-01/terrain_fidelity.json
python3 scripts/stage3_run_monza.py --ai AI-01
~~~

다른 터미널에서 공식 v10.8 방식의 5등 전체 점등은 \`python3 scripts/stage3_lights.py on\`, 순차 점등 후 전체 소등은 \`python3 scripts/stage3_lights.py sequence\`로 실행합니다. 카메라 ROI는 실제 화면 확인 후에만 교정합니다.

## 4단계 — AI-01 검정 차량 RGB 카메라 주행

- 차량 SDF의 차체·바퀴·너클·센서 장착물은 검정색/어두운 재질로 명시.
- AI-01 주행용 RGB 픽셀에서 도로 중심을 추정해 실제 전륜(FWD) 모터·Ackermann 조향 명령 계산.
- 3단계 카메라 소등 안전 허가 없이 움직이지 않으며, 영상 끊김/도로 감지 실패 시 목표 속도 0.
- 전체 Python 단위 시험 63개 PASS. 실제 ROS2 카메라 AI 자율주행과 3바퀴 완주는 아직 별도 검증이 필요.
- 현재 RGB 도로 인식은 색상 기반 기준선이며 CNN 학습 모델은 미구현.

기존 다운로드 폴더에서 변경분만 적용:

~~~bash
cd ~/urrc_2_f1
git pull --ff-only
git submodule update --init --recursive
python3 scripts/stage3_prepare_monza.py --ai AI-01
python3 scripts/stage4_bridge.py --contacts generated/stage3/AI-01/contacts.json --output generated/stage4/ai01_bridge.yaml
~~~

[4단계 Ubuntu 실제 실행법](docs/STAGE4_GUIDE.md) · [4단계 한국어 검증 보고서](reports/STAGE_4.md) · [GitHub Actions](https://github.com/nahee2345/urrc_2_f1/actions)


## 5단계 — Monaco 공식 맵과 AI-01 자동주행 시험

5단계부터 Monaco를 기본 트랙으로 사용합니다. 원본 고저차 약 8.41m, 도로 최소 폭 1.4m를 유지한 검정 AI-01 시뮬레이션을 생성하며, 실제 출발 카메라 5등 ROI 교정과 3바퀴 운영자 랩·충돌 감독을 연동했습니다.

기존 Git clone에서 수정 내용만 업데이트:

    cd ~/urrc_2_f1
    git pull --ff-only
    git submodule update --init --recursive
    python3 scripts/stage5_prepare_monaco.py --ai AI-01
    python3 scripts/stage5_bridge.py
    python3 scripts/stage5_run_monaco.py --ai AI-01

상세 설명: docs/STAGE5_GUIDE.md
단계별 검증 결과: reports/STAGE_5.md

실제 Monaco 카메라·위치·FWD 모터 물리는 GitHub Actions에서 기본 검증에 통과했습니다. 그러나 실제 빨간불 카메라 출발, RGB 주행 제어기 3랩 무충돌 완주는 아직 PASS가 아닙니다.


## Monaco 신호등 및 10대 원색 차량 (2026-10-29)

기존 검정 AI-01은 노랑으로 변경했습니다. AI-01부터 AI-10까지 노랑/빨강/파랑/초록/주황/자홍/청록/보라/연두/분홍 고대비 원색을 각각 사용하며 타이어는 검정입니다.

~~~bash
cd ~/urrc_2_f1
git pull --ff-only
python3 scripts/stage5_prepare_monaco.py --ai AI-01
python3 scripts/stage5_run_monaco.py --ai AI-01 --lights
~~~

위 명령으로 신호등 5등 순차 점등·전체 소등을 볼 수 있습니다. 단독 Monaco Gazebo 실행은 신호등 서비스를 자동 호출하지 않습니다. 실제 RGB 자동주행은 start_camera 5등 교정 후 레이스 통합 실행기를 사용합니다.

상세 사용법: docs/STAGE5_GUIDE.md
