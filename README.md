# URRC 자율주행 AI 실험 프로젝트

**GitHub 수정 허용 저장소:** https://github.com/nahee2345/urrc_2_f1

원본 urrc_f1_ws와 urrc_2는 읽기 전용으로 취급합니다. 두 원본의 경기 맵·신호등·규정을 변경하지 않습니다.

## 실험 목표

물리 엔진 기반 4륜 차량 10대에 서로 다른 센서·알고리즘을 적용하고, URRC 몬자 맵에서 무충돌 3바퀴 완주를 연구합니다. 연구 목적의 RGB·Depth·스테레오·2D/3D LiDAR 주행 센서는 공식 원본 대회 규정과 구분하여 허용합니다.

차량별 공통 출발 카메라는 빨간불 5개가 차례대로 켜진 후 모두 꺼지는 것을 확인하도록 계획했습니다. 출발 판단 알고리즘 및 3바퀴 주행은 아직 구현되지 않았습니다.

## 단계별 진행

| 단계 | 구현한 내용 | 시험 결과 |
|---|---|---|
| 0단계 | urrc_f1_ws v10.8 소스를 고정 커밋 서브모듈로 연결 | 원본 파일 및 기준 확인 |
| 1단계 | 4륜 동적 차량, 질량·관성, 조향 및 구동 | Python 및 Gazebo 평면 전진·제동·좌회전 PASS |
| 2단계 | 10개 차량별 RGB·Depth·스테레오·LiDAR 센서 장착 | 정적 테스트 25개 PASS, 10대 실제 Gazebo 센서 수신 PASS |
| 2단계 확대 검증 | AI-01~AI-10 센서 실제 수신 일괄 검증 | 10대 모두 실제 센서 메시지 수신 PASS (실행 #40455462925) |
| 후속 단계 | 신호등 출발·랩 계측·자율주행 알고리즘 | 미구현·미검증 |

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

Python 단위 검사 44개 PASS 및 원본 몬자 실제 카메라 영상·운영자 포즈 수신 PASS_MONZA_INFRA_ONLY(https://github.com/nahee2345/urrc_2_f1/actions/runs/40458697552). 실제 5등 신호등 인식·ROS2 연동·원본 몬자 3바퀴 주행은 아직 PASS가 아닙니다.

~~~bash
cd ~/urrc_2_f1
git pull --ff-only
git submodule update --init --recursive
python3 -m unittest discover -s tests -v
python3 scripts/stage3_prepare_monza.py --ai AI-01
~~~

[3단계 실행 안내](docs/STAGE3_GUIDE.md) · [3단계 검증 보고서](reports/STAGE_3.md)
