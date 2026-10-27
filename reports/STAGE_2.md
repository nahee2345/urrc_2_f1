# URRC AI 2단계 — 센서 장착 및 실제 데이터 검증 보고서

**기준일: 2026년 10월 27일**

## 1. 수정 대상 및 원본 보호

개발·수정·GitHub push는 **nahee2345/urrc_2_f1**에만 수행합니다.

원본 urrc_f1_ws 및 urrc_2는 읽기 전용으로 유지합니다. urrc_f1_ws 기준 커밋은 a23c7eec41b98b67ec3a3a2a46f9d7ed957f8290입니다. 기존 경기 맵, 원본 운영 코드, 신호등 로직을 수정하지 않았습니다.

## 2. 1단계 물리 주행 사전 통과

사용자가 Ubuntu에서 직접 실행하여 제공한 기록:
- 단위 테스트 12개 모두 통과.
- 전진 거리 +X **1.439996m**.
- 정지 명령 이후 추가 이동 **0.237083m**.
- 좌회전 시 +Y 이동 **0.245785m**.
- 판정: **PASS_FLAT_PHYSICS**.

이는 원본 몬자가 아닌 독립 평면에서 수행한 실제 Gazebo 물리 시험입니다.

## 3. 2단계 구현 내용

- 1단계 물리 차량의 실제 4개 바퀴와 구동·조향 구조 유지.
- 전 차량에 출발 신호용 카메라 추가. 이 카메라는 주행 전용 센서 구성에서 제외.
- RGB 카메라, Depth 카메라, 스테레오 좌·우 카메라, 2D GPU LiDAR, 3D GPU LiDAR를 실제 SDF 센서로 생성.
- 스테레오 카메라는 서로 다른 위치의 실제 물리 센서 2개와 독립 토픽.
- 3D LiDAR는 수평 360샘플 × 수직 16채널로 설정.
- 센서 장착물의 외형과 충돌 형상, 질량, 합성 관성모멘트 및 차체 무게중심 계산.
- 전륜·후륜·사륜 구동방식 유지.
- 센서별 격리 토픽 및 Gazebo → ROS 2 단방향 브리지 설정 YAML 생성.
- 실제 ROS 2 브리지 메시지 수신 자체는 아직 검증하지 않음.

## 4. AI-01~10 정적 검증

GitHub Actions에서 **Python 단위 시험 25개 통과**, SDF 차량 10개 생성 통과, Gazebo SDF 파싱 검사를 확인했습니다.

근거: https://github.com/nahee2345/urrc_2_f1/actions/runs/40449012977

| 차량 | 구동 | 주행용 센서 | 센서 포함 질량 |
|---|---|---|---:|
| AI-01 | 전륜 | RGB | 1.792kg |
| AI-02 | 후륜 | 3D LiDAR | 1.847kg |
| AI-03 | 사륜 | 스테레오 | 1.812kg |
| AI-04 | 전륜 | Depth | 1.802kg |
| AI-05 | 후륜 | RGB + 3D LiDAR | 1.877kg |
| AI-06 | 사륜 | Depth + 3D LiDAR | 1.887kg |
| AI-07 | 전륜 | RGB + Depth | 1.832kg |
| AI-08 | 후륜 | RGB + 2D/3D LiDAR | 1.927kg |
| AI-09 | 사륜 | 스테레오 + 3D LiDAR | 1.897kg |
| AI-10 | 사륜 | RGB + Depth + 3D LiDAR | 1.917kg |

10개 차량 모두 장착 부품을 포함한 외접 치수는 길이 약 **0.477m**, 너비 약 **0.222m**, 높이 약 **0.115~0.138m**입니다. 대회 최대 한계 0.55 × 0.25 × 0.18m 이내입니다.

## 5. Gazebo Harmonic에서 확인한 실제 센서 메시지

GitHub Actions 독립 시험 월드의 Ogre2 렌더링에서 다음 차량은 **실제 센서 데이터 발행·수신을 확인**했습니다.

실행 링크: https://github.com/nahee2345/urrc_2_f1/actions/runs/40449012977

| 차량 | 실제 수신 메시지 | 판정 |
|---|---|---|
| AI-03 | 출발 카메라, 스테레오 좌·우 카메라 영상 | PASS_SENSOR_STREAMS |
| AI-04 | 출발 카메라, Depth 영상 | PASS_SENSOR_STREAMS |
| AI-05 | 출발 카메라, RGB 영상, 3D LaserScan, 3D PointCloudPacked | PASS_SENSOR_STREAMS |
| AI-08 | 출발 카메라, RGB 영상, 2D LaserScan, 3D LaserScan, 3D PointCloudPacked | PASS_SENSOR_STREAMS |
| AI-10 | 출발 카메라, RGB 영상, Depth 영상, 3D LaserScan, 3D PointCloudPacked | PASS_SENSOR_STREAMS |

토픽명, 실제 데이터 수신 여부, 전송 데이터 크기를 포함한 결과는 reports/stage2_ci_5cases_20261027.json에 저장했습니다.

## 5-1. 10대 전 차량 실제 센서 검증 최종 완료

**2026년 10월 27일 GitHub Actions 실행 #40455462925: 전체 성공**

- 최종 결과: https://github.com/nahee2345/urrc_2_f1/actions/runs/40455462925
- 정적 검사: Python 단위 테스트 **25개 PASS**, 10개 차량 생성 및 SDF 파싱 PASS
- 실제 물리·센서 검사: AI-01~AI-05와 AI-06~AI-10 두 그룹 모두 GitHub Actions **success**
- 로그별 확인: 모든 AI에서 시작용 카메라와 해당 주행용 센서가 실제 Gazebo Harmonic Ogre2에서 메시지를 발행했으며 **`PASS_SENSOR_STREAMS`**를 기록함
- 확인된 사례에는 RGB·Depth·스테레오 이미지, 2D·3D LiDAR LaserScan 및 3D PointCloudPacked가 포함됨

| 차량 | 실제 Gazebo 센서 수신 | 결과 |
|---|---|---|
| AI-01 | 출발 카메라 + RGB | PASS_SENSOR_STREAMS |
| AI-02 | 출발 카메라 + 3D LiDAR·점군 | PASS_SENSOR_STREAMS |
| AI-03 | 출발 카메라 + 스테레오 좌·우 | PASS_SENSOR_STREAMS |
| AI-04 | 출발 카메라 + Depth | PASS_SENSOR_STREAMS |
| AI-05 | 출발 카메라 + RGB + 3D LiDAR·점군 | PASS_SENSOR_STREAMS |
| AI-06 | 출발 카메라 + Depth + 3D LiDAR·점군 | PASS_SENSOR_STREAMS |
| AI-07 | 출발 카메라 + RGB + Depth | PASS_SENSOR_STREAMS |
| AI-08 | 출발 카메라 + RGB + 2D·3D LiDAR·점군 | PASS_SENSOR_STREAMS |
| AI-09 | 출발 카메라 + 스테레오 좌·우 + 3D LiDAR·점군 | PASS_SENSOR_STREAMS |
| AI-10 | 출발 카메라 + RGB + Depth + 3D LiDAR·점군 | PASS_SENSOR_STREAMS |

실제 시험 데이터는 GitHub Actions의 `stage2-actual-sensor-AI-01~AI-05` 및 `stage2-actual-sensor-AI-06~AI-10` 아티팩트에 보관했습니다. 10개 SDF 모델은 `stage2-sensor-vehicle-models` 아티팩트로 내려받을 수 있습니다.

**주의:** Node.js 20 지원 중단 경고는 workflow의 `actions/checkout@v4`, `setup-python@v5`, `upload-artifact@v4`에서 발생한 호환성 알림으로, 이번 실행 실패가 아닙니다. 차후 액션 버전 업그레이드를 권장합니다. 이 시험은 아직 실제 ROS2 브리지, 원본 몬자 주행, 빨간 신호등 인식 또는 3바퀴 완주까지 검증하지 않았습니다.

## 6. 아직 검증되지 않은 사항

- 원본 몬자 맵에서 센서들의 실제 시야와 장애물 인식 품질.
- 실제 ROS 2 Jazzy 브리지·PointCloud2 데이터 변환 및 수신.
- 신호등 빨간불 5개 점등→동시 소등의 카메라 인식 및 출발 차단 제어.
- AI 알고리즘 주행 성능과 충돌 회피.
- 흰색 출발선 기반 랩 판정과 무충돌 3바퀴 완주.

## 7. 실행 절차 및 현재 판정

- 실행 명령과 개별 센서 목록: docs/STAGE2_GUIDE.md
- 최신 소스 다운로드: git pull --ff-only
- 10대 모델 생성: python3 scripts/build_sensors.py --ai all
- 실제 개별 센서 시험: python3 scripts/stage2_sensor_smoke.py --run --ai AI-05

**현재 확정:** 2단계 10대 차량 SDF 및 정적 검사 PASS, Gazebo Harmonic 실제 센서 수신 10대 모두 PASS.

**별도 검증 필요:** 실제 ROS 2 브리지, 원본 경기장 센서 성능 및 자율주행·신호등 인식·3바퀴 완주. 이 항목은 완료한 것으로 표시하지 않습니다.
