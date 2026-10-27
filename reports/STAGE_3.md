# URRC 3단계 재진행 — 몬자 원본 지형 및 신호등 일치성

기준일: 2026-10-27

## 원본 재검토

- urrc_f1_ws main: a23c7eec41b98b67ec3a3a2a46f9d7ed957f8290
- 3단계 서브모듈 고정 SHA와 동일: GitHub 원본 변경 없음.
- 원본 monza_geometry_report.json 고저차: 약 2.563m.
- 원본 도로: model://urrc_monza/meshes/road.obj, 충돌: road_collision.obj.
- 공식 run_world.py의 최초 실행에는 v10_assets.enhance_track가 필요하며, 이전 3단계는 이를 생략함.
- 정식 원본 실행기는 5개 빨간등의 임시 모델 전체 제거로 일괄 소등하며, spectator camera 4대·아스팔트 텍스처를 실행시 생성함.

## 원인과 수정

사용자에게 보인 3단계 맵이 원본과 다른 외형/시점을 보인 주된 코드 차이는 **원본 v10.8 에셋 사전 생성 과정과 원본 GUI 시점의 누락**입니다. 원본 자체의 고저차가 제거된 것으로 확인되지는 않았습니다.

- stage3/assets.py: 원본 파일을 generated/stage3/resources 아래로 **복제**하고, 공식 enhance_track를 **복사본에만** 실행.
- stage3/instrument.py: 원본 복사 후 개선된 World SDF에 위치/접촉 감시 차량 추가.
- stage3/terrain.py: 원본/복사본 도로 정점 Z 범위·개수 대조, 충돌 OBJ SHA256 대조, spectator 4대, 5등 단일모델, 지면 충돌 확인.
- scripts/stage3_prepare_monza.py: terrain_fidelity.json으로 실측 정적 검증 결과 보존.
- scripts/stage3_run_monza.py: 원본 monza_grid.gui.config, 격리 리소스 경로와 세션으로 Gazebo 실행.
- scripts/stage3_lights.py: 원본 v10.8 스크립트의 on/off/sequence를 격리 모델에 수행. 원본 GitHub 변경 없음.
- scripts/stage3_monza_smoke.py: 원본 향상 월드의 실제 카메라/포즈 메시지 및 5등 서비스 점등→일괄소등 시험.
- tests/test_stage3_map_fidelity.py: 고저차 삭제 방지 회귀 검사.
- .github/workflows/stage3-race-supervisor.yml: 공식 자원 재생성/높이/충돌 메시 무결성 검사와 실제 Gazebo 신호등 서비스 시험 자동화.

## 증거와 한계

이전 3단계 CI #40458697552에서는 기존 월드로 Python 44개 검사 및 실제 카메라/포즈 수신을 확인했습니다. 이 시험 결과만으로 공식 시각 자산·높이 표현이나 5등 소등 성능을 검증했다고 볼 수 없습니다.

새 CI는 고저차 보존과 원본 신호등 동작을 추가 검증합니다. 결과는 GitHub Actions의 **가장 최근 3단계 실행 완료 로그**를 확인해 판정해야 합니다.

실제 카메라 광학 인식/ROS2 출발 안전 게이트 실연동 및 3바퀴 무충돌 AI 주행은 아직 통과했다고 보고하지 않습니다.

원본 urrc_f1_ws 및 urrc_2에 파일 수정과 Git push를 수행하지 않았습니다.

## 사용자 실행

docs/STAGE3_LOCAL_INTEGRATION.md 를 확인하고 기존 저장소에서 git pull --ff-only만 실행하면 됩니다.
