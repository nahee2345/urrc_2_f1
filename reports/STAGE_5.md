# 5단계 개발·검증 보고서 — Monaco

기준일: 2026-10-27

## 개발 기준

- 공식 urrc_f1_ws의 Monaco를 기반으로 AI-01 실험.
- 길이 약 667.16m, 최소 도로 폭 1.4m, 고저차 약 8.41m.
- 원본 urrc_f1_ws, urrc_2의 수정·push는 금지.
- 작업 대상 저장소는 nahee2345/urrc_2_f1 하나뿐.

## 코드

- stage5/monaco.py: 공식 원본 Monaco 트랙 메시·높이·관전 카메라·신호등을 격리 복제, SHA 무결성 검사.
- scripts/stage5_prepare_monaco.py: 원본 모델 + 검정 AI-01 동적 차량 생성.
- scripts/stage5_run_monaco.py: 공식 Monaco 3D GUI 실행.
- scripts/stage5_lights.py: 공식 다섯 빨간불 순차 점등 후 일괄 소등.
- scripts/stage5_bridge.py: ROS2 카메라/조향/전륜모터 브리지 준비.
- stage5/roi.py 및 scripts/stage5_calibrate_lights.py: 다섯 빨간등 실제 영상의 ROI 자동 교정. 실패 시 출발 차단.
- scripts/stage5_race.py: 카메라 출발 안전 게이트, RGB 추종, 실제 3랩 운영자 계측·충돌 감독 연동.
- scripts/stage5_monaco_sensor_smoke.py: Gazebo Monaco 카메라·감독 포즈 실제 데이터 검사.
- scripts/stage5_monaco_actuator_smoke.py: Gazebo Monaco 전륜 모터 실제 물리 이동 검사.
- tests/test_stage5_*.py: 원본 지형·5등·출발 감지·로그·3랩 무효 판정 회귀검사.

## 검증한 실제 결과

- GitHub Actions #40495144919: 전체 단위 테스트 67개 PASS.
- 원본 Monaco 3D 지형·충돌 메시·5등 임시 모델 보존 확인.
- 실제 Gazebo Monaco 카메라 이미지와 운영자 위치 메시 수신 PASS.
- 실제 Gazebo Monaco AI-01 전륜구동 위치 변위 약 1.642539m PASS.
- 링크: https://github.com/nahee2345/urrc_2_f1/actions/runs/40495144919

이후 빨간불 ROI 자동 교정·레이스 로그 판정 테스트를 추가했고, 최종 GitHub Actions 재시험은 별도로 확인해야 합니다. 한 번의 중간 재시험은 부동소수점 테스트 비교를 너무 엄격하게 설정해 실패했으며 수정했습니다. 다른 중간 재시험은 GitHub runner의 외부 Microsoft APT 403 때문에 Gazebo 설치 전에 중단됐으며 CI 설정만 변경했습니다.

## 아직 PASS가 아닌 것

- 실제 Monaco 시작 카메라 RGB에서 신호등 다섯 렌즈 자동 교정.
- ROS2 메시지 전체 브리지와 출발 안전 인터록의 끝단 통합.
- Monaco 전 구간 헤어핀·언덕 RGB 차선 추종 품질.
- 충돌·코스 단축 없는 실제 자동주행 연속 3랩.
- 원래 계획된 학습형 CNN 주행 모델.

이 모든 사항이 확인되기 전에는 5단계 무충돌 3바퀴 최종 PASS가 아닙니다. 현재 코드는 실제 검증 실행 준비 단계까지 구축했습니다.

사용자 실행은 docs/STAGE5_GUIDE.md에 한국어로 정리했습니다. 모든 코드 수정은 GitHub를 통해 git pull로만 반영합니다.
