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


## 후속 정적 검증 완료

- Python 전체 테스트 **77개 모두 PASS** — Monaco 원본 메시·검정 차량·신호등 ROI 자동교정 모의 프레임·3랩 판정/안전 게이트 포함
- GitHub Actions: https://github.com/nahee2345/urrc_2_f1/actions/runs/40496330149
- Gazebo 최신 동적 재시험: GitHub runner의 일시적인 외부 APT 403 오류 방지 설정을 적용한 뒤 진행 중. 완료 상태 확인 전까지 새 실행의 동적 결과는 미확정.
- 최초 실제 Monaco 물리·센서 PASS는 #40495144919에서 독립적으로 확보했음.

**실제 신호등 카메라 RGB 교정 및 실제 통합 3랩 완주 결과는 여전히 미검증입니다.**


## 5단계 최종 확인된 GitHub 자동 검증

- 실행: https://github.com/nahee2345/urrc_2_f1/actions/runs/40496330149
- 전체 작업 2개 모두 success.
- **Python 단위 테스트 77개 PASS**.
- Monaco 원본 지형 높이·충돌 메시·관전 카메라·원본 5등 모델 구조 PASS.
- 실제 Gazebo Monaco 이미지 메시지 **8,841,025바이트** 수신.
- 실제 Gazebo Monaco 운영자 포즈 메시지 **2,115,497바이트** 수신.
- 실제 Monaco 전륜 모터 주행 변위 **1.674858m** PASS, 포즈 샘플 3,305개.
- 앞·뒤 측정 Z **2.0750m → 2.1566m** (Monaco 3D 노면 변화 반영).
- 위 시험은 **실제 신호등 순차 점등·소등을 Monaco 카메라가 감지하는 시험이나, 실제 RGB 기반 자동 3랩 완주 시험이 아닙니다.**
- 원본 urrc_f1_ws, urrc_2 GitHub는 수정하지 않았습니다.

자동 시험 결과를 과장하지 않도록, 센서 스모크 JSON의 설명 문구에서도 신호등을 실제 검증하지 않은 경우는 분리 표시하도록 수정했습니다.
