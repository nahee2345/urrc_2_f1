# URRC 0·1단계 개발 보고서

**기준일: 2026년 10월 27일**

## 원본 저장소 보호
- 쓰기 저장소: nahee2345/urrc_2_f1
- 원본 읽기 전용: nahee2345/urrc_f1_ws, nahee2345/urrc_2
- 공식 원본 고정 커밋: a23c7eec41b98b67ec3a3a2a46f9d7ed957f8290
- 당시 참고한 urrc_2 커밋: 6a18d72ef3fb2f99229f39b9ad52e7f821ad376d

## 0단계
기준 원본을 upstream/urrc_f1_ws Git 서브모듈로 고정했습니다. docs/source_lock.json에 버전 정보를 기록하고 scripts/check_upstream.py로 기준 커밋, 6개 맵, 신호등 5개, 차량 최대 크기, 원본 스크립트 유무를 확인하도록 만들었습니다.

사용자가 Ubuntu에서 재현한 git clone --recurse-submodules 결과에서 고정 커밋이 정상 체크아웃됐습니다.

## 1단계
- Gazebo Harmonic용 동적 SDF 1.9 차량 생성.
- 차체 1개, 바퀴 4개, 전륜 너클 2개 등 링크 7개.
- 회전 조인트 6개, 차체와 바퀴 충돌 형상 5개.
- 전륜·후륜·사륜구동 선택, 회전속도·토크 제한, 마찰·질량·관성 모델.
- 전륜 Ackermann 조향.
- 기본 후륜구동: 길이 0.460m, 너비 약 0.222m, 높이 0.095m, 질량 1.74kg, 휠베이스 0.34m.

## 실행된 테스트
초기 6개 Python 테스트 통과. Gazebo SDF 플러그인 이름·기준 링크 선언을 수정한 뒤 12개 단위 테스트 통과.

GitHub Actions 평면 Gazebo 실제 시험 결과:
- 전진 1.594m
- 제동 후 추가 이동 0.217m
- 좌회전 +Y 이동 0.254m
- 결과: PASS_FLAT_PHYSICS
- 근거: https://github.com/nahee2345/urrc_2_f1/actions/runs/40435642363

사용자가 Ubuntu에서 실행한 결과:
- 12개 테스트 PASS
- 전진 1.439996m
- 제동 후 추가 이동 0.237083m
- 좌회전 +Y 이동 0.245785m
- 결과: PASS_FLAT_PHYSICS

**미검증:** 실제 몬자 노면·벽 충돌, 신호등 인식 출발, 3바퀴 무충돌 완주.

최신 실행 설명: docs/TEST_PROCEDURE.md
