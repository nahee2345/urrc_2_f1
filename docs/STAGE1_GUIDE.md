# URRC 7단계 중 1단계 — 실제 물리 차량 생성 및 검증

**쓰기 대상은 오직 nahee2345/urrc_2_f1입니다.** urrc_f1_ws와 urrc_2는 변경하지 않습니다.

## 차량 설계

- SDF 1.9, 동적(`static=false`) 차량.
- 7개 질량·관성 링크(차체 1, 전륜 너클 2, 바퀴 4), 6개 회전 조인트, 차체·바퀴의 5개 충돌 형상.
- Gazebo Harmonic 등록 플러그인 클래스 `gz::sim::systems::JointController`(force mode)로 구동, `JointPositionController`(PID mode)로 조향.
- 모델 canonical link는 `canonical_link="base_link"` **속성**으로 지정.
- 대회 +우회전 조향 입력을 -Z 방향으로 변환하고 Ackermann 내륜/외륜 각도를 분리.
- 경기 차량 최대 외형 0.55 × 0.25 × 0.18m. 기본 후륜 차량 보수적 외접 크기 약 0.460 × 0.222 × 0.095m, 질량 1.74kg.
- 차량 pose를 직접 갱신하지 않고 토크·마찰·회전 조인트를 이용함. 실제 물리 안정성은 Gazebo로 검증할 것.

## 실행한 정적 테스트

```bash
python3 -m unittest discover -s tests -v
python3 -m compileall -q vehicle_generator scripts tests
python3 scripts/build_vehicle.py
python3 scripts/stage1_smoke.py --prepare
```

Python 단위 테스트 총 12개가 통과했습니다. 위 테스트는 물리 주행이 아니라 정적 검사입니다.

## Gazebo 자동 물리 시험

Gazebo Harmonic이 설치된 Ubuntu 24.04에서:

```bash
python3 scripts/stage1_smoke.py --run
cat generated/stage1_smoke_result.json
```

이 스크립트는 기존 Monza 월드를 수정하지 않고 별도 평면 세계를 생성합니다. 실제 Gazebo를 headless 구동한 뒤 후륜 모터에 명령을 보내 모델 위치 변화와 높이를 검사하고 결과 JSON을 작성합니다. `PASS_MOTION_ONLY`는 실제 평면 이동만 검증했음을 의미합니다.

GitHub Actions에도 동일한 정적/헤드리스 Gazebo 시험을 추가했습니다: https://github.com/nahee2345/urrc_2_f1/actions

## 1단계 완전 통과 조건

실제 Gazebo에서 스폰, 4륜 접지, 가속, 제동, 코너링, 조향 제한, 충돌 거동을 확인해야 합니다. 별도 평면 주행만 성공한 경우에도 1단계 전체를 PASS로 처리하지 않습니다. 이후 센서 설치는 2단계입니다.

몬자에서 차량을 스폰하는 명령은 docs/TEST_PROCEDURE.md에 있습니다.
