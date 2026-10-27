# 1단계 보강 및 검증 결과 — 2026-10-27

## 코드 변경 범위
쓰기 저장소: https://github.com/nahee2345/urrc_2_f1
원본 읽기 전용: urrc_f1_ws (a23c7eec41b98b67ec3a3a2a46f9d7ed957f8290), urrc_2.
**두 원본 저장소에는 변경 사항이 없습니다.**

## Gazebo 호환성 오류 수정
1. 기존 SDF의 임의 시스템 플러그인 이름을 Gazebo Harmonic이 등록한 `gz::sim::systems::JointController` / `gz::sim::systems::JointPositionController`로 교정.
2. 잘못된 자식 요소 `<canonical_link>`를 모델의 `canonical_link="base_link"` 속성으로 변경.
3. 조인트 기준 자세를 자식 링크 좌표로 명시하고, 회전축 초기 좌표계를 `__model__`로 설정.
4. 재발 방지용 SDF 구조 테스트 여섯 개 추가.

## 차량
- FWD, RWD, AWD 동적 구동 선택 지원.
- 7개 물리 링크, 6개 회전 조인트, 5개 충돌 형상.
- 후륜구동 기본 모델 질량 1.74kg, 보수적 전체 크기 0.460 × 0.222 × 0.095m.
- 물리 엔진 기반 회전·마찰·관성 적용. 모델 순간이동 제어 금지.

## 직접 실행 결과
```text
test_canonical_link_is_model_attribute ... ok
test_independent_ground_world_and_vehicle ... ok
test_invalid_geometry_cannot_enter_gazebo ... ok
test_joint_frames_and_link_references ... ok
test_plugin_system_class_names ... ok
test_pose_telemetry_parser ... ok
기존 6개 단위 테스트 ... ok

Ran 12 tests
OK
```
Python compileall: PASS.
SDF 모델 생성: PASS.
독립 평면 월드 SDF 생성: PASS.

## 실제 물리 주행
로컬에서 `python3 scripts/stage1_smoke.py --run`을 호출했지만 `gz` 명령이 설치되지 않아 결과는 **UNAVAILABLE**. 따라서 실제 주행 검증을 완료했다고 주장하지 않습니다.

GitHub Actions: https://github.com/nahee2345/urrc_2_f1/actions

설정된 CI는 Ubuntu 24.04에 Gazebo Harmonic을 설치하고 격리 월드에서 물리 구동·차량 이동을 자동 검사합니다. 실제 실행 결과는 Actions의 job conclusion 및 첨부된 JSON으로만 판단합니다.

## 판정
- Python 정적 검사 **PASS (12/12)**.
- SDF 형식 설계 오류 수정 **완료**, 실제 Gazebo SDF 파싱은 CI 또는 Ubuntu에서 재확인.
- Gazebo 실제 바퀴 운동 / 접지 / 제동 / 조향 **결과 확인 전까지 보류**.
- AI 주행·3랩 시험 **미실행**.


## 실제 GitHub Actions Gazebo 검증 완료 (업데이트: 2026-10-27)

- **실행:** https://github.com/nahee2345/urrc_2_f1/actions/runs/40435642363
- **검증된 코드 커밋:** ea01a2b8018c95c990c4ec37cc54c436509e6f49
- **결론:** Ubuntu 24.04 GitHub Actions에서 정적 검사와 Gazebo Harmonic 물리 구동 두 작업 모두 성공.
- **Gazebo 판정:** `PASS_FLAT_PHYSICS` — 원본 Monza가 아닌 별도 평면에서의 실제 주행.
- 전진 위치 변화 +X **1.594181m**
- 제동 명령 후 추가 이동 **0.217325m**
- 좌회전 시 +Y 위치 변화 **0.253506m**
- 관측된 차량 pose **1,886개**
- 차체의 마지막 z 좌표 약 **0.040m**로 초기 바퀴 반지름과 일치.
- 이전 초기 테스트에서 발견한 바퀴 명령 부호 반전(전진 명령 시 후진)을 수정한 뒤 통과함.
- 원본 `urrc_f1_ws` 및 `urrc_2` 저장소 변경 없음.
- **미검증:** 몬자 트랙의 메시/노면 접촉, 벽 충돌, 개별 3바퀴 완주, 카메라 출발, 10대 동시 주행.
- 원시 결과: `reports/stage1_flat_physics_ci_20261027.json` (실제 CI 로그 기반).

**판정 갱신:** 1단계 공통 4륜 차량의 *평면상 기본 물리 움직임* 통과. 몬자 환경 통합·안전 검증은 다음 필수 검증 항목으로 유지.
