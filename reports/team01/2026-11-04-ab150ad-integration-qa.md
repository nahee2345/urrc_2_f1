# TEAM 01 독립 재검증 — 차량 커밋 ab150ad

검증일: 2026-11-04 (KST)  
고정된 **차량 개발 커밋:** [`ab150ad7286cd833c083b7fbc391dcf19a7da6b1`](https://github.com/nahee2345/urrc_2_f1/commit/ab150ad7286cd833c083b7fbc391dcf19a7da6b1)  
비교 기준: [`713aa3e36b26430fdd70d285c09cc87b9b55df59`](https://github.com/nahee2345/urrc_2_f1/commit/713aa3e36b26430fdd70d285c09cc87b9b55df59)  
검증팀의 수정 범위: **QA 측정기만 보정**. 차량 경로·구동·센서 코드 수정 없음.

## 1. 확인된 PASS — 개발 코드 및 CI

| 항목 | 판정 | 실제 근거 |
|---|---|---|
| 전체 기존/신규 Python 회귀시험 | **PASS, 178/178** | [빠른 오프라인 #41784297781](https://github.com/nahee2345/urrc_2_f1/actions/runs/41784297781), job 126422760312: `Ran 178 tests`, `OK` |
| 기존 v7 코너 adaptive 선택 | PASS | `test_corner_selects_adaptive_not_constant_arc ... ok` |
| ±0.12m 오프셋 Stanley 조향 반영 | PASS | 신규 v30 `test_offset_changes_stanley_not_just_target_display ... ok` |
| 좁은 도로에서 위험한 오프셋 후보 거절 | PASS | v30 `test_offset_does_not_allow_unsafe_narrow_road ... ok` |
| 최초 RGB/Depth와 LiDAR 불일치 시 fail-closed | PASS | v25 `test_initial_sensor_conflict_rejects_road ... ok` |
| 전체 400줄 검사 | PASS | [독립 QA #41784297810](https://github.com/nahee2345/urrc_2_f1/actions/runs/41784297810) `Enforce 400-line maximum...` step success |
| 핵심 코드 길이 | PASS | `planning.py` 392줄, `node.py` 399줄, `curved_path.py` 156줄, `model.sdf` 189줄 |
| SDF 모델 데이터 보존 (정적 텍스트) | PASS | 713aa3e 대비 `> whitespace <` 정규화 후 전체 SDF 텍스트 일치, normalized 12,082 bytes 양측 동일 |

**정적 시험이 통과했다는 사실과 실제 Gazebo 완주/주행 안정성은 별개입니다.** SDF 실제 로딩·플러그인·바퀴 물리 및 센서 메시지는 Gazebo에서 입증해야 합니다.

## 2. 실제 물리 시험 — 현재 검증 중, 성공 주장 금지

- [차량 `ab150ad` 독립 Gazebo QA #41784297810](https://github.com/nahee2345/urrc_2_f1/actions/runs/41784297810): 오프라인/400줄 단계 PASS 후 **Real Gazebo Monza run 진행 중**.
- [차량 `ab150ad` 원래 실제 Monza #41784297803](https://github.com/nahee2345/urrc_2_f1/actions/runs/41784297803): 실제 주행 단계 **진행 중**.
- 물리 센서 출발, `gz_odom` 실제 이동거리, `GEOMETRIC_LAP_PASS`, 1랩 평균/최대 실측속도, 차체 벽 충돌 및 도로 이탈, S자 조향 증거: **UNVERIFIED**.
- 이 시점에는 두 실행의 완성된 artifact가 없으므로 임의의 숫자나 PASS를 기입하지 않음. GitHub Actions가 끝나면 해당 실행의 `lap.json`, `drive_metrics.json`, `trajectory.csv`, `rosbag_core/metadata.yaml`, controller/gazebo logs로 별도 판정.

## 3. 코드 추적 — 변경 사항별 정확한 구현

**① 대안 경로 탐색 (구현 PASS / 동적 주행 UNVERIFIED)**

- `planning.py` 210~238행: `(이전 안전 오프셋, 0, -0.08, +0.08, -0.16, +0.16)m` 순으로 중복 제거 후, 각 후보의 `14→4→2.3m` horizon을 검사. **첫 번째 안전 후보**가 선택됨.
- `curved_path.py` 78~83행: 1.35m에 걸친 smoothstep으로 offset을 전체 참조경로에 반영하여 Stanley 조향 예측에 적용.
- 기존 도로 경계/장애물/차체 회전 footprint 검사를 그대로 보존 (`curved_path.py` 109~139행).
- `node.py` 310~334행: 20Hz `candidate_shift_m`, `candidate_xy`, `road_bounds_xy`, `candidate_rejections` 진단 제공.

**② 시작 센서 불일치 (구현 PASS / Gazebo UNVERIFIED)**

- `road_belief.py` 59~68행: 초기 `previous=None`이고 현재 두 센서 도로가 0.35m 넘게 불일치하면 `INITIAL_SENSOR_CONFLICT`로 유효 경로를 거절함.
- 초기 false positive, 유효한 코너 진입 시 실제 센서 간 불일치, 일시적 stale frame 후 회복은 물리 데이터로 확인 필요.

**③ SDF 압축 (정적 동일 PASS / Gazebo UNVERIFIED)**

- `model.sdf` 636→189줄. 모델·센서 데이터 XML 태그 사이 공백을 제외한 정적 내용 동일. 실제 SDF 파싱, plugin 로딩/차축 구동·Odometry·카메라·LiDAR 토픽은 반드시 물리시험에서 재확인.

## 4. 제작팀이 추가 검증해야 할 잠재 문제 (현재 FAIL 확정 아님)

### P1: 안전 후보의 선택 기준 ≠ 최소 조향의 최적화

`planning.py`는 후보별 `Candidate.score`가 있더라도 **첫 번째 유효한** `adaptive[0]`을 선택합니다. 이전 오프셋 유지가 안전·부드러움에 유리할 수 있지만, 다른 후보가 더 작은 조향각/더 높은 속도/더 큰 안전 여유를 제공해도 선택되지 않을 수 있습니다.

**요청:** 동일한 Gazebo rosbag에서 선택 후보와 대안 후보들의 실제 횡오차 P95, 예측 yaw/조향각, 도로 여유·실측 속도를 비교하여 과도한 경로 보유인지 확인. 단기적으로 안전 유지 + 좌우 전환 방지를 우선하며, 불필요한 복귀 조향을 만들지 않는 선택 히스테리시스 기준만 개선. **중앙선 편차를 목적함수로 강제하지 말 것.**

### P1: 새 코너에서 좌/우 후보 토글·부호 반전

`candidate_shift_m`이 `0 ↔ -0.08 ↔ +0.08`처럼 20Hz 틱마다 바뀌는지, 실제 `cmd_steer` 부호 반전 및 `gz_odom` yaw-rate와 대응하는지 시계열 검증. 코너 진입/탈출의 경계 검사를 유지하고 장애물 없는 직선에서는 조향 변화량(°/m)을 최소화.

### P0: 실제 시작 및 정지

출발 준비→빨간불 점등→소등→permission→adapter positive command→Odometry 이동 `5m` 이상을 **각 단계의 시간과 증거**로 확인. 양의 명령에도 차가 움직이지 않으면 ROS→Gazebo bridge, 바퀴 관절·접촉, spawn을 분리 분석. sensor stale, `SENSOR_CONFLICT`, `no verified road-safe candidate` 정지 이유를 처음 발생한 시점부터 추적.

## 5. QA 측정기 자체 발견 오류 및 수정

- 이번 새 차량은 안전한 ±0.08m/0.16m **대안 경로**를 따라갈 수 있음.
- 기존 `scripts/qa_team01_metrics.py`는 선택된 경로 대신 `reference_xy`(오프셋 미반영 기준 참조)를 추종 오차 계산에 사용함. 이는 중앙 복귀를 원칙으로 하지 않는 평가 기준에 어긋나고, 의도된 오프셋 경로를 오차로 잘못 계산할 수 있음.
- **QA 전용 수정 [`16a05c9`](https://github.com/nahee2345/urrc_2_f1/commit/16a05c97168fd7fb166d1e79e289dd066fffd33c)**: 0.6~1.5초 전에 실제 선택된 `candidate_xy`를 Gazebo odom 좌표로 변환 후 실제 위치와 비교. `adaptive_pursuit` 후보가 없으면 경로 오차 미검증. 정상 레이싱라인을 중앙으로 몰아가지 않음.
- [수정된 QA 물리검증 #41784937504](https://github.com/nahee2345/urrc_2_f1/actions/runs/41784937504) 별도로 진행. **위 수정은 차량 로직 변경이 아니므로 차량 평가 대상은 계속 ab150ad와 동일합니다.**
- 수정 이전 `ab150ad` Run #41784297810의 `drive_metrics.json` 계획경로 오차 수치는 경로 오프셋 상태와 대조 없이 절대 성능 판정 근거로 사용하지 말 것.

## 6. 최종 승인 조건

1. 178개 오프라인 및 400줄 PASS 유지.
2. Gazebo 실제 모델 로드·센서·조향·명령 전달·실측 이동 정상.
3. 기존 v7 합성 코너뿐 아니라 실제 Monza 첫 코너 경계 안전 여유를 확보.
4. 직선에서 경계 안전, **최소 조향 + 높은 실측속도**, 좌우 S자 반복 없음. 도로 중앙에 있지 않다는 이유만으로 FAIL 금지.
5. 실제 `GEOMETRIC_LAP_PASS` 및 충돌/이탈 없음; rosbag·궤적 CSV와 그래프 저장.
6. 5m 미만 실이동 또는 실측 경로 대응 샘플 15개 미만이면 정확도는 **UNVERIFIED**.
7. 안전 5m/s 1랩 반복 성공 전 8m/s·10m/s 가속 승인 금지.
8. 신규 안전 후보의 장점이 실증되지 않으면 알고리즘 전면 교체 권고 금지. 센서·구동 실패면 해당 영역부터 수정.

## 7. 로컬 저장 및 Gazebo/rosbag 재현 명령

```bash
cd ~/urrc_2_f1
git pull --ff-only
git submodule update --init --recursive
python3 scripts/qa_team01_source_limits.py --limit 400
source /opt/ros/jazzy/setup.bash
bash scripts/qa_team01_gazebo.sh --local
```

로컬 QA 데이터: `~/team01_qa/<날짜_시각>/`, 실행 로그 `qa_runner.log`, rosbag `rosbag_core/`, `trajectory.csv`, `drive_metrics.json`, `qa_run_report.md`.

**중간 보고서:** 실제 물리 run 종료 시 해당 SHA와 run ID의 아티팩트를 검증한 후 확정 FAIL/PASS와 최초 실패 원인, 제작팀 수정 요청을 별도 기록해야 합니다.
