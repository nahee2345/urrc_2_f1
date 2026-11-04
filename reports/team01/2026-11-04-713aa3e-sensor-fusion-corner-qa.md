# TEAM 01 검증팀 분석 보고서 — 713aa3e (2026-11-04)

- **차량 대상 고정 SHA:** `713aa3e36b26430fdd70d285c09cc87b9b55df59`
- **역할:** 독립 검증 및 실패 원인 분석만 수행. 차량 모델·센서·경로·제어 알고리즘 변경 없음.
- [통합 QA #41775763368](https://github.com/nahee2345/urrc_2_f1/actions/runs/41775763368)
- [빠른 회귀 #41775763366](https://github.com/nahee2345/urrc_2_f1/actions/runs/41775763366)
- [기존 Monza 실행 #41775763360](https://github.com/nahee2345/urrc_2_f1/actions/runs/41775763360)
- **검증 전용 실행기 수정:** `27271e5`; [수정 뒤 재검증 #41782415807](https://github.com/nahee2345/urrc_2_f1/actions/runs/41782415807). 후자는 실행 중이며 이 문서의 물리 주행 PASS를 의미하지 않음.

## 1. 확정 판정

| 검증 항목 | 결과 | 근거 |
| --- | --- | --- |
| 센서 융합 신규 회귀시험 | PASS | v25 `test_both_sensor_jump_differently_means_no_safe_corridor`, `test_one_credible_sensor_beats_other_glitch`, `test_current_sensors_agree`, `test_previous_path_not_permission_to_drive` 통과 |
| 오프라인 전체 | **FAIL: 175개 중 1개** | `test_v7_grid_and_depth_corner.V7Tests.test_corner_selects_adaptive_not_constant_arc` |
| 400줄 제한 | **FAIL** | `src/team_01/team01_race/models/team01_car/model.sdf` 636줄, +236줄 |
| 원래 통합 QA 실제 Gazebo | **BLOCKED / 미실행** | ROS 환경 설정 `AMENT_TRACE_SETUP_FILES: unbound variable` |
| 원래 전용 Monza 실제 Gazebo | **BLOCKED / 미실행** | 175개 중 회귀시험 실패로 종료 |
| 실제 센서 융합, 코너 조향, S자 주행, 완주 | **UNVERIFIED** | 해당 커밋의 정상 실주행/rosbag 부재 |
| 수정한 독립 QA 실제 Gazebo | **재실행 중** | `27271e5`은 QA 셸만 수정; 차량 코드 기준은 713aa3e와 동일 |

## 2. 첫 번째 확정 원인: 적응형 코너 경로가 안전 경계에서 탈락

- `planning.py` 207~245행: 안전한 적응형 후보를 14m→4m→2.3m 순서로 검사하고, 전부 실패하면 `constant_arc`로 전환.
- `curved_path.py` 110~122행: 차체 점유 공간 + 회전(Yaw)에 따른 횡방향 폭을 반영하여 `edge_cross`이면 후보 거절.
- **실제 unit-test 진단:** `edge_cross:x=1.96,y=0.69,R=-0.14,L=1.06,yaw=0.52,foot=0.39,shortfall=0.017`가 세 후보에서 반복.
- 해석: 미래 차체 중심 y=0.69m, 좌측 경계 y=1.06m, 유효 반폭 0.39m → **약 0.017m(1.7cm) 부족**. 단지 `adaptive_pursuit` 문자열이 바뀐 것이 아니라, 예측된 차체가 인식된 좌측 안전 경계를 일부 침범하여 탈락한 것.
- **이 자료만으로 실제 차량이 벽에 닿았다고 볼 수는 없음.** 이는 합성 코너 단위시험의 미래 궤적/안전 검증 계산임.

### 제작팀 필수 수정 방향
1. 테스트 재현 상황에서 `reference_scheduler`의 기존 직선→새 코너 경로 강제 갱신, `reference_path` 5차 보간, Stanley의 heading/curvature feedforward, 실제 `first_steer`를 같은 틱으로 출력할 것.
2. `curved_path.py`의 x=1.96m에서 (계획 경로 중심, 예측 차체 중심, 좌/우 인식 경계, yaw, 점유 폭, 장애물) **시각화를 나란히 작성**하여 곡률 선행조향 과다, reference interpolation 지연, 측면 여유 부족을 구분할 것.
3. **안전 경계를 축소해서 테스트만 통과시키지 말 것.** 코너의 곡률 연속 경로 생성/부드러운 조향/선행 감속 또는 더 안전한 오프셋 경로 탐색으로 자동차 전체 점유폭이 도로 안에 들어오게 할 것.
4. 3가지 horizon 재시도 결과가 전부 동일 x=1.96m라면 horizon 축소만 반복해도 해결되지 않음. 안전한 대안 경로가 없으면 BRAKE는 유지.
5. 수정 후 v7 단위시험과 v9/v13 연속 코너, 실제 Gazebo 코너 및 연석 여유·실측 조향을 반드시 재검증. **중앙 복귀 강제 및 직선 S자 조향은 금지**.

## 3. 두 번째 확정 원인: 독립 QA가 물리 검증 시작 전 종료

- `scripts/qa_team01_gazebo.sh` 기존 3~8행: `set -Eeuo pipefail` 상태에서 `source /opt/ros/jazzy/setup.bash` 실행.
- Actions 오류: `AMENT_TRACE_SETUP_FILES: unbound variable`. **ROS 환경 스크립트 중에 `nounset`을 비활성화해야 함.**
- 부가 잠재 실패: 인수 없는 CI 실행에서 `[[ "${1:-}" == "--local" ]] && LOCAL=1`는 false를 반환해 `set -e`로 종료 가능.
- **QA 전용 수정 완료 `27271e5`:** ROS 환경 source 전후 `set +u`/`set -u`, 인수 조건문 `if ...; then ...; fi` 처리. 실제 Gazebo 결과는 새 Actions 실행 결과로만 인정.
- 재검증 아티팩트에 rosbag2 `metadata.yaml`, `drive_metrics.json`, `trajectory.csv`, `lap.json` 유무 확인. 실행기 실패와 차량 물리 실패를 분리.

## 4. 센서 융합 코드 검토 및 미검증 위험

- `road_belief.py` 53~71행: 기존 유효 도로가 있고 양 센서가 서로 >0.35m 불일치하며 각자 기존 도로에서 >0.35m 이동했으면 `SENSOR_CONFLICT_NO_CONSENSUS`로 안전 정지. 새 v25 회귀 통과.
- 그러나 최초 진입 `previous=None`일 때 두 센서가 심하게 불일치하고 시공간 융합이 실패하면 `road_belief.py` 58~75행은 `rgb_depth`를 기본으로 선택하는 경로가 존재. **안전상 추가 검사 대상이며 실제 위험 발생은 아직 입증되지 않음**.
- `node.py`의 RGB·Depth·LiDAR 시간 기준을 실제 20Hz rosbag으로 검증해 오래된 RGB/Depth가 새 경계로 사용되지 않는지 확인.
- 105° RGB vs 92° Depth는 카메라 모델의 `horizontal_fov`, 픽셀 투영과 이미지 실제 마스크의 일치를 시각 검증. 코드 기본값과 물리 모델 불일치 여부 확인 필요.
- BEV의 unknown 영역을 free road로 잘못 간주하지 않는지; 실제 전방 연석과 Depth-LiDAR 벽 대응 오차가 좁은 코너에서 0.35m 임계값을 오가며 STOP 반복하는지 계측.

## 5. 필수 Gazebo 실주행 QA 수치

- `ready_for_lights` → 신호 점등·소등 → camera permission → adapter positive command → 실제 Gazebo `gz_odom` 5m 이동
- 도로 중심선 편차는 **참고 자료만**. 불필요한 중앙 진입 대신 기존 안전 경로를 유지하는지 평가
- 과거 계획 경로 대비 실측 궤적 오차 평균/P95, 도로 경계 최소 여유, 직선 평균 실측속도, 조향 절대값 평균/P95, 조향 변화량(°/m), 좌우 반전, 모드별 BRAKE/STOP 원인
- rosbag·raw timestamp, RGB/Depth/LiDAR 프레임, BEV 및 감지된 경계 시각화 확보
- 실제 5m 미만 이동 또는 경로 대응 표본 15개 미만이면 경로 정확도 **미검증**. 실제 1랩은 독립 `GEOMETRIC_LAP_PASS`만 승인
- 실측 5m/s 첫 랩 안정화 및 재현성 전 8/10m/s 성능을 승인하지 않음.

## 6. 로컬 재현

```bash
cd ~/urrc_2_f1
git pull --ff-only
git submodule update --init --recursive
source /opt/ros/jazzy/setup.bash
python3 scripts/qa_team01_source_limits.py --limit 400

# 실제 Gazebo 및 rosbag, 경로 정확도 평가 (기존 Gazebo 프로세스 종료 후)
bash scripts/qa_team01_gazebo.sh --local
```

아티팩트: `~/team01_qa/<날짜_시각>/`. 주요 파일은 `qa_runner.log`, `rosbag_core/`, `trajectory.csv`, `drive_metrics.json`, `qa_run_report.md`.

**이 문서는 713aa3e의 첫 QA 분석이며, 새로운 실제 Gazebo 실행이 끝나면 그 결과는 별도 SHA/Run ID로 추가 기록해야 함.**
