# TEAM 01 제작팀 인수인계 — 1~6단계 192 PASS / OFF_TRACK / LAP_TIMEOUT 대응

대상 QA 보고: `reports/team01/2026-11-06-9a05a5c-six-stage-final-qa-dev-handoff.md` (차량 SHA `9a05a5c1c62db19e7c7280727008ae844f880b00`).
제작팀은 **GitHub 차량 코드만 수정**했습니다. 독립 QA, 오프라인 회귀 실행 및 Gazebo 실주행은 **검증팀 담당**입니다.
이번 코드 변경을 실제 Monza 1랩 PASS로 간주하면 안 됩니다.

## 입력 증거: 실행을 혼합하지 말 것
| 출처 | 관측 | 한계 |
|---|---|---|
| 독립 QA #42164504112 attempt1 | 9.002m, LAP_TIMEOUT, 경계 밖 제한 조향 2047회, 장애물 제동 3회, 복귀 1회 | 제한 이벤트의 일차 원인은 미확정 |
| 독립 QA #42164504112 attempt2 | 192개 회귀 PASS, WAIT_RED에서 fallback=false/speed=0 관측, LAP_TIMEOUT | 종단 실제 주행거리·전체 RED 이전 Twist 원본 비교는 미완료 |
| 공식 Monza #42164504143 attempt1 | RED 1→5 및 LIGHTS OUT, OFF_TRACK | 최초 이탈 기전·조향 부호·FOV 정합 등 가설 미확정 |
| 공식 Monza #42164504143 attempt2 | CANCELLED, Gazebo 실행 없음 | 주행 성능 증거로 사용 금지 |

## 구현 변경 A — 물리시간 동기화
- `planning.py`: 구동 속도 증감/제동, SteeringLimiter, 전방 정적벽 판단, 선행차 추적, 추월·복귀에 사용되는 `dt`를 ROS 2 `/clock` 기반 `physics_dt`로 변경했습니다. 이전에는 시뮬레이션이 벽시계보다 느려도 조향과 가속이 벽시계 속도로 변화했습니다.
- `node.py`: `RecoveryGuard.update(...,sim_dt)`로 변경; 진단 `/team01/debug/path.control_wall_dt_s`를 추가해 `odom_sim_dt_s`와 대비합니다.
- `steering.py`, `mode_recovery.py`, `rejoin.py`: 정지한 물리 시계에서 임의 최소 경과시간(5~10ms)을 적용하지 않도록 변경.
- 신규 회귀 시나리오: `tests/test_v34_physics_time_controller.py`. 구동 중단(0 sim dt) 시 조향각·복구 타이머가 거짓으로 진전되지 않는지 확인합니다.

## 구현 변경 B — 조향 제한 후 검증된 경계 내 경로 재선택
- 기존 코드: Collision-check된 적응형 경로의 첫 steer→YawFeedback→SteeringLimiter 후, **최종 steer를 일정한 조향으로 가정한 1.25~2.5m 궤적**이 경계 밖이면 매번 `BRAKE: limited steering outside visible road` 처리.
- 새 코드: `steering_recovery.py`에서 **한 동일한 직전 SteeringLimiter 상태**를 기준으로 기존 목표와 작은 대안 목표(±0.05/0.10/0.18rad, 직전 조향, 0)를 각각 정상적인 동일 rate/accel/smoothing limiter에 통과시킵니다.
- 각 **실제 제한된 출력 조향**에 대해 기존 `_within_road`의 원래 차체 너비·안전 여유/관측된 경계, `clearance`의 장애물 여유를 검사합니다. 적절한 대안이 있으면 이를 짧은 구간 `bounded_recovery` 경로로 선택하고 `free`를 실제 검증 구간으로 제한해 속도 제어기의 독립 감속조건을 유지합니다.
- 대안 탐색 허용 조건: 관측된 근거리 구간이 **현재 속도에서 제동거리 + 기존 emergency margin + 0.15m**보다 충분히 길어야 함. 이 조건을 충족하지 못하거나 모든 후보가 불안전하면 종전처럼 `BRAKE`. 실패 후보의 limiter 상태는 원상복구합니다.
- 도로 원본/설정 안전폭/충돌물리/적백색 경계값/조향 부호/Stanley/MPC는 변경하지 않았습니다. **경계 이탈 근본 원인 해결로 단정하면 안 됩니다.**
- 신규 회귀: `tests/test_v35_bounded_steering_recovery.py` (허용 조향, 축소 복귀, 실패시 상태 롤백, 안전거리 부족 시 대안 탐색 차단).
- 새로운 경로 진단 `/team01/debug/path.steering_recovery`: `VERIFIED`, `BOUNDED_REPLAN`, `NO_REACHABLE_SAFE_STEER` 또는 `NONE`. 기존 `candidate_rejections`, `road_bounds_xy`, `candidate_xy`, `cmd_steer`와 같은 시점으로 기록.

## QA팀 P0 재검증 조건
1. 신규 SHA에서 기존 192개+v34/v35 시험 전체, 400줄 제한, SDF 구동 검사. `upstream/urrc_f1_ws` gitlink/Monza 원본 world/model 해시 **실행 전후 동일** 필수.
2. 정상 camera 모드 지연 RED 시퀀스: RED 이전 `sent_linear_mps>0` 0건. `no_signal`의 스폰 7초 동작은 별도로 구분.
3. 같은 Gazebo 시뮬레이션 타임스탬프에서 `/clock`, `/team01/debug/path`(physics/wall dt, `steering_recovery`, `planner`, `candidate_rejections`, `reference_xy`, `road_bounds_xy`), `/team01/debug/adapter`(command freshness/stop reason), `/team01/cmd_vel`, 실제 휠 관절·접촉·`/team01/gz_odom`를 원본 rosbag에 보존.
4. 2047회 이전 제한 이벤트를 **경계 연속성·시간·주행거리별**로 분리: `limited steering outside visible road`가 감소하는지, `BOUNDED_REPLAN`이 실제 장애물/연석 침범 없이 발생하는지, `NO_REACHABLE_SAFE_STEER`가 남는지. 충분한 도로 증거 없이 대안 허가 금지.
5. 공식 최초 OFF_TRACK 시각 ±5초: RGB/Depth/LiDAR 관측 도로 경계, 차량 heading/yaw rate, 요청/적용 steer, 실제 world pose와 관절 및 휠 접촉 비교. 경계 안전여유 위반과 **base_link 0.3m 공식 실격**을 분리.
6. 별도 실제 5m 연속 이동, 첫 코너 무이탈, 직선 최소 조향·S자 억제, Monza 1랩 `GEOMETRIC_LAP_PASS`, rosbag2 metadata/DB, trajectory.csv, drive_metrics.json, lap.json 확보.
7. 새 실주행 실패와 CI/report add/add 충돌은 분리하며, attempt1/attempt2 결과를 혼합하지 않습니다.

**제작 단계 판정:** 차량 코드와 회귀 시나리오 추가 완료 / 실제 개선 효과 및 1랩 PASS **미검증**.
