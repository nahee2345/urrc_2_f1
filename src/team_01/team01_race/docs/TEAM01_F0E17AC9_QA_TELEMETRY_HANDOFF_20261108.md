# TEAM01 f0e17ac9 최종 FAIL 후 제작팀 계측 보강 / 검증팀 인수인계

기준 독립 보고서: `reports/team01/2026-11-06-f0e17ac9-r42173871179-r42173871484-attempt1-final-qa.md`
원본 차량 f0e17ac9, 해당 보고 커밋 090b2114.
제작팀은 **독립 Gazebo 주행/QA 검증을 수행하지 않습니다**. 모든 기존 FAIL은 후속 검증팀이 새 개발 SHA로 다시 판정해야 합니다.

## 1. 이전 검증에서 확정된 사실
- 독립 QA #42173871179 attempt1: 18.176m, LAP_TIMEOUT, BRAKE 993회, `limited steering outside visible road` 991회. 기존 9.002m/2047회 대비 증가·감소는 관측됐으나 같은 조건 A/B 인과는 미확정.
- 공식 Monza #42173871484 attempt1: 17.943m, OFF_TRACK. `max_offroad_m=0.204m`는 **안전 마진 0.145m를 뺀 경계에 대한 초과량**이므로 실제 `base_link`가 도로 외부로 0.204m 벗어났다는 뜻이 아닙니다.
- 공식 판정기 `lap_validation.py`: `outside=max(0,abs(lateral)-(width/2-.145))`, `outside>.025`인 표본 8개면 OFF_TRACK. 따라서 0.204m 보고 수치라면 실제 도로 바깥 중심점 초과량은 **대략 0.204-0.145=0.059m**(표본상 공식 실격 `>0.3m`은 미입증).
- 독립 `drive_metrics.json`은 Odometry 0m/0표본/UNVERIFIED, 별도 `lap.json`은 18.176m/1583표본/SPAWN_RELATIVE. 전자의 메트릭 노드는 데이터 미수신/조기종료/좌표변환 문제 등으로 **원인 미확정**입니다. `qa_metrics.log`·노드 PID/exit code·ROS subscriber QoS·spawn_report 위치·타임스탬프를 검증팀이 확인해야 합니다.
- `trajectory.csv` 헤더만 기록, 독립 rosbag `*.db3.zstd` 수동 해제 오류, 공식 Monza rosbag 누락. 실제 휠 스핀·토크/접촉 데이터 없음.

## 2. 제작팀이 GitHub 차량 소스에 반영한 사항
- `planning_geometry.py`의 **기존 충돌/경계 검사 결과와 여유폭은 그대로** 두고, 탈락한 일정 조향 궤적의 최초 가시 경계 침범 지점 `x_m,y_m`, 좌우 경계, `shortfall_m`, 입력 `steer_rad`, 예측 `horizon_m`를 `last_arc_rejection`에 저장.
- `planning.py`: 매 제어 주기 전 이 필드를 초기화. `node.py`: `/team01/debug/path.arc_rejection` JSON에 추가.
- 기존 `controller.log` 1초 로그에서 `steering_recovery` 상태 및 `arc_reject` 필드를 출력해 rosbag이 손상되더라도 최소한의 원인 위치/초과량을 남김.
- 신규 순수 단위시험 `tests/test_v36_arc_boundary_diagnostics.py`: 좁은 도로에서 잘못된 조향 후보의 실제 초과량 기록과 정중앙 직진에 대한 오탐 방지.
- **주의**: `arc_rejection`은 해당 틱의 경계검사에서 마지막으로 거절된 *예측 후보*이며 실제 차체 도로 이탈 또는 최종 조향의 증거가 아닙니다. `Motion.reason`, `planner`, `steering_recovery`, `candidate_rejections`, 실제 `gz_odom` 및 joint와 같은 sim-time으로 대조.
- 원본 Monza 서브모듈/세계/도로색/연석/차체 물리모델, 안전폭, 조향 부호, Stanley·MPC, QA 전용 시험 및 판정기 **미변경**.

## 3. 검증팀 전용 작업 요청(제작팀은 실행하지 않음)
1. 계측기 fail-closed: `qa_metrics.log`의 첫 에러, `qa_team01_metrics.py` 구동 PID/exit 확인. Odometry 첫 표본 실제 수신/좌표계 판정이 일치하지 않으면 **metrics 실패**로 표시. `lap.json`이 >0이고 `drive_metrics`가 0이면 즉시 `METRICS_MISMATCH` 별도 상태.
2. ROSbag2: 노드 중단을 기다려 flush/close 후 artifact 생성. `zstd -t`, `ros2 bag info`, SQLite `PRAGMA integrity_check` 통과 여부를 파일별로 기록. 원인 확정 전 압축 방식을 변경한 결과와 기존 파일을 혼동하지 않음. 공식 Monza도 동일 raw bag·metadata 보존.
3. 공식 `OFF_TRACK`: `safety_margin_exceeded_m`, `base_link_outside_road_m`, `official_disqualified_0_3m`를 분리. **현재 QA 안전마진 0.145m를 공식 실격으로 취급 금지**. 이 검증기 로직은 검증팀만 수정.
4. `arc_rejection`과 `steering_recovery` 빈도, 가장 처음 BRAKE 및 첫 코너 ±5초의 RGB/Depth/LiDAR bounds, requested/limited steer, yaw_rate, physics_dt, Gazebo wheel steering/spin, contact/slip, actual Odometry를 sim timestamp로 병합. 좌·우 조향 부호 뒤집기나 안전폭 축소를 선행하지 않음.
5. 새 개발 SHA에서 기존 198개+v36 회귀, 파일당 400줄 및 원본 Monza SHA 불변 검사. 실측 5m 연속 무정지·첫 코너 안전 진입·Monza 1랩 `GEOMETRIC_LAP_PASS`를 별도 단계 판정.

## 4. 판정
- 기존 f0e17ac9의 Monza 1랩: **FAIL**.
- 신규 계측 보강: GitHub 개발 소스 반영, **오프라인/실주행 QA 미실행**.
- 로그가 확보되기 전 추가 조향 게인 변경·상수 완화·맵 수정 보류.
