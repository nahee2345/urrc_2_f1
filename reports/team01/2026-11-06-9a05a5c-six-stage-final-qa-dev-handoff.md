# TEAM01 9a05a5c: 1~6단계 QA 최종 분석 및 제작팀 인수인계

기준: 2026-11-06 / 차량 SHA 9a05a5c1c62db19e7c7280727008ae844f880b00 / GitHub Actions ROS2 Jazzy + Gazebo Harmonic.
검증팀은 차량 코드·원본 Monza 맵을 수정하지 않음. 결과는 각 attempt별로 구분.

## 1. 실증 결과
| 단계 | 항목 | 판정 | 근거 |
|---|---|---|---|
| 1 | CI/Gazebo 및 카메라·신호 작동 | PASS(1차 실주행) | 공식 Monza #42164504143 attempt1 |
| 2 | 400줄 소스·오프라인 회귀 | PASS | #42164504138 SUCCESS, 독립 QA attempt2 192 tests OK |
| 3 | ROSbag/로그 자료 수집 | PARTIAL PASS | 독립 QA attempt1 rosbag metadata 확인, attempt2 아티팩트 110MB 수집. joint/contact 정밀 동기화 미검증 |
| 4 | 출발 안전 및 구동 인과 | PARTIAL | 독립 QA attempt2 WAIT_RED 시 fallback=false, speed=0, 이후 LIGHTS OUT. 전체 시각 조기출발 0회 및 물리 인과는 미검증 |
| 5 | 5m·첫 코너·완주 | FAIL | 독립 QA attempt1 9.002m LAP_TIMEOUT; 공식 attempt1 OFF_TRACK, 1랩 PASS 없음 |
| 6 | 최종 PASS | FAIL | GEOMETRIC_LAP_PASS 근거 없음 |

### 실행별 결과 (서로 혼합 금지)
- 독립 QA #42164504112 attempt1: 9.002m, LAP_TIMEOUT, 경계 밖 조향 제한 이벤트 2047회, 장애물 제동 3회, 복귀 1회. 저장소 기존 reports/team01/runs/42164504112-9a05a5c1c62db19e7c7280727008ae844f880b00.md 의 수치는 attempt1 기준.
- 독립 QA #42164504112 attempt2: FAILURE, 192 tests OK, 출발 전 WAIT_RED에서 fallback_mode_authorized=false/fallback_active=false, cmd_speed_mps=0.0 관측; LIGHTS OUT 후 LAP_TIMEOUT. 정확한 이동거리와 조향 제한 회수는 artifact 재처리 전 UNVERIFIED. 원본 https://github.com/nahee2345/urrc_2_f1/actions/runs/42164504112/artifacts/13480388538
- 공식 Monza #42164504143 attempt1: 192 tests OK; RED 1/5→5/5→LIGHTS OUT; 최종 OFF_TRACK. 원본 https://github.com/nahee2345/urrc_2_f1/actions/runs/42164504143/artifacts/13479011350
- 공식 Monza #42164504143 attempt2: CANCELLED, ROS 패키지 저장소 설정 단계에서 취소, Gazebo 주행 SKIPPED. 이 시도를 OFF_TRACK 재현으로 간주 금지.
- 이전 개발판 3d9ebf9: 독립 0.152m, 공식 0.204m DRIVE_STALL. 신규 버전 attempt1이 9.002m까지 이동했으므로 구동 개선 가능성은 있으나 동일조건 A/B 인과 검증 전 확정 불가.

## 2. 확정된 사실과 미확정 원인
확정: 기존 P0 Issue #5에서 RED 전 7초 자동승인 전진 명령이 있었음. 이번 attempt2 대기 시점에는 fallback 비활성, speed 0으로 바뀜. 단, 전체 RED 이전 명령이 전부 0인지는 raw bag 시각 동기 검증 필요.
확정: 공식 1차 실행 OFF_TRACK, 독립 1차/2차 LAP_TIMEOUT, 완주 실패. OFF_TRACK이 조향 부호 때문인지, 센서 경계 인식 때문인지, 충돌 때문인지는 아직 미확정.
확정: 독립 1차 경계 밖 제한 조향 2047회. 제한 이벤트가 매우 많다는 사실은 계획기 목표점과 주행가능 영역의 충돌 가능성을 시사하나, 센서 오류로 단정 불가.
확정: 독립 QA attempt2 텍스트 Git 저장 과정 같은 경로 add/add rebase 충돌로 실패. 원본 아티팩트 업로드 성공. 이는 CI 보고서 충돌이지 차량 주행 원인이 아님.

## 3. 제작팀 수정 권고 (우선순위)
### P0-A: 공식 OFF_TRACK 최초 구간 분석
공식 attempt1의 최초 OFF_TRACK 전후 각 5초에서 sim-time으로 gz_odom(base_link x/y/yaw), 경계 거리와 signed clearance, RGB/Depth/LiDAR, 목표 경로점, 조향 요청/최종 적용, 속도명령, 물리 접촉/충돌을 맞춰 최초 위반 시점을 특정. 안전여유 경고와 base_link 도로 밖 0.3m 이상 실격을 구분.
검증 가설: (1) RGB와 Depth FOV 불일치/투영 오차, (2) 좌우 부호 또는 local/world frame 혼용, (3) 초기 우측연석 추종 목표점, (4) 경계 불확실 상태의 목표점 과도 변경. 원인 확정 전 조향 방향만 뒤집거나 Stanley/MPC 전체 교체 금지.
### P0-B: 주행 불능 및 LAP_TIMEOUT
독립 attempt1의 2047 제한 이벤트를 시간/거리당 빈도로 정규화하고 실제 경계 가시성, 센서 age, 예상 궤적과 비교. 도로 안에서 가능한 대체 목표점 생성, 조향 변화율 제한, 직선 최소조향 유지, 코너 감속을 가설별로 분리해 평가. 경계 양측 미확정 시 fail-closed 유지.
### P0-C: 실제 구동 인과 및 watchdog
동일 simulation timestamp로 controller speed/steer → adapter fresh_speed/fresh_steer/stop_reason/sent_linear_mps → Gazebo Twist 수신 → 4개 wheel spin joint 속도·힘/2개 steer joint → 접촉/마찰/슬립 → gz_odom 기록. 새 제한 sim age<=0.32초 및 wall age<=2.5초에서 반복 0 명령 발생률 확인. Twist 발행만으로 물리 주행 PASS 금지.
### P1: spawn/맵/보고서 정합
safe_grid_spawn 실행 보정 좌표, wheel footprint, 연석/가드레일 접촉, spawn z를 공식·독립 QA에서 비교. upstream Monza 원본/hash 변경 금지. 같은 실행 run에서 attempt별 고유 파일명(run_id-attempt-sha.md)을 써 add/add Git 충돌 방지.

## 4. 재검증 필수 PASS 기준
1) camera 모드 RED 미점등/점등 중 및 소등 이전 sent_linear_mps 양수 0회; 7~30초 지연 신호, RED4→0, 카메라 보정 지연 테스트. 명시적 no_signal 모드만 7초 이후 출발 허용.
2) 400줄 제한/192개 이상 오프라인 전체 PASS, Monza 원본 gitlink/SDF 해시 불변.
3) Gazebo 실제 5m 연속이동 + wheel/joint/odom 정합, 첫 코너 무이탈, 직선 최소조향 및 S자·진자·급조향 없음.
4) 외부 0.3m 실격/안전여유/충돌 개별 기록. 중앙선 편차만으로 FAIL 금지.
5) Monza 1랩 GEOMETRIC_LAP_PASS; rosbag2 metadata 및 DB, trajectory.csv, drive_metrics.json, lap.json, 디버그 로그 확보.

## 5. 결론
종합 FAIL. 안전한 출발 로직과 물리 이동은 기존 대비 개선 근거가 있으나, 공식 OFF_TRACK 및 독립 LAP_TIMEOUT으로 완주 실패. 우선순위는 첫 OFF_TRACK 시점 원인 특정 → 반복 경계 제한/목표 경로 분석 → 명령·휠·접촉·odometry 인과 검증 → 독립 재시험. 위 가설은 아직 확정 고장이 아님.

근거 URL: https://github.com/nahee2345/urrc_2_f1/actions/runs/42164504112 / https://github.com/nahee2345/urrc_2_f1/actions/runs/42164504143 / https://github.com/nahee2345/urrc_2_f1/issues/5 / https://github.com/nahee2345/urrc_2_f1/issues/6