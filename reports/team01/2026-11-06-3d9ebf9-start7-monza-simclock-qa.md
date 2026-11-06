# TEAM 01 `3d9ebf9` 독립 QA: 원본 Monza / 7초 자동출발 / Gazebo 물리시간 거리

검증일: **2026-11-06 KST**  
**차량/런처 고정 SHA:** [`3d9ebf945c38047357baae0ec7e1d6a1b729ac51`](https://github.com/nahee2345/urrc_2_f1/commit/3d9ebf945c38047357baae0ec7e1d6a1b729ac51). 이 커밋 자체는 개발 인수인계 **문서만 추가**했으며 기능 코드 변경은 앞선 조상 커밋에서 이루어짐.  
검증 전용 후속 SHA: [`f67453d`](https://github.com/nahee2345/urrc_2_f1/commit/f67453dde80272e316250e92382c815bbef01cb3): `scripts/qa_team01_frame_test.py` **35줄로 self-test만 수정**, 차량 코드 변경 없음.

**종합: 기능 단위시험 PASS / 신호등보다 먼저 전진 명령 발행 FAIL / 실제 Gazebo DRIVE_STALL / Monza 1랩 FAIL / 장거리 거리 정확도 및 주행 안정성 미검증.**

## 1. 근거별 항목 판정

| 영역 | 판정 | 기준 및 실제 근거 |
|---|---|---|
| 오프라인 전체 | **PASS** | [Fast #42161954918](https://github.com/nahee2345/urrc_2_f1/actions/runs/42161954918): `Ran 184 tests`, `OK`. 기존 코너/안전 오프셋, v31 NO_RED_7S, v32 시뮬레이션 시간 6개 신규 시험 포함 |
| 원본 Monza gitlink | **PASS (정적)** | `upstream/urrc_f1_ws`는 기준 `6340b0e` 및 대상 `3d9ebf9`에서 동일 gitlink `a23c7eec41b98b67ec3a3a2a46f9d7ed957f8290`, `model.sdf` blob도 동일 |
| 원본 SDF 대신 임시 복사본에서 실행 | **PASS (코드/아티팩트 구조)** | `scripts/ci_team01_monza.sh`는 `git archive HEAD` → `STATE/monza_runtime`; 아티팩트에 복사본 `monza_race.sdf`, `monza_signal_active.sdf`, `models/urrc_monza/model.sdf` 존재. 사용자 로컬 실행 전후 원본 SDF 바이트 비교는 미수행 |
| 7초 타이머 계산 | **PASS (단위시험만)** | `test_v31_spawn_fallback.py`: 106.99초 거절 / 107.00초 승인, 이미 RED 본 경우 정지, 스폰 기준 시각 재설정 불가 |
| 양쪽 출발 허가 동기화 | **부분 PASS** | [공식 Monza #42161954931](https://github.com/nahee2345/urrc_2_f1/actions/runs/42161954931) `permission_validation.json`: controller=True, adapter=True, external 허가 4회. `adapter_status.log`에 fallback/race_allowed 각각 표기 |
| **정상 RED 전에 출발 권한 차단** | **FAIL (P0)** | RED 전 `camera_ready_diag.json`: `gate_status=WAIT_RED`, `red_peak=0`, `fallback_active=true`, `spawn_elapsed_s=21.051`, `motion_mode=GRID`, `cmd_speed_mps=0.75`. 어댑터는 `race_allowed=false`+ `fallback_allowed=true`에서 **실제 양의 `sent_linear_mps` 10개 표본** 기록 |
| 빨간불 1→5 및 소등 | **PASS (영상/로그)** | 같은 공식 실행 `lights.log`: RED 1/5→5/5 및 `LIGHTS OUT`, controller `CAMERA LIGHTS OUT verified`. 그러나 이미 자동출발이 활성화됐으므로 전체 규칙 준수 PASS가 아님 |
| Depth/LiDAR/도로 안전 게이트 | **정적 PASS·동적 일부 관측** | 스폰 전후 `depth_ok=true`, `front_lidar_ok=true`, `road=true`. 장애물·오인식·센서 상실을 포함한 전체 동적 안전시험 미완료 |
| Gazebo 주행 거리 | **FAIL** | `lap.json`: `status=DRIVE_STALL`, `distance_actual_odometry_m=0.204`, `progress_m=0.204/1158.473`, `coverage_bins_of_120=1`, `elapsed_wall_s=114.13` |
| 시뮬레이션 시간 거리 적분 | **단위 PASS·짧은 물리시험 정합** | `test_v32_sim_distance.py` 세 시험 PASS. 공식 run 종료 시 controller `lap_distance_est=0.2m`, 독립 Gazebo `0.204m`. 이전 33.4m 과대 누적은 이 **0.204m 구간에서는 재현되지 않았으나 장거리 해결 승인 불가** |
| 어댑터 구동 명령 | **PASS (발행)** | `adapter_status.log` 254표본: `sent_linear_mps>0` 65표본, 최대 1.25m/s. Gazebo 플러그인에서 `AckermannSteering subscribing to ... /team01/cmd_vel` 확인 |
| 실측 목표속도·물리 구동 | **FAIL** | 최대 1.25m/s 명령에 비해 114초 동안 0.204m만 이동하고 DRIVE_STALL. 휠 조인트, 타이어 접촉, 플러그인, `/clock` 실측, 명령 연속성 원인 분석 필요 |
| 도로 안전 여유 vs 실격 | **0.3m 실격은 미확인** | `max_offroad_m=0.0`, `offroad_frames=0`. 실제 이동 자체가 부족. 일반 QA 안전 여유와 `base_link` 중심점 0.3m 공식 실격 별도 평가 |
| 직선 최소 조향/S자 방지/첫 코너 | **UNVERIFIED** | 0.204m 물리 이동; 유효 장거리 궤적 및 rosbag 미확보 |
| RTAB-Map/Monza 1랩 | **UNVERIFIED / FAIL** | 0랩, `GEOMETRIC_LAP_PASS` 없음 |
| 파일당 400줄 전체 | **검증 보류** | 대상 커밋의 독립 QA #42161954917은 source gate 전 frame self-test로 중단. 변경 파일 `node.py` 397, `planning.py` 396, `model.sdf` 189줄이나 전체 자동 gate 결과는 별도 실행 필요 |
| ROSbag 수집 | **공식 run: UNVERIFIED** | 공식 아티팩트에 원본 rosbag2 DB/metadata 없음. 독립 QA #42162624848에서 확인 필요 |

## 2. 확정 P0 — 신호등 점등 전 자동 출발권한 발행

**기대:** 정상 신호가 진행 예정이면 실제 RED→소등 전까지 모든 전진 명령 차단. 7초 대안은 운영자가 명시적으로 허용한 무신호 조건에서만 작동하고, 뒤늦은 빨간불 연출을 선행하지 않아야 함.

**실제:** `start_fallback.py` 18~25행은 `first_bright_time=None`이면 스폰 경과 7초부터 True. CI는 카메라 보정과 외부 permission 대기 후 `traffic_lights_v10.py`를 실행하므로 타이머가 먼저 끝날 수 있음. 이번 `camera_ready_diag.json`에서 신호가 없는 WAIT_RED 단계인데 이미 `fallback_active=true`, 전진명령 0.75m/s. 어댑터 원본 JSON에서 `race_allowed=false`인 동안 `fallback_allowed=true`로 **양의 전진 전송 10회** 확인. 이후 RED 5개→LIGHTS OUT 별도 관측.

**결론:** 실제 타이머가 먼저 출발 **허가 및 구동 명령**을 전달한 것은 확정. 그 시점의 실제 물리 이동은 입증되지 않아 ‘신호 전 물리 이동’으로 과장하지 말 것. **대회 출발 신호 준수 FAIL**.

**제작팀 조치:** 공식 시퀀스 시작 가능성이 남은 동안 fallback 금지. 운영모드/신호 제공 여부의 명시적 합의, 신호 대기 연장 또는 안전한 타임아웃 프로토콜 적용. 정상 RED가 7초 이후 시작하는 회귀시험, RED 시각 경계 테스트, controller-adapter 동시 gate 시험, 7초 대안과 정상 신호 경로 분리. 외부 경주 출발허가 없이 자동전진한 기존 로직을 PASS 처리 금지.

## 3. 확정 실주행 실패 — 0.204m 후 DRIVE_STALL

**기대:** ROS 게이트 허가 후 5m 이상 실제 이동, 첫 코너 안전 통과, 1랩 `GEOMETRIC_LAP_PASS`.

**실제:** 어댑터 254기록 중 양의 `sent_linear_mps` 65표본, 최대 1.25m/s. Gazebo는 cmd_vel 구독 정보를 표시. `lap.json`은 0.204m 이동, 오프로드 0표본, 114.13초 경과 후 `DRIVE_STALL`; 0랩. Controller 내부 0.2m로 정합. 속도·조향 경로 성능을 판정할 주행거리는 부족.

**근본 원인:** 확정 불가. 생성된 `Twist`가 Gazebo 시스템에 적용되는지, 바퀴 실제 각속도/조향 관절/접촉·마찰/차체 스폰 z 높이, update rate/물리 시계, 명령 유지시간을 함께 검증. 원인 없이 Stanley/MPC 교체 또는 중앙 복귀 강제 금지.

## 4. 검증기 자체 BLOCKED 및 수정

`3d9ebf9` 독립 QA [#42161954917](https://github.com/nahee2345/urrc_2_f1/actions/runs/42161954917): `scripts/qa_team01_frame_test.py`가 최초 `transform(1,0,0)` 호출하여 `ODOM_FRAME_UNVERIFIED` 발생. 실제 Gazebo 단계 **미시작**. QA 전용 [`f67453d`](https://github.com/nahee2345/urrc_2_f1/commit/f67453dde80272e316250e92382c815bbef01cb3)에서 최초 원점 상대 `(0,0,0)`으로 좌표계 확정 후 `(1,0,0)` 이동 검사로 수정. [새 QA #42162624848](https://github.com/nahee2345/urrc_2_f1/actions/runs/42162624848) 시작. 차량 제작 코드는 수정하지 않음.

## 5. 후속 PASS 조건

1. 전체 400줄 gate/184개 오프라인 시험 PASS, QA 좌표계 테스트 PASS.
2. `upstream/urrc_f1_ws` 서브모듈 gitlink, 추적된 원본 Monza SDF 해시 실행 전후 동일; 임시 복사본만 변경, 런타임 디렉터리 정상 접근.
3. 정상 RED→소등에서만 전진; 의도된 무신호 테스트 모드에서는 7초 이전 무출발, 이후 controller+adapter 허가·안전 센서 데이터 확인. **RED가 7초 이후 점등되는 경우의 오출발 금지**.
4. 시뮬레이션 시간 기준 내부거리와 실제 Gazebo Odometry 이동거리 5m/코너/1랩에 걸쳐 비교. `odom_sim_dt_s`·`odom_speed`·`lap_distance_est`를 timestamp 동기화.
5. 실제 5m 이상 연속 이동·추진/조향 제어 검증, 첫 코너 통과, 직선에서 불필요한 좌우 조향 최소화. 중앙선 편차만으로 FAIL 판정 금지.
6. 안전 여유 침범·실제 도로 밖 `base_link` 이탈·0.3m 공식 실격을 별도 지표로 보고.
7. 실제 ROSbag DB/metadata·궤적 CSV·독립 `GEOMETRIC_LAP_PASS` 확보 후에만 최종 합격.

## 6. 사용자 PC 재현/원본 복구

```bash
cd ~/urrc_2_f1
git pull --ff-only
git submodule update --init --recursive
bash scripts/team01_restore_original_monza.sh
python3 scripts/qa_team01_source_limits.py --limit 400
source /opt/ros/jazzy/setup.bash
bash scripts/qa_team01_gazebo.sh --local
```

**보고서 증거:** [Actions #42161954918](https://github.com/nahee2345/urrc_2_f1/actions/runs/42161954918), [Actions #42161954917](https://github.com/nahee2345/urrc_2_f1/actions/runs/42161954917), [공식 Monza #42161954931](https://github.com/nahee2345/urrc_2_f1/actions/runs/42161954931) 109MB 아티팩트의 `lap.json`, `controller.log`, `adapter_status.log`, `permission_validation.json`, `camera_ready_diag.json`, `lights.log`, `gazebo.log`, `monza_runtime` 디렉터리. **실제 Monza 최종 PASS 미획득**.


## 7. QA frame self-test 보정 후 실제 독립 Gazebo 최종 결과

**실행:** [#42162624848](https://github.com/nahee2345/urrc_2_f1/actions/runs/42162624848), 차량코드 `3d9ebf9`과 동일 + QA self-test 전용 수정 `f67453d`. Actions 결론 **FAIL**.

| 독립 검증 항목 | 판정 | 아티팩트 근거 |
|---|---|---|
| 좌표계 self-test | PASS | `[QA] coordinate-frame regression PASS` |
| 400줄 검사 | **PASS** | `source_line_limits.json`: 92개 파일, 초과 0건 |
| 오프라인 회귀시험 | **PASS** | CI 독립 오프라인 step success 및 공식 fast regression 184/184 |
| Gazebo 카메라·전후방 LiDAR 구동 | PASS(기본 데이터 확인) | `qa_runner.log`: RGB·Depth·front/rear scan 토픽 검출 |
| 출발 허가 동시 수신 | PASS | `permission_validation.json`: controller=True, adapter=True, 5회 송신 |
| RED 점등 전 타이머 | **FAIL, 재현** | `camera_ready_diag.json`: `WAIT_RED`, `fallback_active=true`, `spawn_elapsed_s=26.334`, `cmd_speed_mps=0.75`. 이후 RED1→RED5/소등. P0 Issue #5와 동일 |
| 실제 Odometry 누적거리 | **FAIL** | 독립 `lap.json`: 0.152m / 1158.473m, `DRIVE_STALL`, 0랩 |
| 실측 최대속도/P95 | **0.154 / 0.078m/s** | `drive_metrics.json`; 주행 목표 미충족 |
| 도로 경계 최소 여유 | 0.455m | `drive_metrics.json`; 이 짧은 이동 구간에서 도로 이탈 0건 |
| 조향 좌우 반전(2° 초과) | 관측 0건 / 성능 UNVERIFIED | 단 0.152m 이동, 직선 안정성 입증 불가 |
| 계획경로 추종오차 평균/P95 | UNVERIFIED | `planned_path_tracking_evaluable=false`, 유효 계획경로 매칭 표본 0개 |
| rosbag 원본 및 metadata | **PASS** | `rosbag_core/metadata.yaml`, `rosbag_core/rosbag_core_0.db3.zstd` 확보 |
| 최종 1랩 | **FAIL** | `GEOMETRIC_LAP_PASS` 없음; `DRIVE_STALL` |

**측정 범위 차이:** `lap.json`은 검증기 전체 시점부터 누적 실제 거리 **0.152m**이고 `drive_metrics.json`의 거리 **0.077m**는 rosbag/metrics 구독 시작 이후 일부 구간만 측정합니다. 두 수치를 서로 교체하거나 두 거리의 차이를 차량 내부 거리 오차라고 단정하지 않습니다.

**카메라 진단 불일치:** `drive_metrics.json`의 `camera_ready=false`는 측정기가 준비 상태 True 이벤트를 늦게 구독한 결과일 수 있습니다. `camera_ready_diag.json`에는 `CAMERA_START_BASELINE_READY` 및 ready_sample_count=5가 명시돼 있으므로 **실제 카메라 준비 실패로 판정하지 않습니다**. 다만 실차의 신호등 준수는 PRE-RED fallback 전진 허가로 여전히 FAIL입니다.

**결론 확정:** 184개 오프라인 및 파일 400줄 검사는 PASS, 원본 맵 gitlink 정적 PASS, **두 Gazebo 실행에서 조기 타이머 승인과 저속/DRIVE_STALL 재현**, 원본 ROSbag 저장 PASS, **Monza 실제 1랩 FAIL**. 차량 제작 코드 미수정. P0 [출발 규칙 #5](https://github.com/nahee2345/urrc_2_f1/issues/5) 및 P0 [실구동 #6](https://github.com/nahee2345/urrc_2_f1/issues/6) 별도 개발팀 조치 대기.
