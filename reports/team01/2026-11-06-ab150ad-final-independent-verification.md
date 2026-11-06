# TEAM 01 `ab150ad` 독립 검증 최종 판정 및 후속 Gazebo 근거 (2026-11-06)

**대상 차량 소스:** [ab150ad7286cd833c083b7fbc391dcf19a7da6b1](https://github.com/nahee2345/urrc_2_f1/commit/ab150ad7286cd833c083b7fbc391dcf19a7da6b1). 이 후 main의 변경은 QA 테스트·스크립트·보고서뿐이며 차량 주행 소스 변경 없음(2026-11-06 compare 확인).  
**후속 QA·구동 기준:** [6340b0e](https://github.com/nahee2345/urrc_2_f1/commit/6340b0e11d88e16091ef7c5de7f510e191238a4a).  
**판정:** **OFFLINE PASS / 물리 Monza 1랩 FAIL / 최소조향 및 전체 경로 추종 UNVERIFIED**.  
**보존 원칙:** 차량 코드 수정 없음. 각 지표는 실제 관측값과 가설을 구분.

## A. 항목별 판정

| 항목 | 판정 | 근거와 기준 |
|---|---|---|
| 기존 및 신규 회귀시험 | PASS | [#41784297781](https://github.com/nahee2345/urrc_2_f1/actions/runs/41784297781) 전체 178/178 OK; 기존 v7 적응형 코너, 신규 v30 안전 오프셋/Stanley, v25 최초 센서 불일치 시험 통과 |
| 파일당 400줄 | PASS | QA 검사 단계 통과, `model.sdf` 189줄, `planning.py` 392줄, `node.py` 399줄 |
| SDF 설정 보존 | 정적 PASS | 기존 파일과 공백 정규화 후 내용 동일; 실제 Gazebo 로딩·Ackermann 명령 구독도 후속 실행에서 확인 |
| 카메라 시동·5등 RED→소등 | PASS | #41784297803 및 #41793368773 logs; 후자 `camera_ready_diag.json` 및 `lights.log` |
| 원본 ab150ad 실제 구동 | FAIL | [#41784297803](https://github.com/nahee2345/urrc_2_f1/actions/runs/41784297803) `lap.json`: `LAP_TIMEOUT`, 실제 Gazebo Odometry 0.0m, 0랩 |
| 원본 출발 허가 | FAIL (시험 게이트) | 단발 Bool 1회 발행 후 controller permission=False 633/633 시계열, adapter allowed=True 1545/1583; 서로 다른 허가 |
| QA 게이트 보정 후 출발 허가 | PASS | [#41793368773](https://github.com/nahee2345/urrc_2_f1/actions/runs/41793368773) `permission_validation.json`: controller=True, adapter=True, 3회 발행, PASS |
| 후속 Gazebo Odometry 5m 이상 | PASS | 후속 `lap.json`: `distance_actual_odometry_m=6.868` |
| 1랩 실제 완주 | **FAIL** | `reference_length_m=1158.473`, `progress_m=6.322`, `coverage_bins_of_120=2`, `OFF_TRACK`, 0랩 |
| 최소 조향/직선 S자 억제 | UNVERIFIED | 약 6.9m 물리이동만 수집, 코너 포함 제어기 샘플의 steer 범위 -13.7°..+10.9°; 실측 긴 직선·복수 랩 데이터 없음 |
| ±0.08/±0.16m 대안 경로의 실제 효과 | UNVERIFIED | 단위시험 PASS만으로 20Hz 선택 후보 사용·차체 footprint 안전성 입증 불가 |
| SLAM 완성·재사용/2랩 | UNVERIFIED | 실제 1랩 미완주 |
| 독립 QA 종합 워크플로 | **BLOCKED / FAIL** | [#41793368717](https://github.com/nahee2345/urrc_2_f1/actions/runs/41793368717) Gazebo 시작 전에 좌표계 self-test 실패. 동적 QA 결과 아님 |

## B. 최초 실패와 근거

### B1. 원본 `ab150ad`: 외부 허가 전송 경쟁 상태 (확정 관측)

`#41784297803` 실제 아티팩트: `permission.log`는 `ros2 topic pub --once` 단발 발행. controller 디버그 `permission=False`가 전 구간 지속, adapter `allowed=True`는 다수. 적색 신호 5→0은 감지됐으나 controller는 계속 STOP, `lap.json` 이동거리 0.0m. **차량 경로 실패로 분류하지 말고 시험 게이트/구독자 수신 불일치로 분류.**

### B2. 게이트를 고친 `6340b0e`: 실제 물리 이동 후 QA OFF_TRACK

`#41793368773`의 `lap.json`과 실제 제어 로그:

- World spawn: `x=0.37867235, y=-2.0560354, yaw=1.45738442`.
- Odometry 프레임: `SPAWN_RELATIVE` → 월드 좌표 변환 적용.
- 실제 누적 이동 `6.868m`, 기준선 진행 `6.322m`, 마지막 횡편차 `1.2609219349m`, 도로 폭 `2.4m`, `max_offroad_m=0.206`, `offroad_frames=8`.
- 독립 lap 판정기는 `edge_margin=0.145m`에 추가 `outside >0.025m`인 표본을 8개 **누적**하면 OFF_TRACK. 이 규칙은 대회의 **base_link 중심점 도로 밖 0.3m 이탈 실격 규칙과 다름**.
- 실제 마지막 중심점 도로 경계 초과는 `1.26092 - 2.4/2 = 0.06092m`. `0.206m`는 **안전 여유 포함 QA 판정치**, base_link 실제 0.206m 이탈이 아님.
- 따라서 `OFF_TRACK`은 **안전 여유 기준 QA FAIL**, 하지만 **공식 0.3m 실격은 입증되지 않음**. 도로 경계를 벗어나려는 궤적 자체는 재현되므로 안전성 개선 필요.

### B3. 위치추정/실제 구동 데이터 불일치 (확정 측정; 근본 원인은 미확정)

`#41793368773` 종료 인접 시점 controller INFO: `lap_distance_est=33.4m`, 내부 `odom_v≈1.5675m/s`, 제어 속도 `v=1.76m/s`. 같은 종료 시각 독립 Gazebo `lap.json`: 실제 `6.868m`. 내부 학습 지도 `active_lap1.json`의 예상 이동거리 `30.151m`도 별도로 확인.

**제작팀 P0 진단 요청:** ICP Odometry 및 wheel/body pose, Gazebo `/team01/gz_odom`, 가상시간/벽시계 및 프레임 변환을 동일 timestamp로 기록하여 30m 이상 추정과 물리 6.9m의 괴리가 발생하는 최초 시점을 찾을 것. controller 입장에서는 차가 약 33m 갔다고 판단해 잘못된 코너·진입 제어를 수행할 수 있으나, 원인/인과관계는 추가 rosbag 대조 전 확정하지 말 것.

### B4. RGB/Depth 도로 경계 및 제어 안정성 우려 (원인 가설)

제어 로그에서 `BRAKE`, `reason=limited steering outside visible road`, `edge=CAUTION`, `planner=constant_arc`, `RECOVER`가 반복되고, 후속에는 `adaptive_pursuit`로 전환. 일부 주행 내부 학습 자료에서는 도로 폭 측정 `~2.39m→~0.79m`로 급감한 구간이 관찰됨. 도로가 실제 좁아졌는지 RGB·Depth·LiDAR 인식 오류인지, world/ICP odometry로 인한 지도의 위치 오류인지 **확정 불가**.

제작팀은 `candidate_xy`, `road_bounds_xy`, `candidate_shift_m`, `candidate_rejections`, 각 센서 age/신뢰도, 실제 wheel steering/Yaw Rate, Gazebo 월드 pose를 동기화해 비교. **중앙 복귀를 해답으로 삼지 말 것.** 안전한 직선에서는 차로 가장자리 위치를 유지해도 최소 조향으로 전진해야 함.

## C. 검증 도구 자체의 확인된 결함

`#41793368717`: `scripts/qa_team01_frame_test.py`가 최초 좌표로 `transform(1,0,0)`을 전달하므로, `OdomWorld`의 초기 frame 판정 거리 조건(`relative_distance<0.60`)에 걸리지 않아 `ValueError: ODOM_FRAME_UNVERIFIED`. **독립 QA는 실제 물리 실행 전 실패**.

- QA 제작자는 최초 `(0,0,0)` 표본으로 SPAWN_RELATIVE 프레임을 확정하고, 다음 `(1,0,0)` 이동 표본의 월드 변환을 확인하도록 단위시험을 보완할 것. 모호한 초기 프레임을 조용히 WORLD로 취급하지 말 것.
- 판정기에 `margin breach`, `base_link center actual outside`, `0.3m DISQUALIFICATION` 세 종류를 각각 기록하고 안전 여유 위반을 대회 실격으로 잘못 명명하지 말 것.
- 정적/오프라인 PASS와 Gazebo/완주 PASS를 독립적으로 표기.

## D. 재검증 승인 조건

1. QA frame self-test PASS → 400줄 검사 PASS → 기존 회귀시험 178개 PASS.
2. 카메라 5 RED→LIGHTS OUT 및 controller/adapter 동시 허가 PASS.
3. Gazebo 실제 world pose / 오도메트리 / ICP 위치추정 일치. 첫 5m~코너 시점에서 축적거리 차이와 yaw 차이를 수치화.
4. 안전 가장자리 주행 허용. 직선에서 중앙으로 가기 위한 반복 좌우 조향이 없고, 평균/P95 조향·실제 주행속도 개선.
5. 도로 안전 여유 위반, 물리 충돌, base_link 실제 0.3m 이상 이탈을 **각각 독립 판정**.
6. 선택한 적응형 후보(±0.08/0.16 포함)와 0.6~1.5s 뒤 실제 Gazebo 좌표 비교. 실이동 5m 이상, 유효 대응 표본 15개 이상.
7. 실주행 Monza 1랩 완료, 독립 `GEOMETRIC_LAP_PASS`, 센서·조향·rosbag 원본 확보 후 최종 PASS.

## E. 재현 명령

```bash
cd ~/urrc_2_f1
git pull --ff-only
git submodule update --init --recursive
python3 scripts/qa_team01_source_limits.py --limit 400
source /opt/ros/jazzy/setup.bash
# 현재 QA frame self-test가 FAIL이므로 QA 수정 전 총합 PASS 불가.
bash scripts/qa_team01_gazebo.sh --local
```

**증거 출처:** GitHub Actions #41784297781, #41784297803, #41784297810, #41793368717, #41793368773의 job logs 및 두 공식 Monza zip 안 `lap.json`, `spawn_clearance.json`, `permission_validation.json`, `controller.log`, `adapter_status.log`, `state/urrc_team01/learning/active_lap1.json`. **GitHub 차량 코드 수정 없음.**
