# TEAM 01 실제 Gazebo 독립 검증 — 취소 원인·실주행 데이터 재판독

- 일자: 2026-11-04 (KST)
- 사용자 제시 최초 결과: 통합 QA 4건 `CANCELLED`
- 분석 차량 커밋: `ab150ad7286cd833c083b7fbc391dcf19a7da6b1` (동일 차량에서 QA 측정기만 변경한 `16a05c9`도 비교)
- 독립 QA 전용 수정: [`6340b0e`](https://github.com/nahee2345/urrc_2_f1/commit/6340b0e11d88e16091ef7c5de7f510e191238a4a)
- **차량 센서·주행·조향·구동 코드 변경 없음.**

## 1. 실행별 판정과 원인

| Actions 실행 | 고정 SHA | 오프라인/400줄 | Gazebo 실행/아티팩트 실측 | 최종 |
|---|---|---|---|---|
| [41784937504](https://github.com/nahee2345/urrc_2_f1/actions/runs/41784937504) | 16a05c9 | PASS/PASS | Odometry **1.115m**, 옛 판정 OFF_TRACK(좌표계 오인) | 워크플로 CANCELLED; 1랩 FAIL |
| [41784297810](https://github.com/nahee2345/urrc_2_f1/actions/runs/41784297810) | ab150ad | PASS/PASS | Odometry **0.0m**, LAP_TIMEOUT, adapter 허가 false | 워크플로 CANCELLED; 1랩 FAIL |
| [41784132144](https://github.com/nahee2345/urrc_2_f1/actions/runs/41784132144) | c71802c | PASS/PASS | 40분 제한으로 워크플로 CANCELLED | 1랩 PASS 미확인 |
| [41783978779](https://github.com/nahee2345/urrc_2_f1/actions/runs/41783978779) | bbe54ed | PASS/FAIL | 40분 제한으로 워크플로 CANCELLED | 1랩 PASS 미확인 |
| [공식 Monza 41784297803](https://github.com/nahee2345/urrc_2_f1/actions/runs/41784297803) | ab150ad | 오프라인 별도 PASS | 실제 Gazebo LAP_TIMEOUT, **0.0m** | FAIL |

**중요:** `CANCELLED`는 실제 Gazebo가 실행되지 않았다는 뜻이 아닙니다. Actions Artifacts를 직접 내려받아 `lap.json`, `drive_metrics.json`, `trajectory.csv`, `rosbag_core/rosbag_core_0.db3`, `controller.log` 및 카메라 프레임까지 판독했습니다.

## 2. 확정 QA 인프라 결함 — 40분 뒤 CANCELLED

- `.github/workflows/team01-qa-validation.yml`은 job `timeout-minutes: 40`.
- 4건 모두 실제 Gazebo 실행이 완료되거나 `LAP FAIL`에 도달한 흔적이 있으나, 내부 프로세스 종료 처리의 **무제한 `wait`**로 job이 40분에 취소될 수 있음.
- `scripts/ci_team01_monza.sh`의 EXIT cleanup은 자식 PID들을 TERM 후 **무기한 wait**.
- `scripts/qa_team01_gazebo.sh`도 bag recorder에 SIGINT를 보내고 **무기한 wait**. 두 최신 취소 아티팩트에 ROS2 bag 원본 DB3는 있었지만 `metadata.yaml`이 없어 정상 종료·재생 준비가 미완료.
- QA 수정 `6340b0e`: 내부 Gazebo 자식 정리 10초 유한 종료 및 KILL escalation, 외부 1050초 상한, bag 녹화기 15초 유한 종료 및 `ros2 bag reindex` 복구 시도. 실제 종료/메타데이터 생성은 **새 CI에서 검증 필요**.

## 3. 확정 출발 허가 경합 — controller와 adapter가 다르게 허가됨

**원인 데이터: 원본 ROS2 bag `41784297810`**

- controller `/team01/debug/start`: `permission=true` 1001/1023 샘플, 실제 주행 요청은 `GRID, cmd_speed=0.75m/s`.
- adapter `/team01/debug/adapter`: `allowed=false` **1279/1279 샘플**, `sent_linear_mps=0` **1279/1279**, 수신 명령 `0.75m/s`는 다수 존재.
- 카메라 기준 RED 5개→소등, 0m `LAP_TIMEOUT`.
- **반대 경우까지 확인:** 공식 Monza `41784297803`에서는 adapter `allowed=true` 1545/1583 로그인데, controller `permission=false` 1266/1266. 따라서 차량 주행 알고리즘보다 **단발성 external /race/start/team01 허가 전달**이 먼저 의심됨.
- QA 수정: `ros2 topic pub --once` 대신 **두 가입자가 모두 수신할 때까지 복수 발행**하는 `scripts/qa_team01_permission.py`를 추가. controller permission과 adapter allowed를 동시 확인하는 `permission_validation.json` 생성. 실제 신호 카메라 소등 판단을 우회하지 않음.
- 신규 테스트에서 두 노드의 허가가 동시에 유지되는지 확인 전에는 구동계 FAIL로 결론내리지 않음.

## 4. 원점 기준 Odometry로 인한 가짜 OFF_TRACK — 좌표 정합 검사

**원인 데이터: 원본 ROS2 bag `41784937504`**

- Gazebo 차량 스폰 `(x=0.37867235, y=-2.0560354, yaw=1.45738442 rad)`.
- 수신 Odometry는 거의 `(x=0.03855,y=0,yaw=0)`에서 시작하여 `(x=1.12074,y=-0.00717,yaw=-0.02003)`로 이동: **스폰 상대좌표**인 정황.
- 기존 `verify_monza_lap.py` 및 `qa_team01_metrics.py`는 이런 좌표를 Monza **월드 기준** 중심선에 그대로 사영하여 마치 차량이 트랙 바깥으로 간 것처럼 계산.
- 실제 공식 centerline CSV (3309점) 기준 독립 계산:
  - 스폰 월드 위치 도로 경계 여유 `+0.4549m`
  - 원점 기준 첫 샘플을 잘못 투영한 여유 `+1.0167m`
  - 마지막 `(1.12074,-0.00717)`을 **그대로** 투영한 여유 `-0.0604m` (옛 OFF_TRACK)
  - 마지막 Odometry에 **스폰 yaw/위치 변환** 후 `(0.51263,-0.94331)`로 투영한 여유 `+0.4451m`.
- 그러므로 옛 OFF_TRACK는 좌표계 혼동의 명백한 거짓양성(단, Gazebo 실제 접촉·충돌 여부는 이 분석만으로 확인 불가). 주행은 약 1.115m에 불과하여 **1랩·코너·안정성을 검증한 것은 아님**.
- QA 수정: `qa_lap_frame.py` 좌표계 감지 `SPAWN_RELATIVE/WORLD/UNVERIFIED`, 좌표 변환을 독립 LapVerifier/trajectory metric에 반영. 변환 불명확하면 `ODOM_FRAME_UNVERIFIED`로 fail-closed. 기존 오프셋 경로에 대한 평가 원칙 유지.

## 5. 실제 구동 성능 — 아직 해결되지 않은 주요 문제

**같은 16a05c9 실주행**에서 controller→adapter 명령은 전달됨.

| 지표 | 아티팩트 값 |
|---|---|
| Adapter `allowed` | True 316/316 |
| Adapter `received_speed_mps` | 최대 5.0m/s |
| Adapter `sent_linear_mps` | 최대 5.0m/s |
| Gazebo Odometry 실이동 누적 | 1.115m |
| Odometry 실측 최고 속도 | **0.387m/s** |
| Odometry 속도 P95 | **0.238m/s** |
| 계획 후보 선택 | adaptive 144 샘플 / constant_arc 109 샘플 |
| 선택된 `candidate_shift_m` | 모두 0.0m (측정된 초기 주행 동안 오프센터 후보 선택 아님) |
| 계획경로 추종오차 평균/P95 | **미검증**, 이동 5m 미달·샘플 부족 |
| 직선 S자/코너 안정성 | **미검증**, 주행거리 부족 |

**제작팀 P0 분석 요청:** ROS→Gazebo `/team01/cmd_vel` 실제 Gazebo 구독, Ackermann 구동 plugin, wheel joint/torque/속도, 타이어 접촉·마찰·차체 초기 높이, world pose 및 odom 속도 측정 차이, time scaling을 구분하십시오. 높은 제어 명령만으로 실제 주행 속도 달성이라 주장하지 마십시오.

추가 확인 전에는 속도제어/Stanley/MPC 전체 교체를 권고하지 않습니다. 직선에서 중앙 복귀를 강제하거나 조향 진동을 유발하는 수정 역시 금지합니다.

## 6. 추가 확인된 QA 시계열 결함

- `verify_monza_lap.py`: `now=time.monotonic()`에서 파일 `st_mtime`(Unix epoch)을 직접 빼는 오류가 있어 **실제 0m 정지가 `DRIVE_STALL`로 조기에 분류되지 않고 `LAP_TIMEOUT`**으로 남을 수 있음. `time.time()`으로 비교하도록 수정.
- `qa_team01_metrics.py`: 프로세스 SIGTERM 시 이미 종료된 rclpy context에 `shutdown()`을 반복 호출해 `RCLError`. 조건부 종료로 수정.
- 최신 주행 후 `ready_for_lights=false`가 bag에서 관찰되더라도 `camera_ready_diag.json`은 먼저 true 확인. 평가기가 진단 구독을 늦게 시작해 true 순간을 놓친 사례일 수 있으므로 **카메라 준비 실패로 오판하지 않음**.

## 7. 정확한 재검증 승인 기준

1. 새 QA 코드 컴파일/프레임 unit test PASS, 전체 회귀시험 PASS, 400줄 제한 PASS.
2. 실제 Gazebo가 **CANCELLED가 아닌 명시적 SUCCESS/FAIL**로 종료.
3. Controller `permission=true` & adapter `allowed=true` 동시 확인, 신호등 RED→소등을 카메라 자체 인식.
4. ROS bag `metadata.yaml` 또는 `reindex` 복원 성공, `ros2 bag info` 및 최소 핵심 9개 토픽 확인.
5. Odom 프레임 `SPAWN_RELATIVE`를 월드로 변환해 기준선과 일치. 불명확하면 FAIL.
6. 실측 5m 이상 물리 이동, 벽 충돌 없이 코너 안전 통과, 실제 계획경로 추종 오차 최소 15개 샘플. 도로 중앙선 편차는 단독 실패 기준 금지.
7. 이후 실제 1랩 `GEOMETRIC_LAP_PASS`, 최소 조향/직선 최고속도 향상, 반복 성공을 구분하여 판정.
8. 각 실패는 첫 오류 시간·해당 프레임·rosbag·코드 위치·정확한 수정 방향으로 제작팀 전달.

## 8. 사용자 PC 명령

```bash
cd ~/urrc_2_f1
git pull --ff-only
git submodule update --init --recursive
python3 scripts/qa_team01_source_limits.py --limit 400
source /opt/ros/jazzy/setup.bash
bash scripts/qa_team01_gazebo.sh --local
```

원본 기록: [16a05c9 아티팩트](https://github.com/nahee2345/urrc_2_f1/actions/runs/41784937504), [ab150ad 아티팩트](https://github.com/nahee2345/urrc_2_f1/actions/runs/41784297810), [공식 Monza 아티팩트](https://github.com/nahee2345/urrc_2_f1/actions/runs/41784297803).

**이 보고서는 과거 커밋의 실제 증거 재분석과 QA 인프라 수리 사실이며, `6340b0e` 물리 1랩 성공을 주장하지 않습니다.**
