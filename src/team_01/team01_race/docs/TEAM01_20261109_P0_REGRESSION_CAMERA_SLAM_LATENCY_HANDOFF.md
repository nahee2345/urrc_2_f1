# TEAM01 제작팀 추가 보완 / 독립 QA 인수인계 (2026-11-09)

**기준 QA:** `0da63070800429d394c564746f3335d5bcbad668`, independent QA #42540958903.
**중요:** 본 보고는 **코드 변경 내역**이며 QA 실행 또는 Gazebo 1랩 PASS를 의미하지 않습니다. 검증·실주행·PASS/FAIL 판정은 검증팀 전담입니다.

## 0. 최신 QA 원문에서 확인된 FAIL
- 오프라인 229건 중 **2건 FAIL**: `test_vehicle_bounds_and_joints`는 신규 IMU 때문에 SDF 센서 6개를 기존 5개로 비교; `test_safe_gap_can_select_overtake_side`는 지도 사용을 요구하는 새 제어기에 `map_ready=True`만 넘기고 실제 안전한 `map_target`을 주지 않음.
- Gazebo 스폰은 성공했으나 `wait_camera_ready.py`는 `ready_samples=0`, `last_start_status=null`로 `START_CALIBRATION_FAILED`. 정상 빨간불 시퀀스를 시작하지 않고 안전하게 중단했고 Odometry 이동거리 0m. 현재 자료만으로 신호 카메라 프레임 실패, DDS 준비 지연, 제어 노드 크래시 중 어느 것이 원인인지 확정 불가.
- 원본 QA 보고서는 `reports/team01/runs/42540958903-0da63070800429d394c564746f3335d5bcbad668.md`. 전체 작업 log는 https://github.com/nahee2345/urrc_2_f1/actions/runs/42540958903.

## 1. 개발팀 반영 P0
1. `tests/test_offline.py`: SDF 센서 개수 6개(도로 RGB, 신호 RGB, Depth, 전후 LiDAR, IMU)와 **실제 SDF 센서 이름** 일치 검사, IMU ≥50Hz 및 Ackermann/IMU/steering-joint 플러그인 전체를 순서 무관하게 확인. **센서 모델이나 원본 맵은 줄이지 않음.**
2. `tests/test_v4_road_and_overtake.py`: 추월 허가 시험에서 `map_ready=True`와 함께 현재 도로 폭 내부의 검증 가능한 `map_target=(3.,0.)`을 제공. 안전한 경로 없는 지도 목표 `(3.,1.8)`에서는 지도 자체가 불신되고 추월이 비활성화되어야 하는 역방향 회귀시험 추가. **실제 주행 제어기의 지도 신뢰도/도로 경계 검사와 추월 허가 조건은 완화하지 않음.**
3. `scripts/ci_team01_monza.sh`(차량 공식 Monza 런처): 신호 카메라 baseline 준비 대기 제한을 고정 6초에서 `TEAM01_CAMERA_READY_TIMEOUT_S` 기본 75초로 연장. 누락 시 fail-closed 정지 유지. 후속 디버그를 위해 `controller_start_tail.log`, `start_nodes.txt`, `start_ready_topic.txt`를 아티팩트로 남김. 7초 자동출발은 정상 `camera` 모드에서 **허용하지 않음**.
4. `scripts/wait_camera_ready.py`: 기존 `START_CALIBRATION_FAILED` 종료 코드/상태를 유지하되 `diagnosis`를 `NO_CONTROLLER_READINESS_OR_DIAGNOSTIC_MESSAGES`/`NO_READY_FLAG_MESSAGES`/`CAMERA_BASELINE_NOT_YET_CONFIRMED`/READY로 구분. 실제 카메라가 안 보이는 상태에서 보정 성공을 만들어내지 않음.

## 2. 개발팀 반영 P1 — SLAM 품질과 시간 동기
1. `map_quality.py`: RTAB-Map `proximity_detection_id>0` 단독으로는 지도의 실제 loop closure를 인정하지 않음. 명시적 `loop_closure_id>0`일 때만 지도 신뢰 승인 요건을 충족할 수 있음. `test_v41_map_quality.py`에 근접 탐지 단독 실패/실제 루프 폐합 통과 시나리오 추가.
2. `slam_alignment.py`: 105° RGB와 92° Depth 페어 최대 timestamp 오차 60ms→**25ms**. `slam_scan_fusion.py`: 전방/후방 LiDAR 융합 최대 시차 100ms→**25ms**. 헤드리스 Gazebo에서 서로 다른 물리시각의 장면을 한 위치로 조합해 경계·SLAM 맵이 흔들리는 현상을 줄임. 입력이 너무 늦으면 관측을 건너뛰며 **없는 도로 공간을 생성하지 않음**. `test_v37_rgbd_slam_geometry.py` 동기화 임계 시험 수정.
3. 지도 신뢰도 저하 시에는 기존 실시간 RGB+Depth+LiDAR 도로를 우선하는 규칙을 유지. 신규 지도 자동 속도 승격·20m/s 설정 개방 없음.

## 3. 검증팀이 꼭 확인할 사항
1. 오프라인 기존 229건 이상 + 신규 시험, `test_offline` SDF/플러그인, 새로운 `test_v4_road_and_overtake`, `test_v41_map_quality`, `test_v37_rgbd_slam_geometry` 및 **파일당 400줄** 검사.
2. 신호 카메라 75초 정상 준비: 카메라 토픽 20Hz, `/team01/debug/start.camera_frames`, `/team01/ready_for_lights` 실제 subscriber/publisher, 정상 RED 점등 전 전진명령 0, RED 4→0 및 LIGHTS_OUT 후에만 출발. 75초 후에도 샘플 0이면 `controller.log` 최초 traceback, `ros2 node list`, DDS/bridge QoS와 시뮬레이션 시간부터 확인.
3. RGBD 25ms/전후 LiDAR 25ms 동시 시각 페어의 수신율·통계. 정렬된 RGBD camera info/TF·RTAB pose & loop closure, `map_quality.reason`. 유효 페어를 확보할 수 없다면 임계값을 무작정 완화하지 말고 실제 sensor stamps·ROS bridge 시각 원인을 검증.
4. Sensor/Controller → adapter → Gazebo 실제 휠 joint → Odometry에 대해 5m 연속 구동, 첫 코너, 조향 부호/IMU 응답, 반복 BRAKE, Monza 1랩 및 rosbag2·trajectory·metrics. QA 판정기의 안전 마진 위반과 **base_link 0.3m 공식 실격** 구분.
5. 이전 `0da6307` FAIL과 **새 개발 커밋의 결과를 절대 혼동하지 말 것**. 오프라인 PASS만으로 Gazebo START, SLAM, 코너, 1랩 PASS를 주장하지 말 것.

## 변경 제한
`upstream/urrc_f1_ws` 원본 Monza 서브모듈/월드/벽/트랙/가시 자산/물리 마찰, QA 전용 `scripts/qa_team01_*`, 공식 0.3m 실격 기준은 수정하지 않았습니다. 추가 코너 controller dynamics 변경은 신호 시작부터 실제 주행 데이터가 다시 확보된 이후 검증팀 로그 근거로 수행합니다.
