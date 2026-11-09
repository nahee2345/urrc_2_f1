# TEAM01 지도 신뢰도·IMU/조향 관절 교정·검증팀 속도 승인 — 개발팀 인수인계

기준 2026-11-09. **코드 변경만 완료**, 실제 ROS/Gazebo/SLAM 실행과 단위·회귀 PASS 판정은 검증팀 전담. Monza 원본 트랙·지도/서브모듈은 변경하지 않음.

## P0 지도 신뢰도 게이트
- 기존에는 `TrackMemory.complete`만으로 SLAM 지도를 경로 계획·추월·속도 상향에 사용할 수 있었습니다. 이제 `map_quality.py` / `map_quality_node.py`에서 `/team01/slam_odom` Odometry와 `/team01_rtabmap/info`(rtabmap_msgs/Info)도 수집합니다.
- 맵 사용 조건 **전부 동시 충족**: (1) 기존 트랙 학습으로 기하학적 폐회로 확인, (2) `team01_map→base_link` TF 최신, (3) RTAB-Map Info의 `loop_closure_id>0` 또는 `proximity_detection_id>0` 이벤트 확인, (4) 최근 3개 이상의 유효 Visual Odometry, (5) ROS 시뮬레이션 기준 최근 0.75초 이내 VO 및 최근 2.5초 이내 RTAB Info, (6) VO 공분산 x/y 각 0~0.09m², yaw 0~0.12rad² (0이면 분산 미제공으로 거부), (7) 순간적인 VO 위치 텔레포트(0.5m+22m/s×dt 초과) 없음.
- `map_target_agrees()`: SLAM 지도 목표가 현재 RGB-D/LiDAR로 실제 **관측된** 좌우 도로 경계 안에서 차체 반폭+0.04m 여유를 갖지 않으면 지도 보조를 거부합니다. 미관측 앞쪽도 지도만으로 주행 가능하다고 인정하지 않습니다.
- 조건 하나라도 실패하면 **기존 실시간 RGB·Depth·LiDAR 기반 제동/코너 인식은 계속 실행**, 지도 기반 경로 보조·지도 기반 추월·상위 속도는 차단. 새 경기 시작 시 과거 loop-closure 신뢰도 초기화.
- `/team01/debug/path.slam_quality`에 `reason`, `loop_confirmed`, `valid_visual_odom_frames` 추가.
- `rtabmap_msgs/Info` 메시지 자체가 설치되지 않거나 topic이 없으면 지도 보조는 비활성으로 유지. 토픽 이름과 QoS가 실제 Jazzy RTAB-Map에서 연결되는지 검증팀이 확인해야 합니다.

## P0 IMU·앞바퀴 부호와 지연 신뢰성
- `feedback_guard.py` 및 `motion_sensors.py`: **명령 조향 좌표계 부호**(Motion.steering의 반대)와 앞바퀴 2개 평균 관절각, IMU Z축 yaw-rate 부호를 비교.
- 명령 |4°| 이상, 실제 조향 |1°| 이상, yaw-rate |0.025rad/s| 이상, 주행속도 0.3m/s 이상, IMU/joint 샘플의 시뮬레이션 시각 기준 각각 0.25초 이내일 때 부호를 평가합니다. 모두 같은 방향인 **4프레임 연속** 관측하고 검증팀이 설정을 승인해야 IMU yaw + 실제 관절각을 폐루프 보정에 사용합니다.
- 조향 명령 4° 이상 변화 시작부터 관절 목표 ±2° 진입까지 **시뮬레이션상 관측 응답 지연**을 계측하며 0.5초 초과가 관측되면 피드백을 차단합니다. 이는 단순 계측이며 폐루프 실제 동특성 전체의 정확한 지연 추정은 아닙니다. 검증팀은 rosbag 시간상관 분석으로 별도 교정해야 합니다.
- 안전 기본값 `qa_approved_imu_joint_calibration: false`: 실제 부호·응답 확인 전에는 새 IMU·관절 폐루프 보정을 적용하지 않고, 기존 odometry yaw 피드백 및 기존 물리 조향 제한기만 사용. 승인 뒤에도 센서 신선도 저하, 실제 반대 부호, 지연 초과 시 즉시 관절 피드백 비활성화. 진단 `/team01/debug/path.motion_sensors`에 허가 상태, sign_match, matching_frames, observed_response_delay_s, raw_joint_steer_rad, active_joint_steer_rad 등.
- **검증팀만** 실제 토픽 Hz, ROS IMU 좌표계(ROS ENU), Gazebo 조향 관절 방향, cmd → joint → yaw 실제 부호와 지연을 확인하여 설정을 승인할 것.

## P0 속도 단계 승인
- 기존 `SpeedStages.cap(map_ready)`는 지도 생성 완료 시 8/10m/s 요청을 허용할 수 있었습니다.
- 변경: `validated_speed_stage_mps`를 8로 설정한 경우에도 **QA가 첫 코너 무이탈 및 실제 Monza 1랩 완료를 각각 승인하고**, SLAM 신뢰도 게이트와 현재 도로 경계 일치를 통과해야 8m/s를 허용. 승인 없이 5m/s 이하 상한 유지.
- 10m/s는 위 조건에 더해 검증팀이 **8m/s 단계 실주행 성능**을 별도 승인해야 허용. 20m/s는 현재 승인 가능한 단계에 포함하지 않음(14~16m 감지 거리·제동거리 문제).
- 설정 변수 (모두 기본 false): `qa_approved_first_corner`, `qa_approved_monza_lap`, `qa_approved_8mps`. `qa_approved_imu_joint_calibration` 역시 기본 false. 단순히 YAML의 값을 true로 바꾸는 것은 실제 검증 결과를 뜻하지 않으며, 검증팀의 아티팩트/실주행 PASS가 근거여야 합니다.
- `/team01/debug/path.speed_stage`에서 `MAP_UNVERIFIED`, `FIRST_CORNER_OR_LAP_NOT_APPROVED`, `EIGHT_MPS_NOT_APPROVED`, `QA_APPROVED` 상태 확인.

## 검증팀 인수인계
1. 이전 전체 회귀 + 수정된 `test_v21_speed_stages.py` + 신규 `test_v41_map_quality.py`, `test_v42_imu_joint_calibration.py`; 파일당 400줄, import 가능 여부, SDF/Gazebo 실행 확인.
2. SLAM Info 이벤트(appearance/proximity), visual-odom 공분산 9999(실패), 중간 데이터 소실, 위치 점프, 오래된 map TF, 미관측·현재 경계 불일치 시 지도 경로 보조 및 맵 기반 상위 속도 차단 확인.
3. 좌·우 실제 조향과 IMU yaw 부호, 원시 joint/IMU timestamp/임계시간, 명령 변동~실제 관절 도달 응답 지연 및 부호 반전. **QA 승인 전에는 기존 odometry/open-loop로 동작하는지** 확인.
4. 5m/s 속도 상한 기본 유지, 단순 map.close로 8m/s 승격되는 상황 금지, 8/10m/s는 검증팀 판정과 설정을 분리해 검증.
5. 출발 신호 RED→OUT, 차체 경계 여유 및 `base_link` 0.3m 공식 실격 분리, 첫 코너·5m 연속·1랩 주행, 원본 rosbag과 `drive_metrics`/`lap.json` 정합 확인. 지도 원본/서브모듈 해시 실행 전후 불변.
6. 검증팀이 직접 PASS 또는 FAIL 판정을 내리고 Issue #6에 사례별 첫 실패 로그·고정된 SHA·재현 단계를 회신.

**이번 인수인계는 개발 완료 보고입니다. 검증팀이 실제 독립 PASS를 확인하기 전까지 Monza 1랩 FAIL을 유지합니다.**
