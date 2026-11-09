# TEAM01 — 점진 코너 조향·RGB-D/LiDAR SLAM·IMU 폐루프 개발 보고
2026-11-09 제작팀 인수인계. **개발 코드는 반영했지만 실제 Gazebo 주행/SLAM 검증은 미실행입니다.** 검증은 검증팀에서 진행합니다.

## 구현 변경
1. **한쪽 조향 증가 유지:** SteeringIntent는 0→5→10→15°를 정상적인 한쪽 코너 명령으로 통과시킵니다. 15→-15°와 같은 급격한 반대방향 명령은 도로 heading에서 반대방향 실제 코너를 **3프레임 연속 확인**하기 전까지 0° 목표로 억제합니다. 긴 직선 이후에도 직전 코너 방향 기억을 보존합니다. 기존 조향 변화율과 차체 충돌검사는 그대로 적용합니다.
2. **IMU + 실제 조향관절 폐루프:** 차량 model.sdf에 100Hz IMU와 50Hz front-left/right steering joint 상태 발행기 추가. ROS bridge: `/team01/imu`, `/team01/steering_joint_states`. `motion_sensors.py`는 IMU yaw-rate를 기존 Odometry yaw-rate의 우선 피드백으로 사용하며, 미수신 시 Odometry 사용. 관절 평균 실측각도 오차를 최대 ±3° 보정한 뒤 기존 경로 안전검사와 SteeringLimiter를 통과합니다. **IMU만으로는 바퀴 조향각을 직접 측정할 수 없으며 관절 상태가 반드시 필요합니다.**
3. **RGB-D Visual SLAM:** 105° 도로 RGB와 92° Depth를 그대로 같은 영상으로 취급하던 문제를 광학 재투영으로 수정하여 Depth 광선에 맞는 RGB와 CameraInfo를 발행. RTAB-Map `rgbd_odometry` + LiDAR ICP `Reg/Strategy=2` 구성. RGB/Depth 유효 시간 차 60ms 초과 시 페어 거부.
4. **전방+후방 LiDAR 지도 보강:** 전방(+0.187m)·후방(-0.208m, 180°) LaserScan을 base_link 좌표로 360° 융합하여 `/team01/slam/scan` 발행. 센서가 보지 못한 영역은 `inf`로 남겨 **미관측 영역을 안전 공간으로 추정하지 않음**. RTAB-Map DB는 실행별 별도 파일로 저장하여 과거 지도를 새 경기 초기에 주입하지 않음.
5. **기존 Monza 원본 맵 및 대회 실격 규칙 불변**. 차량 모델/ROS 런처/제어기/SLAM 파일만 변경. RTAB-Map 미설치 시 기존 안전 인식으로 계속 주행하되 SLAM은 꺼집니다.

## 검증팀 필수 확인 (각 항목 별도 PASS/FAIL)
- 기존 회귀 전체 + 신규 `test_v37_rgbd_slam_geometry.py`, `test_v38_imu_closed_loop_and_turn_intent.py`, `test_v39_rgbd_lidar_slam_fusion.py` + 400줄 검사, 모델 XML/SDF 적재, 원본 Monza 서브모듈 SHA 및 SDF 해시 전후 불변.
- 조향 0→5→10→15° 허용, 15→-15°의 불필요한 급반전 억제, 실제 반대 코너 3연속 인식 후 서서히 전환. 예측 조향각과 실제 2개 앞바퀴 관절각, IMU angular_velocity.z, odom yaw-rate, 실제 이동 궤적의 부호·시간지연 비교. 센서가 없을 때 OPEN_LOOP 전환과 안전제동 유지.
- IMU `/team01/imu`, 조향 `/team01/steering_joint_states` 실제 토픽 Hz/data/TF; Gazebo 모델 수준 IMU 시스템 플러그인 로드 여부를 반드시 검사. 플러그인 미작동이면 IMU 기능 PASS 처리 금지.
- RTAB-Map 설치 여부, 105°→92° RGB-D 정렬, `/team01/slam/scan` 전후방 스캔 360° 범위 및 타임스탬프, RGBD visual odometry와 `team01_map→team01_odom→base_link` TF, 점유 지도·DB 파일 생성, loop closure/드리프트 관측. **지도 정확도 100% 보장은 불가능합니다.**
- 정상 RED 점등 전 0 전진; 실제 5m 연속 및 첫 코너 안전 주행, 조향 제한 991회 개선 여부, BASE_LINK 0.3m 실격과 QA 안전여유 경고 구분, 독립 ROSbag/drive_metrics/lap.json 정합, Monza 실제 1랩 PASS.
- RTAB-Map이 사용 불가할 때 `SLAM_DISABLED`로 분리. Visual odometry 특징점 부족이나 scan ICP 불일치 시 사전 지도만으로 주행 허용하지 말 것.

## 추가 개선 권고
- SLAM 지도 신뢰도/loop closure 품질 게이트와 RGB-D+LiDAR+IMU의 프레임·시각 동기화 계측.
- 좌우 바퀴 실제 각도와 조향 명령의 부호/비례값 교정. IMU는 차체 요 응답을 검증하며 바퀴각 센서가 아닙니다.
- 검증팀이 첫 코너·5m 연속 및 Monza 1랩 PASS를 증명하기 전 10~20m/s 고속 활성화 금지.

**현재 상태: 기능 코드 반영 / 독립 실제 검증 및 완주 PASS 미획득.**
