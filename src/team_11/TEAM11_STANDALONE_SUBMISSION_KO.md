# TEAM 11 — 독립 제출 패키지 및 내부 토픽 규칙

## 제출물

`urrc_2_f1/src/team_11` 폴더를 통째로 전달할 수 있습니다.
이 폴더에는 `race_entry.json`, 차량 SDF, ROS 2 패키지,
launch 파일, 제어기, 6개 맵의 중앙선·그리드·3개 주행 경로가 있습니다.
**대회장 `urrc_f1_ws` 원본 맵·코드·설정은 수정하지 않습니다.**

대회장 접수기는 제출 폴더의 `race_entry.json`과
`package.xml`을 읽어 설치할 수 있습니다.
`team_11` 폴더가 ROS 2 패키지의 루트이며 `package.xml`, `race_entry.json` 및 6개 맵 경로를 포함합니다.

## 내부 토픽 충돌 방지

| 역할 | 토픽 |
| --- | --- |
| 속도 목표 | `/team11/speed_11` |
| 조향 목표 | `/team11/steering_11` |
| 속도·조향 명령 브리지 | `/team11/cmd_vel_11` |
| 차량 자체 PosePublisher | `/team11/pose_11` |
| 출발 신호용 RGB | `/team11/start_image_11` |
| 전방/후방 LiDAR (주행 판단에 사용하지 않음) | `/team11/front_scan_11`, `/team11/rear_scan_11` |
| 차량 IMU | `/team11/imu_11` |
| 차량 오도메트리 | `/team11/odom_11` |
| 조향 조인트 상태 | `/team11/joint_states_11` |

운영자가 제공하는 외부 출발 신호 `/race/start/team_11` 및
중앙 `/clock`은 팀의 내부 토픽이 아닙니다.
대회장 계약에 맞춰 외부 이름을 유지합니다.

## 랜덤 맵 식별

시작할 때 Gazebo의 공개 토픽 이름에서
`/world/<맵 이름>/...`을 찾아 현재 맵을 식별합니다.
맵을 특정하지 못할 경우에는 자기 차량의 초기 스폰 좌표를
TEAM11에 내장된 6개 맵의 출발 그리드와 비교합니다.
정보가 없거나 서로 충돌하면 **속도 0으로 대기**합니다.
맵 원본을 수정하거나 생성하지 않습니다.

## 구동 구조

대회 운영자는 Gazebo 월드와 `team_11` 차량을 스폰합니다.
TEAM11 launch는 `ros_gz_bridge`, `team11_control`,
`team11_adapter`만 기동합니다.
주행 판단은 내장된 맵별 CSV 좌표와
자기 차량 SDF에 포함된 PosePublisher의 현재 위치로 수행합니다.
LiDAR는 주행 경로 결정에 사용하지 않습니다.

미수신/오류/타임아웃 시 속도 명령은 0입니다.
연습용 `three_lane/run_demo.py`는 별도 시연 실행기이며
대회 제출용 `launch/race.launch.py`와 다릅니다.

## 다른 노트북 설치

```bash
# 예시: 전달 받은 team_11 폴더가 ~/team_11 에 있을 때
mkdir -p ~/team11_ws/src
cp -a ~/team_11 ~/team11_ws/src/
cd ~/team11_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select team11_race
source install/setup.bash
# 실제 대회에서는 운영 실행기가 아래 launch를 자동 실행
ros2 launch team11_race race.launch.py
```

`race_start_mode`, `race_start_topic`, `speed_topic`,
`steering_topic`, `lidar_topic`, `use_sim_time`은
운영자의 launch 인자를 그대로 받아 적용합니다.

## 검증 상태

지도·서브패키지·토픽 정적 검사를 실행할 수 있습니다.
대회장 노트북에서 실제 카메라 소등 출발, 물리 주행,
1랩 완주, 다중 차량 추월·후방 방어는 별도로 확인해야 합니다.
기존 `three_lane` 시연 플러그인의 모든 복합 경주 기능이
신규 독립 ROS 제어기에 동일하게 이식된 상태는 아닙니다.
