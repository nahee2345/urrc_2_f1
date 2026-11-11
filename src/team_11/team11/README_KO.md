# TEAM 11 — 공식 대회 제출용 (독립 패키지)

이 폴더 `src/team_11/team11/`을 **그대로 대회 접수기에 제출**합니다.

## 포함 내용
```text
team11/
  race_entry.json
  src/
    team11_race/
      package.xml / setup.py / setup.cfg / requirements.txt
      race_entry.json
      models/team11_car/model.sdf
      launch/race.launch.py
      team11_race/controller.py
      team11_race/adapter.py
      team11_race/geometry.py
      team11_race/signal_vision.py
      tests/test_policy.py
```

## 실행 계약
- **대회장**이 맵·그리드·차량 위치·5등 신호·계측을 설정합니다. 팀 launch에서 절대로 월드/스폰/시계/운영자 신호를 생성하지 않습니다.
- 모델명 `team11`, 루트 링크 `base_link`, 주행 센서 `/team11/front_scan`, 출발 RGB `/team11/front_rgb`.
- 조향 명령 단위 rad, **양수 오른쪽**. 어댑터에서 Gazebo Twist 부호를 변환합니다.
- `/race/start/team11` Bool false면 무조건 정지합니다. `signal`은 true를 받으면 출발하고, `camera`는 true와 빨간불 5개 점등→소등 인식이 모두 필요합니다.
- 카메라 데이터는 출발 신호에만 사용합니다. **주행 판단은 자기 전방 LiDAR만** 이용합니다.
- 대회 운영자 정보/트랙 파일/GPS/IMU/월드 pose/다른 팀 토픽을 주행 알고리즘이 읽지 않습니다.
- RGB·LiDAR 데이터를 받지 못하면 출발하지 않거나 안전 정지합니다.

## 3경로 원칙
1. 전방 LiDAR에서 양쪽 도로 경계를 추정합니다. 실제 도로 중심을 기준으로 좌/중/우 세 목표선을 생성합니다. 미리 제작한 지도와 그리드 좌표는 사용하지 않습니다.
2. 출발 초기 자기 위치가 도로의 왼쪽이면 **1번 경로**, 오른쪽이면 **3번 경로**를 우선합니다.
3. 직선·완만한 코너는 **최대 15m/s**, 일반 코너는 **7m/s→5m/s**, 급코너는 **5m/s**를 목표로 합니다. 실제 주행 속도는 LiDAR 감지거리·안전 간격·조향 제한에 따라 낮아집니다.
4. 앞 차량이 추돌 위험을 만들면 인접 안전 경로로 5차 보간합니다. 가운데에서 양쪽이 같으면 **왼쪽**을 우선합니다. 미래 상대 속도는 연속 LiDAR로 추정합니다.
5. 대회 차량끼리 충돌시켜 상대만 튕기는 연출은 **공식 참가 모델에 넣지 않았습니다**. 실전에서는 물리 충돌이 일어나며 별도 공격 효과가 없습니다.
6. 속도/조향 명령은 어댑터에서 운영자 Bool 출발 허가와 명령 신선도를 재확인한 후 Gazebo 구동기로 보냅니다.

> 기존 `src/team_11/three_lane`은 원본 트랙 좌표와 월드 위치 기반의 개발·시연 환경입니다. 공식 대회에서 그 코드를 그대로 실행하면 LiDAR 전용 주행 규정을 충족하지 못합니다.
> 본 제출본은 **3경로 선택·코너 속도·안전 우회 정책을 전방 LiDAR 입력에 이식한 첫 버전**입니다. 기존 세계좌표 궤적과 100% 동일한 움직임을 보장하지 않습니다.
> 공식 대회 전체 맵 실주행·1랩 완주·15m/s 안정 주행은 아직 통과 확인이 없습니다.

## 제출 방법
```bash
cd ~/urrc_2_f1
git fetch origin
git switch team11-official-entry-v1
git pull --ff-only origin team11-official-entry-v1

# 공식 접수 프로그램 사용: 제출 폴더는 team11
python3 upstream/urrc_f1_ws/race/scripts/pack_entry.py \
  src/team_11/team11 --output ~/team11-official.zip
```
사용할 때는 **`src/team_11/team11` 폴더 또는 생성한 ZIP**을 대회장에 제출하십시오.
`src/team_11`의 상위 폴더에는 별도 시연 환경 `three_lane`이 존재하므로 통째 제출하지 마십시오.

## 로컬 정적 시험
```bash
PYTHONPATH=src/team_11/team11/src/team11_race \
  python3 -m unittest discover -v \
  -s src/team_11/team11/src/team11_race/tests -p 'test_*.py'
```

차량 스폰을 **절대 하지 않는** 참가 launch를 단독 검사할 경우:
```bash
source /opt/ros/jazzy/setup.bash
colcon build --base-paths src/team_11/team11/src \
  --packages-select team11_race --symlink-install
source install/setup.bash
ros2 launch team11_race race.launch.py race_start_mode:=signal
```
ROS2 launch 단독 실행만으로는 대회장 월드가 생성되지 않습니다. 대회 운영자가 맵과 차량을 먼저 생성해야 합니다.

## 다음 검증의 합격 기준
- 공식 ROS2 Jazzy colcon 빌드, Gazebo Harmonic SDF 파싱
- 1~12번 그리드, 신호등 5개 점등·소등 후 출발, false 즉시 정지
- 전방 LiDAR의 시야·도로 경계 유효성, 1·2·3 경로 선택
- Monza, Spa, Monaco, COTA, Hungaroring, Baku 실제 랩별 완주
- 상대 차량 추월 후 복귀 및 충돌/코스 이탈 없음

후방 LiDAR를 이용한 추월 방어는 **별도 설계 단계**로만 관리합니다. 이번 제출본에는 후방 LiDAR 센서나 방어 제어 코드를 넣지 않았습니다.
