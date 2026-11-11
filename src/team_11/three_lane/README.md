# TEAM 11 — Monza / Hungaroring 3중 경로 회피 시연

원본: https://github.com/nahee2345/urrc_2
작업: https://github.com/nahee2345/urrc_2_f1

## 설계
- 원본 Monza/Hungaroring 맵에서 **좌측 1번·중앙 2번·우측 3번**의 경로를 생성합니다. 좁은 도로 구간을 고려해 차체와 벽 사이에 여유를 둡니다.
- **홀수 그리드는 3번, 짝수 그리드는 1번** 경로를 주행합니다. 초기 12m 동안 해당 경로에 합류합니다.
- 카메라가 빨간불 5개 점등 후 5개 전체 소등을 확인해야 출발합니다. 신호 상태 토픽을 직접 읽지 않습니다.
- **기본 주행은 우회 허용**: 앞차 위치·상대 속도를 기반으로 전방 추돌 위험을 판단하고, 앞뒤 공간이 비어 있는 인접 경로로 이동합니다.
- **12m 5차 다항식 차선변경**, 변경 중 최고 7m/s, 연속 변경 억제, 추월 대상과 6m 이상 벌어진 뒤 원래 홀짝 경로로 복귀합니다.
- **직선 최고 목표 15m/s (목표 범위 10~15m/s)**, 코너는 **3~7m/s**, 조향 10도 초과 시 최고 7m/s입니다. 가속 2m/s²·감속 4m/s²로 점진적으로 변경합니다.
- 가용 인접 경로가 없으면 점진적으로 제동하도록 명령합니다.
- 장애물 감지는 현재 **Gazebo의 기존 연습차량 위치 기반**이며 카메라·LiDAR 장애물 인식이 아닙니다.
- 차량은 물리 조향·접지력 모델이 아닌 직접 위치 명령 모델입니다. 충돌에 대한 기존 상대차 튕김 역시 스크립트 연출입니다.
- `--fixed-lane`으로 우회를 끄고 지정 경로만 테스트할 수 있습니다.

## Ubuntu 24.04 / Gazebo Harmonic

```bash
sudo apt update
sudo apt install -y git cmake g++ libgz-sim8-dev
cd ~
git clone https://github.com/nahee2345/urrc_2.git
git clone -b team11-three-lane-practice https://github.com/nahee2345/urrc_2_f1.git
cd ~/urrc_2_f1
python3 src/team_11/three_lane/run_demo.py monza --source ~/urrc_2 --start-mode camera
# 즉시 출발 비교 시험: 끝에 --start-mode immediate 사용
# 다른 맵
python3 src/team_11/three_lane/run_demo.py hungaroring --source ~/urrc_2 --start-mode camera
```

위 두 명령은 **동시에 실행하지 마세요.** 각 실행은 Ctrl+C로 종료 후 다음 맵을 시작합니다.

생성만 확인:

```bash
python3 src/team_11/three_lane/run_demo.py monza --source ~/urrc_2 --prepare-only
python3 src/team_11/three_lane/run_demo.py hungaroring --source ~/urrc_2 --prepare-only
python3 -m unittest discover -v -s src/team_11/three_lane -p 'test_*.py'
```

원본 맵과 메쉬 파일은 수정하지 않습니다. 새 world/CSV는 `~/urrc_2/generated/team11/<track>` 에 생성합니다.

## 시연 내용
- **1번 그리드 노란색 TEAM11 차량**: 스폰 위치에 가장 가까운 3개 경로 중 하나를 선택합니다. 기본값은 **camera 출발**이며 빨간불이 보이다가 꺼지면 출발합니다. `--start-mode immediate`로 즉시 출발을 시험할 수 있습니다.
- **2번 그리드 빨간색 연습차량(10m/s)** 및 **9번 그리드 파란색 연습차량(20m/s)**: 원본 시연차량 유지, 기존 5등 소등 신호 이후 출발합니다.
- **정적 장애물은 더 이상 생성하지 않습니다.** 기존 생성 월드도 매번 덮어써서 제거됩니다.
- TEAM11 차량은 두 연습차량을 Gazebo 위치로 감지하여 필요하면 인접 경로로 이동합니다.
- `generated/team11/<track>/team11_gazebo.log`에서 `[TEAM11] IMMEDIATE start` 로그와 주행 상태를 확인하십시오.
- TEAM11 차량은 아직 *Gazebo 위치 기반 시연 차량*입니다. 전방 카메라 영상의 빨간색 픽셀을 검사하는 출발 게이트가 추가되었으며, 실제 신호 인식 정확도는 Gazebo 실주행으로 검증해야 합니다.

## 경로 1·2·3 단독 테스트

도로 **진행 방향 기준** 왼쪽에서 오른쪽으로 1번(좌측) / 2번(중앙) / 3번(우측)입니다. 파일은 각각 `lane_0.csv`, `lane_1.csv`, `lane_2.csv`이며 **번호가 1씩 차이 납니다.**

```bash
cd ~/urrc_2_f1
git switch team11-three-lane-practice
git pull --ff-only origin team11-three-lane-practice

# 1번 경로 (왼쪽)
python3 src/team_11/three_lane/run_demo.py monza --source ~/urrc_2 --start-mode immediate --lane 1 --fixed-lane

# 2번 경로 (중앙) - 이전 Gazebo 종료 후 실행
python3 src/team_11/three_lane/run_demo.py monza --source ~/urrc_2 --start-mode immediate --lane 2 --fixed-lane

# 3번 경로 (오른쪽) - 이전 Gazebo 종료 후 실행
python3 src/team_11/three_lane/run_demo.py monza --source ~/urrc_2 --start-mode immediate --lane 3 --fixed-lane
```

Hungaroring도 `monza`를 `hungaroring`으로 바꿔 동일하게 시험합니다. 고정 모드는 장애물이 있어도 자동 경로변경을 하지 않으므로 **경로 형상·벽 간격 확인에만 사용**하십시오. 항상 1번 그리드에서 출발하며 선택된 경로까지 처음 12m 구간에 부드럽게 합류합니다. 실전의 가장 가까운 경로 자동 선택과 장애물 회피는 `--lane auto`로 시험합니다.

로그에는 `[TEAM11] ... lane=1`처럼 1부터 시작하는 번호로 출력됩니다.

## 출발 그리드에 따른 기본 경로

- **홀수 그리드(1·3·5·7·9·11)** → **3번 경로(우측)**
- **짝수 그리드(2·4·6·8·10·12)** → **1번 경로(좌측)**
- `--lane auto`에서 위 규칙을 사용합니다. `--lane 1/2/3`은 시험용 강제 지정입니다.
- 실제 출발 위치는 `--grid 1..12`로 선택합니다. 원래 연습차량의 출발 자리 2번 또는 9번을 TEAM11이 사용하면 해당 원본 연습차량을 *생성된 시연 월드에서만* 제외합니다.
- 카메라의 빨간불 5개 점등 → 전부 소등 확인 후 출발합니다.
- 출발 후 약 12m에 걸쳐 해당 경로에 합류합니다. 출발 직후에는 1번 그리드의 원래 위치에서 점진적으로 오른쪽으로 이동하므로 잠시 중앙 부근에 있을 수 있습니다. 기본 실행에서 앞차가 주행 경로를 막으면 안전한 인접 경로로 우회합니다. 원래 경로를 고정하려면 `--fixed-lane`을 사용합니다.

```bash
python3 src/team_11/three_lane/run_demo.py monza --source ~/urrc_2 --grid 1 --lane auto --start-mode camera
# 종료 후 홀수 다른 예시
python3 src/team_11/three_lane/run_demo.py monza --source ~/urrc_2 --grid 9 --lane auto --start-mode camera
# 종료 후 짝수 예시
python3 src/team_11/three_lane/run_demo.py monza --source ~/urrc_2 --grid 12 --lane auto --start-mode camera
```

## TEAM11 전용 충돌 연출

- 생성된 월드에서 노란 TEAM11 차의 **명목 질량 50kg**과 연습차량 **명목 질량 1.5kg**을 설정합니다.
- `pace_bounce/PaceCars.cc`는 두 차량이 약 0.39m 거리로 접근하면 **상대 페이스카에만** 측면 이탈·상승·회전 효과를 2초간 적용하고 원래 경로로 부드럽게 복귀시킵니다.
- 노란 TEAM11 차량의 속도·경로·위치는 충돌 연출 때문에 변경하지 않습니다.
- **이는 물리 엔진으로 무거운 차가 가벼운 차를 밀어내는 진짜 충돌이 아닙니다.** 두 모델 모두 `static=true`이며 매 업데이트에 `SetWorldPoseCmd`로 움직이므로 해당 명목 질량은 실제 충격량 계산에 사용되지 않습니다.
- 이 효과는 현재 **시연용 빨간색·파란색 연습차량에만 적용**됩니다. 다른 팀의 임의 차량에 적용하려면 그 차량의 동적 충돌·제어 인터페이스가 별도로 필요합니다.
- 트랙·벽·기존 `urrc_2` 파일을 수정하지 않으며, `urrc_2_f1/src/team_11`에 전용 플러그인을 보관합니다.
- 실제 Gazebo 내 물리·시각 동작, 상대차 벽 접촉과 튕김 정도는 아직 실주행 검증이 필요합니다.

## 전방 카메라 5등 인식 출발 (기본값)

- 1번 그리드 노란색 차량에 고정된 전방 RGB 카메라(`/urrc/<track>/team11/front/image`)가 신호등을 관찰합니다.
- 밝은 빨간색 영역 중 **가로 한 줄에 정렬된 5개 램프**를 구분하고 **2프레임 연속** 5개 점등을 확인해야 합니다.
- 그 후 **빨간 램프 0개를 4프레임 연속** 확인해야 출발합니다. 1~4개에서 소등되지 않았는데 출발하는 것은 금지합니다.
- 영상이 없거나 형식이 잘못되었거나 5등을 본 적이 없으면 계속 대기합니다. 실제 영상 인식 정확도는 Gazebo 주행 검증 필요.
- 진단: `grep 'TEAM11 CAMERA' ~/urrc_2/generated/team11/monza/team11_gazebo.log`로 `images`, `red_lamps`, `all_five_seen`, `consecutive_off` 확인.
- 물리 충돌: TEAM11은 원본 `add_car()`가 만든 **static=true 차량**이고 `SetWorldPoseCmd`로 매 프레임 이동합니다. **벽 접촉 시 반동·자동정지·물리 충돌 응답은 구현되어 있지 않습니다.**

## 검증 범위·주의
- Python 경로·설정 자동 검사는 실제 시뮬레이션 주행 성공을 보장하지 않습니다.
- 우회 감지 대상은 현재 그리드 02·09의 연습차량입니다. 다른 팀 차량을 범용 감지하는 기능은 별도 작업이 필요합니다.
- 직접 위치제어 모델이므로 실제 타이어·조향·충돌 물리의 정확한 재현이 아닙니다.
- Gazebo 실주행에서 신호 인식, 1랩 완주, 추월 및 복귀, 충돌 여부를 추가 검증해야 합니다.

## 중앙 경로로 이동하는 이유

1번 그리드는 기본적으로 **3번 경로에서 출발**합니다. 앞차 추월을 위해 2번 경로로 **일시적으로 이동한 뒤**, 상대 차량을 추월하고 안전 간격이 확보되면 다시 3번 경로로 복귀하도록 설계했습니다. 각 이동은 12m 동안 부드럽게 실행하고, 앞뒤 차량 간격을 확인합니다.

그리드 선택만 검증할 경우에는 `--fixed-lane`을 추가해 우회를 비활성화하십시오. 기본 실행의 로그에서 `[TEAM11 AVOID] pass`, `[TEAM11 AVOID] return` 및 `lane=...` 기록을 확인할 수 있습니다.

## 그리드별 신호등 인식 카메라 보정

차량이 그리드 1~12에서 출발하면 카메라의 설치 상대 방향을 그리드에서 시작 신호등까지의 위치로 자동 계산합니다. 기존에 모든 그리드에 고정 적용된 카메라 pitch=-0.43rad 대신 해당 그리드에 맞는 pitch/yaw를 적용합니다. 1280×720 해상도와 0.52rad 화각으로 멀리 있는 신호등 5개를 더 크게 표시합니다. 기존 카메라 기반 **빨간불 5개 동시 점등 후 전체 소등** 판정 규칙은 유지합니다.

- 실제 신호등 색상 판정은 TEAM11에 부착된 카메라 영상으로 처리합니다. 월드의 신호 상태를 직접 읽어 출발하지 않습니다.
- `team11_manifest.json`의 `camera_target_distance_m`, `camera_pitch_rad`, `camera_yaw_rad`에서 해당 그리드 보정값을 확인할 수 있습니다.
- `grep 'TEAM11 CAMERA' ~/urrc_2/generated/team11/monza/team11_gazebo.log`에서 수신 프레임·인식 개수·연속 소등 횟수를 점검합니다.
- **주의:** Gazebo 카메라 실제 렌더링에서 5개 인식 및 출발은 별도로 검증해야 합니다. 월드 생성 검사만으로 성공을 단정하지 않습니다.
