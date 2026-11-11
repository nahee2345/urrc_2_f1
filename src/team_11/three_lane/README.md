# TEAM 11 — Monza / Hungaroring 3중 경로 회피 시연

원본: https://github.com/nahee2345/urrc_2
작업: https://github.com/nahee2345/urrc_2_f1

## 설계
- 도로 중심선을 기준으로 **좌측·중앙·우측 세 개의 경로**를 생성합니다. 순서는 lane_0 / lane_1 / lane_2입니다.
- 원본 CSV road_width_m의 **한 바퀴 전체 구간에서 가장 좁은 도로 폭**으로부터 벽 안전 여유(벽 0.08m, 추가 0.03m, 차체 반폭 0.125m)를 뺀 최대 공통 횡방향 오프셋을 계산합니다. 벽 충돌이 없다는 사실은 별도 Gazebo 검증이 필요합니다. 맵 폭은 실측이 아닌 추정치입니다.
- 차량은 스폰 위치에서 가장 가까운 경로를 선택합니다.
- 35m 전방에 장애물이 있으면 **인접한 빈 경로**를 선택합니다.
- 9m 이동 동안 quintic smoothstep 5차 함수를 사용하여 측면 이동합니다.
- 경로 변경 후 **원래 경로로 자동 복귀하지 않습니다.**
- 전방 20m 곡률과 조향각을 함께 판단합니다. 직선 목표 **10m/s**, 코너 목표 **3~7m/s**입니다. 조향각 |10°| 초과 시 7m/s 이하를 목표로 합니다.
- 속도를 급격하게 변경하지 않습니다. **최대 감속도 4m/s², 최대 가속도 2m/s²**로 제한합니다. 코너 진입 전 감속하고 탈출 후 서서히 가속합니다.
- 현재 시스템은 **물리 조향 모델이 아닌 Gazebo 직접 위치 명령 시연차량**입니다. 실제 조향 관절, 타이어 접지, 가속 제한의 정확한 물리 검증에는 별도 차량 제어기 통합이 필요합니다.

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
- 출발 후 약 12m에 걸쳐 해당 경로에 합류합니다. 출발 직후에는 1번 그리드의 원래 위치에서 점진적으로 오른쪽으로 이동하므로 잠시 중앙 부근에 있을 수 있습니다. 기본 실행에서는 지정된 경로를 계속 유지하도록 자동 차선 변경을 잠갔습니다. 장애물 회피 경로 변경을 시험하려면 `--allow-lane-change`를 명시합니다.

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
- 단위시험은 경로 생성, 장애물 판정, 인접 경로 선택, 원래 경로 비복귀, 5차 보간, 감속 규칙을 확인합니다.
- 코너 벽 간격은 **맵 데이터 기반 설계값**이며, 실제 Gazebo 접촉 없는 1랩/3랩 검증을 통과하기 전까지 실주행 성공으로 판단하지 않습니다.
- 실측 평균속도 10m/s는 아직 검증되지 않았습니다. 직선 기준 목표 속도는 10m/s이며 코너 감속 시 전체 평균은 낮아질 수 있습니다.
- 정적 장애물은 제거했습니다. 기존 두 연습차량의 Gazebo 위치를 직접 참고합니다. 영상/라이다 기반 장애물 인식이나 실제 충돌 없는 추월은 아직 검증되지 않았습니다.
- 블로킹된 두 인접 경로에서 **정지**하며, 불가능한 추월을 강행하지 않습니다.

## 지정 경로에서 중앙으로 이동하는 경우

기본 `--grid 1 --lane auto` 실행은 **3번(우측) 경로를 고정**합니다. 이전에는 3번으로 시작했더라도 35m 전방의 연습차량을 장애물로 판단하면 2번(중앙)으로 자동 변경할 수 있었습니다. 기본 모드는 이를 막고, 원래 그리드에서 12m 합류 구간 이후 우측 경로에 남도록 수정했습니다. 로그의 `target_lane=3 fixed_lane=1`, `lane=3 fixed=1 merge_progress_m=12`를 확인하십시오.

회피 시험에서만 `--allow-lane-change`를 추가합니다. 원본 맵에는 변경이 없습니다.
