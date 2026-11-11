# URRC TEAM 11 — 신호등 출발·3경로 주행·충돌 연출·추월 시연

## 1. 현재 기본 시연 모드

- **연습차량 없는 단독 주행이 기본값**입니다. TEAM 11 차량 1대만 생성합니다.
- 원본 Monza/Hungaroring 맵, 벽, 그리드 표시와 교통신호는 수정하지 않습니다.
- Gazebo가 카메라 토픽을 공개하고 TEAM 11의 RGB 이미지 콜백에서 **유효한 첫 프레임을 실제 수신**해야 신호등 점등을 시작합니다.
- 차에 달린 RGB 카메라에서 빨간불 5개 점등을 연속 **2프레임**, 이후 0개 소등을 연속 **4프레임** 확인하면 출발합니다. 시간이 지나기만 해서 자동 출발하지 않습니다.
- 카메라 시야각은 시작 그리드의 신호등 상대 좌표로 계산합니다. 해상도 1280×720, 수평 화각 0.52rad이며 개별 램프의 예상 영상 위치를 이용해 인식합니다.
- 1번 그리드의 실제 Gazebo 카메라 출발은 이전 시연에서 확인했습니다. **그리드 1~12 전체에서 영상 인식 성공은 아직 검증되지 않았습니다.**

## 2. 경로·그리드 선택

- 진행 방향 기준 **1번=왼쪽, 2번=중앙, 3번=오른쪽**입니다.
- 원본 트랙의 그리드 교대 배치를 따라 **홀수 1·3·5·7·9·11번 → 3번(오른쪽)**, **짝수 2·4·6·8·10·12번 → 1번(왼쪽)** 경로를 선택합니다.
- 원래 위치에서 약 12m 동안 5차 부드러운 보간으로 선택된 경로에 합류합니다.
- 단독 주행은 선택 경로를 유지합니다. `--lane 1/2/3 --fixed-lane`으로 각 경로를 별도로 점검할 수 있습니다.
- 세 경로는 전체 트랙 중 최소 도로 폭을 이용해 생성합니다. 벽 여유와 실제 차체 폭을 고려한 도로 영역 검사와 실제 Gazebo 접촉 검사는 구별해야 합니다.

## 3. 속도 제어

| 구분 | 전방 36m 경로 곡률(1/m) | 목표 속도 |
|---|---:|---:|
| 직선·완만한 코너 | 0.030 미만 | 15m/s (10~15m/s 목표 대역) |
| 일반 코너 | 0.030~0.080 | 7m/s에서 5m/s로 점진 감속 |
| 급코너 | 0.080 이상 | 5m/s |

- 직선 복귀는 곡률 0.022/m 이하로 낮아졌을 때 허용하는 **히스테리시스**를 사용합니다.
- 조향각 10° 초과 시 목표 최대 7m/s, 20° 초과 시 최대 5m/s입니다.
- 코너는 X·Y 실제 경로좌표에서 미리 계산하여 감속을 시작합니다.
- 최고 가속도 2m/s², 최고 감속도 4m/s². 출발 직후의 0~10m/s 가속 구간과 추돌 방지 정지는 목표 대역 예외입니다.
- 실제 차량이 물리 구동식이 아니라 `SetWorldPoseCmd`로 움직이는 **키네마틱 시연 모델**이라는 한계가 있습니다.

## 4. 선택적으로 차량을 추가하는 우회·충돌 시험

기본값에는 다른 차량이 없습니다. `--practice-cars`를 지정하면 원본 빨간색 **10m/s** 연습차량(2번 그리드)과 파란색 **20m/s** 연습차량(9번 그리드)을 추가합니다. TEAM 11이 해당 그리드를 쓰면 겹치는 원본 차량만 해당 시연 월드에서 제외합니다.

- 우회는 현 단계에서 **Gazebo 모델 위치**로만 상대차를 감지합니다. 카메라·LiDAR로 다른 차량을 인식하는 구현은 아닙니다.
- 상대 차량은 **전방 50m**에서 탐색하고 각 경로의 차선 위치에 투영해 위치·속도를 추정합니다.
- 전방 35m 이내이고 충돌예상시간(TTC) **6초 이하**, 내 속도가 앞차보다 **2m/s 이상 빠를 때** 우회를 검토합니다.
- 1·2·3번 경로 중 **가까운 안전한 인접 경로**를 우선하며, 중앙에서 양쪽이 모두 안전하면 **왼쪽 1번**을 선택합니다.
- 최소 4초 미래 위치로 이동 중 충돌 여부를 확인합니다. 중간 차선이 점유 중이면 2개 차선을 한 번에 가로지르지 않습니다.
- 이동 거리는 최소 18m이며 횡가속도 2.5m/s²에 맞춰 늘립니다. 부드러운 5차 보간 경로를 따릅니다.
- 직선·완만한 코너는 우회 중에도 최고 15m/s이므로 10m/s 연습차량을 추월할 수 있습니다. 일반·급코너는 5~7m/s입니다.
- 급코너에서는 움직이는 차량 신규 추월을 보류합니다. 정지 장애물은 속도 7m/s 이하에서만 조건부 회피합니다.
- 우회 중 새 위험을 만나면 경로를 급반전하지 않고 감속하며, 모든 경로가 위험하면 앞차와 거리를 유지합니다.
- 앞차보다 10m 이상 앞서고 복귀 경로의 전후방이 안전하면 원래 경로로 복귀합니다.
- 정상 주행의 신호등 및 속도 로직은 변경하지 않고 유지합니다.
- TEAM 11 명목 질량 50kg, 연습차량 명목 질량 1.5kg입니다. 약 0.39m 이내에서는 **상대차만 2초간 튕겨나가는 연출**을 수행하고 TEAM 11의 위치·속도 명령은 유지합니다. **실제 질량에 따른 물리 충돌이 아닙니다.**
- 우회 없이 상대차만 넣어 충돌 효과를 시험하려면 `--practice-cars --fixed-lane`을 함께 사용합니다.

## 5. 실행

```bash
cd ~/urrc_2_f1
git fetch origin team11-three-lane-overtake-v1
git switch team11-three-lane-overtake-v1
git pull --ff-only origin team11-three-lane-overtake-v1

# 기본: 연습차량 없이 1번 그리드 → 3번 경로, 카메라 신호 대기
python3 src/team_11/three_lane/run_demo.py monza --source ~/urrc_2 --grid 1 --start-mode camera

# 기본: 2번 그리드 → 1번 경로, 카메라 신호 대기 (이전 Gazebo 종료 후)
python3 src/team_11/three_lane/run_demo.py monza --source ~/urrc_2 --grid 2 --start-mode camera

# 다른 차량을 추가해 우회·충돌 연출 시험
python3 src/team_11/three_lane/run_demo.py monza --source ~/urrc_2 --grid 1 --practice-cars --start-mode camera

# 테스트 전용: 빨간 시험차 5m/s를 내 차보다 25m 전방에 배치
python3 src/team_11/three_lane/run_demo.py monza --source ~/urrc_2 --grid 1 --practice-cars --scenario slow-ahead --start-mode camera

# 3경로 형상만 각각 확인할 때 (신호 없이 즉시 출발)
python3 src/team_11/three_lane/run_demo.py monza --source ~/urrc_2 --lane 1 --fixed-lane --start-mode immediate
```

`monza`를 `hungaroring`으로 변경해 같은 방식으로 실행할 수 있습니다. 이전 Gazebo는 종료한 다음 새 시험을 시작하십시오.

## 6. 진단과 검증

- 실행 로그: `~/urrc_2/generated/team11/monza/team11_gazebo.log`
- 신호 진단: `grep 'TEAM11 CAMERA' ~/urrc_2/generated/team11/monza/team11_gazebo.log`
- 출발 확인: `grep 'CAMERA GREEN start' ~/urrc_2/generated/team11/monza/team11_gazebo.log`
- 1랩 확인: `grep 'TEAM11 LAP' ~/urrc_2/generated/team11/monza/team11_gazebo.log`
- 속도·코너 확인: `grep 'corner_mode=' ~/urrc_2/generated/team11/monza/team11_gazebo.log`
- 우회·충돌 시연: `grep -E 'TEAM11 AVOID|PACE BOUNCE' ~/urrc_2/generated/team11/monza/team11_gazebo.log`

자동 생성·정적 경로 검사 PASS는 실제 충돌 없는 1랩 완주를 보증하지 않습니다. 주행 및 속도 정책의 최종 판정은 Gazebo 실제 1랩 검증으로 합니다.

## 7. 고정 기준과 우회 실험 분리

- 고정된 단독 주행 브랜치: `team11-solo-baseline-20261111` (수정하지 않음)
- 우회 구현 브랜치: `team11-three-lane-overtake-v1`
- 기본 실행은 연습차량 없음. `--practice-cars` 추가 시 원본 빨간 10m/s, 파란 20m/s 모델을 시연 월드에 생성합니다.
- `--scenario slow-ahead`는 **테스트 전용**이며 생성 월드에서만 빨간 차를 5m/s, 25m 앞에 배치합니다. 원본 맵·연습 차량은 변경하지 않습니다.
- 단위시험·그리드 생성·Gazebo 컴파일 및 주행 로그 검증을 `.github/workflows/team11-overtake-qa.yml`에 분리했습니다.
