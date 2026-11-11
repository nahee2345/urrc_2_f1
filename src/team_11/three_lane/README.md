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
- 단독 주행도 코너 접근 시 1·3번에서 중앙 2번 경로로 합류합니다. `--lane 1/2/3 --fixed-lane`을 지정한 경우에만 해당 경로를 계속 고정해 점검합니다.
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
- 기본 차량은 `SetWorldPoseCmd`로 움직이는 **키네마틱 시연 모델**이지만, 실제 위치·자세 명령은 ±30° 조향·최대 회전속도·곡률 한도를 만족하도록 바이시클 모델로 적분합니다. 별도 `--physics-contact` 실험에서 실제 힘/토크 구동을 사용합니다.

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
git fetch origin team11-corner-center-steer30-v1
git switch team11-corner-center-steer30-v1
git pull --ff-only origin team11-corner-center-steer30-v1

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

## 8. 후방 LiDAR 추월 방어 (독립 추가 기능)

기존 주행·5등 RGB 신호 출발·3경로 고정 좌표·전방 추월/복귀 알고리즘은 그대로 유지합니다.
새 기능은 `RearScan.hh`(실제 레이저 측정 검증), `RearDefense.hh`(후방 위협/경로 선택)
에 분리되어 있습니다. `--practice-cars`를 생략하거나 `--fixed-lane`이면 방어를 실행하지 않습니다.

**발동 조건:** 후방 LiDAR 거리 데이터가 수신되고, 기존 연습 상대차 추적 데이터에서
후방 60m 이내·뒤차 상대속도 2m/s 이상·예상 추월시간 5초 이하를 모두 충족할 때만
후방 추월 방어 후보를 검토합니다. 근접해도 상대속도가 부족하면 현재 경로를 유지합니다.

**우선순위:** ① 기존 전방 추월·우회·복귀 ② 뒤차 추월 방어 ③ 정상 경로 추종.
기존 전방 추월을 시작할 수 있거나 현재 경로 변경 중이면 방어를 시작하지 않습니다.
중간·급코너에서는 방어를 새로 시작하지 않습니다.

**경로가 확인된 뒤차:** 뒤차가 추월하려는 좌/중/우 경로를 향해 이동하되,
3→1과 같은 2차선 이동은 반드시 2번을 거쳐 각각 5차 경로변경으로 수행합니다.
예측한 이동 중 실제 차량 겹침이 있거나 진입 순간 후방 거리가 부족하면 방어하지 않습니다.
이는 정해진 *시뮬레이션 충돌 연출*의 진로 선점 기능입니다.

**S자 주행:** 후방 차량의 횡방향 이동이 크거나 경로가 불확실하면,
1·3번 경로를 양끝 목표로 번갈아 선정하고 2번 경로를 중간 경유합니다.
경로변경 거리 최소 18m, 횡가속도 제한 2.5m/s², 급격한 방향 반전 금지.
각 단계 전에 미래 차량 위치를 검사하고, 안전하게 이동할 수 없으면 S자 이동을 보류합니다.
추월 위험이 해소되면 원래 경로로 복귀합니다.

**센서·물리 한계:** 실제 후방 `gpu_lidar`를 설치하고 30Hz 후방 스캔을 받지만,
대상 차량의 ID·진행거리·속도는 기존의 Gazebo 월드 pose 기반 연습차 추적을
후방 LiDAR의 실제 거리 반환점과 대조해 얻습니다. 따라서 *후방 LiDAR만으로*
상대차를 분류하거나 코너 너머 차량을 인식하는 단계는 아닙니다.
기존 충돌 시 상대차가 튕기는 것은 `pace_bounce`의 스크립트 연출이지 실제
물리 충돌에서 TEAM11에 영향이 없다는 보장은 아닙니다.
공식 대회 환경 호환성도 별도 검증이 필요합니다.

**재현 명령 (생성 월드만 변경, 원본 Monza는 변경하지 않음):**

```bash
cd ~/urrc_2_f1
git fetch origin team11-rear-lidar-defense-v1
git switch team11-rear-lidar-defense-v1
git pull --ff-only origin team11-rear-lidar-defense-v1
python3 src/team_11/three_lane/run_demo.py monza \
  --source ~/urrc_2 --grid 1 --start-mode camera \
  --practice-cars --scenario rear-fast
```

`--scenario rear-fast`는 청색 20m/s 연습 차량을 중앙 2번 경로의
TEAM11 후방 약 110m에 배치하고, 전방 추월 충돌을 피하려고
해당 시나리오에서만 적색차를 생략합니다. 일반 `--practice-cars`의
적색 10m/s·청색 20m/s 원래 설정은 변하지 않습니다.

**로그:** `TEAM11 REAR`(실제 scan),
`TEAM11 DEFENSE`(방어 경로 이동/복귀),
`TEAM11 AVOID`(기존 전방 추월),
`PACE BOUNCE`(상대차 충돌 연출).

## 9. 후방 방어 안정화 및 물리 충돌 시험 (v2)

- 개발 브랜치: team11-defense-physics-v2 (기존 고정 주행 및 전방 추월 브랜치는 변경하지 않았습니다).
- 후방 LiDAR는 서로 다른 시간에서 세 번 이상의 유효한 차량 반사 확인 후에만 TTC 5초 이하 방어를 시작합니다.
- 3번→2번 방어 경로 변경 후 자동으로 3번 복귀하지 않습니다. 2번 경로에서 정상 주행합니다.
- 실제 S자 접근 또는 추월 임박한 뒤차에 대해 차단 가능한 단일 경로가 없을 때 조건부 S자 주행 후보를 만듭니다.
- 기존 전방 추월 및 복귀, 3경로 좌표 추종, 5등 RGB 출발 및 코너 속도 정책이 우선합니다.
- 후방 차량을 차단할 때 직접 충돌하도록 좌우 급조향하지 않으며, 단계적 5차 경로 이동만 허용합니다.

### 물리 충돌 실험 모드 (기본값 유지)

```bash
cd ~/urrc_2_f1
git fetch origin team11-defense-physics-v2
git switch team11-defense-physics-v2
git pull --ff-only origin team11-defense-physics-v2
python3 src/team_11/three_lane/run_demo.py monza \
  --source ~/urrc_2 --grid 1 --practice-cars \
  --scenario rear-fast --physics-contact --start-mode camera
```

- 기본 주행은 과거와 똑같이 키네마틱/스크립트 튕김 연출입니다. 새로운 물리 시험은 --physics-contact를 지정한 경우에만 활성화됩니다.
- 물리 시험 모델은 TEAM11=50kg, 연습차량=1.5kg, 동적 강체와 관성·마찰·실제 충돌 형상을 사용합니다.
- 물리 모드는 두 차량을 위치 텔레포트로 이동하지 않고 제한된 힘과 요 토크로 주행시킵니다. 실제 충돌이 두 차량 모두에 영향을 미칠 수 있습니다.
- 기존 스크립트로 상대 차량만 튕기는 기능은 물리 모드에서 꺼지며, 완전한 충돌 무반동은 보장되지 않습니다.
- 물리 힘 기반 15m/s 주행의 완주·안정성·충돌 결과는 실제 Gazebo 시험을 통과해야만 인정합니다.
- 변경되는 맵은 generated/team11 아래의 생성 월드뿐이며, 원본 맵·그리드는 변경하지 않습니다.

## 10. 코너 중앙 합류 / 최대 조향 ±30° (별도 개발 브랜치)

- 현재 개발 브랜치: `team11-corner-center-steer30-v1`.
- 본 변경에서는 기존 3경로 CSV, 전방 추월 `OvertakePlanner.hh`, 후방 방어 `RearDefense.hh`, RGB 신호등 출발 및 연습차량 물리 시험 코드를 삭제하지 않습니다.
- 새 `CornerPlanner.hh`는 원본 도로 중앙선의 52m 앞 곡률을 미리 검사해 코너가 가까우면 1번/3번 경로에서 **2번(중앙) 경로**로 합류를 계획합니다.
- 합류 전부터 7m/s(급코너 5m/s) 이하로 감속하며, 진행 중인 전방 추월과 경로변경을 우선합니다. 중앙 경로가 충돌 위험이 있으면 강제 합류하지 않습니다.
- 2번 경로 진입 완료 후에는 중앙 경로를 유지하며 코너 통과 뒤 자동으로 기존 1/3번 경로에 돌아가지 않습니다. 다음 전방 추월이나 새로운 안전한 방어 상황은 기존 정책으로 판단합니다.
- 사용자 요청에 맞춰 **연습차량 없는 기본 솔로 주행도 코너 중앙 합류를 실행합니다.** 특정 경로 고정 시험이 필요하면 `--fixed-lane`을 지정합니다.
- `BoundedTracker.hh`는 경로 CSV의 위치와 방향을 즉시 강제하지 않고, **키네마틱 바이시클 모델**로 다음 위치·방향을 계산합니다. 조향각 ±30°와 최대 회전속도 약 68.75°/s(1.2rad/s)를 지키도록 경로 곡률을 제한합니다.
- 차량 축거 0.33m 기준 실행 가능한 최대 곡률은 tan(30°)/0.33m ≈ **1.75m⁻¹**, 최소 회전반경은 약 **0.572m**입니다. 횡가속도 기준 2.5m/s²도 적용하며, 도달 불가능한 곡률을 요구하면 진행을 중단합니다.
- 목표 속도·조향각은 실제 목표 경로와 실시간 추적 오차에 따라 낮아질 수 있습니다. 현재 `--physics-contact` 모드는 별도 힘/토크 동역학 시험이며, 실제 접촉에 의한 추가 회전까지 절대 제한하는 것을 의미하지 않습니다.
- 진단 로그: `TEAM11 CORNER`(코너 중앙 합류), `yaw_rate_deg_s`, `max_steer_deg`, `track_error_m`, `steer`, `preview_curvature`.

### 실행

```bash
cd ~/urrc_2_f1
git fetch origin
git switch team11-corner-center-steer30-v1
git pull --ff-only origin team11-corner-center-steer30-v1
python3 src/team_11/three_lane/run_demo.py monza \
  --source ~/urrc_2 --grid 1 --start-mode camera
```

코너 합류만 검증하려면 `--start-mode immediate --server`로 실행할 수 있습니다. 기본 `--start-mode camera`에서는 기존의 5등 적색 신호등 소등 인식 후 출발합니다.

기존 전방/후방 추월·충돌 연습은 `--practice-cars` 옵션을 추가합니다. 원본 맵은 수정하지 않으며 실제 주행 중 도로 이탈 여부는 별도의 Gazebo 장시간 시험으로 검증해야 합니다.

## 11. 코너 합류 감속 보정 / 전방 추월 TTC 3초 (별도 브랜치)

- 개발 브랜치: `team11-corner-speed-late-pass-v1` (기존 3경로·전방 추월/복귀·후방 방어·카메라 출발·물리 실험 기능 유지).
- **추월 시작:** 같은 경로 전방 차량과 실제 상대 접근속도를 기반으로 `(전방 경로상 거리-차량 충돌 판정거리 2m)/접근속도`를 계산하며, **예상 충돌까지 3초 이내일 때만** 추월을 검토합니다. 기존 35m 미만이라는 이유만으로 미리 추월하지 않습니다.
- 전방 차량이 느려도 TTC 3초를 넘으면 기본 경로를 유지하며, 필요하면 추돌 방지를 위해 점진적으로 감속합니다. 옆 경로가 충돌 위험으로 막혀 있으면 억지로 추월하지 않고 감속합니다.
- 3초는 **추월 시작 허용 기준**이지 충돌 보장 시간이나 강제 충돌 명령이 아닙니다. 코너·진행 중 경로변경·안전 경로 확인 등의 제약은 그대로 유지됩니다.
- **코너 감속:** 곡률 최대치를 36~52m 앞에서 발견했다는 이유만으로 현재 속도를 바로 5~7m/s 혹은 2~3m/s까지 낮추던 방식을 개선합니다.
- `SpeedProfile.hh`에서 각 곡률이 나타나는 **남은 거리와 최대감속도 4m/s²**를 고려해 필요한 시점부터 제동하며, 회전 지점에 도달할 때만 실제 조향·횡가속·회전속도에 맞는 목표속도를 적용합니다.
- 1·3번 경로에서 2번 중앙 경로로 합류하는 규칙과 조향각 ±30°, 최대 yaw 약 68.75°/s, 횡가속도 2.5m/s²는 그대로 적용합니다. 급격한 코너에 실제 도달했을 때는 여전히 5m/s보다 느릴 수 있습니다.
- `--scenario late-pass`: 원본 맵을 수정하지 않고 **5m/s 적색 연습차량을 같은 경로 45m 앞**에 배치한 별도 검증 모드입니다. 기존 `slow-ahead` 25m 및 후방 차량 시나리오는 보존합니다.
- 로그: `[TEAM11 AVOID] pass`의 `front_gap_m`, `collision_ttc_s`; `[TEAM11]`의 `speed_profile_cap`, `corner_approach_cap`으로 추월 거리·속도판정을 직접 확인할 수 있습니다.

### 추월 3초 실제 시험 실행

```bash
cd ~/urrc_2_f1
git fetch origin
git switch team11-corner-speed-late-pass-v1
git pull --ff-only origin team11-corner-speed-late-pass-v1
python3 src/team_11/three_lane/run_demo.py monza \
  --source ~/urrc_2 --grid 1 --practice-cars \
  --scenario late-pass --start-mode camera
```

코너 속도 보정만 확인하려면 `--practice-cars --scenario late-pass`를 생략하고 실행합니다. 생성된 연습 월드와 원본 트랙은 분리됩니다.
