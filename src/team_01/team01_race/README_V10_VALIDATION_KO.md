# TEAM01 v10 검증 후보 — 실제 Monza 1랩 미인증

## 분석 근거

- v9는 출발 GRID 상태에서 먼 거리의 중앙/우측 연석 좌표를 목표로 하는 Pure Pursuit 조향각을 만들고, 그 뒤 넓은 후보 조향각 중 clearance가 높은 방향을 선택할 수 있었습니다. 전방 곡선이 출발 조향에 섞이는 원인입니다.
- 3도 조향 제한만으로는 3초 동안 차체가 벽 쪽으로 충분히 이동할 수 있습니다. 이미지 한 장만으로 실제 차량의 조향 방향 반전 여부를 확정할 수는 없습니다.

## v10 수정

- GRID (정상 직선, 출발 3초): 오른쪽 연석의 **동일 전방 위치(약 0.75m)** 에서 측정한 시작 간격과 현재 간격을 비교합니다. 오차 0.07m 이내면 0도 조향, 이상이면 최대 +/-0.8도 보정만 합니다. 멀리 보이는 코너가 출발 명령을 결정하지 않습니다. GRID에서 후보 선택의 clearance 편향과 adaptive_pursuit는 사용하지 않습니다.
- 가까운 연석 간격이 급격히 좁아지고 실제 전방 코너가 확인되면 3초 이전에도 GRID를 종료해 충돌을 피합니다. 무조건 직진 명령을 유지하며 벽에 충돌하지 않도록 하는 안전 예외입니다.
- 1랩 우회/추월 없이 RGB+Depth 기반 도로 중심을 탐색합니다. 1랩 목표 직진 7 m/s, 2랩 12 m/s, 20 m/s는 목표만 보존하며 활성화하지 않습니다. 실속도는 거리/제동 제한으로 낮아질 수 있습니다.
- 기존 4개 이상의 적색 신호 인식 -> 모두 소등 후 출발 유지합니다.
- 독립 **실제 Gazebo 1랩 검증기**를 `scripts/verify_monza_lap.py`로 추가했습니다. `scripts/run_local_monza.sh` 실행 시 기본적으로 자동 실행됩니다. QA 도구만 `src/urrc_track_gazebo/tracks/processed/monza_centerline.csv`를 읽고, 주행 제어기는 기존대로 사전 트랙 맵을 읽지 않습니다.

## 실행 (기존 버전 삭제 후 설치, 백업 없음)

아래 명령의 `rm -rf`는 `team01_race` 폴더만 삭제합니다. 이미 동작하는 Gazebo는 Ctrl+C로 종료하세요.

```bash
cd ~
rm -rf ~/urrc_2/src/team_01/team01_race
unzip -o ~/Downloads/team01_f1_start_direction_v10_candidate.zip -d ~/urrc_2/src/team_01
cd ~/urrc_2/src/team_01/team01_race
bash scripts/run_offline_tests.sh
bash scripts/run_local_monza.sh 2>&1 | tee ~/team01_v10.log
```

QA 결과: `~/.local/state/urrc_team01/qa/monza_lap_v10.json`

```bash
cat ~/.local/state/urrc_team01/qa/monza_lap_v10.json
```

- `status: "GEOMETRIC_LAP_PASS"`: Gazebo odometry가 기준선 한 바퀴 진행, 주행 거리, 경로 구간 관측, 출발 위치 재방문 조건을 만족했습니다. **실제 주행 물리 성공 판정은 사용자의 Gazebo 실행 후에만 가능합니다.** 접촉 센서 검사가 없어 충돌 없음까지 자동 보증하지는 않습니다.
- `status: "TRACKING"`: 주행 중이거나 완료 실패. 숫자를 보고 원인을 추적해야 합니다.
- `status: "OFF_TRACK"`: 원본 트랙 기준선의 주행 범위 이탈. `POSE_JUMP`, `STALE_ODOM`, `NO_ODOM`, `LAP_TIMEOUT` 등도 불합격입니다.
- 1랩 제어기 자체의 랩 판단은 아직 잠정적인 센서/데드레커닝 추정이며, 독립 Gazebo QA 판정과 별개입니다.

## 검증 상태

- Python 오프라인 테스트 97개 통과 (참고: 환경에 따라 테스트 수는 추가될 수 있습니다).
- 합성 경로에서 17m 코너 추종 테스트 통과.
- 독립 QA 모듈의 합성 원형 트랙 한 바퀴 판정 통과.
- **실제 Gazebo / Monza 한 바퀴: 이 산출물에서는 아직 검증되지 않았습니다.** 실제 완주 인증을 받은 v10 최종본이라고 주장할 수 없습니다.
