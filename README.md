# TEAM 11 — 공식 6개 트랙 좌표 직접 재생

**실행 명령어는 하나입니다.**

```bash
cd ~/urrc_2_f1 && git pull --ff-only origin main && bash run.sh monza 1
```

최초 다운로드 후에는 `bash run.sh monza 1`만 실행합니다.

## 인자

- 트랙: `monza`, `spa`, `monaco`, `cota`, `hungaroring`, `baku`
- 경로: `1` 오른쪽, `2` 중앙, `3` 왼쪽
- 선택적 속도: 세 번째 인자 m/s. 기본값 **10 m/s**. 예: `bash run.sh monza 2 15`

## 실제 동작

1. `~/urrc_f1_ws`에서 공식 6개 트랙 메시·월드·중앙선·5등 신호등을 읽기 전용으로 복사하고 검증합니다.
2. 트랙별 오른쪽/중앙/왼쪽 3개 경로를 만들어 `routes/<track>/lane_1.csv` 등 **18개 CSV**를 저장합니다.
3. `urrc_2/race/pace_cars/plugin/PaceCars.cc`와 같은 **Gazebo 시스템 플러그인**을 빌드합니다.
4. 항상 해당 트랙 **1번 그리드 X·Y 정중앙 및 Yaw**에 기존 노란 차량을 배치합니다.
5. 원래의 `traffic_lights_v10.py`를 실행합니다. **5개 빨간등 순차 점등 → 한 번에 전체 소등**에 성공한 후에만 주행을 시작합니다.
6. **선택된 경로 하나**의 누적거리, X/Y/Z/Yaw를 보간해 `SetWorldPoseCmd`로 차량의 월드 위치를 직접 갱신합니다.
7. 1랩 경로 재생이 끝나면 마지막 위치에서 정지합니다. Gazebo 창은 관찰할 수 있도록 유지되며 `Ctrl+C`로 종료합니다.

출발 그리드에서 선택 경로까지의 최초 약 8m만 부드럽게 연결합니다. 주행 중 **차선 변경, 추월, 방어, Pure Pursuit, Ackermann 조향, 센서 인식은 없습니다.**

차체 원본은 `src/team_11/models/team11_yellow_car/model.sdf`에 그대로 보존합니다. 실제 재생 때만 `log/kinematic/` 아래에 **정적 위치 재생용 사본**을 생성하여 불필요한 물리 구동과 충돌하지 않게 합니다.

## 검증

플러그인 구동 및 신호등 로그는 `log/kinematic/<track>_lane<번호>/<실행시각>/`에 남습니다. `[FIXED ROUTE] PASS_LAP` 출력은 플러그인이 **사전 경로 1랩의 위치 명령 재생을 완료했다는 의미**이며, 센서 기반 주행 성공이나 물리 차량 조향 성공을 뜻하지 않습니다.

현재 GitHub에 코드는 반영되어 있으나 Ubuntu/Gazebo 실환경 실행 및 표시 위치는 아직 확인되지 않았습니다.
