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
- 기존 차량 조향 관절 한도는 **±30°**이며, 직접 위치 재생 시에는 실제 바퀴 조향이 아닌 **휠베이스 0.36m 기준 환산 조향각 30° 이하인 경로만 허용**합니다.
- 1·3번 경로는 기존 벽 여유를 약 절반으로 줄입니다. 단, **최소 0.075m의 여유**와 경로 환산 조향각 제한을 우선하여 Monaco·COTA 등 급커브에서는 벽과 더 멀어질 수 있습니다.
- 구간별 조향 제약은 `routes/<track>/manifest.json`의 `max_route_equivalent_steering_deg` 및 `geometry_limit_reduced_offset`으로 확인할 수 있습니다.

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

## 1·2·3번 경로별 6개 맵 연속 시험

```bash
cd ~/urrc_2_f1 && git pull --ff-only origin main && bash run_all.sh
```

위 명령 하나로 다음 **18개 테스트를 순서대로** 실행합니다.

- 1번(오른쪽): Monza → Spa → Monaco → COTA → Hungaroring → Baku
- 2번(중앙): Monza → Spa → Monaco → COTA → Hungaroring → Baku
- 3번(왼쪽): Monza → Spa → Monaco → COTA → Hungaroring → Baku

각 테스트는 **1번 그리드**에서 출발하며, 공식 5등 신호 소등 후 선택한 고정 경로를 1랩 재생합니다. 1랩 완료를 확인하면 해당 Gazebo를 자동 종료하고 다음 트랙으로 넘어갑니다. 실패한 테스트도 로그에 남기고 다음 테스트를 진행합니다.

테스트 결과는 `log/batch/<실행시각>/summary.csv` 및 각 케이스 `.log`에 저장됩니다.

전체 실행 속도는 기본 10m/s이며 `bash run_all.sh 15`처럼 선택할 수 있습니다.

### 트랙 벽 높이와 차량 접지

- **공식 `urrc_f1_ws` 트랙 벽: 0.80m → 실행용 트랙 벽: 0.30m**, `urrc_2` 연습 트랙의 높이와 동일합니다.
- 내부·외부 벽 모두 **시각 메시 및 충돌 메시**를 함께 변경합니다. 노면·트랙 폭·그리드 위치는 변경하지 않습니다.
- **차량 바퀴는 노면에 닿도록 유지**합니다. TEAM11 `base_link`는 노면보다 0.090m 위로 배치하며, 차량 원본 SDF는 수정하지 않습니다.
- 벽 변경은 `log/kinematic/<track>_lane<번호>/<실행시각>/prepared/resources/urrc_track_gazebo/models/`의 **실행용 복사본에만** 적용합니다. 원본 `urrc_f1_ws` 및 복사해 둔 원본 트랙 파일은 변경하지 않습니다.
- 벽 형상은 기존 도로 높이를 기준으로 상단만 0.50m 낮춥니다. 충돌 메시까지 함께 변환합니다.

**주의:** PASS는 저장된 좌표 1랩 재생 완료를 뜻하며, Gazebo 실주행 및 벽과의 실제 접촉 검증이 끝났다는 뜻은 아닙니다.
