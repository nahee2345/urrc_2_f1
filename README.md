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

## 추월·방어·S자 주행·충돌 통합 시뮬레이션

한 개 명령어로 Monza 테스트를 실행합니다.

```bash
cd ~/urrc_2_f1 && git pull --ff-only origin main && bash run.sh battle monza
```

- TEAM 11: **1번 그리드**, 목표속도 10m/s, 전방 **TTC 3초 이하**일 때 안전 경로를 검토해 추월
- 전방 붉은 연습차량: TEAM 11보다 12m 앞, 기본속도 **9m/s**, **2초마다 1→2→3→2→1번 경로**로 S자 주행
- 후방 파란 연습차량: **10번 그리드**, 기본속도 **11m/s**, TEAM 11에 접근하며 방어 조건 검증
- 연습차량은 **진행 경로에 앞차가 있으면 감속**, 앞차가 사라지면 목표속도로 복귀
- TEAM 11 후방 TTC가 **5초 이하로 연속 감소**하는 조건에서 방어 경로 검토
- 좌표 직접 이동 주행이지만 **횡이동 중 차량 방향(Yaw)은 실제 대각선 이동 궤적에 정렬**되므로 옆으로 평행이동하지 않음
- 차선 변경은 2.5초 **5차 다항 보간**, 코너에서 추월·방어 금지 및 3초 미래 충돌 예측
- **0.53×0.25m** 차량 충돌 외형을 회전 사각형으로 계산하고 프레임 사이 이동도 분할 검사
- 연습차량들이 쓰는 속도는 목표속도이며 장애물 감속 시 실제 속도가 낮아질 수 있음
- 공식 원본 5등 신호등의 **전체 소등 성공 신호** 이후에만 출발. 카메라 인식 없음
- 벽 높이 **0.30m**, 차량 바퀴 지면 접촉, 원본 Gazebo 월드·차량 SDF 보존

다른 맵은 `bash run.sh battle spa`처럼 첫 번째 하위 인자만 변경합니다.

검증 결과: `log/battle/<track>/<시각>/gazebo.log`의 `[BATTLE] RESULT` 및 `prepared/battle_state.csv`에서 추월 여부, 방어 시도, 연습차량 감속, 충돌을 확인합니다.

**주의:** 이는 저장 좌표를 직접 이동하는 가상 테스트입니다. 실제 Ackermann 물리 주행 성능과 동일하지 않습니다. `PASS_BATTLE` 판정은 충돌 없는 1랩 재생 중 추월·방어 시도가 충족된 경우에만 출력됩니다. Ubuntu/Gazebo 실환경의 최종 테스트는 아직 진행되지 않았습니다.

## 추월·방어 18개 자동 검증

```bash
cd ~/urrc_2_f1 && git pull --ff-only origin main && bash run.sh battle all
```

실행 순서는 1번 경로×6트랙 → 2번 경로×6트랙 → 3번 경로×6트랙입니다.

각 케이스에 TEAM 11(1번 그리드 10m/s), 앞차(9m/s, 2초마다 1↔2↔3번 S자 이동), 후방 추격차(10번 그리드 11m/s)를 동일하게 배치합니다. TEAM 11은 선택된 1·2·3번 경로로 부드럽게 진입한 후 추월과 방어를 시도합니다. 카메라 인식 없이 공식 신호등 소등 서비스로 출발합니다.

각 1랩 테스트의 충돌/추월/방어 성공·실패와 종료 코드를 `log/battle_batch/<실행시각>/summary.csv`에 기록합니다. 케이스별 자세한 실행 로그는 같은 폴더의 `<경로번호>_<트랙>.log`이며, 차량 좌표·속도·TTC CSV는 로그에 표시된 `log/battle/<track>_lane<경로>/<시각>/prepared/battle_state.csv`에서 확인합니다. 실패 시에도 다음 케이스를 진행합니다. 개별 실행: `bash run.sh battle monza 2`.

## 1차 시험: 맵별·출발 그리드별 개별 실행

부분 구현된 **추월·방어 판단, 후보 경로 선택, 차선 변경 중 안전 재평가, 5초 차량 충돌 예측**을 시험합니다. 카메라 인식과 실제 Ackermann 물리 제어를 사용하지 않는 직접 좌표 재생입니다.

```bash
cd ~/urrc_2_f1
git pull --ff-only origin main

# Monza의 1번 그리드 시험
bash run.sh battle monza 1

# Monza의 2번 그리드 시험
bash run.sh battle monza 2
```

다른 맵도 `monza` 대신 `spa`, `monaco`, `cota`, `hungaroring`, `baku`를 넣어 **맵별로 별도 실행**합니다. 총 **6맵 × 2그리드 = 12개 개별 시험**이며 자동 연속 실행하지 않습니다.

- `bash run.sh battle <map> 1`: TEAM 11을 **공식 1번 그리드**에 스폰하며 기본적으로 **3번(왼쪽)** 경로에 합류
- `bash run.sh battle <map> 2`: TEAM 11을 **공식 2번 그리드**에 스폰하며 기본적으로 **1번(오른쪽)** 경로에 합류
- 선택적으로 `bash run.sh battle <map> <grid> <lane>`에서 경로 1·2·3을 지정할 수 있습니다.
- 전방 연습차량 9m/s: TEAM 11보다 12m 전방, **2초마다 1→2→3→2→1번 S자 경로** 변경
- 후방 연습차량 11m/s: **공식 10번 그리드**에서 출발, 앞차 접근 시 감속
- TEAM 11: 기본 10m/s, 경로 이동 중 8.5m/s 상한(1.5m/s 감속), 전방 TTC 3초 이하에서 추월, 후방 TTC 5초 이하 감소 시 방어
- 미래 차량 충돌 예측: **5초, 0.1초 간격**, 경로별 최소 간격 평가, 차선 변경 중 위험하면 재계획
- 곡률 제한: 환산 조향각 30° 초과 후보 거절, 코너에서는 새 추월·방어 금지
- 로그: `log/battle/<map>_grid<1|2>_lane<lane>/<실행시각>/`; `gazebo.log`의 `[BATTLE]` 기록 및 `prepared/battle_state.csv`

**판정 참고:** 출력되는 PASS는 직접 좌표 이동 기반 연습 알고리즘의 결과일 뿐이며, 벽 접촉 방지·실제 물리 조향 검증과 동일하지 않습니다. 코드 반영 후 실제 Gazebo 환경에서의 실행 성공은 아직 확인되지 않았습니다.

## 1차 테스트 12회 연속 실행 (6개 맵 × 1·2번 그리드)

```bash
cd ~/urrc_2_f1 && git pull --ff-only origin main && bash run.sh battle stage1
```

Monza(1·2번) → Spa(1·2번) → Monaco(1·2번) → COTA(1·2번) → Hungaroring(1·2번) → Baku(1·2번) 순서입니다. 각 케이스는 1랩이 끝나거나 충돌/실패 판정을 받으면 종료하며 다음 케이스를 실행합니다. 1번 그리드는 기본 3번 경로(왼쪽), 2번 그리드는 기본 1번 경로(오른쪽)입니다. 기존 개별 실행 및 18회 경로 테스트 명령은 그대로 유지합니다. 요약: `log/battle_stage1/<실행시각>/summary.csv`, 상세: 같은 폴더 내 `<맵>_grid<번호>.log`. 케이스별 실행 제한시간은 기본 480초이고 실패 시에도 다음 케이스를 진행합니다.

## 1차 테스트 실패 원인 대응 (2026-11-13)

12회 결과(기존): 추월 완료 10, 방어 실행 0, COTA grid1 충돌 1. 코드상 충돌 발생과 '방어가 시험에서 발생하지 않음'을 분리합니다.

- 경로 변경은 완료하거나 실제 미래 차체 충돌 예측이 확인되기 전까지 **현재 목표 경로를 유지**합니다. 미래 중심 거리 점수가 0보다 작다는 이유만으로 반복 재계획하지 않도록 변경
- 목표 경로 변경 후 2초간 새 반대 방향 변경 금지, 위험 재계획은 최소 1초 간격
- 현재 경로에서도 5초 미래 연습차량 S자 주행을 예측하여 **0.1초마다 TEAM 11 속도 상한을 재산정**; 예상 충돌 시 선제 감속
- 1·2·3번 후보 비교와 ±30도 환산 조향 제약 유지, 경로 변경 시 기본 10 → 최대 8.5m/s
- 방어는 **실제 후방 TTC 5초 이하 접근 기회**가 있었는지를 기록하고 `NOT_TRIGGERED`, `SAFE_ABORT`, `POSITIONED`, `HELD_SAFE_LANE`을 분리
- 결과는 **PASS**(추월+방어 완료, 무충돌 1랩), **PARTIAL_DEFENSE_UNVERIFIED**(추월+무충돌 1랩이나 방어 미검증), **FAIL**(충돌, 추월 미완료, 시간초과 등)로 분리해 `summary.csv`에 기록
- 1차 12회 재검증 명령: `cd ~/urrc_2_f1 && git pull --ff-only origin main && bash run.sh battle stage1`
- 원래 6개 맵×3개 경로 고정 주행 검증 `bash run_all.sh`과 1차 주행 테스트는 별개
- **Gazebo 실제 실행·컴파일은 로컬 사용자 환경에서 재검증이 필요합니다.** 이 변경은 공식 벽 충돌 검사나 실제 Ackermann 조향 제어를 구현하지 않습니다.

## 1차 경주 주행감 및 직선 추월 개선 (2026-11-13)

사용자가 제공한 12회 재시험 요약: PASS 0 / PARTIAL 8 / FAIL 4 (COTA grid1·2, Baku grid1·2). PARTIAL은 추월 완료·무충돌 1랩이지만 방어 검증 부족을 뜻합니다.

- TEAM 11 빈 경로 목표속도 **12m/s**. 앞 차량이 *옆* 경로에 있고 경로가 겹치지 않으면 **경로를 바꾸지 않고 가속해 통과**하는 것도 추월로 인식
- 전방 차량과 동일한 실효 경로에 있는 경우에만 거리 기반 추종 감속. 다른 경로에 있는 9m/s 차량에 일률적으로 속도를 맞추지 않음
- 충돌 예측: 경로 선택은 5초 예측을 유지하되, 실시간 속도 제한은 **가까운 교차 경로의 1.8초 예측** 중심으로 적용하여 먼 미래 위험 때문에 지속적으로 제동하지 않음
- 차선 변경 중에는 12→10.5m/s(1.5m/s 감속). 변경 완료 후 점진적으로 목표 속도 복귀
- Gazebo 실제 `simTime` 변화 반영(기존 최대 0.05초 잘림 완화), 가감속 제한 3.0/5.0m/s², 위치 사다리꼴 적분, 진행 방향 노이즈 완화
- 추월 기록 `OVERTAKE COMPLETED mode=LANE_CHANGE` 또는 `mode=CLEAR_LANE_ACCELERATION`으로 구분
- 기존 12회 1차 테스트: `cd ~/urrc_2_f1 && git pull --ff-only origin main && bash run.sh battle stage1`

**검증 제한:** 수정 코드의 ROS2 Jazzy/Gazebo Harmonic 현장 실행은 미검증입니다. 현재는 저장 좌표 직접 이동으로 실제 Ackermann 차체 동역학이나 벽 충돌까지 검증하지는 않습니다.

## 차체 간격·속도 정책 및 방어 전용 재검증 (2026-11-15)

1. 직선 구간: TEAM11과 다른 차량 사이 **차체 외곽 기준 최소 0.50m**. 코너 구간: **최소 0.70m**. 코드의 `clearance`는 회전된 0.53m × 0.25m 차체 사각형의 SAT 축별 간격 최댓값을 사용하여 **보수적으로 판정**하는 하한값입니다. 최소 간격을 위반하면 `FAIL_CLEARANCE`로 기록합니다.
2. 앞차 추종: 서로 겹치는 경로의 앞차에만 적용합니다. 직선에서 충돌까지 **1초 초과**이면 급제동 대신 부드러운 속도 조절·추월 후보 경로 평가를 우선합니다. 급제동은 1초 이내 미래 위험·거리 부족 시에만 활성화하도록 조정했습니다.
3. 코너: 근접한 동일 경로 앞차 속도에 부드럽게 맞추고, 간격 요구치를 0.7m로 변경합니다. 코너 진입 자체의 속도 상한 및 실제 차량 조향 동역학은 후속 구현 대상입니다.
4. 기존 6맵×2그리드 12회 일반 시험:
```bash
cd ~/urrc_2_f1 && git pull --ff-only origin main && bash run.sh battle stage1
```
5. **별도 후방 방어 12회 시험**: 6맵×그리드1·2에 대해 추격 차량을 TEAM11보다 약 7m 뒤, 동일 주행 경로, 14m/s 목표속도로 스폰하여 후방 TTC 5초 방어 판단의 발생 여부를 별도로 검증합니다.
```bash
cd ~/urrc_2_f1 && git pull --ff-only origin main && bash run.sh battle defense
```
로그는 `log/battle_defense/<실행시각>/summary.csv`에 저장됩니다. 개별 실행은 `bash run_battle.sh monza 1 3 defense` 형태입니다. 방어 전용 시험은 연습용 상황 재현이며 공식 10번 그리드에서 출발하는 일반 시험을 변경하지 않습니다.

**검증 제한:** 현재 GitHub 코드 변경 내용은 점검했지만 Ubuntu 24.04 / ROS2 Jazzy / Gazebo Harmonic 환경에서의 빌드 및 12회 실주행 PASS는 아직 확인되지 않았습니다. **차체 간격 0.5/0.7m 요구는 좁은 인접 경로에서 동시에 주행하기 어렵거나 불가능할 수 있으므로, 우선 경로를 보수적으로 선택하며 FAIL_CLEARANCE가 발생하면 실제 경계·차량 이동 로그를 통해 원인을 분석해야 합니다.**

## 2026-11-15 1차/방어 로그 기반 수정

- 오류: 0.499949m처럼 직선 기준 0.5m에서 수십 마이크로미터만 부족해도 즉시 실패 처리하던 문제. **허용 최소 직선 0.50m·코너 0.70m는 유지**, 실제 속도 제어 목표를 각각 **0.60m·0.80m**로 조정. 판정 시 계산 허용오차 0.0001m를 두어 부동소수점 경계 오차만 구분
- 후방 추격 판단: 후방 실제 거리 25m 이내일 때만 TTC 계산, **차체 길이**를 차감하여 실제 접근 시간을 평가, 5초 이내 접근 상황 기록. 후방 대처를 전방 추월보다 우선 검토하되, 측면의 차를 급히 가로막지 않고 안전 경로 유지 또는 가속을 우선
- 후방 차량 위험을 이유로 **TEAM11이 오히려 급제동하던 문제** 개선: 긴급 감속 예측에서는 전방 차량만 평가. 후방 TTC 2초 이내이고 전방이 안전하면 속도 유지/가속
- 충돌 발생 시 `COLLISION pair=<차량1>/<차량2> positions=...` 출력. 차체 간격 위반은 `CLEARANCE_BREACH pair=... actual=... required=...`로 구별하여 원인 추적
- 방어 시험의 후방 차량은 GRID 1/2 **TEAM11 실제 출발 횡좌표를 기준으로** 약 7m 뒤에 스폰(14m/s)
- Baku 등 느린 GUI 환경을 고려해 실행 제한 시간은 Gazebo 실시간 기준 최대 600초, 개별 배치 케이스는 720초로 연장. 시뮬레이션 내부 완주 판정 기준은 변경하지 않음
- 주행: `cd ~/urrc_2_f1 && git pull --ff-only origin main && bash run.sh battle stage1`
- 방어: `cd ~/urrc_2_f1 && git pull --ff-only origin main && bash run.sh battle defense`
- **실제 ROS2 Jazzy / Gazebo Harmonic 컴파일 및 시뮬레이션 완주 검증은 사용자 환경에서 수행해야 합니다.**

## 2026-11-15 S자 차량 예측·방어 성공 검증 수정

직전 24회 상세 로그 확인: 일반 주행 Monaco/COTA 네 번의 `FAIL_CLEARANCE`는 모두 `team_11/practice_s_09`이며, 0.4998m 전후에서 발생. 방어 시험 Baku/COTA/Hungaroring 그리드 1 세 번의 충돌은 **연습차량 S와 연습 후방차** 사이에서 발생하여 TEAM11 직접 충돌과 구별이 필요했음.

- TEAM11 속도 제어에 **2.5초 S자 횡방향 경로 예측을 반영**해 위험 경로 접근을 약하게 사전 감속. 실제 TTC 1초 이내 강한 제동 원칙은 그대로 유지. 직선 최저 허용 0.5m, 코너 0.7m, 제어 목표 0.6m·0.8m 유지
- 예측 주행거리 `(미래 속도 × 전체 미래시간)` 과대평가를 상수 가속도 적분으로 변경하여 충돌 예측의 거리 편차 감소
- `FAIL_SCENARIO_COLLISION`: practice_s_09 ↔ practice_rear_11 간 충돌은 팀 차량 직접 충돌(`FAIL_COLLISION`)과 **구별하여 실패 처리**. 연습 차량 충돌 시 해당 시험을 유효한 팀 방어 성공으로 처리하지 않음
- 방어 단계: `REAR_PRESSURE`(감지) → `HOLD_START`(안전 경로 유지 시작) → `DEFENSE VERIFIED`(2초 이상 유지 및 추격차 후방 잔류) / `OVERTAKEN`(추격차 통과). 단순 순간 안전 유지 선언은 이제 방어 성공으로 인정하지 않음
- 12회 주행: `cd ~/urrc_2_f1 && git pull --ff-only origin main && bash run.sh battle stage1`
- 12회 후방방어: `cd ~/urrc_2_f1 && git pull --ff-only origin main && bash run.sh battle defense`

한계: 전방 S자 차량은 시나리오에서 지정한 주기적 움직임을 미리 알고 예측하는 **리플레이 검증**이며 실차 센서 인식/조향의 검증은 아님. 실제 ROS2 Jazzy + Gazebo Harmonic 빌드 및 24회 주행은 로컬 환경에서 미검증.

## 2026-11-15 15:15/15:42 최신 24회 로그 대응

- 일반 주행 PASS 4, PARTIAL 6, FAIL 2(Monaco 1·2). Monaco 1은 코너 구간에서 TEAM11↔rear 간격 0.5936m로 0.7m 위반, Monaco 2는 추월 미완료·후방 차량 추월 허용
- 방어 주행 PASS 0, PARTIAL 6, FAIL 6. Baku/COTA/Hungaroring grid1의 3건은 **연습 S ↔ 연습 후방 차량** 충돌, Monaco 양 그리드는 TEAM11↔후방 차량 코너 간격 위반, Monza grid2는 TEAM11↔S 차량 0.699884m(코너 0.7m) 위반
- 수정: 연습 후방 차량이 앞 TEAM11 또는 S차량에 접근할 때 추종 차량 역할에 맞게 미리 감속하도록 설정. **14m/s는 전방이 비어 있을 때의 목표속도**이며 충돌 위험에는 감속합니다. 스폰 위치와 차량 충돌은 유지
- 수정: 방어 성공 기준은 2초간 뒤에 있었다는 중간 기록이 아닌 **최종 완주 시 후방 차량이 추월하지 않았으며 실제 접근 상황이 존재한 경우**입니다. `DEFENSE HOLD_MONITOR`는 중간 상태, `DEFENSE VERIFIED_FINISH`만 최종 성공
- 수정: S자 차량이 실제로 다른 차선에 있으며 앞 방향으로 충분히 떨어져 있고 단기 예측이 안전하면 불필요하게 앞차 속도에 맞추지 않고 직진 가속
- 직선 차체 최소 간격 0.5m, 코너 0.7m 유지. 팀 11 또는 상대 연습차량 충돌의 실패 분류는 유지
- 개발은 **kinematic 리플레이 시뮬레이터** 전용입니다. ROS 카메라/실제 차량 조향 및 센서 처리 로직을 변경한 것이 아닙니다
- 단독 검증: `bash run.sh battle monaco 1 3` / `bash run.sh battle monaco 2 1`
- 전체: `bash run.sh battle stage1` / `bash run.sh battle defense`

**실제 Jazzy/Gazebo 빌드 및 재시험은 아직 미수행**.

## 2026-11-15 Monaco 16:37/16:38 재현 로그와 수정
- Monaco 일반 grid1: `team_11/practice_rear_11` 코너 간격 **0.699822m** (기준 0.7m)에서 FAIL_CLEARANCE, grid2: **0.647175m**에서 FAIL_CLEARANCE. 두 경우 모두 추격 연습 차량이 TEAM11 코너 추종속도(약 9m/s)를 늦게 따라가면서 간격 축소
- 후방 연습 차량은 기존 2~3m 근처에서 급히 감속하던 패턴을 폐기. 전방 자동차의 속도 및 양쪽 코너 구간 여부, 상대 접근 속도와 제동거리에 따라 **4m 이상 선제 감속**. 코너 시 제어 목표 **0.8m**로 유지, 최소 허용 0.7m를 변경하지 않음
- `PRACTICE_YIELD`에 `corner` 표기. 앞차 속도 추종은 가감속 제한 하에서 실행. 이는 연습 상대 차량의 운전 품질 수정이며 TEAM11 자체 ROS/카메라 기반 센서 주행은 아님
- 일반 주행 이전 12건 PASS3/PARTIAL6/FAIL3, 방어 12건 PASS10/FAIL2. 이 수정 후 재시험 결과는 아직 없음
- 재확인: `bash run.sh battle monaco 1 3` 와 `bash run.sh battle monaco 2 1`, 해당 사례가 개선되면 `bash run.sh battle stage1` 및 `bash run.sh battle defense`

## 2026-11-15 19:03 Monaco 재시험 대응

- `17735ee` 재시험: Monaco 일반 grid1 0.699720m / grid2 0.663981m, 모두 팀11-후방 연습 차량의 코너 0.7m 기준 위반. 전체 stage1 PASS 3, PARTIAL 6, FAIL 3; defense PASS 10, FAIL 2. 기존과 동일한 실패 맵
- 원인: practice rear 운전자의 `PRACTICE_YIELD`가 주로 `corner=0`으로 기록됨. 코너에 진입한 후에야 간격 증가 필요성을 평가하며 0.7m 목표에 비해 약하게 감속함
- 수정: 연습 후방 차량의 코너 진입 감지를 현재 위치만이 아니라 **2초 미래 이동거리**에서 미리 검사. 목표 차체 간격은 평지 0.60+0.35=0.95m, 코너 0.80+0.35=1.15m로 설정. 차간거리 6m 이상에서 닫힘 속도에 따라 조기 추종 감속을 시작; 목표가 너무 가까우면 전방 차보다 최대 0.8m/s 느리게 주행하여 여유 복원. 연습 후방 차량의 실제 감속 한계 -5m/s² 유지
- **평가 최소 기준은 직선 0.5m / 코너 0.7m로 변경하지 않음.** TEAM11 차량 제어 로직 및 완주/추월/방어 성공 판정도 이번 커밋에서는 바꾸지 않음
- 확인: `cd ~/urrc_2_f1 && git pull --ff-only origin main && bash run.sh battle monaco 1 3 && bash run.sh battle monaco 2 1`
- 미검증: 실제 Gazebo 구동 결과. 통과하면 `bash run.sh battle stage1` 및 `bash run.sh battle defense`로 나머지 맵 회귀 검증

## 2026-11-15 20:15 Monaco geometric-clearance diagnosis

- Monaco grid1: `FAIL_NO_OVERTAKE`, 무사고 완주 및 방어 기록, 최종 최소 차체 간격 **1.01993m**. 문제는 간격이 아니라 S자 차량을 추월하지 못한 데 있음
- Monaco grid2: `FAIL_CLEARANCE`, `team_11/practice_rear_11` 간격 **0.672013m**(코너 0.7m 요구)
- **기존 `clearance()`는 SAT 축의 최대 분리 간격을 실제 차체간 최단거리처럼 사용함.** 두 회전 직사각형의 대각선 분리 시 실제 거리보다 작게 측정하는 보수적 하한값으로서 실측 0.7m 위반 판단을 과대평가할 수 있음
- 변경: 차체 사각형의 4개 모서리와 반대 차체의 4개 변 간 점-선분 거리(양방향 총 32개)를 계산하고, 겹침은 SAT로 구별하여 0m 처리. **실제 유클리드 차체 경계 거리**에 직선 0.5m, 코너 0.7m 규정을 그대로 적용
- 변경: 추월 실패 진단을 위해 5초마다 `PASS_DIAGNOSTIC`(팀속도, 선행차속도, 차량 사이 전방 거리·횡방향 거리, 곡률, 안전속도 제한, rear TTC, 최초 차선합류 미완료 여부) 기록. 추월 성공 조건은 완화하지 않음
- **검증 필요:** `cd ~/urrc_2_f1 && git pull --ff-only origin main && bash run.sh battle monaco 1 3 && bash run.sh battle monaco 2 1`. 두 실행 후 PASS 여부와 `PASS_DIAGNOSTIC` 기록을 비교. 기존 전체 방어 10/12 PASS도 반드시 회귀시험
- Gazebo 실주행 결과와 실제 컴파일은 아직 미검증

## 2026-11-15 Monaco 22:52/22:55 재시험: 9m/s 동일속도 고착 대응
- 두 그리드 모두 `FAIL_NO_OVERTAKE`, `attempts=2`, `defense=1`, `min_body_clearance >1.28m`. 실제 추월을 시작하는 후보 선택은 발생했으나 TEAM11 실제 속도와 S 차량 속도가 끝까지 **9m/s**로 같아 전방 거리 약 2.47m에서 고착
- 원인: 기존 `passingSpeedWindow`는 장기 2초 예측과 `cruise(team)`의 앞차 추종 9m/s 상한에 가로막힘. 앞차를 통과하기 위한 속도 증가 경로를 실행하지 못함
- 수정: 현재 실제 차체 간격이 최소 0.5m보다 **0.25m** 이상 여유 있고 코너/차선 변경 중이 아니며 **1초 구간 0.58m 이상**의 몸체 간격 예측을 통과하면 9.5~10.5m/s 안전 가속을 허용, 매 시뮬레이션 스텝 재확인. 기존 1초 충돌 예측 제동, 직선 0.5m / 코너 0.7m 실패 기준 및 추월 조건은 유지
- `PASS_DIAGNOSTIC`에 `pass_window`와 `speed_target`을 출력하도록 추가. 안전한 가속 창이 없으면 속도를 강제로 올리지 않음
- 테스트: `cd ~/urrc_2_f1 && git pull --ff-only origin main && bash run.sh battle monaco 1 3 && bash run.sh battle monaco 2 1`. **Gazebo 실행 결과는 아직 검증되지 않음**

## 2026-11-16 Monaco 단계별 추월속도 유지 수정
- 2026-11-15 23:00/23:02 Monaco grid1/2: 둘 다 `FAIL_NO_OVERTAKE`, 방어 성공, 최소 차체간격 grid1 0.614299m / grid2 0.799432m. 추월 시도 각각 2/1회. `PASS_SPEED_WINDOW`이 9.5m/s에서 단발로 등장했지만 결국 9m/s로 다시 고착
- 수정: 속도 차이가 0.4m/s 미만일 때에만 가속 판단을 실행하던 구간을 없애고 `team.v < 11m/s`, 앞차 거리 7m 미만 및 단기 예측 간격 통과 조건으로 유지. 가속 후보 11, 10.5, 10, 9.5m/s를 매 스텝 검토
- `SPEED_DECISION`에는 최종 목표속도, 근접 추종속도, 편안한 선제 속도, 1초 위험 감속 상한, 추월 가속 허용 창, 차선변경 여부, 앞차 거리를 표시. 안전 예측에 걸리면 가속하지 않는 기존 제한 유지
- 컴파일 및 Gazebo 주행은 아직 재검증되지 않음. 우선 `bash run.sh battle monaco 1 3`와 `bash run.sh battle monaco 2 1`, 통과 후 다른 맵 및 방어 전체 회귀 테스트

## 2026-11-16 직선·완만한 코너 추월 통합 및 급격한 코너 사전 감속

- 범위: `kinematic/BattleReplay.cc`의 **Gazebo 위치 직접 지정 방식 3차량 배틀 테스트**. 실제 ROS 2 센서·카메라/라이다·실제 조향 제어는 이 변경으로 수정되지 않습니다
- 곡률(절댓값) 단계: 직선 < 0.08 1/m, 완만한 코너 0.08~0.18 1/m, 급격한 코너 >= 0.18 1/m. 경계값은 시뮬레이션 검증을 위한 초기 설정이며 Gazebo 실제 시험 후 조정해야 합니다
- 코너 접근 판정은 현재 위치에서만 하지 않고 **최소 8m 또는 현재 속도에서 계산한 감속거리+4m**까지 8분할로 미리 곡률 검사. 직선 최대 12m/s, 완만한 코너 최대 10m/s, 급격한 코너 7m/s로 목표속도 제한. 감속은 기존 한계 -5m/s² 적용
- 직선 및 완만한 코너에서 앞차 TTC <=3초 또는 앞차 거리 <5m일 때 3개 경로×여러 속도 후보를 향후 3초간 비교. 완만한 코너에서 계획 간격 0.7m, 직선 0.5m 이상 확보하고, 후방 차량의 미래 위치·조향 30° 제한을 통과한 후보만 채택
- 차선 변경을 선택하면 `passingActive`와 `plannedPassSpeed`로 추월 목표를 기억하고, 실제 변경 중인 보간 경로의 미래 차체 간격을 매 스텝 확인하여 가능한 경우 앞차 추종 9m/s 상한보다 높은 목표속도를 허용. 급격한 코너 접근 시 신규 추월을 중지하고 가속 계획을 보류. 안전 경로가 없으면 무리하게 차선변경하지 않음
- 로그: `CORNER_SPEED`, `OVERTAKE ATTEMPT`, `OVERTAKE PAUSE`, `PASS_SPEED_WINDOW`, `OVERTAKE COMPLETED`, `SPEED_DECISION` 참고
- 차체 간격 실패 기준 직선 0.5m / 급격한 코너 0.7m, 판정 로직과 전체 배틀 결과 규칙 유지
- 로컬 Gazebo 컴파일·실주행은 아직 검증되지 않음. 우선 `bash run.sh battle monaco 1 3`, `bash run.sh battle monaco 2 1` 후 `bash run.sh battle stage1` 및 `bash run.sh battle defense`로 회귀 검증 필요

## 2026-11-16 09:17~09:19 Monaco 회귀 원인 및 코너 감속 보정
- 최신 실제 실행: 일반 Monaco grid1/grid2 모두 `FAIL_NO_OVERTAKE`, 후방 방어 완료, 최소 간격 1.29486m / 1.3348m, 추월 시도 각 3회. 70초 지점 선행 S차량과 종방향 거리 33.42m / 33.71m까지 벌어졌으며 TEAM11의 12m/s 가속이 뒤늦게 발생
- 원인: 이전 로직은 현재 위치에서 최대 18m 이상 선행한 구간에 급커브가 한 지점만 있어도 **즉시 최고속도 7m/s**로 제한함. 리플레이 S차량은 9m/s를 유지하므로 코너에서 앞차와 거리가 크게 벌어짐. 또한 곡률 임계점에서 `MILD/STRAIGHT` 반복 출력
- 개선: 현재 차량에서 감속 가능한 첫 코너까지 **0.5m 간격**으로 샘플하고, `v_allowed = sqrt(v_corner²+2*5*max(0, distance-buffer))`로 현재 최고속도를 계산. 실제 급코너 위치에서는 7m/s, 완만한 코너에서는 10m/s, 직선 최대 12m/s 기준 유지. 코너 이전에 감속을 시작하되 먼 구간에서부터 무조건 7m/s 주행하는 현상 제거
- 곡률 분류가 낮아지는 방향 전환은 0.5초 유지로 로그 토글 억제. `CORNER_SPEED`에 급코너까지의 거리 출력
- 주의: 후행차의 코너 감속과 선행 S 차량의 9m/s 일정속도 때문에 무조건 추월 성공을 보장하지 않음. 선행 S차량의 속도/경로를 조정하여 인위적으로 PASS를 만들지는 않음
- 확인: `bash run.sh battle monaco 1 3` / `bash run.sh battle monaco 2 1` → 전체 `stage1` 및 `defense` 회귀 테스트. 실제 Gazebo 재시험은 아직 수행되지 않음

## 2026-11-16 09:31~09:33 Monaco 추월 재검증과 사전 차선 위치 선정
- 일반 Monaco grid1 및 grid2: 각각 `FAIL_NO_OVERTAKE`, `attempts=6`, 방어는 `HELD_TO_FINISH`. 차체 최소 간격 1.00544m / 1.02661m, 충돌·간격 위반 없음. 71초경 마지막 `OVERTAKE ATTEMPT` 후 곧 완주하여 추월 미완료. `HAZARD_REPLAN future=-1 stay=-1 back=-1` 발생해 당시 예측상 안전한 재계획 후보도 없었음
- 수정: 추월 검토를 전방 차량 5m 이내에 도달하기 전에 시작할 수 있도록 조건 확장. 직선·완만한 코너에서 앞차 거리 18m 미만이고 도달 가능한 곡률 제한 속도가 선행 S차량(9m/s)보다 0.5m/s 이상 높으며, 결승까지 25m 이상 남았을 때 차선 변경 후보를 미리 점검
- 기존처럼 3초 미래 시뮬레이션의 양쪽 상대차량 차체 간격/30도 조향 등 안전검사를 통과한 후보만 채택. 급격한 코너 접근 시 새 추월은 시작하지 않고, 직선 0.5m·코너 0.7m 기준 완화하지 않음. 안전한 경로 후보가 없으면 추월 시도 보류
- `OVERTAKE ATTEMPT` 로그에 `front_gap`, `lap_remaining` 추가하여 추월 시도 시점 확인. 이 패치는 선행 S차량의 움직임이나 배틀 성공판정을 변경하지 않음
- 검증: `bash run.sh battle monaco 1 3`, `bash run.sh battle monaco 2 1`; 두 결과 확인 후 `bash run.sh battle stage1`, `bash run.sh battle defense`. 실제 Gazebo 실행 결과는 아직 미확인

## 2026-11-16 09:37~09:39 Monaco 추월 11회 시도/0회 완료 대응
- Monaco grid1/grid2: `FAIL_NO_OVERTAKE`, `attempts=11`, `defense=1`; 최소 차체간격 1.00984m / 1.00904m
- 추월 시도는 초반(앞차 약 10m, 남은 124~126m)부터 발생하였지만 곧 급격한 코너를 만나 `OVERTAKE PAUSE`와 `HAZARD_REPLAN future=-1 stay=-1 back=-1` 발생. 급코너에선 TEAM11 7m/s, 선행 S 차량 9m/s라 간격이 다시 벌어짐
- **명확한 코드 충돌:** `plannedPassSpeed=11`로 안전 후보를 선택했더라도 `if(team.changing) vt=min(vt,10.5)`가 무조건 적용되어 2.5초 차선 변경 중 10.5m/s를 초과할 수 없었음
- 수정: 차선 변경 중에도 추월 목표속도로 양쪽 상대차량 포함 1.2초 차체간격을 재검증할 때만 해당 **10.5m/s 일반 차선 변경 제한 예외** 허용. 그 외 일반 차선 이동은 10.5m/s 제한 유지. 미래 위험 1초 긴급감속·곡률별 최고속도·차체 최소 간격 판정은 유지
- 앞차와 같거나 느린 속도의 차선 변경을 `OVERTAKE ATTEMPT`로 집계하지 않게 `trial > lead.v + 0.5m/s` 요구. `PASS_LANE_SPEED` 진단 로그 추가
- 코드 정적 중괄호/수정 위치 확인, **실제 컴파일 및 Gazebo 완주 여부 미검증**. `bash run.sh battle monaco 1 3` 및 `bash run.sh battle monaco 2 1`로 재검증 후 `stage1` / `defense` 회귀시험 필요

## 2026-11-16 13:00~13:09 Monaco 재시험: timeout 및 추월 실패 대응
- grid1: `FAIL: no battle result within 600 seconds`; 후속 `Exception sending a multicast message:Network is unreachable` 다수. Gazebo 시작/차량 스폰/출발 메시지는 확인됐으나, 최종 `[BATTLE] RESULT`가 없으므로 **추월 실패로 분류하지 않고 환경/진행 중단으로 구분**. 멀티캐스트 오류만으로 네트워크 원인 또는 코드 원인을 확정하지 않음
- grid2: `FAIL_NO_OVERTAKE`, `attempts=10`, `defense=1`, `min_body_clearance=0.803867m`. `PASS_LANE_SPEED`는 실행됐으나 선행차를 통과하기 전 레이스 종료. `HAZARD_REPLAN future=-0.111646` 등 재계획 위험 판정 반복
- 진짜 결함: `forecast()`가 차체 간 실제 최소거리 `clearance()`가 기준을 충족하더라도, **차체 중심거리에서 차량 길이와 직선 간격을 뺀 값**이 음수면 안전 후보를 음수로 반환했음. 차선 옆으로 지나가는 경우 안전한 위치가 잘못 거부될 수 있음. 미래 0.1초 샘플의 차체 최단거리−해당 구간 법정 최소거리로 점수 및 안전 여부를 통일. 실제 충돌·차체 거리 판정 기준 변경 없음
- 속도 선택: 안전 검사를 통과한 후보들 중에는 필요 이상 여유거리보다 앞차 대비 높은 추월 속도를 우선하도록 후보 점수 보정. 제한속도·최대조향±30도·곡률 감속·미래 전후방 상대 위치 확인 유지
- `run_battle.sh` 600초 시간초과 시 반복되는 Gazebo 멀티캐스트 오류를 제외한 마지막 로그, `battle_state.csv` 마지막 3행 및 Gazebo PID 상태를 표시하도록 진단 강화. 타임아웃 값을 단순히 증가시키지 않음
- 실제 Gazebo 재시험은 아직 확인되지 않음: `bash run.sh battle monaco 1 3` 및 `bash run.sh battle monaco 2 1`. 반복되는 `Network is unreachable`는 호스트 네트워크 설정도 별도 확인 필요

## 2026-11-16 TEAM11 배틀 속도·차간거리 단순화 및 정적 검증

**적용 범위:** `kinematic/BattleReplay.cc` 3대 차량의 *운동학적 Gazebo pose replay*. 실제 Team11 ROS2 노드의 카메라/라이다 감지 및 휠 조향 제어는 변경하지 않음.

- TEAM11 목표 최고속도: 직선 **15m/s**, 완만한 코너 **11m/s**, 급격한 코너 **7m/s**. 곡률 분류 및 급코너 접근 시 감속거리 기반 속도 상한 유지. `run_battle.sh` 시작 로그의 최고속도도 15m/s로 수정
- 앞차 추종 시 **차체 사이 목표 간격 1.0m**(직선/코너), 기존 **안전 최소 간격 직선 0.50m / 급코너 0.70m** 규칙 변경하지 않음. 목표 간격은 제어값이지 보장되는 결과가 아님. 추월 중에는 센서가 아닌 스크립트의 3차원 차량 pose 예측을 사용하며 법정 최소 간격을 계속 확인
- 차선변경 속도: 일반 합류 **10.5m/s** 상한 유지, 안전한 추월 후보에 한하여 높은 목표속도 허용. 앞차/뒤차 미래 위치·±30도 조향 후보 필터, 실제 차체 충돌 및 스윕 검사 유지
- 선행 S차량 9m/s, 후방 차량 일반 11m/s / defense 14m/s, 신호등 일괄 소등 게이트, 스폰 위치, 추월/방어 성공 기준 변경하지 않음
- **변경 코드·실행 스크립트 정적 검사 19/19 PASS:** 속도값, 추종 목표, 최소 간격, 감속거리식, 급코너 추월 금지, 15m/s 추월 후보, 30도 예측 제한, 후방 미래 차량 포함, 충돌/간격 판정, 가감속 범위, 일반 합류 상한, 로그, 두 개 12회 배치의 6맵×2그리드, 신호 게이트, C++ 중괄호 균형. **정적 텍스트 검사이지 컴파일/실주행 검증이 아님**
- **필수 로컬 검증:** `bash run.sh battle monaco 1 3` 및 `bash run.sh battle monaco 2 1`; 그다음 `bash run.sh battle stage1`(일반 12개)과 `bash run.sh battle defense`(방어 12개). 총 24개 전체 케이스의 `PASS_BATTLE`, `FAIL_CLEARANCE`, `FAIL_COLLISION`, `FAIL_NO_OVERTAKE`, 600초 타임아웃 확인. `log/battle_stage1/*/summary.csv`, `log/battle_defense/*/summary.csv` 결과 제출 필요
- 현재 **GitHub 변경 및 19항목 정적 확인만 완료**. 사용자 Gazebo / ROS Jazzy에서 전체 24회 실주행 성공 여부는 미검증. 특히 최신 12m/s 사례에서 차체 최소거리 0.508m / 0.518m처럼 근접했던 점을 고려해 속도 상향 후 충돌 회귀검증이 필요

## 2026-11-16 배틀 FAIL 원인 세분화 및 기존 주행 로그 재진단

- 기존 `run_stage1.sh` 및 `run_defense.sh`의 `FAIL`은 종료코드만 표시하여 차량 충돌/간격 위반/추월 실패/방어 미달/실행 오류를 식별하지 못했음. 최근 로컬 시험 일반(stage1) 8/12 케이스까지 기록 후 사용자 중단, 방어(defense) 12개 중 PASS 1 / PARTIAL 8 / FAIL 3
- `kinematic/diagnose_battle.py` 추가: 각 `summary.csv`와 개별 `*.log`, 원본 `gazebo.log`에서 `[BATTLE] RESULT`, `COLLISION pair`, `CLEARANCE_BREACH pair/actual/required`, `defense_status`를 검색. `diagnosis.csv`에 실제 원인 및 관련 차량 저장. **과거 로그를 재주행하지 않고도 분석 가능**
- 원인 범주: `VEHICLE_COLLISION`(팀 차량 충돌), `PRACTICE_VEHICLE_COLLISION`(연습 차량끼리 충돌), `VEHICLE_CLEARANCE_BREACH`(접촉은 없지만 규정 간격 위반), `TASK_NO_OVERTAKE`(추월 미달), `TASK_DEFENSE_UNVERIFIED`(추월 성공/방어 미확인), `EXECUTION_TIMEOUT` 및 기타 실행 실패. `FAIL_NO_OVERTAKE`는 반드시 충돌을 뜻하는 것이 아님
- **벽 충돌은 `NOT_MEASURED`로 구분**. 현재 `BattleReplay.cc`는 차량 위치를 직접 설정하고 차량 간 차체 간격을 자체 계산하는 테스트입니다. 벽과의 실제 물리 접촉 이벤트를 수집하지 않으므로 기존 로그에서 `벽 충돌 없음`이라고 단정할 수 없습니다. 벽 충돌 증명에는 별도 벽 충돌 센서나 실제 충돌 이벤트 연동이 필요합니다
- 새 전체 시험은 한 케이스마다 `cause=<실패원인> replay=<원본판정> wall=NOT_MEASURED`를 출력하고, `summary.csv`에 원인 범주 컬럼을 추가하고 `diagnosis.csv`를 자동 작성합니다. 사용자가 중간에 Ctrl+C로 중단해도 현재까지 쌓인 `summary.csv`는 아래 명령으로 재진단 가능
- **기존 로그 즉시 분류:** `cd ~/urrc_2_f1 && git pull --ff-only origin main && python3 kinematic/diagnose_battle.py --all`
- 특정 실행: `python3 kinematic/diagnose_battle.py --summary log/battle_stage1/20261116_132922/summary.csv --summary log/battle_defense/20261116_135232/summary.csv`
- 분류기 회귀 단위시험: `python3 kinematic/test_diagnose_battle.py` (6개 케이스). GitHub 소스 정적 확인만 완료했으며 **사용자 환경의 파이썬 단위시험·Gazebo 실제 재실행 결과는 아직 확인되지 않았음**. 배틀 물리/속도 로직은 이 변경에서 수정하지 않음

## 2026-11-16 3대 차량, 6개 맵별 3바퀴 순위·속도·도로 경계 검증

이번 기능은 **kinematic/Gazebo SetWorldPoseCmd 배틀 리플레이 전용**이며 실제 ROS 2 센서/조향 제어 로직과 별개입니다. 기존 `bash run.sh battle ...`의 **1바퀴 시험은 유지**됩니다.

### 주행 설정
- 모든 맵: 3대 차량(team_11, practice_s_09, practice_rear_11), 지도당 3바퀴
- TEAM11: **15m/s 최고속도**, 평상시 **10m/s 목표**, 완만한 코너 **10m/s 목표**, 급격한 코너 **7.5m/s 목표**. 실제 평균속도는 `TEAM11 주행거리 / 경과 시간`으로 기록하여 목표 10m/s에 도달했는지 검증합니다. 충돌 회피가 필요하면 목표보다 느려질 수 있습니다
- 선행 S 연습차량 기준속도 9m/s, 후방 연습차량 일반 11m/s (기존 시나리오 동일)
- **벽 판정은 도로 폭과 차량(0.53×0.25m)의 회전 외형이 도로 경계를 넘는지 검사하는 기하학적 대리 지표**입니다. `WALL_BOUNDARY` 발견 시 즉시 `FAIL_WALL_BOUNDARY`, 그 밖에는 `wall_boundary_status=CLEAR_GEOMETRIC`으로 보고. **Gazebo 충돌 이벤트와 충돌 센서를 읽는 구조는 아니므로 실제 벽 메시 충돌과 같다고 단정하지 않습니다.**
- 참가 3대의 동일 결승선을 사용해 TEAM11이 3바퀴를 완료한 시점에 이미 결승선을 통과한 상대 수를 비교하여 순위 `1/3`, `2/3`, `3/3` 계산. 결승 전에 충돌/경계 침범/제한시간 종료가 있으면 `UNRANKED`로 구분
- **측정 출력:** `[BATTLE] LAP_COMPLETED` (랩), `[BATTLE] RACE3_RESULT` (순위/실제 최고속도/실제 평균/최소 도로 경계 여유/차체 최소 간격), `[BATTLE] RESULT` (성공·실패)
- 배틀 안전 검사: 3대 차체 간 충돌, 차체 간 최소 간격, 기하학적 도로 경계, 기존 30° 조향 후보 제한은 유지

### 실행
```bash
cd ~/urrc_2_f1
git pull --ff-only origin main
python3 kinematic/test_diagnose_battle.py
python3 kinematic/test_three_lap_report.py

# 6개 맵 × 그리드1 × 3바퀴 = 6회
bash run.sh race3

# 선택: 6개 맵 × 그리드 1·2 × 3바퀴 = 12회
bash run.sh race3 both

# 개별 Monaco 3바퀴
bash run.sh battle monaco 1 3 race3
```

결과: `log/race3/<YYYYMMDD_HHMMSS>/summary.csv`. 컬럼: `track,grid,lane,result,rank,team_peak_mps,team_average_mps,min_wall_margin_m,wall_boundary_status,min_body_clearance_m,failure_category,exit_code,wall_measurement,case_log`. 실패/중단 후에도 각 완료한 케이스까지 CSV 기록.

**검증 한계:** GitHub 코드 작성 및 정적 확인은 가능하지만, 이 대화 실행 환경에는 사용자 Ubuntu 24.04/ROS2 Jazzy/Gazebo Harmonic 실행 환경이 없어 실제 3바퀴 완주, 순위, 물리적 벽 접촉 여부는 사용자 실행 로그가 필요합니다. 평균속도 10m/s 달성 또한 보장값이 아니라 측정 대상입니다.

## 2026-11-16 race3 비접촉 차간거리 및 도로 경계 처리 변경

- **적용 범위는 `race3`(3바퀴 기하학적 리플레이) 한정**. 기존 1바퀴 `battle`/일반/방어 테스트의 최소 차간거리 FAIL 정책은 유지
- 요청 기준: 실제 **차체 겹침(기하학적 차량 충돌)**만 `FAIL_COLLISION` 또는 `FAIL_SCENARIO_COLLISION`. 차량 간 간격이 직선 0.5m, 코너 0.7m보다 작더라도 실제 차체가 겹치지 않으면 `CLEARANCE_WARNING`만 기록하며 완주를 허용. 3바퀴 `collide()`의 구형 25mm 가상 패딩을 제거하여 실제 회전 직사각형 겹침을 검사
- **도로 경계 보정:** 차량의 위치·방향 및 공식 도로폭 프로파일을 이용해 3대 모두 안전한 횡방향 범위를 계산. TEAM11의 예정 이동은 향후 3초간 최소 도로폭을 고려해 제한하고, 연습 S차량·후행차의 좌우 오프셋도 도로폭 내로 제한. 벽 가까이 가면 `ROAD_CORRECTION` 로그 출력
- 도로 경계 기하학적 침범은 실제 Gazebo 벽 접촉으로 확인된 것이 아님. `WALL_BOUNDARY_WARNING`으로 **경고·기록만** 하며 `FAIL_WALL_BOUNDARY`로 즉시 실격하지 않음. 최종 결과에 `wall_margin_min_m`과 `wall_warning_count` 기록. `wall_boundary_status=CROSSED`는 여전히 보고되어야 하며, 이는 **물리 접촉 여부가 아니라 계산상 도로 경계 침범**만 의미함
- 경로 후보의 미래 위치가 허용 도로폭 밖이면 선택하지 않도록 차단, 선행차 예측도 도로폭 보정값 사용. 최소 차간거리 경고 건수 `clearance_warning_count` 별도 기록
- 최종 `log/race3/<실행시각>/summary.csv`에는 `min_body_clearance_m`, `clearance_warning_count`, `wall_warning_count`, `wall_boundary_status`, 순위 및 평균/최고속도 포함
- **중요 제한:** 차량 차체 충돌은 경로 리플레이의 회전 직사각형 기하학적 충돌로 판정됩니다. Gazebo 벽 접촉 센서는 아직 구현되지 않았으므로 **실제 벽과 접촉하지 않았다는 것을 검증한 것은 아닙니다**. 벽 관련 결과는 기하학적 대리 지표일 뿐입니다
- 재검증:
```bash
cd ~/urrc_2_f1
git pull --ff-only origin main
python3 kinematic/test_three_lap_report.py
bash run.sh race3 both
```
- 이번 수정은 아직 사용자 ROS2 Jazzy/Gazebo Harmonic에서 컴파일·12회 재주행 확인 전입니다. 기존 12회 로그(8회 통과, 4회 FAIL)는 이전 코드 버전이므로 새 기준 결과가 아닙니다
