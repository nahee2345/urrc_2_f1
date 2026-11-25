# 4단계: 관측 경계 안의 로컬 레이싱 라인

3단계 원본 소스 ZIP·시험 기록은 artifacts/stage4/baseline 및 baseline.json으로 보존한다. 원래 중앙 경로 계산은 centerline_planner.cpp에 함수명만 바꾸어 그대로 유지했다. 모든 신규 실행 기록은 artifacts/stage4에 저장한다. 트랙·world·중앙 스폰·신호·계측·차량 물리·센서 구성은 변경하지 않는다.

## 제출 설정과 검증 범위

버전0.4.0의 제출 manifest는 autonomous·racing_line·상한4.5m/s를 지정한다. 직접 launch의 driving_mode 기본값은 manual이다. 자율 설정은 horizon10m, 계획 횡가속도1.0m/s², 제동2.5m/s², 가속0.9m/s², 관측 축소 여유 최대1.8m, 기준 속도3.5m/s다. timeout.18초·반응 여유.23초·경계 여유.16m를 유지한다.

Monza에서는 grid 접근 뒤 전체2랩을282.315/281.635 sim초에 완료했다. 출발 허가 기준574.403wall초, 빨간불 시작 포함582.308wall초, 접촉0·주행 안전정지0이었다. 자체 관측 결과이며 공식 랩 인증은 아니다. Hungaroring에서는 같은5.0m/s 설정과 상한3.5m/s가 각각 안전 정지로 실패했다. 같은 알고리즘의 상한2.5m/s에서는355.520sim초에 무접촉·주행 안전정지0으로 완주했다. 트랙별 정답 좌표나 조향표는 없으며, 운영자가 상한을 명시한다. 모든 트랙에서5.0m/s를 검증했다고 해석하면 안 된다.

5.0m/s는 이후 R01 반복 시험에서 약395m 지점의 관측거리 축소로 안전 정지했다. 따라서 제출 기본값은 연속3랩을 무접촉·주행 안전정지0으로 완료한4.5m/s로 선택했다. 랩별 실제 시간은 [291.0499999999998, 290.0, 289.9550000000006]sim초다. 5.0m/s Q01 기록과 최종 기본값의 반복 검증은 구분한다.

중앙이 차량/world/clock을 제공한 상태에서:

```bash
# Monza에서 검증한 고속 설정
ros2 launch team04_race race.launch.py driving_mode:=autonomous
# Hungaroring H03의 정확한 검증 프로필(같은 알고리즘, 상한2.5)
ros2 launch team04_race race.launch.py driving_mode:=autonomous autonomy_max_speed:=2.5 \
  autonomy_config:="$(ros2 pkg prefix --share team04_race)/config/autonomy_racing_500_observed.yaml"
```

새 연속 주행과 최종 ZIP 재빌드·기본값 주행의 실제 결과는 패키지 외부 REPORT_STAGE4.md와 artifacts/stage4/acceptance_stage4.json에 기록한다. 원래3.0m/s 중앙 경로로의 복귀 명령은 문서 마지막에 있다.

## 모드와 경로

- centerline: 원래 중앙 경로·속도 계산. 3.0m/s 복귀 설정은 config/autonomy_stage3_rollback.yaml이다.
- racing_line: 동일한 LiDAR 노면/양측 경계 추정에서 시작해 27개의 횡방향 오프셋 후보를 비교한다. 두 개의 부드러운 중간 오프셋과 끝단 오프셋을 독립적으로 선택해 코너 연결과 apex 위치를 바꿀 수 있다. 특정 트랙·코너 이름·world 좌표를 쓰지 않는다.
- 후보 비용은 관측된 구간의 예상 통과시간, 조향 변화, 오프셋 크기와 이전 명령 조향과의 차이다. 속도 프로파일은 전방 곡률·횡가속도·조향 변화율·제동·가속·관측 끝에서의 정지를 함께 반영한다.
- 현재 차체 위치에서 시작하는 부드러운 연결 경로를 만든다. 이미 바깥/안쪽을 주행 중일 때 중앙선으로 급히 돌아가도록 만들지 않고 현재 횡방향 위치를 약4m에 걸쳐 연결한다. 명령/센서 프레임의 정보만 사용한다.
- 실제 양측 관측 구간 밖으로 경로를 연장하지 않는다. 관측 지평선이 짧으면 전역 F1 레이싱 라인을 알 수 없으므로, 이것은 제한된 시야의 로컬 최적화다. 모든 코너에 late apex를 강제하지 않는다.

차체 검사에는 길이0.520m·조향을 포함한 폭0.238958m의 회전된 사각형 전체를 사용한다. 기존0.16m 안전 여유에 표본 간격 여유를 추가하고, 양쪽 실제 벽 선분과 사각형의 교차 및 원본 장애물 반사를 검사한다. 센터 점과 벽의 거리만으로 통과를 허용하지 않는다. 신뢰할 수 있는 후보가 없으면 정지한다. racing_line 안에서 중앙 후보를 선택할 때에도 같은 차체 검사를 적용한다. 별도 centerline 모드는 과거 결과 재현용 원래 계산을 유지한다.

근거리 곡률과 실제 가용 경로에 따라 Lookahead를1.0~2.0m 안에서 조정한다. 현재 명령 속도의 정지거리 예산이 실제 가용 경로보다 크면 목표점을 짧게 만드는 대신 안전 정지한다. 첫3.5m/s 시도는 약132m에서 고정 Lookahead가 차체 여유 이후 경로보다 길어 정지했으며, 실제 실패 점군 재생과 짧은 경로/과속 정지 회귀 시험으로 수정 사항을 확인했다.

성긴 원거리 반사점은 실제 관측된 양 끝점 사이에서만 연결한다. 기존 연속성 조건을 만족하지 않는 경우에도1m 이하의 제한된 간격에 대해 최근 관측 벽 방향, 전방 진행, 방향 일치, 수직 잔차, grazing angle을 반영한 광선–벽 교차 잔차가 모두 일치해야 한다. 큰 각도 공백·측면 단절·측정 끝은 연결하거나 연장하지 않는다. 원래 centerline 백엔드는 이 개선과 분리되어 그대로 보존된다.

## 속도와 관측거리

새 모드는 정지거리에 센서 지연을 포함한다:

    v² / (2 * braking_deceleration) + reaction_time_s * v <= observed_usable_distance - reserve

reaction_time_s 기본0.23초는 sensor_timeout_s=0.18초 이상이다. 관측 끝보다 차체가 앞서지 않도록 길이 여유도 차감한다. LiDAR 최대12m가 완전한12m 경계 인식을 보장하지 않는다. 실제 정지 직선 점군에서는 horizon8→10→12를 지정해도 대응 경계 길이가 약7.13m에서 제한되는 표본을 확인했다. 실제 주행에서는 이 길이가 계속 바뀌며, 그때그때 관측한 값만 사용한다.

기존 단순 선형 곡률 감속 항 대신 racing_line에서는 물리 계획값·곡률·조향 변화율·제동거리로 속도를 정한다. 계획 횡가속도는 차체의 실제 횡가속도를 엄격히 보장하는 상한이 아니다. pose/엔코더/IMU는 시험 관측·물리 진단에만 쓰며 자율 노드로 전달하지 않는다. 엔코더가 먼저 멈춘 뒤 차체가 미끄러질 수 있으므로, 제동 성능은 실제 차체 이동거리로 확인한다.

## 빌드와 회귀 시험

저장소 루트에서:

```bash
source /opt/ros/jazzy/setup.bash
CMAKE_BUILD_PARALLEL_LEVEL=2 colcon --log-base src/team_04/artifacts/stage4/log build \
  --base-paths src/team_04/team04_race \
  --build-base src/team_04/artifacts/stage4/build \
  --install-base src/team_04/artifacts/stage4/install --merge-install
source src/team_04/artifacts/stage4/install/setup.bash
colcon --log-base src/team_04/artifacts/stage4/log test \
  --base-paths src/team_04/team04_race \
  --build-base src/team_04/artifacts/stage4/build \
  --install-base src/team_04/artifacts/stage4/install --merge-install
```

이전 검증용 복사 패키지가 artifacts 아래에 있으므로 --base-paths를 생략하지 않는다. 기존 중앙 경로12개와 신규 계획기14개 시나리오를 검사한다. ROS 안전 시험은 실제 차량과 분리된 도메인97에서 실행한다.

```bash
PYTHONDONTWRITEBYTECODE=1 python3 -s src/team_04/team04_race/tests/autonomy_safety.py \
  --max-speed 5.0 --path-mode racing_line \
  --config src/team_04/team04_race/config/autonomy.yaml \
  --output src/team_04/artifacts/stage4/new-safety-result.json
```

## 실제 시험 및 시간 기준

label은 항상 새 이름을 쓴다. 기본 제출값을 검증하기 전까지 개발 프로필을 제출 기본값으로 간주하지 않는다.

```bash
PYTHONDONTWRITEBYTECODE=1 python3 -s src/team_04/team04_race/tests/autonomy_trial.py \
  --isolated --root . --track monza --label stage4-new-trial \
  --autonomous --max-speed 3.5 --path-mode racing_line \
  --autonomy-config src/team_04/team04_race/config/autonomy_racing_development.yaml \
  --duration 1500 --stop-after-laps 1 --stop-on-active-failure
```

속도는 직전 단계의 실제 전체 무접촉 완주 뒤에만 올린다. 실패한 구간의 시간을 랩타임으로 쓰지 않는다. parameter/config와 실행파일 해시는 각 run_config.json에 저장한다.

2랩 시간 시험은 --stop-after-laps 2와 --qualifying-distance를 함께 쓴다. 관측자는 초기 grid→출발선 접근을 분리하고 첫 선 통과를0랩으로 처리한 뒤 전체2랩을 확인한다. 이는 중앙 계측을 수정하거나 대체 발행하는 기능이 아니다. 결과는 자체 관측이며 공식 인증과 다르다.

- lap_seconds: 첫 접근 이후 선→선 랩 시간.
- elapsed_seconds: 출발 허가 시각부터 랩 종료까지 sim시간, 접근 포함.
- wall_elapsed_from_arm_s: Bool 허가 직전부터 목표 관측까지 실제시간. 소등을 위한 일시정지 시간도 포함하는 보수적인 값.
- wall_elapsed_from_resume_s: world 재개 확인 후 목표 관측까지 실제시간.
- goal_wall_elapsed_from_sequence_s: 빨간불 출발 절차 시작부터 목표까지 실제시간.
- wall_duration: 종료 후 정지 관측 약1초까지 포함한 전체 시험시간.

600초 판정은 실제 wall 시간을 사용한다. 표본 간 관측 지연이 있으므로 공인 밀리초 정확도는 주장하지 않는다. 중앙 계측 토픽의 기존 호환성 문제는 그대로 남겨 두며 공식 예선 통과 인증을 주장하지 않는다.

## 진단과 분석

기존 path/target_point/track_boundaries/target_speed/state 토픽을 유지한다. centerline Path, racing_candidates MarkerArray, speed_profile JSON도 제공한다. 시험 관측자는 선택 경로·중앙 경로·후보·속도 프로파일을 별도 기록한다. RViz GUI를 자동 시각 인증했다고 주장하지 않는다.

```bash
PYTHONDONTWRITEBYTECODE=1 python3 -s src/team_04/team04_race/tests/analyze_stage4.py \
  src/team_04/artifacts/stage4/<label> --root .
```

기준선과의 거리는 의도된 레이싱 오프셋일 수 있다. 별도 executed_path_error_0_25s_m은 과거 선택한 로컬 경로를 관측 pose로 변환해0.25초 뒤 실제 궤적과 비교하는 사후2D 추종 오차다. 이 관측 정보는 주행 노드에 입력하지 않는다. 제한 원인·직선 속도·센서 Hz/age·CPU/RSS·RTF·제동거리를 함께 기록한다. 최종 추천 설정과 실제 실행 결과는 REPORT_STAGE4.md를 따른다.

## 원래3.0m/s로 복귀

중앙이 world·차량·clock을 제공한 상태에서:

```bash
ros2 launch team04_race race.launch.py driving_mode:=autonomous \
  autonomy_path_mode:=centerline autonomy_max_speed:=3.0 \
  autonomy_config:="$(ros2 pkg prefix --share team04_race)/config/autonomy_stage3_rollback.yaml"
```

주행 모드와 속도를 바꿀 때는 기존 명령 발행자를 먼저 종료한다. Bool false·timeout·시각/형식 검사·노면/경계/경로 실패·명령 충돌·제어 watchdog은 유지한다.
