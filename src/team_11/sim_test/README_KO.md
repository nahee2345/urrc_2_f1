# TEAM11 공식 GitHub 맵으로 Gazebo 주행·신호 시험

기본 실행은 `nahee2345/urrc_f1_ws` **GitHub main의 공식 3D 맵**을
선택 트랙만 다운로드하고, 도로·벽·연석·노면 표시·출발 게이트를
실제 Gazebo 메시(SDF+OBJ)로 사용합니다.

다운로드 위치:
`~/urrc_2_f1/build/team11_official_maps_cache/urrc_f1_ws/`

- **기존 `~/urrc_f1_ws`는 읽거나 수정하지 않습니다.**
- 기존 `src/team_11/maps/` 경로 데이터와 차량 모델은 덮어쓰지 않습니다.
- 다운로드된 공식 중앙선 SHA-256과 TEAM11 경로 중앙선이 일치해야 실행합니다.
- GUI는 공식 저장소의 `{track}_grid.gui.config`를 사용합니다.
- 시험 월드는 `build/team11_gazebo_tests/`에 생성하며 백업 폴더는 없습니다.
- 공식 맵 파일은 변경 없이 그대로 사용하지만, 시험용 TEAM11 차량과
  빨간 신호등 모델은 임시 월드에 **별도로 추가**합니다.
- 이는 공식 트랙 메시 기반의 독립 시험으로, 대회장 운영 매니저의
  랩 계측, 실제 출발 순서, 다중 차량 스폰 등을 그대로 재현하지는 않습니다.

## 1. 기본 Monza 1번 그리드 GUI 실행

```bash
cd ~/urrc_2_f1
git switch main
git pull --ff-only origin main
bash src/team_11/sim_test/run.sh --track monza --grid 1 --seconds 90
```

처음 실행할 때 필요한 공식 모델 파일만 GitHub에서 내려받습니다.
다음 실행에는 같은 캐시를 재사용합니다.

11번 그리드 실행:

```bash
bash src/team_11/sim_test/run.sh --track monza --grid 11 --seconds 90
```

다른 트랙: `--track spa`, `--track monaco`, `--track cota`,
`--track hungaroring`, `--track baku`.

GitHub에 변경된 공식 맵을 새로 반영할 때:

```bash
bash src/team_11/sim_test/run.sh --track monza --grid 1 --seconds 90 --refresh-maps
```

만약 공식 중앙선이 실제로 변경되어 내장 경로와 달라졌다면,
차량이 잘못된 경로를 추종하지 않도록 오류를 내고 실행을 차단합니다.

## 2. 카메라 신호등 시험 과정

1. 공식 원본 `{track}_race.sdf`를 복사해 임시 Gazebo 월드를 만듭니다.
2. 정식 `team11_car/model.sdf` 차량을 지정 그리드에 배치합니다.
3. 공식 출발 게이트의 다섯 렌즈 위치에 발광 빨간불을 추가합니다.
4. ROS 제어기에서 **실제 카메라 빨간불 감지 및 자체 위치 수신**을 확인합니다.
5. Gazebo 요청 한 번으로 다섯 신호등을 동시에 제거합니다.
6. 소등 성공 후 `/race/start/team_11`에 TRUE를 발행합니다.
7. 카메라 소등 감지, 실제 출발 근거와 차량 이동 거리를 로그에 표시합니다.

`CAMERA_LIGHTS_OUT`가 아닌 `OFFICIAL_GATE_SOLO`로 출발하면,
차량이 이동하더라도 **카메라 출발 인식 성공으로 판정하지 않습니다.**

기본 90초는 소등 이후의 관찰 시간이며, 초기 공식 메시 다운로드와
Gazebo 초기화 과정은 별도입니다.

## 3. 기존 CSV 사각형 시험장 (선택 사용)

종전의 내장 CSV를 직사각형 물리 도로로 재구성하는 시험장은
`--csv-test`를 명시한 경우에만 사용합니다.

```bash
bash src/team_11/sim_test/run.sh --track monza --grid 1 --seconds 90 --csv-test
```

이 시험장은 공식 도로·벽 모양과 일치하지 않습니다.

## 4. GUI·로딩 오류 진단

오류가 발생하면 Gazebo GUI/서버 및 ROS 로그의 마지막 내용을
터미널에 표시합니다. 별도 로그는 다음에서 찾을 수 있습니다.

```bash
cd ~/urrc_2_f1
LOG_DIR=$(find build/team11_gazebo_tests -type f -name gazebo.log \\
  -printf '%T@ %h\n' | sort -nr | head -1 | cut -d' ' -f2-)
echo "$LOG_DIR"
tail -n 60 "$LOG_DIR/gazebo.log"
tail -n 60 "$LOG_DIR/gazebo_gui.log"
tail -n 60 "$LOG_DIR/team11_ros.log"
```

- GUI가 있는 Ubuntu 데스크톱에서 위의 기본 명령을 실행하십시오.
- 별도 화면이 없는 환경은 `--headless`를 추가할 수 있습니다.
- 준비가 느리면 `--startup-timeout 240`을 사용할 수 있습니다.
- Gazebo 자체 GUI 진단: `gz sim shapes.sdf -v 4`.
- 공식 메시가 무겁고 대형 트랙에서 GUI/OGRE2 문제가 발생할 수 있습니다.

**전체 성공 판정:** 빨간불 검출, 소등 검출, `CAMERA_LIGHTS_OUT`
출발, 자기 위치 수신, 맵/그리드 선택, 실측 이동거리 1m 초과가
모두 확인되어야 합니다.
