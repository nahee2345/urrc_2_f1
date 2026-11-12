# TEAM11 내장 맵 Gazebo 신호·주행 검증

`urrc_2_f1`의 `src/team_11/maps/{track}/centerline.csv`와 `track.json`만
읽어 **별도의 시험용 Gazebo SDF**를 생성합니다. 모델, 조향/속도 제어기,
카메라, 전/후 LiDAR는 제출용 `team11_race`와 **동일한 것**을 사용합니다.
기존 경로, 원본 대회장, 타 참가팀 파일은 수정하지 않습니다. 백업하지 않습니다.

**주의:** 내장 데이터는 CSV 경로이지 공식 Gazebo 맵 월드/연석/환경 메시가
아닙니다. 이 시험에서 생성되는 도로는 CSV 중앙선을 따라 박스를 연결한
근사 도로이므로 **공식 대회 맵에서의 충돌/2랩 완주 통과를 의미하지 않습니다.**

## 시험 실행

```bash
cd ~/urrc_2_f1
git pull --ff-only origin main
bash src/team_11/sim_test/run.sh --track monza --grid 1 --seconds 90
```

- 화면 없이 실행하려면 `--headless`를 추가합니다. OGRE2/EGL 환경이 필요합니다.
- 11번 그리드: `--grid 11`
- 다른 트랙: `--track spa`, `--track monaco`, `--track cota`,
  `--track hungaroring`, `--track baku`
- `Ctrl+C`로 중단하면 관측된 부분 결과가 출력됩니다.

## 시험 순서 및 검증 항목

1. ROS 2 Jazzy용 실제 `team11_race` 패키지를 새로 빌드합니다.
2. 내장 CSV로 트랙 전체를 연결한 물리 충돌 도로를 생성하고,
   선택 그리드에 팀 차량을 실제 물리 시뮬레이션으로 스폰합니다.
3. 5개의 빨간 램프를 **하나의 모델**로 켜서 전방 카메라에 8초간 노출합니다.
4. Gazebo UserCommands의 **단 한 번의 제거 명령**으로 5개를 함께 끕니다.
5. 제거 성공 후에만 `/race/start/team_11` BOOL TRUE를 지속 발행합니다.
6. ROS 카메라, 차량 자체 pose, 맵/그리드 인식, 출발 출처,
   실제 이동 거리(own pose 기반)를 확인합니다.
7. 결과와 로그는 `build/team11_gazebo_tests/`에 보존합니다.
   **백업 폴더를 만들지 않습니다.**

**검증 성공 기준:** 카메라에서 빨간불 감지 및 소등 감지,
TEAM11 자체 위치 수신, 지도/출발 그리드 식별,
`TEAM11 START source=CAMERA_LIGHTS_OUT`,
실측 이동거리 1m 초과가 모두 확인되어야 PASS입니다.

`OFFICIAL_GATE_SOLO` 등으로 출발하면 이동했더라도
**카메라 신호 인식 성공으로 판정하지 않습니다.**

Gazebo 센서 렌더링/그래픽 드라이버 환경 문제는
`gazebo.log`를 확인하십시오.
