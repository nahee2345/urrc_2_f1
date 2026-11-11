# TEAM 11 — 6개 맵 경로 내장 연습 시스템

이 코드는 **사전 경로 추종 연습 전용**입니다. 공식 URRC 대회 참가 규정은 LiDAR 이외의
저장된 트랙 좌표 기반 주행 판단을 금지하므로, 그대로 제출하면 규정에 맞지 않습니다.

## 폴더 구조

- `three_lane/maps/<맵 이름>/centerline.csv`: 공식 원본 중앙선 사본
- `three_lane/maps/<맵 이름>/lane_0.csv`: 왼쪽(1번) 경로
- `three_lane/maps/<맵 이름>/lane_1.csv`: 중앙(2번) 경로
- `three_lane/maps/<맵 이름>/lane_2.csv`: 오른쪽(3번) 경로
- `three_lane/maps/<맵 이름>/track.json`: 원본 SHA, 그리드, 경로 간격 등

지원 맵은 `monza`, `spa`, `monaco`, `cota`, `hungaroring`, `baku`입니다.
원본 트랙 파일을 수정하지 않으며 GitHub의 `upstream/urrc_f1_ws` 서브모듈에서
맵·메시·신호등 리소스만 불러옵니다.

## 실행

```bash
cd ~/urrc_2_f1
git submodule update --init --recursive
python3 src/team_11/three_lane/run_demo.py monaco --grid 1 --prepare-only
python3 src/team_11/three_lane/run_demo.py monza --grid 1 --start-mode camera
# 물리 충돌 모드에서는 연습 차량도 추가됩니다.
python3 src/team_11/three_lane/run_demo.py monza --grid 1 \
  --practice-cars --physics-contact --start-mode camera
```

맵 자동 인식은 **운영자가 지정한 SDF의 `<world name>`** 기준입니다.
파일을 안 주거나 미등록 이름이면 실행을 중단하며 맵을 임의 추정하지 않습니다.

```bash
python3 src/team_11/three_lane/run_demo.py auto \
  --world upstream/urrc_f1_ws/src/urrc_track_gazebo/worlds/monaco_race.sdf \
  --grid 1 --prepare-only
```

외부 대회장 폴더에 TEAM11만 복사하는 경우에는 해당 폴더를 `--source`로 지정합니다.
단, 맵 원본이 내장된 데이터와 다른 버전이면 해시 검사에서 실행을 막습니다.

## 검증 기준

1. 내부 6개 지도·3개 경로의 정합성 검사
2. 맵별 파생 시뮬레이션 월드·스폰 위치·신호등 연결 검사
3. 실제 Gazebo 물리 주행 1랩 및 도로 이탈 0건 검사
4. TTC 3초 추월·코너 중앙 합류·후방 방어 회귀
5. 물리 모드 실제 접촉 및 복구 동작 검증

정적 경로 검증과 Gazebo 1랩 실주행 성공은 별개입니다.
