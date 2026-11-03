# TEAM01 v8: 2026-11-02 현장 로그 기반 코너 이탈 방지 보완

## 현장 로그 사실

- 신호등 카메라: `peak=5`, `out=True`, `permission=True`; 출발 성공.
- 직선/초기 구간: `GRID v=0.75` 이후 `CORNER`/`STRAIGHT` 상태로 변경됨.
- 코너: 실제 odometry `odom_v≈1.8m/s`, `planner=adaptive_pursuit`.
- 14.2m 근처 `curb_left=0.33m`; 16.2m `0.27m`; 18.0m `0.20m`; 20.0m `0.04m`; 21.4m `-0.07m`.
- 21.4m에서 `BRAKE reason=no verified road-safe candidate`; 이후 도로 확인 실패로 정지.
- `CORNER_EXIT` 상태가 큰 음의 도로 헤딩에서도 계속 이어짐.
- `curb_left/right`는 실센서 기반 **추정값**으로 실제 차량과 연석의 물리 접촉 거리 측정치는 아님.

## 수정 사항

1. 가까운 연석과 차체 여유를 가변 속도 제어에 연결: `min(left,right) < path_half_width+.055+.12`일 때 사전 감속.
2. `min(left,right) <= path_half_width+.055`인 경우 0m/s 명령, 검증 불가한 코너 탈출 중단. 이 상황에서는 추가적인 센서/차선 검증 없이는 자동 복구 시도하지 않음.
3. Pure Pursuit 후보 경로를 생성하기 전 가까운 연석 기준 차폭 확보 검사 추가. 회전 중 차체 길이에 따른 옆으로 벌어지는 footprint 반영.
4. 진행 중 다른 방향의 코너가 나타나면 이전 `CORNER_EXIT` 단계를 초기화. 코너가 계속 가파르면 바깥쪽으로 옮기는 요청 중단.
5. 로그에 `edge`, `edge_m`, `edge_vcap` 추가. 도로 카메라 진단 이미지 `~/.local/state/urrc_team01/road/{latest,edge_caution,edge_critical,first_brake}.png` 저장.
6. 빨간불 점등→소등, 차량 모델/SDF/센서 토픽, 규정 속도 목표(6.0m/s, 코너 30%, 20m/s 장기 목표) 유지.

## 자동 검증

- 기존 v7 자동 테스트 71개 회귀 통과.
- v8 신규 9개 테스트: 좌측 여유 0.27/0.04/-0.07m, 정상 중앙, 후보 경로 차체 점유폭, 반대 방향 커브, 급한 코너에서 exit offset 중지, near-edge에서 중앙 복귀, 시작 신호 회귀.
- 총 **80개 PASS**. Python compileall, 쉘 문법 검사 통과.
- **한계**: 제공된 로그를 바탕으로 만든 합성 경계 테스트입니다. 이 환경에서 ROS2 Jazzy/Gazebo Harmonic 월드 실행은 진행하지 못했습니다. 실제 Monza 코너 통과, 완주, 무충돌은 검증되지 않았습니다. 센서의 실제 깊이 투영/경계 오검출 가능성도 남아 있습니다.

## 실행

```bash
unzip -o ~/Downloads/team01_f1_corner_safety_v8.zip -d ~/urrc_2/src/team_01
cd ~/urrc_2/src/team_01/team01_race
bash scripts/run_offline_tests.sh
bash scripts/run_local_monza.sh 2>&1 | tee ~/team01_v8.log
```

속도가 줄거나 다시 정지하면 `edge`, `edge_m`, `curb_left`, `curb_right`, `road_heading`, `planner`, `odom_v`, `road_source`와 `~/.local/state/urrc_team01/road/first_brake.png`를 확인하십시오.
