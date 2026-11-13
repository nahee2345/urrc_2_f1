# TEAM 11 실제 Gazebo 주행 1랩 검증 (신호등 제외)

기존 오프라인 260개 PASS는 실제 차량 움직임이 아닙니다. 이 단계는 Gazebo에서
모델의 실시간 Odometry를 받아 차량을 움직이고, 1랩 진행·이탈·코너 속도를 기록합니다.
공식 `urrc_f1_ws` 파일은 수정하지 않습니다.

## 실행

기존 Gazebo 종료 후:

```bash
cd ~/urrc_2_f1
git pull --ff-only origin main
bash src/team_11/tests/run_physical_lap.sh monza 1
```

사용 토픽: `/model/team_11/odometry_11` (Gazebo→ROS),
`/model/team_11/cmd_vel_11` (ROS→Gazebo).
경량 월드만 사용하고 카메라 신호·관전용 카메라·신호등 시작 시퀀스는 사용하지 않습니다.
Follow는 GUI의 추적 기능이며 차량 센서가 아닙니다.
이 실행기의 로컬 연습 전용 출발 허용은 대회용 신호 게이트를 변경하지 않습니다.

## 결과

- `log/team11_physical_lap/monza_grid_01/result.json`: PASS_LAP 또는 실패 이유
- `log/team11_physical_lap/monza_grid_01/result.csv`: 주행 거리, 실제속도, 목표속도, 조향, 경로, 곡률, 도로 중앙 이격
- `log/team11_physical_lap/monza_grid_01/trial.log`: 실시간 진행 기록
- `log/team11_physical_lap/monza_grid_01/bridge.log`: 토픽 브리지 진단

PASS_LAP는 *Gazebo 오도메트리 기준* 랩 통과입니다. 오도메트리·벽 충돌·전체 맵 모델의 정합성은 추가 검토해야 하며,
실제 8m/s 코너 주행의 물리적 안전성을 보증하지 않습니다.

## 이후 순서

Monza 1랩이 실제 PASS일 때만 나머지 다섯 맵으로 확장하고,
이후 추월·방어 다차량 검증을 진행합니다. 신호등 검증은 가장 마지막에 진행합니다.
