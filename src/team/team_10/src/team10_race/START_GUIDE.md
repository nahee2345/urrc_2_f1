# F1 Monaco 출발 재현

현재 워크스페이스는 ~/urrc_2. 기존 build/install 및 install_team103d는 유지한다.
이번 실행은 build_f1/install_f1/log_f1만 사용한다. preview는 하나만 실행한다.
아래 source는 각 터미널에서 별도로 실행해야 한다. f1_env.bash는 현재 원래
preview 프로세스의 새 GZ_PARTITION을 찾으며 ROS_DOMAIN_ID=42를 설정한다.
중복 Monaco preview가 있으면 임의 선택하지 않고 실패한다.

터미널 1 — 빌드와 원래 preview. READY는 아직 입력하지 않는다.

```bash
cd ~/urrc_2
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=42
bash src/team/team_10/tools/build_f1.sh
bash race_v24.sh preview qualifying --track monaco
```

터미널 2 — clock·영상·LiDAR·제어 브리지와 차량 노드. 스폰 전에 실행한다.

```bash
cd ~/urrc_2
source src/team/team_10/tools/f1_env.bash
ros2 launch team10_race race.launch.py race_start_mode:=camera
```

터미널 3 — metadata 1번 위치 스폰, 실제 센서·시간 준비 점검, 준비 허가.
check_f1.py가 PASS여야 다음 단계로 간다. 스폰 요청 성공만으로 준비 완료가 아니다.

```bash
cd ~/urrc_2
source src/team/team_10/tools/f1_env.bash
python3 src/team/team_10/tools/spawn_f1.py
python3 src/team/team_10/tools/check_f1.py
python3 src/team/team_10/tools/permission.py true
```

터미널 4 — 실제 카메라·명령·검증 전용 pose 기록. READY 전에 실행한다.
기존 chain.jsonl은 실행 시 자동으로 다른 이름으로 보존한다. 카메라 최신 프레임은
validation_f1/camera_latest.png, 상태/명령/pose는 validation_f1/chain.jsonl이다.

```bash
cd ~/urrc_2
source src/team/team_10/tools/f1_env.bash
python3 src/team/team_10/tools/observe_f1.py
```

도로·그리드·신호등, 준비 PASS, 카메라 수신을 확인한 후 원래 터미널 1에 입력:

```text
READY
```

5등이 켜지는 동안 정지하고, 실제 카메라가 FIVE_RED_CONFIRMED ->
LIGHTS_OUT_CONFIRMED를 기록한 뒤 전진한다. 영상 단절 후에는 자동 재출발하지
않는다. 새 출발 시험은 false로 정지하고 preview/차량 실행을 새로 시작한다.

터미널 3 — 이동 측정(제어 입력으로 사용하지 않음), 안전 정지:

```bash
cd ~/urrc_2
source src/team/team_10/tools/f1_env.bash
python3 src/team/team_10/tools/summarize_trial.py validation_f1/chain.jsonl
python3 src/team/team_10/tools/permission.py false
```

종료는 false 수신·출력 0을 확인한 다음 터미널 4, 2, 1 순서로 Ctrl+C.
전체 시험 결과와 실패 이력은 워크스페이스 validation_f1/REPORT.md에 있다.

변경 파일:
- models/team10_car/model.sdf: 카메라·하우징·고정 조인트, 명시적 플러그인 토픽
- race_entry.json: team_10와 정규화 경로
- launch/race.launch.py, setup.py, package.xml: 카메라·브리지·clock·실행 의존성
- team10_race/start_vision.py, start_camera_node.py, start_interlock.py: 영상 게이트
- team10_race/navigation_node.py, race_adapter.py: 각각의 게이트와 freshness 계측
- tools/: 독립 빌드·환경·스폰·준비·정지·읽기 전용 계측 도구
- test/test_camera_start.py, test/test_contract.py: 출발 안전/계약 회귀 테스트

단독 1랩 실행과 기록·정리 명령은 `LAP_GUIDE.md`를 참조한다. 이번 랩 작업에서는 영상 전송을 400×300으로 줄이고, 주 LiDAR 센서 원점만 15mm 올려 타이어 가림을 제거했다. F1 차체·바퀴·구동 및 속도 설정은 유지한다.

2026-11-22 후속 급커브/랩 작업의 현재 전체 명령은 `RUN_GUIDE.md`, 실제 결과는
`validation_f1/bend_20261122/REPORT.md`를 참조한다. 위의 2026-11-19 결과는 당시 기록이다.
