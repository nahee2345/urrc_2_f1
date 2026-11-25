# Monaco 단독 1랩 실행

~/urrc_2 기준. 이전 출발 시험 REPORT.md는 이전 결과이며 이번 랩 시험 결과는
validation_f1/lap_20261119/에 분리되어 있다. 기존 설치본과 운영자 파일은 유지한다.
현재 최종 실주행 결과는 **미완주**다. 약241m 급커브에서 안전 정지했으며,
원인·측정값·출발 안전 회귀 결과는 `validation_f1/lap_20261119/REPORT.md`에 있다.
아래 명령은 현재 파일 재현용이며 완주를 보장하는 결과가 아니다.

Monaco preview는 한 개만 실행한다. 준비 허가 Bool로 카메라 게이트를 우회할 수 없다.
아래 예제의 lap_manual 폴더에 기록이 이미 있으면 다른 새 폴더명을 선택한다.
기록기는 기존 poses.jsonl을 덮어쓰지 않고 중단한다.

터미널 1 — 전용 빌드와 원래 preview. 아직 READY를 입력하지 않는다.

```bash
cd ~/urrc_2
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=42
bash src/team/team_10/tools/build_f1.sh
bash race_v24.sh preview qualifying --track monaco
```

터미널 2 — clock·영상·LiDAR·제어 브리지와 차량 노드.

```bash
cd ~/urrc_2
source src/team/team_10/tools/f1_env.bash
ros2 launch team10_race race.launch.py race_start_mode:=camera
```

터미널 3 — 1번 스폰, 센서·clock 준비 PASS 확인, 준비 허가.

```bash
cd ~/urrc_2
source src/team/team_10/tools/f1_env.bash
python3 src/team/team_10/tools/spawn_f1.py
python3 src/team/team_10/tools/check_f1.py && \
  python3 src/team/team_10/tools/permission.py true
```

터미널 4 — 읽기 전용 원시 LiDAR·센서/명령 체인·검증용 pose 기록.

```bash
cd ~/urrc_2
source src/team/team_10/tools/f1_env.bash
python3 src/team/team_10/tools/record_lap.py --output validation_f1/lap_manual
```

지도·카메라 수신·준비 PASS를 확인하고, 원래 터미널 1에 입력한다.

```text
READY
```

FIVE_RED_CONFIRMED -> LIGHTS_OUT_CONFIRMED 뒤 실제 주행한다.
진행은 validation_f1/lap_manual/lap_state.json에서 확인한다.
preview에는 운영자 경기 랩 기록기가 없으므로 검증기는 원본 운영자 timing.py의
출발선과 순서 체크포인트를 읽는다. first_line 이후 1..66을 순서대로 통과하고
출발선을 다시 통과해야 finish_line과 lap_seconds가 기록된다.
이것은 검증용 기록이며 공식 경기 기록으로 주장하지 않는다.
정답 pose는 기록기에만 연결되며 navigation·adapter에는 공급하지 않는다.
벽 접촉 센서는 없으므로 물리 접촉 여부는 미측정이다.

터미널 3 — 완료/실패 후 차량 정지.

```bash
cd ~/urrc_2
source src/team/team_10/tools/f1_env.bash
python3 src/team/team_10/tools/permission.py false
```

출력 0과 실제 정지를 확인한 뒤 터미널 4에서 Ctrl+C를 누른다.
세 기록기가 종료되며 chain.jsonl도 해당 실행 폴더에 보존된다. 그 다음 분석:

```bash
cd ~/urrc_2
python3 src/team/team_10/tools/lap_metrics.py --run validation_f1/lap_manual
```

마지막으로 터미널 2, 1 순서로 Ctrl+C. 영상 단절/false 후 재출발은 새 preview와
차량으로 위 순서를 다시 수행한다. 안전 정지를 timeout 증가로 우회하지 않는다.
현재 속도 설정은 max=4.0, medium=1.5, corner=1.0 m/s이며 이번 작업에서 올리지 않았다.
10m/s 이상이라는 향후 목표는 이 1랩 검증의 성공 조건과 별개다.

2026-11-22 후속 급커브/랩 작업의 현재 전체 명령은 `RUN_GUIDE.md`, 실제 결과는
`validation_f1/bend_20261122/REPORT.md`를 참조한다. 위의 2026-11-19 결과는 당시 기록이다.
