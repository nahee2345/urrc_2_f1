# F1 Monaco 전체 실행

작업 경로는 ~/urrc_2, 전용 설치본은 install_f1/team10_race.
Monaco preview는 하나만 실행한다. 각 터미널에서 환경을 별도로 source한다.
GZ_PARTITION은 f1_env.bash가 현재 원래 preview에서 읽고 ROS_DOMAIN_ID=42를 설정한다.
최신 실행본·설정·증거는 SPEED_HANDOFF.md에 기록한다. 현재 선택본은
validation_f1/speed_20261123/latest_scan_final_lap의226.453495sim초 완주본이다.
출발 포함 평균2.933369m/s, 최고 원시10.167571/약.2초평활10.104984m/s.
실제>=10 총1.321sim초/.57905%/최장.599초로 연속 유지 시간은 아직 짧다.
접촉/이탈/지속정지/역주행0, 최종 false 실제정지 확인. primary scan 큐만 최신1개로 줄였다.
이전248.994552/254.778385sim초 성공 스냅샷도 보존했다. 출발/false/영상정지 안전 코드는 유지.

아래 기록 폴더가 이미 있으면 모든 예시에서 같은 새 폴더명으로 바꾼다.
기존 기록을 덮어쓰지 않는다. 정상 실행에는 reproduce_bend_pose.py를 사용하지 않는다.

터미널 1: 빌드(접촉 계측 바이너리 포함)와 원래 preview. 아직 READY 입력 금지.

```bash
cd ~/urrc_2
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=42
bash src/team/team_10/tools/build_f1.sh &&
bash race_v24.sh preview qualifying --track monaco
```

터미널 2: clock·영상·LiDAR·제어 브리지와 차량 노드.

```bash
cd ~/urrc_2
source src/team/team_10/tools/f1_env.bash
export HANLA_SPEED_AUDIT_DIR="$PWD/validation_f1/lap_manual_20261123/audit"
ros2 launch team10_race race.launch.py race_start_mode:=camera
```

터미널 3: metadata 1번 스폰, 물리 접촉 계측 연결, 실제 센서 준비 확인, 준비 허가.

```bash
cd ~/urrc_2
source src/team/team_10/tools/f1_env.bash
python3 src/team/team_10/tools/spawn_f1.py &&
python3 src/team/team_10/tools/check_f1.py &&
python3 src/team/team_10/tools/permission.py true
```

터미널 4: READY 전에 원시 입력·명령·접촉·관절각·검증용 pose 기록.

```bash
cd ~/urrc_2
source src/team/team_10/tools/f1_env.bash
export HANLA_SPEED_LIGHT_RECORDING=1
python3 src/team/team_10/tools/record_lap.py --output validation_f1/lap_manual_20261123
```

준비 PASS·화면·카메라 수신을 확인하고 원래 터미널 1에 입력:

출발 반복 시험 후 준비 검사는 연속 수신과 영상 안정화도 확인한다. 기록기 준비가
끝난 뒤 터미널 3에서 READY 직전에 다시 검사한다. NOT READY면 READY를 입력하지
말고 실제 생성·수신 회복을 확인한다. 검사 대기 20초와 차량 freshness 제한은 그대로다.

```bash
cd ~/urrc_2
source src/team/team_10/tools/f1_env.bash
python3 src/team/team_10/tools/check_f1.py
```

PASS 뒤 원래 터미널 1:

```text
READY
```

실제 카메라의 5등 확인과 전체 소등 인식 후 출발한다. Bool은 준비 허가뿐이다.
터미널 3에서 아래를 실행하면 진행을 볼 수 있다. finish_line이 생겨야 출발선
정상 방향 재통과다. next_gate만 보고 완주로 판정하지 않는다.

```bash
cd ~/urrc_2
watch -n 1 cat validation_f1/lap_manual_20261123/lap_state.json
```

완료 또는 실패 시 터미널 3의 watch를 Ctrl+C로 끝내고 차량 정지:

```bash
cd ~/urrc_2
source src/team/team_10/tools/f1_env.bash
python3 src/team/team_10/tools/permission.py false
```

출력 0과 실제 정지를 확인한다. 터미널 4에서 Ctrl+C로 네 기록기를 마감한 뒤 분석:

```bash
cd ~/urrc_2
MPLCONFIGDIR=/tmp/mpl_f1 python3 src/team/team_10/tools/track_integrity.py --run validation_f1/lap_manual_20261123
python3 src/team/team_10/tools/lap_metrics.py --run validation_f1/lap_manual_20261123
python3 src/team/team_10/tools/speed_metrics.py --run validation_f1/lap_manual_20261123
python3 src/team/team_10/tools/speed_response.py --run validation_f1/lap_manual_20261123
```

track_integrity는 finish_line이 있으면 first_line..finish_line 구간만 분석한다.
공식 랩 기록기가 없는 preview에서는 운영자 원본 timing.crossing/track_gates로
0,1..66,0 순서와 정상 방향을 검증한다. 이 결과를 공식 경기 기록으로 주장하지 않는다.
실제 road_collision mesh에서 차량 중심·샘플 중점 이탈과 접촉 스트림도 검사한다.
pose·맵·체크포인트는 검증기에만 쓰고 차량 주행 입력에 넣지 않는다.
초기화/중간 코드 교체 기록은 metrics가 최종 코드 랩으로 인정하지 않는다.

마지막으로 터미널 2, 1 순서로 Ctrl+C. 영상 단절 뒤 새 5등 점등 없이 재출발하지
않는다. 새 출발 시험은 새로운 preview/차량과 새 기록 폴더로 다시 수행한다.

현재 planner/adapter/model 상한은10.1m/s로 일치한다. continuous_limits 모드이며
중간/코너 단계1.5/1.0, 반응.40초, 제동3m/s²를 유지한다. 횡가속 예산3m/s²,
고속 lookahead L=1.4+.4*max(0,v-3), 최대4.4m다. 실제 속도/유지 시간은
기록으로 판정한다. 기존 완주본·사용자 파일·운영자 소스는 보존한다.
