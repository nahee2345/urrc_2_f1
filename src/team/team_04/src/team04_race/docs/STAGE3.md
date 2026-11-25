# 3단계 실험과 재현

기존 차량·센서·경계 추적·Pure Pursuit 구조를 유지한다. 노면·world·신호등·운영자 스크립트·중앙 계측을 변경하지 않는다. 모든 시험 결과는 `src/team_04/artifacts/stage3/`에 저장하며 stage2 ZIP/로그를 덮어쓰지 않는다.


검증된 제출 기본 최고 목표속도는 **3.0m/s**다. Monza 단계별 단일1랩과 같은 차량의 연속5랩을 확인한 뒤 적용했다. 연속 랩 시간(sim s): 460.773, 456.120, 456.225, 456.345, 456.240이다. 접촉0·주행 중 안전정지0이며, 공식 출발 일시정지 중 원시 timeout1회는 별도 기록했다. 최종 ZIP은 이 기본값으로 청정 재빌드·실제 주행을 다시 검증한다.

## 계산 성능 변경

- `TEAM04_OPTIMIZE_PLANNER=ON`은 `autonomy_04`와 대응 단위시험만 `-O2`로 빌드한다. `-ffast-math`·하드웨어 전용 플래그를 쓰지 않는다.
- RANSAC에서 같은 3개 표본 저장 공간을 재사용한다. 난수 표본 순서와 계산식은 같다.
- 실제 직선·급코너 점군에서 경로/명령 일치를 비교했고, 연산 지연과 실제 주행을 별도로 측정한다.
- 차량 물리 플러그인에는 이 최적화 옵션을 추가하지 않았다.

## 빌드

저장소 루트 기준:

```bash
source /opt/ros/jazzy/setup.bash
colcon --log-base src/team_04/artifacts/stage3/log build \
  --base-paths src/team_04/team04_race \
  --build-base src/team_04/artifacts/stage3/build \
  --install-base src/team_04/artifacts/stage3/install --merge-install
source src/team_04/artifacts/stage3/install/setup.bash
```

## 단일 및 연속 시험

시험마다 새 label을 지정한다. --duration은 실시간 최대 대기 한도이며, 목표 랩에 먼저 도달하면 종료한다.

```bash
PYTHONDONTWRITEBYTECODE=1 python3 -s src/team_04/team04_race/tests/autonomy_trial.py \
  --isolated --root . --track hungaroring --label stage3-hunga-new \
  --duration 1600 --autonomous --max-speed 1.2 --stop-after-laps 1
PYTHONDONTWRITEBYTECODE=1 python3 -s src/team_04/team04_race/tests/autonomy_trial.py \
  --isolated --root . --track monza --label stage3-monza-new \
  --duration 1800 --autonomous --max-speed 2.5 --stop-after-laps 1 --stop-on-active-failure
PYTHONDONTWRITEBYTECODE=1 python3 -s src/team_04/team04_race/tests/autonomy_trial.py \
  --isolated --root . --track monza --label stage3-continuous-new \
  --duration 4500 --autonomous --max-speed 2.5 --stop-after-laps 5 --stop-on-active-failure
```

최고 목표속도는 상한이다. 횡가속도·제동거리·관측된 경로 길이·실제 조향 곡률 때문에 실제 목표 속도가 더 낮을 수 있다. 미관측 경로를 가정해 가속하지 않는다.

`--start-sequence official`(기본)은 원본 공식 함수를 통해 5개 빨간불을 1초 간격으로 켠 뒤, world 일시정지·공통 Bool 허가·소등·재개를 수행한다. 실험의 소등 지연 .2초는 공식 허용 범위의 고정값이다. 참가 알고리즘은 이 순서나 운영 파일을 수정하지 않는다.

정상 시작 순서의 world 일시정지 동안 기존 wall-clock timeout이 0 목표를 낼 수 있다. 이 원시 카운터를 숨기거나 reset하지 않는다. `raw_stop_counter`와 출발 후 `active_safety_stops`를 분리한다. 출발 후 최초의 정상 TRACKING 이후 오류는 시각이 역행하더라도 주행 중 오류로 기록한다. 안전 timeout·감지 조건은 그대로다.

`--stop-on-active-failure`는 시험 관측자의 조기 종료 옵션이며, 참가 안전 로직을 바꾸지 않는다. 오류 상태를 기록한 뒤 외부 Bool false를 요청한다. 다음 최고속도로 진행하지 않는다.

## 자동 속도 단계

```bash
PYTHONDONTWRITEBYTECODE=1 python3 -s src/team_04/team04_race/tests/stage3_ladder.py \
  --root . --prefix stage3-new-sweep --speeds 1.5 2.0 2.5 3.0 --repeat-laps 5
```

각 속도의 실제 1바퀴·접촉0·주행 중 안전 정지0을 확인해야 다음 단계로 진행한다. 어느 단계라도 실패하면 중단한다. 600초 미만인 성공 설정 중 가장 빠른 설정으로 연속5바퀴를 시험한다. 같은 실행에서3랩/5랩 누적 체크포인트와 각 랩 시간을 기록하며 중간에 차량/노드를 재시작하지 않는다.

## 관측 및 사후 분석

```bash
PYTHONDONTWRITEBYTECODE=1 python3 -s src/team_04/team04_race/tests/analyze_stage3.py \
  src/team_04/artifacts/stage3/<label> --root .
```

JSON과 CSV에 랩별 시간, 실제 관측 속도, 엔코더 속도(진단), 조향·앞바퀴 각도, 경로 오차, 제동 시작 위치, RTF, CPU/RSS, 점군 지연·처리시간을 저장한다. pose·기준선 CSV·엔코더·접촉·world 통계는 관측자만 사용하고 주행 노드에 전달하지 않는다. 실제 경로와 Pure Pursuit 로컬 경로도 별도 기록한다.

`corner_comparison`은 사후 기준선 곡률 절대값 .04(1/m) 초과 구간을5m 이내 간격으로 병합하고 길이2m 이상만 남긴 비교표다. 코너 구간을 주행 노드에 제공하지 않는다. `braking_events`는 실제 속도 명령이 지속 하강한 구간의 시작 위치·속도·최저 엔코더 속도를 기록한다. 모든 작은 감속을 물리 코너라고 간주하지 않는다.

최종 시험 관측기는 매 물리 step의 접촉을 누적해100Hz로 전송한다. 짧은 접촉을 전송 간격 사이에서 놓치지 않도록 한 관측 개선이며, 참가 차량의 원본 모델·물리 구동은 그대로다. 시험 모델 복사본에만 이 계측기를 부착한다. 초기 단계별 단일 시험은 기존100Hz 표본 방식이었다.

CPU는 코어1개=100% 기준, RSS는 시험 자식 프로세스들의 합으로 공유 메모리를 중복 포함할 수 있다. 순간 RTF와 sim/wall 비율을 구별한다. 정지 이동거리는 Bool false 직전과 정지 관측 후의 실제 pose 차이이며 약0.1초 pose 표본 간격의 오차가 포함된다. 0 명령 발행과 즉시 물리 정지는 다르다.

반복 랩의 각 시간은 출발/이전 랩 시각과 독립 관측자의 코스 진행률 교차 시각 차이다. 기준선 표본·pose 표본 오차가 있으므로 공인 밀리초 계측이 아니다. 공식 예선은2바퀴, 기본600초는 운영자 프로세스의 실시간 제한이다. 자체1바퀴600초 결과로 공식 합격을 주장하지 않는다.

## 최종 ZIP에서 기본 설정 검증

빈 별도 디렉터리에 최종 ZIP만 풀어 빌드하고 그 install을 source한다. `tests/release_check.py`는 참가팀 내부 fixture에서 원본 공식 validator를 호출하고 ZIP·소스·모델 해시와 설치 리소스를 검사한다. 실제 주행은 설치된 manifest의 인자를 그대로 사용한다:

```bash
PYTHONDONTWRITEBYTECODE=1 python3 -s src/team_04/team04_race/tests/autonomy_trial.py \
  --isolated --root . --track monza --label stage3-release-new \
  --entry-defaults --duration 1800 --stop-after-laps 1 --stop-on-active-failure
```

`--entry-defaults`는 속도·config override와 함께 쓸 수 없다. 원본 운영자의 launch 인자 생성 함수를 사용한다. run_config.json에는 실제 install prefix·실행파일·모델·관측기 해시, result.json에는 실행한 인자를 기록한다. 기본 설정을 확인하는 이 시험도 공식 계측 인증과 구별한다.

고속 설정의 기존 안전 회귀 시험은 시험 전용 ROS 도메인97에서 수행한다:

```bash
PYTHONDONTWRITEBYTECODE=1 python3 -s src/team_04/team04_race/tests/autonomy_safety.py \
  --max-speed 3.0 --output src/team_04/artifacts/stage3/autonomy_safety-new.json
```

이 합성 ROS 시험은 충분히 가속한 뒤 수신 중단 등의0명령을 검사한다. 실제 Gazebo 차량의 정지 이동거리는 별도 주행 결과에서 확인한다.

## 1.2m/s 복귀

중앙이 world·차량·clock을 제공한 상태에서:

```bash
ros2 launch team04_race race.launch.py driving_mode:=autonomous \
  autonomy_max_speed:=1.2 \
  autonomy_config:="$(ros2 pkg prefix --share team04_race)/config/autonomy_stage2_rollback.yaml"
```

정확한 과거 소스는 `artifacts/stage3/baseline_stage2_source.zip`과 보존된 `artifacts/stage2/team04_stage2.zip`에 있다. 복구할 때는 새 별도 디렉터리에 풀어 빌드하며 기존 결과를 덮어쓰지 않는다. 자세한 실측과 최종 제출 기본값은 `src/team_04/REPORT_STAGE3.md`에 기록한다.
