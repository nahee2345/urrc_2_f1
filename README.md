# URRC F1 통합 대회장 — 운영 안내

> **업로드 브랜치: `integration-test`**. 아직 `main`에 병합하지 않았습니다. `urrc_2_f1_next` 통합본을 기반으로 하며, 예선·본선·결승 전체 실주행 검증은 **미완료**입니다. 현재 TEAM08만 접수되어 있습니다.

## 환경과 다른 컴퓨터에서 복제

- Ubuntu 24.04 / ROS 2 Jazzy / Gazebo Harmonic
- 맵: `monza`, `spa`, `monaco`, `cota`, `hungaroring`, `baku`
- `~/urrc_f1_ws`, `~/urrc_22`, `/home/ww` 경로를 설치 전제로 사용하면 안 됩니다. 작업 폴더 이름은 자유롭게 지정합니다.
- Git에는 일반적으로 빌드 결과(`build/`, `install/`, `race/install/`), 실행 로그, 비밀정보를 포함하지 않습니다. 새 컴퓨터에서 종속 패키지와 빌드 작업은 별도 필요합니다.

```bash
sudo apt update
sudo apt install -y git ros-jazzy-ros-gz
cd ~
git clone --branch integration-test https://github.com/nahee2345/urrc_2_f1.git urrc_2_f1
cd ~/urrc_2_f1
source /opt/ros/jazzy/setup.bash
bash race_v24.sh sync
bash race_v24.sh prepare
```

`prepare`가 성공하면 다음 환경 설정을 같은 터미널에 적용합니다. `race/install/setup.bash`는 빌드 후 생성되는 파일입니다.

```bash
cd ~/urrc_2_f1
source /opt/ros/jazzy/setup.bash
source race/install/setup.bash
export GZ_SIM_SYSTEM_PLUGIN_PATH="$PWD/race/install/lib:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"
export LD_LIBRARY_PATH="$PWD/race/install/lib:${LD_LIBRARY_PATH:-}"
export PYTHONPATH="$PWD/race/scripts:$PWD/src/urrc_track_gazebo:${PYTHONPATH:-}"
```

## 참가팀 접수 및 차량 검수

참가팀은 `src/team/team_01`부터 `src/team/team_12`까지 팀별 폴더를 사용합니다. 현재는 `team_08`만 있습니다.

```bash
bash race_v24.sh sync
python3 race/scripts/team_sync_v7.py check --root "$PWD"
bash race_v24.sh prepare
bash race_v24.sh vehicles --team team_08
bash race_v24.sh inspect-all
python3 race/scripts/stage_team_gate.py qualifying --team team_08
```

각 단계 직전 정적 검수 (결과/진출자 확정 후 실행):

```bash
python3 race/scripts/stage_team_gate.py qualifying
python3 race/scripts/stage_team_gate.py semi_a
python3 race/scripts/stage_team_gate.py semi_b
python3 race/scripts/stage_team_gate.py final
```

`STATIC_PASS` 또는 `REVIEW`는 **Gazebo 완주 합격이 아닙니다**. `REVIEW`의 base_link 회전·복수 링크 형상 등은 운영자가 별도로 확인합니다.

## 공식 경기 운영

**현재 맵 배정**(`race/results/v3_maps.json`):

| 단계 | 맵 | 랩 |
| --- | --- | ---: |
| 예선 | Hungaroring | 2 |
| 본선 A | COTA | 3 |
| 본선 B | Baku | 3 |
| 결승 | Monaco | 5 |

맵 배정은 `refresh-maps` 실행 시 변경됩니다. 실제 경기 기록이 생긴 뒤에는 재추첨하지 마십시오.

```bash
bash race_v24.sh show-maps
# 최초 배정이 없고 공식 결과가 없는 경우에만:
bash race_v24.sh refresh-maps
```

**예선**: 참가팀마다 별도 실행합니다. 1번 그리드, 2랩. **본선 A/B**: 예선 순위에 따라 진출자를 자동 선정하며 각각 3랩. **결승**: 본선 결과와 운영자 승인 후 5랩.

```bash
bash race_v24.sh qualifying --team team_08
bash race_v24.sh semi_a
bash race_v24.sh semi_b
bash race_v24.sh final-approve --preview
bash race_v24.sh final-approve
bash race_v24.sh final
```

운영자 수동 제어(경기를 실행한 터미널과 별도 터미널에서):

```bash
bash race_v24.sh spawn team_08
bash race_v24.sh operator-status
bash race_v24.sh dnf team_08 --reason "주행 불가능"
bash race_v24.sh standings
```

운영 모드에 따라 수동 스폰 시점과 허용 팀이 다릅니다. 운영자 화면에서 **정확히 `READY`를 입력하기 전에는 신호등 점등 순서를 시작하면 안 됩니다.** 5개 빨간불 순차 점등 후 일괄 소등합니다.

## TEAM08: 6개 맵 예선 모의주행(실제 경기 기록과 분리)

6개 맵에서 동일한 **TEAM08 한 대 / 1번 그리드 / 예선 2랩 / 카메라 출발 모드**로 개별 시험합니다.

```bash
bash race_v24.sh rehearse qualifying --track monza       --team team_08 --seconds 900 --start-mode camera
bash race_v24.sh rehearse qualifying --track spa         --team team_08 --seconds 900 --start-mode camera
bash race_v24.sh rehearse qualifying --track monaco      --team team_08 --seconds 900 --start-mode camera
bash race_v24.sh rehearse qualifying --track cota        --team team_08 --seconds 900 --start-mode camera
bash race_v24.sh rehearse qualifying --track hungaroring --team team_08 --seconds 900 --start-mode camera
bash race_v24.sh rehearse qualifying --track baku        --team team_08 --seconds 900 --start-mode camera
```

각 테스트마다 Gazebo 화면에서 맵·차량·신호등을 확인한 후 터미널에 `READY`를 입력합니다. `READY`는 대소문자를 구분합니다.

결과는 `race/previews/v17_qualifying_*/report.json`에 저장됩니다. 공식 `race/results/qualifying.json`을 갱신하지 않습니다. Monaco의 이전 테스트는 출발·제어 토픽 수신에 성공했지만 **0랩 / 51번 체크포인트 / 시뮬레이션 시계 정지**로 완주 검증에 실패했습니다. 아직 여섯 맵 모두 성공한 것으로 간주하면 안 됩니다.

## 참가팀 추가 및 본선 A/B 검증

`src/team/team_XX` 폴더를 추가하고 `sync` → `prepare` → `vehicles` → 단계별 정적검수 순서로 검사합니다. 예선 결과가 확정되면 A/B 분리와 진출자 선발을 검사합니다. 현재 TEAM08만 있으므로 실제 다중 차량 동시 스폰/주행은 미검증입니다.

## GitHub 업로드와 정확한 복제 확인

로컬 `urrc_2_f1_next` 작업본을 GitHub `integration-test` 브랜치와 동기화하는 명령은 사용자 컴퓨터에서 실행해야 합니다.

```bash
cd ~/urrc_2_f1_next
git status --short
git fetch origin integration-test
git diff --exit-code HEAD origin/integration-test
git ls-files --others --exclude-standard
```

`git diff`가 0이어도 **ignored 파일**은 포함하지 않으므로 `git status --ignored --short`로 별도 확인합니다. `race/install/`과 컴파일 결과는 다른 컴퓨터에서 다시 빌드해야 합니다. `race/results/`, `race/previews/`에는 운영 기록·실행 로그 등이 존재하므로 업로드 범위를 신중하게 결정합니다.
