# URRC F1 — 다운로드 및 맵 실행

**지원 환경:** Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic  
**지원 맵:** `monza`, `spa`, `monaco`, `cota`, `hungaroring`, `baku`

## 1. 다른 컴퓨터에서 다운로드

ROS 2 Jazzy와 Gazebo Harmonic이 설치된 Ubuntu에서 터미널을 실행합니다.

```bash
sudo apt update
sudo apt install -y git ros-jazzy-ros-gz ros-jazzy-rqt-image-view
cd ~
git -c http.version=HTTP/1.1 clone --depth 1 https://github.com/nahee2345/urrc_f1_ws.git
cd ~/urrc_f1_ws
source /opt/ros/jazzy/setup.bash
```

GitHub 웹사이트에서 **Code → Download ZIP**으로도 내려받을 수 있습니다. ZIP을 사용하면 압축을 해제한 폴더에서 실행합니다.

## 2. 랜덤 맵 실행 (신호등 + 카메라)

**터미널 1 — 랜덤 맵과 5개 빨간 신호등 실행**

```bash
cd ~/urrc_f1_ws
source /opt/ros/jazzy/setup.bash
bash run_v10.sh random
```

**터미널 2 — 랜덤으로 선택된 맵의 관전 카메라 실행**

Gazebo 화면이 열린 뒤 별도 터미널에서 실행합니다.

```bash
cd ~/urrc_f1_ws
source /opt/ros/jazzy/setup.bash
TRACK=$(python3 -c 'import json, os; from pathlib import Path; p=Path(os.environ.get("XDG_STATE_HOME", str(Path.home()/".local/state")))/"urrc_f1"/"last_run.json"; print(json.loads(p.read_text())["track"])')
bash watch_v10.sh "$TRACK"
```

## 3. 지정 맵 실행 (신호등 + 카메라)

아래 예시는 `monaco` 맵입니다. 다른 맵을 실행하려면 두 명령어의 `monaco`를 같은 맵 이름으로 변경합니다.

**터미널 1 — 맵과 5개 빨간 신호등 실행**

```bash
cd ~/urrc_f1_ws
source /opt/ros/jazzy/setup.bash
bash run_v10.sh monaco
```

**터미널 2 — 관전 카메라 실행**

Gazebo 화면이 열린 뒤 실행합니다.

```bash
cd ~/urrc_f1_ws
source /opt/ros/jazzy/setup.bash
bash watch_v10.sh monaco
```

## 4. 카메라 키

| 키 | 화면 |
| --- | --- |
| `U` | 출발·결승선 |
| `I` | 전체 코스 |
| `O` | 두 번째 구간 확대 |
| `P` | 세 번째 구간 확대 |
| `Q` | 카메라 관전 종료 |

**카메라 전환:** 관전용 터미널에 포커스를 두고 키를 누릅니다.  
**신호등:** 맵 실행 시 자동 시작되며, 빨간불 5개가 순서대로 켜진 후 모두 꺼집니다.  
**종료:** 맵 실행 터미널에서 `Ctrl+C`를 누릅니다.


### 운영자 수동 스폰·DNF (v26)

공식 경기 실행 명령을 **첫 번째 터미널**에 실행한 뒤, **두 번째 터미널**에서 차량을 선택합니다.
운영자가 명령을 내려야 Gazebo에 차량이 스폰됩니다. 일반 Gazebo GUI 수동 모델 추가는
base_link 공식 계측 플러그인이 빠질 수 있으므로 사용하지 마십시오.

```bash
cd ~/urrc_22
bash race_v24.sh qualifying --team team_01  # 터미널 1: 예선 시작
bash race_v24.sh spawn team_01              # 터미널 2: 해당 팀 스폰
bash race_v24.sh dnf team_01 --reason "주행 불가능"  # 필요 시 수동 DNF
bash race_v24.sh operator-status            # 진행 상태
```

본선·결승도 각각 `semi_a`, `semi_b`, `final` 실행 후 동일하게
`spawn team_XX`, `dnf team_XX`를 사용합니다.
센서(카메라·LiDAR)와 base_link 계측 **연결 실패**만 개별 자동 DNF 처리하며,
제어기 종료·차량 정지·벽 충돌·제한시간 초과·기타 주행 불능 판정은 운영자가
수동 DNF 처리합니다. Gazebo 또는 중앙 시계 오류는 개별 DNF가 아니라 경기 무효입니다.

차량 스폰 요청의 `접수` 메시지는 Gazebo 실제 스폰 완료가 아닙니다.
실제 완료는 경기를 실행한 터미널에서 확인하십시오.

### v27 대회 운영 규칙 (예선 3랩·본선 3랩·결승 5랩)

- 11팀 정적 검진: `bash race_v24.sh inspect-all`; 실제 Gazebo 검증: `bash race_v24.sh inspect-all --live --track monza --seconds 120`
- 맵 배정: `bash race_v24.sh refresh-maps` / 확인: `bash race_v24.sh show-maps` (예선 Monaco 제외, 본선 A/B=COTA·Baku, 결승은 미사용 맵)
- 예선: `bash race_v24.sh qualifying --team team_01` (원하는 순서로 한 팀씩 3랩)
- 예선 순위: `race/results/qualifying_ranked/` 의 `rankings.txt`, `rankings.csv`, 순위별 JSON
- 본선: `bash race_v24.sh semi_a`, `bash race_v24.sh semi_b` (각각 3랩)
- 결승 진출 후보 확인: `bash race_v24.sh final-approve --preview`; A/B 각 상위 3팀 승인: `bash race_v24.sh final-approve`
- 결승: `bash race_v24.sh final` (5랩, 본선 동일거리 환산시간 순으로 그리드 배정)
- 각 경기 터미널에 약 2초마다 base_link 기준 잠정 순위, 앞차·선두와의 같은 체크포인트 통과 시간차를 표시합니다.
- 흰 출발선을 첫 번째 통과하면 1랩 진입, 두 번째 통과하면 1랩 완주·2랩째입니다.
- 별도 터미널에서 스폰: `bash race_v24.sh spawn team_01`; 수동 DNF: `bash race_v24.sh dnf team_01 --reason "충돌"`
- 예선 수동 DNF는 DQ(실격), 본선·결승 수동 DNF는 기록 후 차량 제거합니다. 센서·계측 연결 실패만 자동 DNF입니다.
- 같은 Wi-Fi 관전은 이번 버전에 구현하지 않았습니다.

