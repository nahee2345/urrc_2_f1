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
