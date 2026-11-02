# URRC 맵 · 관전 카메라 실행 방법

**환경:** Ubuntu 24.04 / ROS 2 Jazzy / Gazebo Harmonic

## 1. 다운로드 (최초 1회)

```bash
cd ~
git clone --recurse-submodules https://github.com/nahee2345/urrc_2_f1.git
cd ~/urrc_2_f1
source /opt/ros/jazzy/setup.bash
```

이미 다운로드했다면:

```bash
cd ~/urrc_2_f1
git pull --ff-only
git submodule update --init --recursive
source /opt/ros/jazzy/setup.bash
```

> GitHub 웹사이트의 Download ZIP에는 서브모듈 안의 맵 데이터가 포함되지 않습니다. 위 `git clone --recurse-submodules` 명령을 사용하세요.

## 2. 맵 실행 (터미널 1)

```bash
cd ~/urrc_2_f1
source /opt/ros/jazzy/setup.bash
```

원하는 맵 하나를 선택해 실행합니다. 5개 빨간 신호등이 순서대로 켜진 뒤 한꺼번에 꺼집니다.

| 맵 | 명령어 |
| --- | --- |
| Monza | `bash run_v10.sh monza` |
| Spa | `bash run_v10.sh spa` |
| Monaco | `bash run_v10.sh monaco` |
| COTA | `bash run_v10.sh cota` |
| Hungaroring | `bash run_v10.sh hungaroring` |
| Baku | `bash run_v10.sh baku` |
| 랜덤 | `bash run_v10.sh random` |

## 3. 관전 카메라 실행 (터미널 2)

맵이 열린 뒤 **새 터미널**에서 실행합니다.

```bash
cd ~/urrc_2_f1
source /opt/ros/jazzy/setup.bash
bash watch_v10.sh monaco
```

`monaco`를 터미널 1에서 실행한 맵 이름으로 변경하세요.

카메라 전환은 관전 카메라 터미널에서 키를 입력합니다.

| 키 | 기능 |
| --- | --- |
| `U` | 출발·결승선 |
| `I` | 전체 코스 |
| `O` | 두 번째 구간 |
| `P` | 세 번째 구간 |
| `Q` | 관전 종료 |

맵 종료: 터미널 1에서 `Ctrl+C`.
