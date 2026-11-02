# URRC 맵 · 관전 카메라 실행

**지원 환경:** Ubuntu 24.04 / ROS 2 Jazzy / Gazebo Harmonic

## 1. 다운로드

기존 실험용 `~/urrc_2_f1` 폴더와 구분하여 맵 실행용 폴더에 받습니다.

```bash
cd ~
git clone --recurse-submodules https://github.com/nahee2345/urrc_2_f1.git urrc_2_f1_maps
cd ~/urrc_2_f1_maps
source /opt/ros/jazzy/setup.bash
```

이미 `~/urrc_2_f1_maps`를 받았다면:

```bash
cd ~/urrc_2_f1_maps
git pull --ff-only
git submodule update --init --recursive
source /opt/ros/jazzy/setup.bash
```

> GitHub의 `Download ZIP`에는 서브모듈의 맵 파일이 포함되지 않으므로 위 `git clone --recurse-submodules` 명령어를 사용합니다.

## 2. 맵 실행 — 터미널 1

```bash
cd ~/urrc_2_f1_maps
source /opt/ros/jazzy/setup.bash
```

아래에서 **한 개의 맵**을 선택합니다. 빨간 신호등 5개가 순차 점등된 뒤 동시에 소등됩니다.

| 맵 | 실행 명령어 |
| --- | --- |
| Monza | `bash run_v10.sh monza` |
| Spa | `bash run_v10.sh spa` |
| Monaco | `bash run_v10.sh monaco` |
| COTA | `bash run_v10.sh cota` |
| Hungaroring | `bash run_v10.sh hungaroring` |
| Baku | `bash run_v10.sh baku` |
| 랜덤 맵 | `bash run_v10.sh random` |

## 3. 관전 카메라 — 터미널 2

맵 실행 후 **새 터미널**에서 진행합니다.

```bash
cd ~/urrc_2_f1_maps
source /opt/ros/jazzy/setup.bash
bash watch_v10.sh monaco
```

`monaco`를 터미널 1에서 실행한 **동일한 맵 이름**으로 변경합니다.

| 키 | 화면 |
| --- | --- |
| `U` | 출발·결승선 |
| `I` | 전체 코스 |
| `O` | 두 번째 구간 |
| `P` | 세 번째 구간 |
| `Q` | 관전 종료 |

종료: 맵 터미널에서 `Ctrl+C`.
