# URRC 대회 맵·관전 카메라 실행 방법

**환경:** Ubuntu 24.04 / ROS 2 Jazzy / Gazebo Harmonic

## 1. 다운로드

처음 설치하는 경우:

```bash
cd ~
git clone --recurse-submodules https://github.com/nahee2345/urrc_2_f1.git
cd ~/urrc_2_f1
source /opt/ros/jazzy/setup.bash
```

이미 다운로드한 경우:

```bash
cd ~/urrc_2_f1
git pull --ff-only
git submodule update --init --recursive
source /opt/ros/jazzy/setup.bash
```

> GitHub의 ZIP 다운로드에는 서브모듈의 실제 맵 파일이 포함되지 않습니다. 위 `git clone --recurse-submodules`를 사용하세요.

## 2. 맵 실행 — 터미널 1

```bash
cd ~/urrc_2_f1
source /opt/ros/jazzy/setup.bash
```

다음 중 한 가지 명령어를 실행하세요. 신호등 5개가 순서대로 켜지고 동시에 꺼집니다.

| 맵 | 명령어 |
| --- | --- |
| Monza | `bash run_v10.sh monza` |
| Spa | `bash run_v10.sh spa` |
| Monaco | `bash run_v10.sh monaco` |
| COTA | `bash run_v10.sh cota` |
| Hungaroring | `bash run_v10.sh hungaroring` |
| Baku | `bash run_v10.sh baku` |
| 랜덤 맵 | `bash run_v10.sh random` |

## 3. 관전 카메라 실행 — 터미널 2

맵 실행 후 별도 터미널에서:

```bash
cd ~/urrc_2_f1
source /opt/ros/jazzy/setup.bash
bash watch_v10.sh monaco
```

`monaco` 대신 실행 중인 맵 이름을 입력하세요. 랜덤 맵을 실행한 경우 `bash watch_v10.sh random`을 사용하면 최근 실행된 맵의 카메라가 열립니다.

| 키 | 관전 화면 |
| --- | --- |
| `U` | 출발·결승선 |
| `I` | 전체 코스 |
| `O` | 두 번째 구간 확대 |
| `P` | 세 번째 구간 확대 |
| `Q` | 관전 종료 |

맵 종료: 맵 실행 터미널에서 `Ctrl+C`.
