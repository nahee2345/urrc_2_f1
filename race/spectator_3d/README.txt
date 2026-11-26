URRC MONZA WebGL spectator prototype v1

운영 PC: Ubuntu 24.04, ROS2 Jazzy, Gazebo Harmonic, Git, Node/npm, colcon,
Python aiohttp/numpy, Gazebo transport13 / msgs10 Python bindings 필요.
설치 전 ROS 환경: source /opt/ros/jazzy/setup.bash
필요시: sudo apt install git npm python3-aiohttp python3-numpy python3-colcon-common-extensions
Gazebo Python 모듈이 없다면 기존 Gazebo 설치 방식에 맞는 바인딩 패키지를 설치해야 합니다.

1. unzip 파일을 Downloads에 풀고 bash ~/Downloads/urrc_spectator_v1/install.sh
2. bash ~/urrc_2_f1_next/spectator/run.sh
3. 운영 PC에서 맵/차량 확인 후 READY 입력. 표시된 관전 주소를 동일 LAN 브라우저에 입력.
4. 왼쪽 TEAM_08 클릭. 휠로 Follow 확대, 버튼으로 전체맵/상공/전체화면.
5. Ctrl+C 종료. 로그 ~/urrc_2_f1_next/logs.

원본 저장소는 임시 디렉터리에 다운로드만 하며 push하지 않습니다. 기존 ~/urrc_2를 참조하지 않습니다.
새 폴더만 작성하며 기존 폴더가 있으면 중단합니다. 설치 도중 실패한 폴더는 자동 삭제하지 않습니다.
새 폴더는 원본 저장소의 Git 메타데이터를 복사하지 않습니다.
운영 PC에서만 Three.js 설치. 관전 PC에서는 별도 설치 불필요.
LAN 관전 포트 8765/TCP. GZ_IP=127.0.0.1, 별도 GZ_PARTITION 사용.
이 테스트에는 공식 경기 운영 시스템을 포함하지 않습니다. 원본 signal-only team_08 플러그인을 직접 실행하고
원본 신호등 스크립트의 일괄 소등 성공 이후에만 native start topic을 발행합니다. 공식 출발 자격 검증이 아닙니다.

제한: 실제 Monza OBJ/텍스처 + SDF 시각 형상으로 렌더링하되 Gazebo와 동일한 조명/동적 신호등/센서 효과는 미구현.
카메라 뒤 8m, 높이 6m, 전방 5.5m 응시, FOV 50도(초기값). 실제 캡처 화면과 비교해 조정 필요.
WebSocket 송신 최대 목표 60Hz, 85ms 보간. 실제 pose publish rate 및 FPS는 실환경에 따라 다름.
HUD 순위/랩/격차는 중심선 진행도 기반 잠정치. 3랩 표시이며 자동 종료 및 공식 랩 판정은 아직 미구현.
단독 TEAM08 테스트에서 앞뒤 차량 격차는 —. 다른 차량은 외형 모델이 아직 없어 동일 템플릿 사용.
Python/JavaScript/Bash 문법 검사만 완료. 실제 ROS/Gazebo 빌드, 원격 브라우저, 3랩, FHD 60FPS 검증은 미실행.
