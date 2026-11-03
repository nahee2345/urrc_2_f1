# TEAM01 v5 출발 실패 수정 및 검증

**제공된 현장 로그:** `RGB=9 intensity=20544 gate=WAIT_RED ready=False`
가 `RED 1/5`부터 `LIGHTS OUT`까지 바뀌지 않았음. 카메라의
빨간 물체 전역 카운트/초기 밝기 기준값이 실제 신호등 상태를 반영하지 않음.
추가로 `road=False black=0.00`으로 시각 도로 인식 결과가 거짓이므로,
신호등 판정만 바꾸면 차량이 여전히 정지할 가능성이 존재함.

**v5 변경:**
- `signal_vision.py`: 초기 암영상을 기준으로 한 RGB 차영상,
  적색 증가량, 수평 점등 배열의 독립 점멸 감지.
- `model.sdf`: start camera 상향 및 화각 변경. 차량 외형/색상/바퀴 유지.
- `planning.py`: 첫 번째 가시 조명이 4개인 경우에도 4→0 판정 허용.
  단, 미점등/짧은 플리커는 출발하지 않음.
- `node.py`: 상태 출력 강화(`permission`, `calibrated`, `peak`, `reason`),
  현장 카메라 PNG 저장 기능.
- `planning.py`: RGB 도로 인식 실패 시에도 양쪽 벽을 LiDAR가 연속 확인하면
  최고 0.75m/s의 느린 경계 주행 허용. 미확인 경계는 정지.

**검증 유형:** Python 오프라인 단위/모의 테스트. ROS 2/Gazebo 실행 환경은
현재 제작 환경에 없으므로 실제 신호등 영상, 차륜 구동 및 트랙 완주
검증은 통과 판정하지 않았음.
