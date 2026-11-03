# TEAM01 v7 검증 보고서

- 자동화 테스트: Python `unittest` **71개 PASS**, 실패 0개.
- 추가 12개: GRID 고착 재현/탈출, 탐색 목표속도, 경계 미확인 GRID 유지, Depth 곡선 가변 조향 경로, 동적 코너 플래너 선택, 전방 충돌 거부, optional odom 거리 적분, 최종 목표 20 비활성, LiDAR 노후 데이터 정지, 도로 없는 경우 정지, 합성 곡률 변화 도로에서의 폐루프 주행.
- 합성 곡률 변화 도로: 간단한 bicycle 운동학에서 약 17m 전진, 도로 중심에서 최대 오차 약 0.20m, BRAKE 상태 0회. **실제 Gazebo 검증 아님.**
- 모델/환경 정적 점검: Python compileall, `gz sdf -p` 및 ROS2 launch 실제 실행은 이 환경에 ROS2/Gazebo가 없어 미수행.
- v6 현장 로그: 속도 명령 0.65m/s 상태에서 GRID 고착, 333.5m는 명령 적분 거리, 전방 약 3.4m 위험 추정 이후 후보 경로 없음으로 제동.
- 현장 필요 사항: 실제 `/team01/gz_odom`, `planner=adaptive_pursuit`, `curb_ahead`, `line_phase`, 제어속도/실속도/코너 진입, 1랩 완주/충돌 여부.
- 연석/앞차 오분류 추가 검증: 굽은 도로의 양쪽 연석 Depth 점이 영상 중앙으로 들어와도 `lead_distance`를 앞차로 기록하지 않도록 수정; 같은 점은 충돌 위험 판단에서 제외하지 않음.
