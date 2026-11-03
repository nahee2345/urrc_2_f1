# TEAM 01 v12 실제 주행 수정

- 적용 저장소: nahee2345/urrc_2_f1 (기존 urrc_2 미변경)
- 근거: Actions 41560616915, lap.json 실제 이동거리 0m. 카메라 RGB 1→3→4→0, 게이트 WAIT_RED 유지.
- 출발: 점진적으로 켜진 2종 이상의 빨간 렌즈 수 + 전체 4개 이상 검출로 1프레임 ARMED 허용. 단독 4개 플래시 차단, 3초 이상 점등 순서 확인 및 연속 3프레임 소등 검증 유지.
- 센서: RGB/Depth 각각의 비동기 수신 순서를 처리하고 실제 속도에 따라 허용 정합 시간을 제한. LiDAR·Depth 유효성은 계속 필수.
- 주행: Nav2 Regulated Pure Pursuit의 속도별 전방 주시 거리, 곡률 기반 감속 및 전방 TTC 속도 제한을 실제로 관측된 국소 도로 안에서 적용. 사전 트랙 경로를 차량 제어에 제공하지 않음.
- 속도: 첫 랩 직진 목표 8m/s, 두 번째 랩 목표 10m/s. 코너·위험 구간 감속과 신선도 조건이 우선.
- CI: 카메라 미출발 65초 경과 시 SIGNAL_GATE_STALL 보고. OFF_TRACK, DRIVE_STALL, LAP_TIMEOUT은 모두 FAIL.
- 오프라인: 111개 테스트 통과. 실제 Monza 완주는 GitHub Actions 재검증에서만 판정 가능.
- 연구 출처: https://github.com/ros-navigation/navigation2/tree/main/nav2_regulated_pure_pursuit_controller ; https://github.com/f1tenth-dev/pure_pursuit
