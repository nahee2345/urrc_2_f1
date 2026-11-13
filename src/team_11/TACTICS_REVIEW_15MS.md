# TEAM 11 — 신호등 없이 주행·추월·방어 우선 점검

## 목표 속도 및 현황

- **직선 목표 최고속도 15m/s (54km/h)**
- 경로 계획기 최대속도 15m/s; 코너 횡가속도 계산 및 가감속 경사 제한 유지
- ROS→Gazebo stage5 어댑터 전진 상한 15m/s, 후진 상한 2m/s
- 타이어 반경 0.045m이므로 목표 15m/s에는 약 333.33rad/s 필요; 네 바퀴 관절 상한 400rad/s로 설정
- 이 값은 **명령 한계**이며 실제 Gazebo에서 15m/s 도달 여부는 계측 전에는 확정할 수 없음

## 빠른 오프라인 검증 (Gazebo·신호등·차량 스폰 불필요)

```bash
cd ~/urrc_2_f1 && git pull --ff-only origin main && PYTHONPATH=src/team_11 python3 -m unittest discover -s src/team_11 -p "test_*.py" -v
PYTHONPATH=src/team_11 python3 src/team_11/review_tactics.py
```

두 번째 명령은 기본 회귀시험과 독립적으로 13개 전술 시나리오를 검사하고 불일치를 FAIL로 표시합니다. 전체 13개 PASS가 되지 않으면 실전 완성으로 간주하지 않습니다.

## 미해결·미검증 사항

- 추월 임계 3초로 수정, 오프라인 재검증 필요
- 뒤차 방어 임계 5초로 수정, 오프라인 재검증 필요
- 뒤차 S자 궤적 추적과 연속 접근 검증 미구현
- 코너에서 2번 중앙 경로 목표 설정(경로 점유 시 보류) 추가, 실제 주행 검증 필요
- `ros_controller.py`의 `/cmd_vel_11`은 현 테스트 어댑터의 `/speed_11`, `/steering_11` 입력에 직접 연결되어 있지 않아 통합 제어 검증 필요
- 실제 물리 주행·벽 충돌·완주, 15m/s 도달, 차선 변경 안전거리 검증 필요

신호등 검증과 카메라 입력 검증은 위 주행·추월·방어 경로가 검증된 후 마지막으로 재개합니다.

## 공식 6개 맵 × 11개 그리드 × 3개 경로 검증

공식 `~/urrc_f1_ws/src/urrc_track_gazebo/tracks/processed`의 중심선 CSV·그리드 JSON을 사용합니다.
경로별 조향 명령, 초기 경로와 그리드의 좌우 정합성, 경로 안전 폭 추정치,
직선 최고속도 15m/s와 실제 각 맵 급커브 지점의 감속 명령을 198개 조합으로 검사합니다.

```bash
cd ~/urrc_2_f1
git pull --ff-only origin main
PYTHONPATH=src/team_11 python3 -m unittest discover -s src/team_11 -p 'test_*.py' -v
PYTHONPATH=src/team_11 python3 src/team_11/review_tactics.py
PYTHONPATH=src/team_11 python3 src/team_11/review_six_maps.py --official ~/urrc_f1_ws
```

결과: `~/urrc_2_f1/log/team11_six_maps_routes.csv`

이 테스트는 오프라인 경로 계획 검증이며 Gazebo 실제 주행·완주 또는 물리적 충돌 안전성을 입증하지 않습니다.
