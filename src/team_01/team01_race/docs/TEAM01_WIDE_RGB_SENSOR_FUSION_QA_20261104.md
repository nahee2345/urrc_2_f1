# TEAM01 넓은 RGB·Depth·LiDAR 융합 및 단계적 주행 개선 인수인계

검토 기준: 2026-11-04. 이 문서는 **코드 구현 내용**이며 실제 Monza 완주 PASS가 아닙니다.

## 현재 이미 구현된 요소
- 도로 RGB 105°(신호 RGB는 독립), Depth 약 92°. `rgb_depth_projection.py`에서 서로 다른 화각을 정렬합니다. 센서 모델의 실제 값과 YAML/기본값의 일치 여부도 검증해야 합니다.
- 아스팔트의 청회색 영역 + Depth 지면점 결합: `road_surface.py`.
- 적백색 경계벽의 색상과 높이점 및 LiDAR 공간 일치 **진단**: `road_node.py` / `rgb_depth_projection.py`. 색상 검출만으로 안전 주행 허가를 부여하지 않습니다.
- 센서 기반 차량 중심 BEV 그리드, 미관측 영역 = unknown: `bev_grid.py`, `/team01/debug/bev`.
- 가까운/중간/먼 영역의 곡률·폭 진단: `road_roi.py`.
- 차량 이동·yaw-rate 보정 과거 경로 융합: `road_belief.py`.
- 점진적 5차 경로와 Stanley 조향, 직선 데드밴드: `reference_path.py`, `reference_scheduler.py`, `stanley_control.py`.
- MPC는 `enable_shadow_mpc=false` 기본값으로 실제 액추에이터와 분리된 평가 기능입니다. 학습된 이득도 기본 비활성입니다.

## 이번 개발팀 변경
- `road_belief.py`: RGB·Depth 경로와 LiDAR 경로가 0.35m 이상 불일치하고, **두 결과 모두** ego-motion 보정 이전 경로에서 0.35m 넘게 벗어난 경우에는 신뢰할 수 있는 새 도로가 없다고 판정합니다. 한 센서만 튀면 나머지 센서와 이전 측정 일치도를 우선합니다. 실제 코너처럼 두 센서가 같은 방향으로 바뀌는 경우는 이 조건에 해당하지 않습니다.
- `planning.py`: 위 충돌이 발생하면 그 틱에서 안전 정지, 오래된 기준경로 폐기, 조향 제한기 초기화. 이전 경로가 도로 존재를 증명하지 않도록 합니다.
- `node.py`: 과거 `max(road_at,ground_at)` 때문에 한 센서가 오래되어도 결합 경계를 쓸 수 있던 검사를 각각의 freshness와 동기화 조건으로 교체했습니다.
- `/team01/debug/path`에 `road_disagreement_m`, `road_innovation_m`, `road_conflict_reason` 추가.
- `test_v25_bev_road_belief.py`: 두 센서 상충 시 안전 정지 전제 및 한 센서 이상치 회귀시험 추가.

## 검증팀 P0
1. 현재 최신 SHA 기준 전체 오프라인 회귀시험. 직전 테스트에서 `test_corner_selects_adaptive_not_constant_arc` 1건이 FAIL했습니다. **후보 거절 원인을 밝혀야 하며 테스트 기대값만 변경 금지**.
2. 실제 카메라 105° 화면 위에 도로 마스크(청회색), 적백색 벽, Depth 정렬점, LiDAR 대응점 및 좌/우 경계·생성 경로가 일치하는지 시각화하여 확인.
3. `/team01/debug/path` 20Hz에서 ROI 유효성, 센서간 거리 오차, 기준경로 생성, `adaptive_pursuit`/ `constant_arc`, BRAKE 원인, yaw-rate 및 조향 부호 전환 기록.
4. 센서 단독 잡음, 두 센서 동시 실제 코너 감지, 양쪽 불일치, 오래된 RGB, 오래된 Depth를 각각 재현해 정지/복귀 확인.
5. 신호등 4→0 카메라 검출, 별도 출발 허가, ROS/Gazebo 구동 명령, odom 실제 이동과 1랩 거리를 각각 확인. 카메라가 점등/소등을 검출하지 않았는데 출발시켜 PASS 처리 금지.
6. 5m/s 미검증 상태에서 8/10m/s 활성화 금지. 장거리 실주행과 첫 코너 진입/연석 거리 전후가 핵심입니다.

## 개발 판정
- 기존 구현 재사용 + 센서 충돌 보호 및 입력 freshness 개선: 소스 반영.
- 기존 빠른 오프라인 QA는 1건 코너 회귀 FAIL, 실제 Gazebo 검증 결과는 별도 확인 필요.
- 새로운 MPC나 RL을 메인 제어로 전환하지 않았습니다. 사고를 줄이는 것보다 검증된 경로를 유지하는 것이 우선입니다.
