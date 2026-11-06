# TEAM01 개발팀 P0 인수인계 — 늦은 신호등 7초 오출발 / Gazebo DRIVE_STALL

기준 2026-11-06, 대상 Issues #5·#6. **제작팀은 독립 QA와 Gazebo 실주행을 수행하지 않았습니다.** 모든 항목은 코드 반영 상태이며 PASS는 검증팀만 판정합니다.

## 출발 규칙 P0 — 오출발 차단
- 기존에는 스폰 후 빨간불이 아직 없다는 사실만으로 7초 뒤 `StartFallback.active`가 True였습니다. 정상 신호 시퀀스가 준비/카메라 보정 대기 중일 때 이미 전진 명령이 발생했습니다.
- 이번에는 `StartFallback.active(..., confirmed_no_signal=False)` 기본값을 안전 정지로 설정했습니다. `StartDiagnosticsMixin`은 **명시적** `race_start_mode=no_signal`에서만 `confirmed_no_signal=True`를 넘깁니다.
- **정상 경기(`camera`)는 7초든 21초든 RED→LIGHTS_OUT 전까지 fallback=False입니다.** 빨간불을 감지하면 타이머는 언제나 무효이고, 정상 카메라 소등+경기 허가를 우선합니다.
- 운영자가 **진짜로 신호등이 없는 별도 시험**임을 확인한 경우에만 `TEAM01_NO_SIGNAL_MODE=1`을 런처 환경에 설정하거나 `ros2 launch team01_race race.launch.py race_start_mode:=no_signal`로 선택합니다. 이 경우 스폰 완료→7초 타이머와 Depth/LiDAR/도로·장애물 제동 게이트가 적용됩니다. 일반 대회에서는 설정 금지.
- 정상 실행기의 신호 RGB/보정 실패가 자동으로 `no_signal`로 바뀌지 않으며, **오출발보다 안전 중단을 우선**합니다. 정상 카메라 인식 실패를 7초 타이머만으로 무신호 상태라고 판단하는 것은 원리적으로 불가능합니다.
- `/team01/debug/start`: `fallback_mode_authorized`, `start_mode`, `fallback_active`, `spawn_elapsed_s`, `start_source`; 어댑터 `/team01/debug/adapter`: `fallback_allowed`, `race_allowed`, `sent_linear_mps`.
- `test_v31_spawn_fallback.py`: 기본 대회 모드에서 7초/21초 대기, 늦은 빨간불 점등, 명시적 무신호 모드에서만 7초 타이머 허가, 빨간불 감지 시 타이머 무효를 구분합니다.

## DRIVE_STALL P0 — 벽시계/시뮬레이션 시간 혼용
- 실제 QA는 제어기/어댑터가 양의 구동 명령을 전송하더라도 실물 Odometry가 0.152~0.204m뿐이라고 확인했습니다. **실제 휠 동력 또는 충돌 원인은 아직 확정되지 않았습니다.**
- 기존 `adapter.py`는 속도·조향 명령에 대해 **벽시계 시간 0.3초**로 신선도 조건을 검사했습니다. Gazebo 소프트웨어 렌더링이 느리면 정상적인 20Hz *시뮬레이션* 제어에도 0.3초 이상 실제 시간이 지나 구동 명령을 반복 0으로 만들 수 있습니다.
- 수정: `command_freshness.py` 순수 모듈로 `/clock` **시뮬레이션 경과 0.32초 이내** + **벽시계 2.5초 이내**를 둘 다 요구합니다. `/clock`이 아직 준비되지 않았으면 종전 안전한 벽시계 0.3초 제한을 사용합니다. 시계가 뒤로 가면 즉시 stale 처리합니다.
- 어댑터 디버그에 `speed_sim_age_s`, `steer_sim_age_s`, `speed_wall_age_s`, `steer_wall_age_s`, `watchdog_clock`, `stop_reason`을 추가해 `gate` / `stale_speed` / `stale_steering`을 구분합니다.
- `test_v33_adapter_sim_watchdog.py`: slow-headless 사례(시뮬레이션 0.06초 / 벽시계 1.1초), 실제 stale clock, 벽시계 통신 끊김 3초, /clock 미수신, 시계 리셋, 명령 미수신을 구분합니다.
- 이 변경은 **의심 가능한 명령 인가 중단 요인**을 제거한 것이며, AckermannSteering joint/torque/접촉/초기 스폰 높이 등 실제 물리 이상은 QA의 독립 관측이 필요합니다.

## 변경 금지 영역
- `upstream/urrc_f1_ws` Monza SDF/트랙 원본, 맵 생성 파라미터, 트랙 색·높이·경계, QA 전용 판정기, 대회 0.3m 실격 규칙.
- 원본맵 파일은 임시 런타임 복사본에서만 시각화 자산을 생성하고, 기존 immutable 보호를 유지합니다.

## 검증팀 요청 항목 — 개발 커밋 이후만 검증
1. 기본 `camera` 모드: 스폰+7~30초 동안 RED 미발견/늦은 점등/RED4→0/소등 전 `sent_linear_mps` **모두 0**, 신호 소등과 허가 후에만 전진.
2. 명시적 `no_signal` 모드: 스폰+7.0초 이전 0, 7.0초 이후 도로/장애물 안전조건이 허용하면 구동; 이 실행을 카메라 인식 PASS로 판정하지 않기.
3. Gazebo 저실시간율에서 컨트롤러 → Float32 명령 → 어댑터 타임스탬프 → `/team01/cmd_vel` → Gazebo Ackermann plugin의 실제 4개 wheel spin/2개 steering joint, 접촉/슬립 → `/team01/gz_odom`을 **시뮬레이션 시간으로 동기화**. 양의 Twist만으로 휠 회전·물리 이동 PASS 불가.
4. 어댑터 `watchdog_clock=simulation` 중 `fresh_speed`, `fresh_steer`, `send_enabled`, `stop_reason` 각각의 비율과 명령의 0 전환 횟수.
5. 기존 오프라인 전체, 400줄, 원본 Monza 파일 hash 불변, 실제 5m 연속 주행, 첫 코너, OFF_TRACK 안전여유/실격 분리, Monza 1랩 및 rosbag.

**판정:** 출발 정책/명령 신선도 개선 소스 GitHub 반영. 실제 DRIVE_STALL 해결, 안전출발 PASS, Monza 1랩 완주 모두 미검증.
