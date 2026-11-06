# TEAM01 개발팀 인수인계: 원본 Monza 보존 / 7초 출발 / 물리시간 거리 보정

기준: 2026-11-06. **코드 변경 내역이며 독립 QA PASS나 Monza 완주를 의미하지 않습니다.**
개발팀은 검증·Gazebo 실주행을 수행하지 않습니다.

## P0 원본 맵 불변
- GitHub 루트에서 원본 트랙은 `upstream/urrc_f1_ws` 서브모듈 SHA `a23c7eec41b98b67ec3a3a2a46f9d7ed957f8290`을 그대로 참조합니다. 저장소 원본 맵을 덮어쓴 개발 커밋은 없습니다.
- 과거 `enhance_track(..., "monza")`는 원본 `worlds/monza.sdf`, `worlds/monza_race.sdf`, `models/urrc_monza/model.sdf`를 현장에서 수정하고 PNG·OBJ·신호등 출력을 생성했습니다.
- 변경 후 로컬/CI 실행은 **서브모듈 HEAD의 원본을 `git archive`로 읽어 임시 런타임 디렉터리를 구성**하고, 그 복사본에만 `enhance_track`을 적용합니다.
- 로컬 실행 시작 시 `scripts/team01_restore_original_monza.sh`가 과거 실행에 의해 변경된 추적 Monza SDF 3개를 서브모듈 HEAD로 되돌리고, 해당 함수가 생성한 *정확한 이름*의 불추적 에셋만 정리합니다. 다른 맵·원본 커밋·다른 사용자 파일은 수정하지 않습니다.
- 사용자가 복구만 할 때: `bash scripts/team01_restore_original_monza.sh`. CI에서는 GitHub 소스 트리를 복구하지 않고 원본에서 복사본을 생성하여 사용합니다.
- QA는 실행 전후 `git -C upstream/urrc_f1_ws status --short`, 서브모듈 SHA, 원본 SDF blob SHA가 불변인지 구분해 보고하십시오.

## P1 신호등 미인식 시 스폰 7초 타임아웃
- **진짜 빨간불 인식 없이** 스폰 완료 후 실시간 7초가 지나면 `/team01/start_fallback` Bool이 True가 되어 controller와 adapter가 같은 허가 상태를 봅니다.
- 로컬·CI 스폰 스크립트는 `TEAM01_SPAWN_MONOTONIC` 환경 변수를 스폰 완료 시 설정합니다. 독립 운영자 스폰에서 해당 변수가 없으면 **첫 Gazebo Odometry 수신 시각**을 대체 기준으로 사용합니다.
- 빨간불 점등이 감지되었다면 자동 타이머를 중단하고 실제 소등을 기다립니다. 정상 `LIGHTS_OUT`은 기존 카메라 절차를 사용합니다.
- 타이머 허가는 **차량 진행 허가**일 뿐이며, Depth 유효성·LiDAR·도로 안전 경계·장애물 제동 조건을 통과하지 못하면 계속 정지합니다.
- 신호 RGB 영상 또는 카메라 보정이 실패한 경우 로컬/CI 스크립트는 즉시 비정상 종료하지 않고 타이머 경로를 기록합니다. 해당 실행을 **CAMERA RED→OUT PASS로 간주하면 안 됩니다.**
- `/team01/debug/start`: `spawn_elapsed_s`, `fallback_active`, `start_source`=`CAMERA` 또는 `NO_RED_7S`. `/team01/debug/adapter`: `race_allowed`, `fallback_allowed`, `sent_linear_mps`로 양쪽 허가를 확인합니다.
- 신호등 연출이 스폰 후 7초보다 늦게 시작되는 경우 **타이머가 먼저 작동할 수 있습니다.** 실제 대회 규칙과 운영자 신호 타이밍이 충돌하는지 검증팀이 확인해야 합니다.

## P2 거리 추정 33.4m 대 실측 6.868m
- 기존 내부 `LapLearner.integrate`는 ROS callback의 `time.monotonic()` 간격(벽시계)로 속도를 적분하므로 소프트웨어 렌더링이 느리면 Gazebo 물리 시간에 비해 거리를 크게 누적할 수 있었습니다.
- 새 `node.py`는 ROS `/clock` 기준 **시뮬레이션 경과시간**을 얻어 `SafetyPlanner.step(distance_dt=...)`에 보냅니다.
- 초기 `/clock` 미수신/멈춤 시 물리 경과시간을 0으로 적용합니다. 실제 Odometry 속도가 없으면 **명령속도를 주행거리로 적분하지 않습니다.**
- 센서 시간 보정 및 경로 예측에도 동일한 물리시간을 사용하고, `road_belief.py`/`reference_scheduler.py`에서 시계가 멈췄을 때 허위 최소 이동을 적용하지 않습니다.
- `/team01/debug/path.odom_sim_dt_s`, `odom_speed`, `odom_yaw_rate`, `lap_distance_est`와 Gazebo world Odometry 실제 거리 및 타임스탬프를 **동일 시각**에 비교해야 합니다. 약 33.4m 추정 오류가 이것만으로 해결됐다고 주장하지 않습니다.

## QA팀에 요청하는 별도 판정
1. 원본 맵 SHA/파일 상태 보존, SDF 임시복사본으로 정상 Gazebo 적재, 기존 400줄 제한.
2. 새 v31 스폰7초 신호없음, 소등 인정, 빨간불 검출 중 타이머 금지, adapter fallback 권한 동기화.
3. 새 v32 Gazebo 물리시간 기반 적분, 시계 정지·Odometry 부재, 33.4m→6.868m 괴리의 최초 발생 지점.
4. 검증팀 소유 `qa_team01_frame_test.py` 좌표계 기준 오류는 QA팀이 자체 수정. 차량 모델에 QA 판정기를 맞추기 위한 코드/맵 변경 금지.
5. 경계 **안전 여유 위반**, base_link **실제 도로 밖 이탈**, **0.3m 공식 실격**을 별도 기록하십시오. 기존 6.868m 후 안전 여유 위반은 알려졌으나 공식 0.3m 실격을 뜻하지 않습니다.
6. 도로 좌우 경계/B EV, adaptive 후보, 실제 yaw-rate/조향, 첫 코너·긴 직선·1랩 완주 확인. 이전 178/178 단위 PASS를 최신 개발 커밋의 PASS로 재사용하지 마십시오.

**검증·실주행은 QA팀 전담이며, 개발팀은 QA 실패 보고를 받아 차량 코드만 수정합니다.**
