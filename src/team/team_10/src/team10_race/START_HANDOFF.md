# 출발·정지 인계 — 2026-11-23

> 아래는 출발 수정 종료 당시의 기록이다. 이후 주행·속도 설정은 변경됐으며 현재 상태는
> [SPEED_HANDOFF.md](SPEED_HANDOFF.md)를 따른다. 출발/false/영상 단절 안전 코드는 이번 속도 작업에서 변경하지 않았다.

- 워크스페이스 ~/urrc_2, 소스 src/team/team_10, 실행 install_f1/team10_race.
  ROS2 Jazzy / Harmonic, ROS_DOMAIN_ID=42. 각 터미널은 tools/f1_env.bash를 별도 source.
  GZ_PARTITION은 새 원래 preview에서 읽는다. 현재 시험 프로세스는 모두 종료됐다.
- 주행 코드·속도4.0/1.5/1.0m/s·차량 SDF는 이전 run05와 같다.
  이번 변경: 최근 dark 기준 선택(start_vision.py), 연속 신선도/영상 안정화 준비 검사
  (check_f1.py), 선택적 카메라 계측과 짧은 시험 기록 도구·회귀 테스트.
- 최종 코드 독립 출발2회 완료: final02는 주행 중 false로 실제 정지, final03은
  영상 브리지 단절로 정지하고 복구 후28.76wall초 자동 재출발 없음. 183 tests PASS.
  속도 향상·전체 랩 재시험은 하지 않았다. 완주 증거는 이전 bend_20261122/run05다.
- 증거: validation_f1/start_repeat_20261122/REPORT.md, short_results.json,
  baseline_replay.json, unchanged_driving.json, final_source_sha256.json, cleanup.json.
  final02/·final03/에 실제 RGB, 생성 stamp/수신·처리 시각, 명령·pose·정지 결과가 있다.
  diag01은 상류 지연 계측, final01은 수정 완료된 계측 JSON 예외 실패이며 성공 횟수 제외.
- 남은 문제: Gazebo 직접 영상부터 최대 약10s 공백이 생기는 일시 정체의 물리/렌더/
  전송 세부 원인은 미분리. 안전 차단은 정상. 기존 첫 실패에는 RGB가 없어 정확한
  픽셀 원인은 확정 불가하나 실제 영상의 오래된 기준 선택 반례는 수정 전후 재현됐다.
- 정상 명령은 RUN_GUIDE.md. 기록기를 준비한 뒤 READY 직전에 check_f1.py를 다시
  실행해 PASS를 확인한다. NOT READY이면 READY를 보내지 말고 생성/수신 회복 확인.
  검사 한도20s, camera age0.25sim초/0.35wall초, lease0.25wall초는 그대로다.
- 짧은 계측: 차량 launch 터미널에서 HANLA_START_AUDIT_DIR를 새 폴더/camera로
  export한 뒤 기존 camera launch 실행. 다른 터미널에서 동일 env를 source하고
  python3 src/team/team_10/tools/record_start.py --output 새폴더 실행.
  native camera_timing 바이너리는 tools/build_f1.sh가 함께 빌드한다. 미설정 시 영상
  저장은 비활성이다. 정지는 permission.py false 후 실제0 확인, 기록기→차량→preview 종료.
- 다음 최소 원인 시험은 정지 스폰1회에서 native world stats 및 팀 System
  Pre/PostUpdate wall 시각과 직접 image 수신을 맞추는 것. 안전 timeout 변경/GO 우회나
  동일 조건 신호 재시도, 추가 랩은 필요 없다.
