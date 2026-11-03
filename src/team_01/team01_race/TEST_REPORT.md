# TEAM01 v4 검증 기록 — 2026-11-02

- `python3 -m unittest discover -s tests -v`: **39/39 PASS**
- `python3 -m compileall -q team01_race launch`: PASS
- `bash -n scripts/run_local_monza.sh`: PASS
- `xml.etree.ElementTree.parse(models/team01_car/model.sdf)`: PASS
- 단위 시험: 빨간불 4→0, 점등 전 검은 영상 불출발, 깜빡임 방지, 3개 불출발, 도로/비도로 컬러 분리, 평지·경사 지면, 좌우 경로, 급박한 차량 제동, 추정 상대속도, 후방 안전, 출발 차단, 노면 경계 제한.
- 가상 운동학 시험: 사인 곡선 도로 따라 150스텝, 노폭 1.6m 내에서 주행 후보를 반복 생성, 물리 충돌은 시뮬레이션하지 않음.
- **미검증:** 실제 Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic에서 실행/출발/센서 영상 확인, 전체 트랙 완주, 경쟁 차량과 동시 추월, 무충돌 보증.

이 기록은 실제 Gazebo 실험의 PASS 판정이 아닙니다. 실제 실행 로그 검증이 필요한 상태입니다.
