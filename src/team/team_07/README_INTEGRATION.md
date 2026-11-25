# TEAM07 URRC F1 연결 패키지

원본 hybrid_racer.py 주행 알고리즘은 수정하지 않았습니다. ROS2 패키지, 공식 등록 JSON, launch, SDF 생성본만 추가했습니다.

- 설치: `unzip -o team_07_compatible.zip -d ~/urrc_2_f1_next/src/team/`
- 실행 전 `bash race_v24.sh check && bash race_v24.sh sync && bash race_v24.sh prepare`
- 단독 테스트: `bash race_v24.sh rehearse qualifying --team team_07 --track monza --seconds 600 --start-mode camera`

주의: 실제 빌드/스폰/출발/완주는 미검증입니다. 원본 모델의 `base_link_07`은 기하 중심이 아닐 수 있어 공식 geometry 검수에서 별도 수정이 필요할 수 있습니다. 원본 컨트롤러는 레이스 모드 0.3 m/s 상한을 강제합니다. 원본 실행 시 `gz.transport13`, `gz.msgs10` 파이썬 바인딩이 필요합니다.
