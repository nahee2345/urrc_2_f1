# URRC F1 v24 — 대회용 전체 워크스페이스

지원 맵: monza, hungaroring, monaco, spa, cota, baku

- ROS 2 Jazzy + Gazebo Harmonic은 별도 설치가 필요합니다.
- src/team/team_01~team_12는 빈 폴더입니다. 참가자는 자기 팀만 채우십시오.
- 팀별 ROS 패키지명과 속도·조향·카메라·LiDAR 토픽을 고유하게 작성하십시오.
- 두 배포본의 차량 규격·제어·카메라 출발 인터페이스 및 검수기는 동일합니다.
- 참가자에게 연습용만 제공하고, 대회장에서는 대회용을 사용하십시오.
- 공식 예선·본선 A/B·결승은 v23의 6맵 공통 호환성 검사를 선행합니다.
- 실제 6개 맵 완주 여부는 참가팀 알고리즘 성능에 따라 달라집니다.

```bash
cd ~/urrc_22
bash verify_integrated_v24.sh
bash verify_integrated_v24.sh --gui
bash race_v24.sh preview qualifying --track monza
bash race_v24.sh qualifying team_01
```

사용하는 PC마다 소스 빌드, ROS 환경 설정 및 Gazebo 실행 검증이 필요합니다.
