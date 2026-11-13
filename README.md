# TEAM 11 — 6개 트랙 고정 좌표 경로 검증

**한 개 명령어만 실행합니다.**

```bash
cd ~/urrc_2_f1 && bash run.sh monza 1
```

- 첫 인자: `monza`, `spa`, `monaco`, `cota`, `hungaroring`, `baku`
- 두 번째 인자: **1번 오른쪽 / 2번 중앙 / 3번 왼쪽**
- 별도 차선 변경 기능 없음. 선택한 경로 **하나만** 따라 주행
- 6개 트랙의 공식 중앙선에서 **트랙별 3개, 총 18개 CSV 경로** 자동 생성·저장
- 모델 스폰은 항상 해당 트랙의 **공식 1번 그리드 X·Y 정중앙 및 Yaw**
- 출발 신호: 공식 5등 순차 점등 → 일괄 소등 확인 → 고정 경로 주행 시작
- 시작 검증 속도 **최대 2m/s**; 실제 물리식 Ackermann 스티어링
- 주행 및 경로 이탈 확인 CSV: `log/fixed_routes/<track>_lane<no>.csv`
- 최종 PASS/FAIL JSON: `log/fixed_routes/<track>_lane<no>.json`

경로는 `routes/<track>/lane_1.csv`, `lane_2.csv`, `lane_3.csv`로 저장됩니다.
모든 실행에서 경로 18개를 다시 생성해 공식 중심선과의 일치성을 유지합니다.
`src/team_11/models/team11_yellow_car/model.sdf` 차량 모델은 수정하지 않았습니다.

## 각 경로 단독 주행 예시

```bash
bash run.sh monza 1
bash run.sh monza 2
bash run.sh monza 3
bash run.sh spa 1
```

각 명령어는 독립적인 한 번의 실행입니다. 다음 실행 전 이전 Gazebo가 종료되어야 합니다.
`run.sh`가 최초 한 번 `~/urrc_f1_ws`의 6개 원본 트랙 메시와 신호 자료를
`src/urrc_track_gazebo`에 복사합니다. **원본은 변경하지 않습니다.**

이번 단계에는 추월, 방어, 차선 이동, SLAM, 신호등 영상 인식 등의 알고리즘은 없습니다.
신호등 소등 후 출발 허용은 공식 신호 제어 스크립트가 성공한 시점을 기준으로 합니다.
카메라/월드가 준비되지 않으면 출발하지 않습니다.

실제 Gazebo 1랩 완주와 물리적 경로 추종 성능은 아직 검증되지 않았습니다.
