# URRC 공식 6개 트랙 + 5등 신호등 (주행 로직 없음)

현재 차량 파일: `src/team_11/models/team11_yellow_car/model.sdf`.

공식 원본: `~/urrc_f1_ws/src/urrc_track_gazebo`. **원본은 읽기 전용으로 사용**합니다.

복사 대상은 `monza`, `spa`, `monaco`, `cota`, `hungaroring`, `baku` 6개만입니다.
각 트랙의 모델·메시·충돌·도로 시각자료, 레이스 월드, 그리드/관전 GUI 설정,
중앙선·그리드 메타데이터가 `src/urrc_track_gazebo/` 아래로 복사됩니다.

공식 `traffic_lights_v10.py`를 원본과 동일하게 유지합니다. 별도 생성 파일인
`<track>_signal_active.sdf`는 복사된 월드의 기존 5개 신호등 렌즈로부터 생성합니다.
출발 신호는 5개 빨간등을 순차적으로 점등하고 **한 번의 모델 삭제 요청으로 일괄 소등**합니다.
신호 스크립트는 Gazebo visual_config 서비스와 출발선·오버뷰 관전 카메라 토픽이
준비될 때까지 기다립니다.

## 한 번만 복사

```bash
cd ~/urrc_2_f1
git pull --ff-only origin main
python3 import_6_tracks.py --official ~/urrc_f1_ws
python3 import_6_tracks.py --official ~/urrc_f1_ws --verify-only
```

`status: PASS`를 확인한 다음, 복사한 트랙 자료를 GitHub에 올리려면:

```bash
cd ~/urrc_2_f1
git add src/urrc_track_gazebo track_import_report.json
git commit -m "Import six official track assets and signal templates"
git push origin main
```

트랙 메시가 크므로 업로드에는 시간이 걸릴 수 있습니다. 가져오기 전에는
GitHub에 모델 메시가 존재하지 않습니다.

## 월드 실행

첫 터미널:

```bash
cd ~/urrc_2_f1
bash run_track.sh monza
```

월드 로딩과 카메라 준비가 완료된 뒤 두 번째 터미널:

```bash
cd ~/urrc_2_f1
bash run_lights.sh monza
```

다른 트랙에서도 `monza` 이름만 교체합니다.

**주의:** 출발 신호등의 실제 이미지 인식·차량 자동 출발·경로 추종·레이스 운영 로직은
포함하지 않습니다. 이는 트랙 및 신호등 작동 구성만 가져오는 단계입니다.
