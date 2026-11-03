# TEAM 01 검증 전용 자동화 — Gazebo 물리 실주행 / rosbag / 궤적 정확도

## 자동 실행
- main 브랜치 차량 코드가 바뀔 때마다 .github/workflows/team01-qa-validation.yml이 자동 실행됩니다.
- ROS2 Jazzy + Gazebo Harmonic **실제 물리 시뮬레이션**, 카메라/Depth/LiDAR, 신호 카메라 출발과 외부 승인, 어댑터, 실측 Odometry 및 독립 LapVerifier를 검증합니다.
- 오프라인 실패 후에도 Gazebo는 독립 실행합니다. 어느 한쪽이라도 실패하면 전체 QA는 FAIL입니다.
- 주행 코드, 차량 SDF, 실제 컨트롤러는 QA가 수정하지 않습니다.

## 제작자가 GitHub Actions → 해당 실행 → Artifacts에서 받을 자료
| 파일 | 검증 목적 |
| --- | --- |
| rosbag_core/ | ROS 2 rosbag2 sqlite3 + zstd: /team01/gz_odom, /team01/debug/path, /team01/debug/start, /team01/debug/adapter, /team01/ready_for_lights, /team01/speed, /team01/steering, /team01/front_scan, /team01/rear_scan, /race/start/team01 |
| trajectory.csv | Gazebo 실제 x/y/yaw/속도, 독립 Monza 중심선 편차, 도로 안전 여유거리, 과거 계획경로 추종 오차 |
| drive_metrics.json | 계획경로 추종 오차 평균/P95, 중심선 편차 평균/P95, 최소 도로 여유, Odometry 최대/P95 속도, 조향 부호 반전, BRAKE/STOP 원인 |
| trajectory_overlay.svg | Monza 공식 중심선(회색)과 실제 주행 궤적(파란색) |
| qa_run_report.md | 커밋별 시험 요약; GitHub Actions Summary에도 표시 |
| qa_events.jsonl | 준비 승인→카메라 신호 소등→출발 권한→어댑터 전진 및 BRAKE/STOP 모드 변경 |
| lap.json 및 기존 *.log | 독립 완주/도로이탈 판정과 센서·모델·SLAM 실패 원인 |
| ci_artifacts/state/urrc_team01/camera 및 road | 컨트롤러가 보존한 신호/도로 이미지 |

바이너리 rosbag은 Git 이력에 직접 커밋하지 않습니다. Actions 아티팩트에 커밋 SHA 및 실행 ID별로 보관하고, 텍스트 QA 기준은 Git 저장소에서 공동 확인합니다. 보존 기간은 14일입니다. 기본 rosbag은 진단·라이다·Odometry 중심입니다. 카메라 전체 원본의 장시간 bag 저장은 고용량이므로 컨트롤러가 보존하는 프레임으로 보완합니다.

## 정확도 계산 원칙
1. 먼저 신호등 보정 준비, 실제 카메라 소등, permission, adapter의 양의 속도 송신, Gazebo Odometry 5m 이상 이동을 순서대로 확인합니다.
2. 현재 생성된 base_link 계획경로는 차량 위치 (0,0)에서 시작하여 현재 오도메트리와 단순 비교하면 오차가 **가짜 0**이 됩니다. 따라서 0.6~1.5초 전에 작성한 경로를 그때의 Gazebo Odometry 위치·yaw로 전역 변환하고, 이후 실제 차량 위치와 비교합니다.
3. 의도적으로 도로 중앙에서 벗어나 안전한 경로를 유지할 수 있으므로 공식 중심선 오프셋만으로 주행 실패 처리하지 않습니다. 센터 편차와 도로 경계 안전 여유, 계획경로 추종 오차를 분리합니다.
4. 실제 이동 5m 미만 또는 과거 계획경로 비교 15개 미만에서는 **정확도 미검증**으로 표시합니다. 목표 속도 명령 대신 실측 Odometry 속도만 보고합니다.
5. Monza 완주는 독립 GEOMETRIC_LAP_PASS 와 충분한 코스 커버리지, 도로 경계 유지가 있어야 승인됩니다. 1랩 PASS 없이 8m/s 및 10m/s 안정화를 인증하지 않습니다.

## 실패에 따른 제작팀 수정 방향
| 증거 | 진단/제작 권고 |
| --- | --- |
| ready=false | 각각의 vision_ready, gate_baseline, rgb_age, depth_ok, lidar_ok 동시 기록, ROS topic discovery/지속 구독 점검. 신호 안전 게이트 우회 금지 |
| motion.cmd_speed=0 | road.valid, BRAKE reason, reference geometry 유효성, 센서 지연/경계값, recovery 히스테리시스 확인 |
| adapter.sent_linear>0인데 actual_v≈0 | 브리지 방향, Ackermann plugin, 스폰 높이·접촉, 구동바퀴 joint·마찰·토크 확인. 상위 경로 알고리즘부터 교체하지 않음 |
| planned reference가 없거나 adaptive→constant_arc 빈번 전환 | RGB-D 인식과 LiDAR 도로 경계 신뢰성, 경로 후보 거절 원인(near-curb/obstacle/곡률/길이) 기록 |
| 경로는 유효하지만 S자 주행 | yaw-rate 부호, Stanley 측방 오차·방위 오차 이득, 조향 제한 후 지연, path scheduling 10Hz↔control 20Hz 분리 시험 |
| 코너에서 BRAKE 반복 | 소거리 충돌검사와 rollout 경로 차이, 유효 전방거리 축소, 조향 제한 후 안전성 확인(안전 검사 우회 금지) |
| SLAM ICP/TF 불안정 | timestamp, TF frame chain, scan–RGBD 동기화, 지도 loop closure 검증 |

## 교체 여부 결정
- **센서/도로 경계 부재**라면 Stanley→LQR/MPC 변경은 근본 해결책이 아닙니다. 센서 인식·유효 경로를 먼저 복구해야 합니다.
- **실제 계획경로는 유효하지만 제어 출력이 발산**할 때는 현재 Stanley + yaw feedback + steering limiter 조합의 부호/시간지연/중복 보정부터 확인합니다. 이후 동일한 rosbag을 사용해 Stanley / LQR / 제약형 MPC의 조향 RMS·오차 P95·안전거리·연산시간을 비교해 도입 여부를 결정합니다.
- **곡률 큰 코너에서 국소 후보가 계속 사라진다면** 단일 상수 곡률 후보 fallback보다 곡률 연속 경로 생성+모델예측제어(MPC)가 대안입니다. 안전 경계/충돌 제동/신호 조건은 유지합니다.
- **시뮬레이터 실제 구동 전달 실패**라면 고수준 알고리즘 전면 교체 전에 모델·구동 플러그인부터 검증해야 합니다.

## 사용자 PC 업데이트·실행 (Ubuntu 24.04 / ROS 2 Jazzy)
```bash
cd ~/urrc_2_f1
git pull --ff-only
git submodule update --init --recursive
python3 - <<'PY'
from pathlib import Path
import sys
p=Path('upstream/urrc_f1_ws/src/urrc_track_gazebo')
sys.path.insert(0,str(p))
from urrc_tracks.v10_assets import enhance_track
enhance_track(p,'monza')
PY
sudo apt install -y ros-jazzy-rosbag2 ros-jazzy-rosbag2-storage-default-plugins
source /opt/ros/jazzy/setup.bash
bash scripts/qa_team01_gazebo.sh --local
```

- 저장 위치: ~/team01_qa/날짜_시각/
- ros2 bag info 경로/rosbag_core 로 녹화를 확인하고 ros2 bag play 경로/rosbag_core 로 재생합니다.
- QA 시각화·요약은 같은 폴더의 trajectory_overlay.svg 및 qa_run_report.md입니다.
