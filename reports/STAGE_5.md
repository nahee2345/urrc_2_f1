# 5단계 개발·검증 보고서 — Monaco

기준일: 2026-10-27

## 개발 기준

- 공식 urrc_f1_ws의 Monaco를 기반으로 AI-01 실험.
- 길이 약 667.16m, 최소 도로 폭 1.4m, 고저차 약 8.41m.
- 원본 urrc_f1_ws, urrc_2의 수정·push는 금지.
- 작업 대상 저장소는 nahee2345/urrc_2_f1 하나뿐.

## 코드

- stage5/monaco.py: 공식 원본 Monaco 트랙 메시·높이·관전 카메라·신호등을 격리 복제, SHA 무결성 검사.
- scripts/stage5_prepare_monaco.py: 원본 모델 + 검정 AI-01 동적 차량 생성.
- scripts/stage5_run_monaco.py: 공식 Monaco 3D GUI 실행.
- scripts/stage5_lights.py: 공식 다섯 빨간불 순차 점등 후 일괄 소등.
- scripts/stage5_bridge.py: ROS2 카메라/조향/전륜모터 브리지 준비.
- stage5/roi.py 및 scripts/stage5_calibrate_lights.py: 다섯 빨간등 실제 영상의 ROI 자동 교정. 실패 시 출발 차단.
- scripts/stage5_race.py: 카메라 출발 안전 게이트, RGB 추종, 실제 3랩 운영자 계측·충돌 감독 연동.
- scripts/stage5_monaco_sensor_smoke.py: Gazebo Monaco 카메라·감독 포즈 실제 데이터 검사.
- scripts/stage5_monaco_actuator_smoke.py: Gazebo Monaco 전륜 모터 실제 물리 이동 검사.
- tests/test_stage5_*.py: 원본 지형·5등·출발 감지·로그·3랩 무효 판정 회귀검사.

## 검증한 실제 결과

- GitHub Actions #40495144919: 전체 단위 테스트 67개 PASS.
- 원본 Monaco 3D 지형·충돌 메시·5등 임시 모델 보존 확인.
- 실제 Gazebo Monaco 카메라 이미지와 운영자 위치 메시 수신 PASS.
- 실제 Gazebo Monaco AI-01 전륜구동 위치 변위 약 1.642539m PASS.
- 링크: https://github.com/nahee2345/urrc_2_f1/actions/runs/40495144919

이후 빨간불 ROI 자동 교정·레이스 로그 판정 테스트를 추가했고, 최종 GitHub Actions 재시험은 별도로 확인해야 합니다. 한 번의 중간 재시험은 부동소수점 테스트 비교를 너무 엄격하게 설정해 실패했으며 수정했습니다. 다른 중간 재시험은 GitHub runner의 외부 Microsoft APT 403 때문에 Gazebo 설치 전에 중단됐으며 CI 설정만 변경했습니다.

## 아직 PASS가 아닌 것

- 실제 Monaco 시작 카메라 RGB에서 신호등 다섯 렌즈 자동 교정.
- ROS2 메시지 전체 브리지와 출발 안전 인터록의 끝단 통합.
- Monaco 전 구간 헤어핀·언덕 RGB 차선 추종 품질.
- 충돌·코스 단축 없는 실제 자동주행 연속 3랩.
- 원래 계획된 학습형 CNN 주행 모델.

이 모든 사항이 확인되기 전에는 5단계 무충돌 3바퀴 최종 PASS가 아닙니다. 현재 코드는 실제 검증 실행 준비 단계까지 구축했습니다.

사용자 실행은 docs/STAGE5_GUIDE.md에 한국어로 정리했습니다. 모든 코드 수정은 GitHub를 통해 git pull로만 반영합니다.


## 후속 정적 검증 완료

- Python 전체 테스트 **77개 모두 PASS** — Monaco 원본 메시·검정 차량·신호등 ROI 자동교정 모의 프레임·3랩 판정/안전 게이트 포함
- GitHub Actions: https://github.com/nahee2345/urrc_2_f1/actions/runs/40496330149
- Gazebo 최신 동적 재시험: GitHub runner의 일시적인 외부 APT 403 오류 방지 설정을 적용한 뒤 진행 중. 완료 상태 확인 전까지 새 실행의 동적 결과는 미확정.
- 최초 실제 Monaco 물리·센서 PASS는 #40495144919에서 독립적으로 확보했음.

**실제 신호등 카메라 RGB 교정 및 실제 통합 3랩 완주 결과는 여전히 미검증입니다.**


## 5단계 최종 확인된 GitHub 자동 검증

- 실행: https://github.com/nahee2345/urrc_2_f1/actions/runs/40496330149
- 전체 작업 2개 모두 success.
- **Python 단위 테스트 77개 PASS**.
- Monaco 원본 지형 높이·충돌 메시·관전 카메라·원본 5등 모델 구조 PASS.
- 실제 Gazebo Monaco 이미지 메시지 **8,841,025바이트** 수신.
- 실제 Gazebo Monaco 운영자 포즈 메시지 **2,115,497바이트** 수신.
- 실제 Monaco 전륜 모터 주행 변위 **1.674858m** PASS, 포즈 샘플 3,305개.
- 앞·뒤 측정 Z **2.0750m → 2.1566m** (Monaco 3D 노면 변화 반영).
- 위 시험은 **실제 신호등 순차 점등·소등을 Monaco 카메라가 감지하는 시험이나, 실제 RGB 기반 자동 3랩 완주 시험이 아닙니다.**
- 원본 urrc_f1_ws, urrc_2 GitHub는 수정하지 않았습니다.

자동 시험 결과를 과장하지 않도록, 센서 스모크 JSON의 설명 문구에서도 신호등을 실제 검증하지 않은 경우는 분리 표시하도록 수정했습니다.


## 2026-10-29 수정 기록

- 직접 실행하는 scripts/stage5_race.py의 sys.path에 프로젝트 루트 누락으로 ModuleNotFoundError 발생한 것을 수정.
- 단독 Monaco Gazebo 실행은 차량 스폰만 처리하는 것이 원인. 실행 후 원본 신호등 서비스를 호출하는 --lights 옵션 추가.
- 차량 10대 차체 및 센서 장착물을 10가지 고대비 원색으로 변경.
- 테스트에 Python 직접 실행 오류 회귀, 10종 색상 검증, 신호등 옵션 검사 추가.
- 이 수정은 원본 저장소가 아닌 urrc_2_f1만 변경하며 실제 3랩 완주 PASS로 간주하지 않음.


## 2026-10-29 한 터미널 통합 실행기 추가

- \`scripts/stage5_start.py\` 신규 구현: Monaco 파일 생성 및 ROS 2 브리지 설정, 실제 5등 점등 카메라 ROI 교정, 신호등 초기화, Gazebo 정리, 안전 출발 및 AI-01 3랩 주행 실험까지 일괄 관리.
- \`calibration.json\` 미존재시 이전처럼 오류만 출력하지 않고 실제 Gazebo/브리지를 실행하여 교정 시도.
- 신호등 5개가 실제 카메라로 검출되지 않으면 fail-closed: 모터를 구동하는 레이스 실행을 시작하지 않음.
- Ctrl+C 및 실패 시 Gazebo/브리지 자식 프로세스 종료, 카메라 진단 이미지/로그 출력.
- 옵션 \`--calibrate-only\`, \`--recalibrate\`, \`--dry-run\` 포함.
- \`tests/test_stage5_one_terminal.py\`에서 직접 실행 경로·단일 실행 순서·무효 교정값·속도 범위·실패 후 구동 차단 검증.
- 실제 Ubuntu ROS2+Gazebo 끝단 자동출발 및 3바퀴 완주 시험 결과는 **미검증**.


## 사용자 실측 7~8개 빨간 그룹 문제 후속 수정 (2026-10-29)

- 사용자 실제 Gazebo에서 ROS 카메라 수신 277프레임/18초, 발행자 1개. 센서·브리지는 정상.
- 기존 교정기는 붉은 연결영역 7~8개가 있으면 무조건 FAIL; 정적 빨간 물체를 신호등 후보에서 구분하지 않았음.
- 이제 실제 신호등 OFF/ON 카메라 RGB 픽셀을 비교하여 새로 빨개진 영역으로부터 다섯 개 정렬된 렌즈를 검출.
- 준비→소등 기준 캡처→점등→차분 검출→교정 파일 생성→소등 및 임시 월드 정리를 한 터미널에서 실행.
- 7~8개 붉은 후보, 새로 켜진 렌즈 4개, 변경 없는 영상, 카메라 데이터 오류에 대한 회귀검사 추가.
- Jazzy Ctrl+C 예외 중복 shutdown 방지.
- 기존 교정 파일이 있더라도 OFF/ON 실제 차분 검증 표시가 없으면 레이스 실행 금지.
- 실제 ROS2 카메라 영상에서 최종 5등 검출과 Monaco 3랩 완주는 아직 미검증.


### 교정 오류 수정 자동검사 결과

- GitHub Actions 실행: https://github.com/nahee2345/urrc_2_f1/actions/runs/40818551110
- 0~5단계 Python 회귀시험: **100/100 PASS**.
- 새 교정 방식의 7~8개 붉은 연결영역, 5개만 새로 켜진 LED 식별, 4개 이하 fail-closed, OFF/ON 동일 영상의 출발 불허, PPM 저장·재로드 정상 테스트 PASS.
- CI 실제 Gazebo 동적 시험은 별도 작업이며, 실제 ROS2 카메라에서 5등을 인식했다는 실측 근거는 아직 없음.
- 사용자가 Ctrl+C로 교정 중단해도 ROS 종료 두 번 호출하지 않고 마지막 카메라 이미지를 진단용으로 보존하도록 수정.


## 2026-10-29 교정 PASS 후 GUI 재기동 실패 수정

- 사용자 실제 실행 로그: 소등 기준 RGB 촬영 → 빨간불 5개 ON → 차분 교정 **PASS**.
- 기존 scripts/stage5_start.py: 교정 성공 직후 정상 Gazebo GUI 및 ROS 브리지를 종료하고 별도 scripts/stage5_race.py에서 다시 실행. 이 경로에서 화면이 사라지고, 재실행 실패 로그는 화면에 표시되지 않았음.
- 신규 stage5/session.py: Gazebo GUI와 ROS 브리지 실행을 딱 1회로 통합. 교정 OFF/ON 시퀀스 뒤 Gazebo를 유지한 상태에서 레이스를 연결.
- scripts/stage5_race.py --reuse-gazebo: 이미 존재하는 Gazebo와 브리지에는 새 프로세스를 생성하지 않고 출발 게이트/주행 제어/계측만 실행. 기존 GUI 서비스 존재 확인, 런타임 노드 로그 위치 표시.
- Ctrl+C로 레이스를 종료하면 Python KeyboardInterrupt traceback 없이 종료하도록 처리.
- tests/test_stage5_session_reuse.py: 교정 전후 동일 GUI 프로세스, 재교정 생략, GUI 중복 실행 차단 및 새 CLI 스위치 확인.
- 실제 사용자 Ubuntu에서 GUI 유지 및 카메라 자동 출발·3바퀴 완주 검증은 **다음 현장 실행 필요**.


## 2026-10-29 사용자가 제출한 Monaco GUI 및 ROS 로그 후속 검증

실제 출발 영상 5개 점등/전체 소등 허가 확인. ros_gz_bridge의 ROS Float64→Gazebo Double 송신 기록 있음. 허가 로그 시점 1793249835.692과 SIGINT 로그 1793249836.801 사이 약 1.1초. 실제 바퀴 명령 크기 또는 물리 차량 이동 증거는 사용자 제공 자료만으로 미확인.

수정: stage3/ros_gate.py의 출발 로그 반복과 shutdown 충돌 방지, stage4/ros_drive.py의 실제 RGB·명령·도로검출 진단과 rgb_camera.ppm 저장, stage3/pose_monitor.py의 실제 물리 이동량 계측, scripts/stage5_race.py의 3초 진단 출력과 종료 시 모터 0명령 전송.

tests/test_stage5_live_diagnostics.py에서 미수신 진단값, 실제 진단 데이터 및 Gazebo 4개 액추에이터 0속도 명령을 검증. 110개 단위 시험 PASS 확인. 실제 원본 Monaco의 자동 3바퀴는 별도 검증이 필요합니다.


## 2026-10-29 AI-01 실제 Monaco 3.8m 후 도로 추적 상실

현장 실측 신호등 순차 점등→동시 소등 4.372초, RGB 카메라 173~448프레임, 출발 허가 True, 도로 추종 정상 상태 전륜 목표 11~13.75rad/s, 누적 3.8043m 이동. 이후 `연속 도로 경계 부족`으로 속도 0·랩 0.

stage4/lane.py: Monza 고정 17~67 밝기 분할을 어두운/그림자/밝은 Monaco 아스팔트·녹갈색 배경 구별로 변경. 수평 이미지 하단부터 먼 행 방향 연속 도로 연결과 곡선 선제 추종. stage4/safety.py: Monaco 코너 감속 및 선제 조향, 실영상 유효성/watchdog 실패 정지 유지. vehicle_generator/sensors.py: AI-01 주행 RGB 카메라만 pitch -0.26rad 하향, start_camera -0.55rad 유지. stage4/ros_drive.py 및 scripts/stage5_race.py: 최초 경계 상실 순간 실제 주행용 RGB 프레임 rgb_camera_road_lost.ppm 저장(이전 결과 삭제 후 새 실행).

tests/test_stage5_monaco_lane.py에서 그림자·강한 코너·반대 방향 코너·모든 픽셀이 검정인 잘못된 화면·도로 소실/정지 회귀검사 추가. 이번 정적 단위 검사 122개 PASS. 실제 Gazebo 연속 주행·무충돌 3바퀴는 미검증.
