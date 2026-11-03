# TEAM01 단계별 개발 및 검증 안내

## 구현 코드 (실제 주행 PASS와 별개)
- 1: Odometry yaw-rate 피드백과 ±4도 조향 보정
- 2: 10Hz 기준 경로 생성, 좌표 이동 보정, 20Hz 조향 제어/경로 로그
- 3: 정지 원인별 분류, 재출발 히스테리시스, 직선/코너 판단 히스테리시스
- 4: Stanley 기반 국소 예측 경로 추종
- 5: Q-learning 오프라인 실험, 실주행 적용은 기본 비활성
- 6: 명시적인 5/8/10m/s 상한 설정

## 속도 상승 조건
기본 validated_speed_stage_mps: 5.0
실제 Gazebo에서 5m/s 반복 완주가 검증된 뒤에만 8.0으로 변경하세요.
8m/s 반복 완주를 검증한 뒤에만 10.0으로 변경하세요.
RTAB-Map 기반 지도 목표가 유효하지 않으면 언제나 5m/s가 상한입니다.
도로 경계/장애물/횡가속/제동거리 감속은 이 상한보다 우선합니다.

## 오프라인 Q-learning 실험 명령
cd ~/urrc_2_f1/src/team_01/team01_race
python3 scripts/train_gain_qlearning.py --episodes 1200 --out /tmp/team01_rl_gains.json

훈련기는 토이 자전거 모델만 사용하며 Gazebo 완주를 보증하지 않습니다.
enable_learned_gains는 기본 false입니다.

## 별도 검증 담당자에 요청할 자료
- 커밋 SHA, 실제 1랩 거리/시간, 카메라 소등 출발 여부
- /team01/debug/path (20Hz): 경로 생성, 선택, reason, mode
- /team01/steering 및 /team01/gz_odom: 실제 yaw-rate/횡오차/역조향 수
- STOP/BRAKE/RECOVER 최초 원인, 센서 유효성, 경로 생성 실패
- 5m/s 통과 뒤 8m/s, 그 뒤 10m/s에서 별도의 안전/완주 실측
- Loop Closure와 Map TF, 우회/재합류 결과
