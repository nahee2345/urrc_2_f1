# TEAM 01 시그널 출발 P0 진단

## 기존 증거
- 이전 독립 Gazebo v15에서는 CAMERA LIGHTS OUT verified가 있었지만 실제 odom 거리는 약 1.1m였습니다.
- 따라서 화면상 정지는 신호 인식 실패뿐 아니라 구동 명령, 안전정지, 시작 위치 간섭일 수 있습니다.
- 이 문서와 추가 진단은 실제 출발 PASS를 의미하지 않습니다.

## 이번 제작 변경
- `run_local_monza.sh`는 실제 CI와 동일한 upstream Monza 경로를 우선 사용하고, `safe_grid_spawn.py`로 초기 연석 간섭을 완화한 생성 위치를 적용합니다.
- 실제 출발 전 `/team01/ready_for_lights`가 true인지 기다립니다. RGB 어두운 기준 프레임과 TrafficGate 밝기 baseline, Depth/LiDAR 신선도가 조건입니다.
- 신호 인식 코드/공식 신호등 점등·소등 순서는 바꾸지 않았습니다.
- `/team01/debug/start`(20Hz): permission, camera_calibrated, gate_status, lights_out, red_peak, depth_ok, motion_reason, 실제속도.
- `/team01/debug/adapter`(25Hz): allowed, fresh_speed, fresh_steer, 송신된 Twist linear/yaw.

## 검증팀에게 필요한 순서
1. 신호등 점등 전 `ros2 topic echo /team01/ready_for_lights` 에 true가 한 번 이상 있는지.
2. 점등 시 `/team01/debug/start`의 red_peak>=4, gate_status ARMED, 최종 LIGHTS_OUT인지.
3. `permission=true` 및 `motion_mode`, `motion_reason`을 함께 보십시오.
4. `/team01/debug/adapter`: allowed true 및 fresh_speed/fresh_steer true, sent_linear_mps>0가 나오는지.
5. `/team01/gz_odom`: 실제 실측 linear.x 및 차량 위치가 움직이는지.
6. 4단계의 값이 >0인데 5단계가 거의 0이라면: Gazebo bridge 방향, Ackermann plugin cmd_vel, 초기 연석·차체/바퀴 충돌을 점검하십시오.
7. 신호 승인 없이 우회하는 강제 출발을 성능 PASS로 인정하지 마십시오.

위 내용을 각 실행 커밋 SHA와 함께 보고하고, 최초 실패 시각·ROS/Gazebo 명령·실제속도 기록을 붙여주십시오.
