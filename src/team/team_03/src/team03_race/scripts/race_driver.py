#!/usr/bin/env python3
"""
URRC Team03 Driver - curve wall heading + lateral position tracking

수정 사항:
1. __init__에서 초기화 전 control_count 접근 제거
2. control_step 끝에 속도/조향 토픽 발행 복구
3. LiDAR 수신/제어 진단 로그를 각각 올바른 콜백에 배치

추가: 코너에서는 좌우 벽 거리 오차 + 벽 접선 방향으로 조향.
직선 제어, 속도/횡가속도/긴급 제동 설정은 그대로 유지함.
"""

import math
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Imu
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float32, Bool


class RaceDriver(Node):
    def __init__(self):
        super().__init__('team03_race_driver_03')

        self.declare_parameter('speed_topic', '/team03/speed_03')
        self.declare_parameter('steering_topic', '/team03/steering_03')
        self.declare_parameter('lidar_topic', '/team03/scan_03')
        self.declare_parameter('imu_topic', '/team03/imu_03')
        self.declare_parameter('odom_topic', '/team03/odom_03')

        # 운행 중 ros2 param set으로 속도/피드백을 변경할 수 있다.
        self.declare_parameter('straight_speed', 3.0)
        self.declare_parameter('anti_hunting_enabled', True)
        self.declare_parameter('curve_imu_feedback', True)

        speed_topic = self.get_parameter('speed_topic').value
        steering_topic = self.get_parameter('steering_topic').value
        lidar_topic = self.get_parameter('lidar_topic').value
        imu_topic = self.get_parameter('imu_topic').value
        odom_topic = self.get_parameter('odom_topic').value

        self.speed_pub = self.create_publisher(Float32, speed_topic, 10)
        self.steering_pub = self.create_publisher(Float32, steering_topic, 10)

        # 콜백에서 사용하는 상태는 구독 생성 전에 초기화
        self.scan = None
        self.scan_count = 0
        self.control_count = 0

        # Phase 1: 센서 입력 보관 (미수신이어도 기존 LiDAR 주행 유지)
        self.imu_msg = None
        self.odom_msg = None
        self.imu_count = 0
        self.odom_count = 0
        self.last_scan_mono = None
        self.last_imu_mono = None
        self.last_odom_mono = None
        self.imu_yaw_filtered = 0.0
        self.imu_filter_initialized = False
        self.last_stale_warn_mono = -1e9

        self.scan_sub = self.create_subscription(
            LaserScan,
            lidar_topic,
            self.on_scan,
            qos_profile_sensor_data
        )

        self.imu_sub = self.create_subscription(
            Imu, imu_topic, self.on_imu, qos_profile_sensor_data
        )
        self.odom_sub = self.create_subscription(
            Odometry, odom_topic, self.on_odom, qos_profile_sensor_data
        )

        # [주행 파라미터]
        self.straight_speed = 3.0   # 직선 최고 속도 (m/s)
        self.corner_speed = 0.85    # 코너링 안전 속도 (m/s)
        self.lookahead_dist = 6.5   # 사전 감속 인지 거리 (m)
        self.stop_distance = 0.65

        # 횡가속도 및 선제 감속 파라미터
        self.max_lateral_accel = 1.5  # m/s^2, 약 0.15G
        self.brake_decel = 1.2        # m/s^2, 가정 감속도
        self.corner_buffer = 1.2      # m, 코너 접근 여유 거리

        # 직진 헌팅 억제용 변수
        self.prev_lateral_error = 0.0
        self.prev_time = None
        self.current_steering = 0.0

        # 코너 벽 방향 추정 필터: LiDAR 10Hz에 맞춰 새 스캔에서만 갱신
        self.wall_heading_filtered = 0.0
        self.last_processed_scan_count = -1
        self.curve_lateral_error = 0.0
        self.straight_heading_filtered = 0.0
        self.straight_lateral_filtered = 0.0
        self.straight_filter_initialized = False

        self.official_start_03 = False
        self.create_subscription(Bool, '/race/start/team_03',
                                 self.on_official_start_03, 10)
        self.timer = self.create_timer(0.05, self.control_step)
        self.get_logger().info('>>> Team03 Driver Node Initialized. Waiting for LiDAR... <<<')

    def on_official_start_03(self, msg):
        if msg.data:
            self.official_start_03 = True

    def on_scan(self, msg):
        self.scan = msg
        self.scan_count += 1
        self.last_scan_mono = time.monotonic()

        if self.scan_count == 1 or self.scan_count % 50 == 0:
            self.get_logger().info(
                f'[LiDAR RX] count={self.scan_count}, '
                f'rays={len(msg.ranges)}'
            )

    def on_imu(self, msg):
        self.imu_msg = msg
        self.imu_count += 1
        self.last_imu_mono = time.monotonic()
        rate = float(msg.angular_velocity.z)
        if math.isfinite(rate):
            if not self.imu_filter_initialized:
                self.imu_yaw_filtered = rate
                self.imu_filter_initialized = True
            else:
                self.imu_yaw_filtered = 0.70 * self.imu_yaw_filtered + 0.30 * rate
        if self.imu_count == 1:
            self.get_logger().info('[IMU RX] /team03/imu_03 connected')

    def on_odom(self, msg):
        self.odom_msg = msg
        self.odom_count += 1
        self.last_odom_mono = time.monotonic()
        if self.odom_count == 1:
            self.get_logger().info('[ODOM RX] /team03/odom_03 connected')

    def get_valid_min_range(self, ranges, start_idx, end_idx):
        # 차체 범퍼/바퀴 노이즈를 배제하기 위한 거리 하한 0.40m
        valid = [
            r for r in ranges[start_idx:end_idx]
            if 0.40 < r < 25.0 and not math.isinf(r) and not math.isnan(r)
        ]
        return min(valid) if valid else 15.0

    def get_sector_median_range(self, ranges, mid, deg_to_idx,
                                angle_deg, half_width_deg=3.0):
        """지정 각도 주변의 유효 LiDAR 값 중앙값. 반사 노이즈 최소값의 튐 억제."""
        start = max(0, int(mid + (angle_deg - half_width_deg) * deg_to_idx))
        end = min(len(ranges), int(mid + (angle_deg + half_width_deg) * deg_to_idx) + 1)
        valid = sorted(
            r for r in ranges[start:end]
            if math.isfinite(r) and 0.40 < r < 25.0
        )
        if len(valid) < 2:
            return None
        return valid[len(valid) // 2]

    def estimate_wall_heading(self, ranges, mid, deg_to_idx, side):
        """한쪽 벽의 앞/옆 측정점 2개의 접선이 차량 전방과 이루는 각도."""
        front_angle = side * 55.0
        rear_angle = side * 82.0
        front_range = self.get_sector_median_range(
            ranges, mid, deg_to_idx, front_angle
        )
        rear_range = self.get_sector_median_range(
            ranges, mid, deg_to_idx, rear_angle
        )
        if front_range is None or rear_range is None:
            return None

        a = math.radians(front_angle)
        b = math.radians(rear_angle)
        dx = front_range * math.cos(a) - rear_range * math.cos(b)
        dy = front_range * math.sin(a) - rear_range * math.sin(b)

        # 같은 벽의 전후 두 점이 아닌 경우(벽의 끝단 등) 제거
        if dx < 0.12:
            return None
        heading = math.atan2(dy, dx)
        if abs(heading) > 0.90:
            return None
        return heading

    def estimate_corridor_heading(self, ranges, mid, deg_to_idx,
                                  side_left, side_right, straight=False):
        """양쪽 벽 접선 추정. 직선에서는 서로 모순되는 벽을 거부한다."""
        left = self.estimate_wall_heading(ranges, mid, deg_to_idx, 1)
        right = self.estimate_wall_heading(ranges, mid, deg_to_idx, -1)
        if left is not None and right is not None:
            if abs(left - right) < (0.20 if straight else 0.40):
                return (left + right) * 0.5
            if straight and abs(side_left - side_right) < 0.25:
                return None  # 양쪽 거리가 비슷한데 접선 불일치 -> 신뢰하지 않음
            return left if side_left < side_right else right
        return left if left is not None else right

    def control_step(self):
        if not self.official_start_03:
            self.speed_pub.publish(Float32(data=0.0))
            self.steering_pub.publish(Float32(data=0.0))
            return

        # 센서 데이터가 멈춘 경우 이전 명령으로 계속 달리지 않도록 정지.
        now_wall = time.monotonic()
        if self.scan is None or (self.last_scan_mono is not None
                                 and now_wall - self.last_scan_mono > 0.50):
            self.speed_pub.publish(Float32(data=0.0))
            self.steering_pub.publish(Float32(data=0.0))
            self.current_steering = 0.0
            if now_wall - self.last_stale_warn_mono > 2.0:
                self.get_logger().warn('[WATCHDOG] LiDAR stale/missing; publish speed=0')
                self.last_stale_warn_mono = now_wall
            return

        # 직선 목표속도: 3.0 -> 3.5 -> 4.0 (ROS 파라미터로 무재시작 변경).
        self.straight_speed = min(4.0, max(0.0, float(self.get_parameter('straight_speed').value)))
        anti_hunting_enabled = bool(self.get_parameter('anti_hunting_enabled').value)
        curve_imu_feedback = bool(self.get_parameter('curve_imu_feedback').value)
        imu_valid = (self.imu_filter_initialized and self.last_imu_mono is not None
                     and now_wall - self.last_imu_mono <= 0.25)
        imu_yaw = self.imu_yaw_filtered if imu_valid else None
        odom_valid = (self.odom_msg is not None and self.last_odom_mono is not None
                      and now_wall - self.last_odom_mono <= 0.30)
        actual_speed = (math.hypot(self.odom_msg.twist.twist.linear.x,
                                   self.odom_msg.twist.twist.linear.y)
                        if odom_valid else self.straight_speed)

        now = self.get_clock().now().nanoseconds / 1e9
        dt = 0.05 if self.prev_time is None else max(0.001, now - self.prev_time)
        self.prev_time = now

        ranges = self.scan.ranges
        total = len(ranges)
        mid = total // 2
        deg_to_idx = total / 360.0

        # 1. 핀포인트 정면 감속 레이더 (±4도)
        front_dist = self.get_valid_min_range(
            ranges,
            int(mid - 4 * deg_to_idx),
            int(mid + 4 * deg_to_idx)
        )

        # 2. 측면 레이더 (±50도 ~ ±80도) -> 횡방향 위치 오차
        side_left = self.get_valid_min_range(
            ranges,
            int(mid + 50 * deg_to_idx),
            int(mid + 80 * deg_to_idx)
        )
        side_right = self.get_valid_min_range(
            ranges,
            int(mid - 80 * deg_to_idx),
            int(mid - 50 * deg_to_idx)
        )
        lateral_error = side_left - side_right

        d_error = (lateral_error - self.prev_lateral_error) / dt
        self.prev_lateral_error = lateral_error

        # 3. 직선 / 코너 주행 모드
        corner_mode = front_dist < self.lookahead_dist
        turn_clearance = 0.0
        new_scan = self.scan_count != self.last_processed_scan_count
        if new_scan:
            self.last_processed_scan_count = self.scan_count

        if corner_mode:
            # 코너 방향 예견
            far_left = self.get_valid_min_range(
                ranges,
                int(mid + 10 * deg_to_idx),
                int(mid + 22 * deg_to_idx)
            )
            far_right = self.get_valid_min_range(
                ranges,
                int(mid - 22 * deg_to_idx),
                int(mid - 10 * deg_to_idx)
            )
            lookahead_error = far_left - far_right

            ratio = max(
                0.0,
                min(
                    1.0,
                    (front_dist - self.stop_distance)
                    / (self.lookahead_dist - self.stop_distance)
                )
            )

            speed = (
                self.corner_speed
                + ratio * (self.straight_speed - self.corner_speed)
            )

            # 코너 벽 접선 + 좌우 위치 + 전방 예견 (완주 버전 수식 유지).
            wall_heading = self.estimate_corridor_heading(
                ranges, mid, deg_to_idx, side_left, side_right
            )

            if new_scan:
                if wall_heading is not None:
                    self.wall_heading_filtered = (
                        0.65 * self.wall_heading_filtered
                        + 0.35 * wall_heading
                    )
                else:
                    # 벽 방향을 못 구하면 이전 추정값을 점차 해제
                    self.wall_heading_filtered *= 0.90

            # 측면 최소값 대신 중앙값으로 위치 오차를 부드럽게 추정
            left_gap = self.get_sector_median_range(
                ranges, mid, deg_to_idx, 78.0
            )
            right_gap = self.get_sector_median_range(
                ranges, mid, deg_to_idx, -78.0
            )
            if left_gap is not None and right_gap is not None:
                curve_error = left_gap - right_gap
            else:
                curve_error = lateral_error
            curve_error = max(-2.0, min(2.0, curve_error))
            self.curve_lateral_error = curve_error

            # 벽의 진행 방향 우선, 좌우 편차와 전방 예견은 보조
            # [rad/s], 유효한 벽 방향이 없을 때는 예견 오차로 복구
            if wall_heading is not None:
                target_steering = (
                    1.8 * self.wall_heading_filtered
                    + 0.14 * curve_error
                    + 0.08 * max(-2.0, min(2.0, lookahead_error))
                )
            else:
                target_steering = (
                    0.30 * max(-2.0, min(2.0, lookahead_error))
                    + 0.14 * curve_error
                )

            # 작은 IMU 요각속도 피드백만 추가. 기존 코너 회전 성능 보호.
            if curve_imu_feedback and imu_valid:
                target_steering += 0.12 * (target_steering - imu_yaw)
            target_steering = max(-1.0, min(1.0, target_steering))

            # 회전 방향 공간 확인
            turn_clearance = (
                far_left if target_steering >= 0.0 else far_right
            )

            # 회전량 증가 시 감속
            steer_penalty = 1.0 - 0.40 * abs(target_steering)
            speed *= steer_penalty

        else:
            speed = self.straight_speed
            if anti_hunting_enabled:
                # 1) 직선에서도 벽 접선 방향을 추정 (새 LiDAR 스캔에서만 필터 갱신).
                if new_scan:
                    heading = self.estimate_corridor_heading(
                        ranges, mid, deg_to_idx, side_left, side_right,
                        straight=True
                    )
                    left_gap = self.get_sector_median_range(ranges, mid, deg_to_idx, 78.0)
                    right_gap = self.get_sector_median_range(ranges, mid, deg_to_idx, -78.0)
                    position_error = (left_gap - right_gap) if (
                        left_gap is not None and right_gap is not None
                    ) else lateral_error
                    position_error = max(-2.0, min(2.0, position_error))
                    if not self.straight_filter_initialized:
                        self.straight_lateral_filtered = position_error
                        self.straight_filter_initialized = True
                    else:
                        self.straight_lateral_filtered = (
                            0.70 * self.straight_lateral_filtered + 0.30 * position_error
                        )
                    if heading is not None:
                        self.straight_heading_filtered = (
                            0.60 * self.straight_heading_filtered + 0.40 * heading
                        )
                    else:
                        self.straight_heading_filtered *= 0.90
                # 2) 중앙선 주변의 작은 측정 잡음은 연속적인 소프트 데드밴드로 무시.
                e = self.straight_lateral_filtered
                e_soft = math.copysign(max(0.0, abs(e) - 0.06), e)
                # 속도 증가 시 횡위치 보정 과민도를 완화.
                k_lateral = 0.045 * min(1.0, 3.0 / max(3.0, actual_speed))
                yaw_ref = 1.15 * self.straight_heading_filtered + k_lateral * e_soft
                # 3) IMU 실제 회전속도를 이용해 반대 방향으로 감쇠.
                target_steering = yaw_ref - (0.28 * imu_yaw if imu_valid else 0.0)
                target_steering = max(-0.10, min(0.10, target_steering))
            else:
                # 즉시 원래 완주 버전 직선 제어로 되돌리는 라이브 스위치.
                target_steering = max(-0.06, min(0.06, 0.05 * lateral_error))

        # 코너에서는 빠르게, 직선에서는 부드럽게 조향
        alpha = 0.55 if corner_mode else 0.35
        self.current_steering = (
            (1.0 - alpha) * self.current_steering
            + alpha * target_steering
        )

        # 4. 횡가속도 제한
        # vehicle_adapter에서 angular.z는 yaw rate(rad/s)
        yaw_rate = abs(self.current_steering)
        if yaw_rate > 0.01:
            lateral_speed_limit = self.max_lateral_accel / yaw_rate
            speed = min(speed, lateral_speed_limit)

        # 5. 코너 진입 전 거리 기반 선제 감속
        available_distance = max(0.0, front_dist - self.corner_buffer)
        preview_speed = math.sqrt(
            self.corner_speed ** 2
            + 2.0 * self.brake_decel * available_distance
        )
        speed = min(speed, preview_speed)
        speed = max(0.0, speed)

        # 6. 코너와 장애물 긴급 정지 구분
        if front_dist < self.stop_distance:
            # 회전 방향 공간이 충분할 때만 초저속 진행
            can_creep = (
                corner_mode
                and 0.50 <= front_dist
                and abs(self.current_steering) > 0.25
                and turn_clearance > 1.5
            )
            if can_creep:
                speed = min(speed, 0.25)
            else:
                speed = 0.0
                self.get_logger().warn(
                    f'EMERGENCY BRAKE! Front: {front_dist:.2f}m'
                )

        # 7. 실제 속도/조향 명령 발행 (원본에서 누락된 부분)
        self.speed_pub.publish(Float32(data=float(speed)))
        self.steering_pub.publish(
            Float32(data=float(self.current_steering))
        )

        # 진단 로그: control_step에서만 제어 카운터 증가
        self.control_count += 1
        if self.control_count == 1 or self.control_count % 20 == 0:
            self.get_logger().info(
                f'[CTRL] count={self.control_count}, '
                f'speed={speed:.2f}, '
                f'yaw={self.current_steering:.2f}, '
                f'front={front_dist:.2f}, '
                f'wall_heading={self.wall_heading_filtered:.2f}, '
                f'curve_error={self.curve_lateral_error:.2f}, '
                f'imu_rx={self.imu_count}, odom_rx={self.odom_count}, '
                f'gyro={imu_yaw if imu_yaw is not None else float("nan"):.3f}, '
                f'v_odom={actual_speed:.2f}, '
                f'straight_e={self.straight_lateral_filtered:.2f}, '
                f'straight_h={self.straight_heading_filtered:.3f}'
            )


def main():
    rclpy.init()
    node = RaceDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.speed_pub.publish(Float32(data=0.0))
        node.steering_pub.publish(Float32(data=0.0))
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
