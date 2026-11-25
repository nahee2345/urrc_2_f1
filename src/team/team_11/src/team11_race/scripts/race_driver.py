#!/usr/bin/env python3

import math
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from std_msgs.msg import Bool, Float32


# V4.1 고속 안정 제어: 20Hz / 18m LiDAR + 속도 비례 감쇠 PD(ζ≈1.2 불변) + 고립 장애물 추월 + 연속 제동·횡가속도 제어
MAX_SPEED = 7.7  # m/s: 직선 최고 속도 (어댑터/물리 상한 8.0 m/s)
CORNER_MIN_SPEED = 2.75  # m/s: 급코너(시케인) 최소 통과 속도
BRAKE_DECEL = 4.3  # m/s^2: 코너 전 감속 곡선 기준 감속도 (물리 제동 여유 확보)
BRAKE_MARGIN = 1.85  # m: 코너 외측 벽 여유 거리
MAX_LAT_ACCEL = 6.0  # m/s^2: 조향각 기반 코너 횡가속도 한계 (mu=1.0 기준 안전 여유)
OVERTAKE_MAX_SPEED = 5.6  # m/s: 추월 중 최고 속도

STEERING_EMA_ALPHA = 0.90  # 조향 위상 지연(Phase lag) 최소화
MAX_STEERING_CHANGE = 0.20  # 코너 진입/탈출 최대 변화량
CORRIDOR_MAX_STEERING_CHANGE = 0.08  # 직선/완만 구간에서 급격한 발진 방지
# 기준 속도(3.5 m/s)에서의 연속 PD 게인 (v > 3.5에서는 v_scale로 자동 감쇠)
KP_NEAR = 0.013  # 측면 위치 오차(right_near - left_near) 비례 게인
KP_FAR = 0.011  # 전방 대각선(right_far - left_far) 선행 조향 게인
KD_SEC = 0.032  # 측면 오차 시간 미분(de/dt, m/s) 감쇠 게인
CORRIDOR_MAX_STEER = 0.12  # 직선/완만 구간 기본 조향 포화 한계 (고속 시 자동 축소)
# 출발 직후 급격한 대각선 돌진 방지 (초)
LAUNCH_SOFT_DURATION = 1.2  # 실제 이동 시작 후 1.2초 동안 최대 조향 완만히 제한
LAUNCH_MAX_STEERING = 0.048  # 출발 직후 Grid 편향 복귀 시 부드러운 합류 각도 (rad)
# 추월(Pace Car 회피) 파라미터
OVERTAKE_DETECT_DIST = 3.60  # m: 고속 주행 대비 전방 차량 조기 감지 거리
OVERTAKE_OPEN_MARGIN = 0.75  # m: 장애물 대비 옆 차선이 이 이상 열려 있으면 추월 판단
OVERTAKE_WALL_SAFE = 0.48  # m: 추월 중 외측 벽 최소 안전 거리
# 끼임(Stuck) 탈출 보험 파라미터 (정상 주행 무영향: 출발 3초 후 + 3.5초 이상 완전 정지/끼임 시에만 발동)
STUCK_ARM_AFTER_LAUNCH = 3.0  # 초: 출발 직후 그리드 혼전 구간에서는 발동 금지
STUCK_TRIGGER_SEC = 3.5  # 초: 2~3초 앞차 AEB 대기 중 불필요 후진 방지, 진짜 끼임 시에만 발동
RECOVERY_DURATION_SEC = 0.85  # 초: 약 0.6m 짧은 후진 (1회 회전각 ~30도 이내로 역주행 원천 차단)
RECOVERY_SPEED = -0.75  # m/s: 저속 안전 후진 (어댑터/물리 하한 -1.0 m/s)
RECOVERY_STEER = 0.28  # rad: 후진 중 코를 열린 방향으로 완만히 돌리는 조향각
RECOVERY_COOLDOWN_SEC = 3.0  # 초: 복구 종료 후 재발동 최소 쿨다운
RECOVERY_SAME_DIR_BLOCK_SEC = 15.0  # 초: 이 시간 내 연속 복구 시 동일 방향 조향 금지(직선 후진으로 90도+ 역주행 차단)
REAR_CLEAR_START = 1.25  # m: 후진 시작 최소 후방 LiDAR 거리 (저차고 뒤차 센서 마스트까지 고려)
REAR_CLEAR_ABORT = 1.05  # m: 후진 중 뒤차/벽 근접 시 즉시 중단 거리
# vehicle_adapter와 같은 중앙 조향 한계 (안쪽 바퀴 30도 제한).
MAX_STEERING = math.atan(0.32 / (0.32 / math.tan(math.pi / 6) + 0.085))
SCAN_TIMEOUT = 0.5  # 초: 입력이 끊기면 이전 명령으로 계속 달리지 않음
SECTORS = {
    "left": (45.0, 80.0),
    "front_left": (15.0, 45.0),
    "front": (-11.0, 11.0),
    "front_right": (-45.0, -15.0),
    "right": (-80.0, -45.0),
}


class RaceDriver(Node):

    def __init__(self):
        super().__init__("team11_race_driver")

        # Topics & start mode
        self.declare_parameter("race_start_mode", "signal")
        self.declare_parameter("race_start_topic", "/race/start/team_11")
        self.declare_parameter("race_start_transient_local", False)
        self.declare_parameter("speed_topic", "/team_11/speed")
        self.declare_parameter("steering_topic", "/team_11/steering")
        self.declare_parameter("lidar_topic", "/team_11/scan")
        self.declare_parameter("camera_topic", "/team_11/camera/image")

        raw_start_mode = str(
            self.get_parameter("race_start_mode").value
        ).strip().lower()
        self.start_mode = (
            raw_start_mode if raw_start_mode in ("signal", "camera") else "signal"
        )

        # 출발 조건
        self.race_allowed = False
        self.saw_red = False
        self.lights_off = False
        self.off_count = 0
        self.bg_frames = 0
        self.bg_red_mask = None
        self.peak_new_red = 0
        self.launch_time = None
        self.start_ready_time = None
        self.initial_left_near = None
        self.prev_near_error = None
        self.prev_scan_sec = None
        self.d_near_rate = 0.0
        self.stuck_since = None
        self.stuck_anchor_sig = None
        self.recovery_until = None
        self.recovery_steer = 0.0
        self.last_recovery_end = None
        self.last_recovery_steer = 0.0

        # LiDAR
        self.scan = None
        self.scan_time = None
        self.scan_pending = False
        self.steering = 0.0
        self.speed = 0.0

        # 명령 publisher
        self.speed_pub = self.create_publisher(
            Float32,
            self.get_parameter("speed_topic").value,
            10
        )

        self.steering_pub = self.create_publisher(
            Float32,
            self.get_parameter("steering_topic").value,
            10
        )

        # 공통 출발 허가 (내부 relay 사용 시 TRANSIENT_LOCAL로 초기 상태 보존)
        start_qos = (
            QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )
            if bool(self.get_parameter("race_start_transient_local").value)
            else 10
        )
        self.create_subscription(
            Bool,
            self.get_parameter("race_start_topic").value,
            self.on_start,
            start_qos
        )

        # LiDAR
        self.create_subscription(
            LaserScan,
            self.get_parameter("lidar_topic").value,
            self.on_scan,
            qos_profile_sensor_data
        )

        # Camera (camera 출발 모드에서만 신호등 감지에 사용)
        if self.start_mode == "camera":
            self.camera_sub = self.create_subscription(
                Image,
                self.get_parameter("camera_topic").value,
                self.on_image,
                qos_profile_sensor_data
            )
        else:
            self.camera_sub = None

        self.create_timer(0.05, self.control_step)

        self.get_logger().info(f"Team11 driver ready (start_mode={self.start_mode})")
        if self.start_mode == "camera":
            self.get_logger().info(
                "Waiting for race permission + red lights ON -> OFF"
            )
        else:
            self.get_logger().info("Waiting for race permission (Bool=true)")


    # --------------------------------------------------
    # 공통 출발 허가
    # --------------------------------------------------
    def on_start(self, msg):

        allowed = bool(msg.data)
        prev = self.race_allowed
        self.race_allowed = allowed

        if not self.race_allowed:
            self.launch_time = None
            self.start_ready_time = None
            self.initial_left_near = None
            self.prev_near_error = None
            self.prev_scan_sec = None
            self.d_near_rate = 0.0
            self.stuck_since = None
            self.stuck_anchor_sig = None
            self.recovery_until = None
            self.recovery_steer = 0.0
            self.last_recovery_end = None
            self.last_recovery_steer = 0.0
            self.speed = 0.0
            self.steering = 0.0
            self.publish_command(0.0, 0.0)
            if prev:
                self.get_logger().info("Race permission disabled -> STOP")
        elif not prev:
            self.get_logger().info(
                f"RACE PERMISSION RECEIVED (mode={self.start_mode})"
            )


    # --------------------------------------------------
    # LiDAR
    # --------------------------------------------------
    def on_scan(self, msg):
        self.scan = msg
        self.scan_time = self.get_clock().now()
        self.scan_pending = True
        self.control_step()


    # --------------------------------------------------
    # Camera 출발등 판독
    # --------------------------------------------------
    def on_image(self, msg):

        # 이미 소등 판독까지 완료했다면 더 볼 필요 없음
        if self.lights_off:
            return

        if msg.encoding != "rgb8":
            self.get_logger().warning(
                f"Expected rgb8, got {msg.encoding}"
            )
            return

        try:
            image = np.frombuffer(msg.data, dtype=np.uint8)
            image = image.reshape((msg.height, msg.width, 3))

        except ValueError:
            return

        h, w, _ = image.shape

        # 전방·후방 그리드(Grid 01~12) 모두 신호등이 잡히도록 상단 85% 영역 검사
        # (정적 배경의 빨간 연석·방호벽·정차 차량은 아래 bg_red_mask로 제외)
        y1 = 0
        y2 = int(h * 0.85)

        x1 = int(w * 0.10)
        x2 = int(w * 0.90)

        roi = image[y1:y2, x1:x2]

        r = roi[:, :, 0].astype(np.int16)
        g = roi[:, :, 1].astype(np.int16)
        b = roi[:, :, 2].astype(np.int16)

        # 순수 발광(emissive) 빨간 LED 픽셀만 추출 (주간 반사광 빨간 연석·방호벽·차량 원천 배제)
        red_mask = (
            (r > 215)
            & (g < 45)
            & (b < 45)
        )

        # 초기 정차 3프레임으로 정적 배경 빨간 요소(연석·방호벽 등)를 마스킹하여 맵별 오감지 차단
        if self.bg_frames < 3:
            if self.bg_red_mask is None or self.bg_red_mask.shape != red_mask.shape:
                self.bg_red_mask = red_mask.copy()
            else:
                self.bg_red_mask |= red_mask
            self.bg_frames += 1
            if self.bg_frames == 3:
                # ±2px 팽창으로 배경 경계 깜빡임 흡수
                dilated = self.bg_red_mask.copy()
                for dy in (-2, -1, 0, 1, 2):
                    for dx in (-2, -1, 0, 1, 2):
                        if dy == 0 and dx == 0:
                            continue
                        src_y0 = max(0, -dy)
                        src_y1 = red_mask.shape[0] - max(0, dy)
                        src_x0 = max(0, -dx)
                        src_x1 = red_mask.shape[1] - max(0, dx)
                        dst_y0 = max(0, dy)
                        dst_y1 = red_mask.shape[0] - max(0, -dy)
                        dst_x0 = max(0, dx)
                        dst_x1 = red_mask.shape[1] - max(0, -dx)
                        dilated[dst_y0:dst_y1, dst_x0:dst_x1] |= self.bg_red_mask[
                            src_y0:src_y1, src_x0:src_x1
                        ]
                self.bg_red_mask = dilated
                self.get_logger().info(
                    f"Camera baseline calibrated (static_red={int(np.count_nonzero(self.bg_red_mask))})"
                )
            return

        if self.bg_red_mask is not None and self.bg_red_mask.shape == red_mask.shape:
            new_red = int(np.count_nonzero(red_mask & (~self.bg_red_mask)))
        else:
            new_red = int(np.count_nonzero(red_mask))

        # 후방 그리드(Grid 07~12) 원거리 신호등까지 대응하는 상대 증가량 임계값
        ON_THRESHOLD = 10

        # 한 번이라도 출발등 점등을 확인
        if not self.saw_red:

            if new_red >= ON_THRESHOLD:

                self.saw_red = True
                self.peak_new_red = new_red

                self.get_logger().info(
                    f"RED LIGHT DETECTED! new_pixels={new_red}"
                )

        # 점등을 본 이후 → 소등 확인 (최대 점등량 대비 82% 이상 감소)
        else:
            if new_red > self.peak_new_red:
                self.peak_new_red = new_red
            off_threshold = max(5, int(0.18 * self.peak_new_red))

            if new_red <= off_threshold:
                self.off_count += 1
            else:
                self.off_count = 0

            # 2프레임(약 0.2초) 연속 소등이면 즉시 출발 인정
            if self.off_count >= 2:

                self.lights_off = True

                self.get_logger().info(
                    f"LIGHTS OFF -> CAMERA START CONDITION OK (peak={self.peak_new_red}, now={new_red})"
                )
                if self.camera_sub is not None:
                    self.destroy_subscription(self.camera_sub)
                    self.camera_sub = None


    # --------------------------------------------------
    # 특정 LiDAR 영역의 최소 거리
    # --------------------------------------------------
    def sector_min(self, start_deg, end_deg):

        if self.scan is None:
            return float("inf")

        values = []
        no_return = False

        for i, distance in enumerate(self.scan.ranges):

            angle = (
                self.scan.angle_min
                + i * self.scan.angle_increment
            )

            angle_deg = math.degrees(angle)

            if start_deg <= angle_deg <= end_deg:

                if (
                    math.isfinite(distance)
                    and max(0.01, self.scan.range_min)
                    <= distance
                    <= self.scan.range_max
                ):
                    values.append(distance)
                elif distance == float("inf"):
                    # Gazebo의 +inf는 측정 범위 내 반사체가 없다는 뜻.
                    no_return = True

        if not values:
            # NaN, -inf, 0만 있는 영역은 열린 공간으로 오인하지 않음.
            return self.scan.range_max if no_return else float("nan")

        return min(values)


    def sector_percentile(self, start_deg, end_deg, q=0.50):
        """측면/대각선 벽 거리 측정 시 좁은 Pace Car에 의해 값이 왜곡되지 않도록 분위수를 반환."""
        if self.scan is None:
            return float("inf")

        values = []
        no_return = False

        for i, distance in enumerate(self.scan.ranges):
            angle = self.scan.angle_min + i * self.scan.angle_increment
            angle_deg = math.degrees(angle)

            if start_deg <= angle_deg <= end_deg:
                if (
                    math.isfinite(distance)
                    and max(0.01, self.scan.range_min)
                    <= distance
                    <= self.scan.range_max
                ):
                    values.append(distance)
                elif distance == float("inf"):
                    values.append(self.scan.range_max)
                    no_return = True

        if not values:
            return self.scan.range_max if no_return else float("nan")

        values.sort()
        idx = int(max(0, min(len(values) - 1, round(q * (len(values) - 1)))))
        return values[idx]

    def rear_clearance(self):
        """후진 탈출 안전 확인용 후방 60도(±150~180도) 최소 LiDAR 거리."""
        if self.scan is None:
            return 0.0

        values = []
        no_return = False
        r_min = max(0.01, self.scan.range_min)
        r_max = self.scan.range_max

        for i, distance in enumerate(self.scan.ranges):
            angle_deg = math.degrees(
                self.scan.angle_min + i * self.scan.angle_increment
            )
            if angle_deg >= 150.0 or angle_deg <= -150.0:
                if math.isfinite(distance) and r_min <= distance <= r_max:
                    values.append(distance)
                elif distance == float("inf"):
                    no_return = True

        if not values:
            return r_max if no_return else 0.0
        return min(values)

    def find_corridor_obstacle(self, left_near, right_near):
        """전방 -20~+20도 내에서 곡선 벽과 분리된 실제 전방 차량(고립 장애물)만 검출."""
        if self.scan is None:
            return float("inf"), float("inf"), float("inf")

        # left_near/right_near(45~80도 중앙값, sin62≈0.88)로부터 좌/우 벽까지의 수직 거리 추정 후
        # 벽에서 최소 0.35m 이상 안쪽에 있는 점만 차선 내 후보로 고려
        left_wall_y = max(0.22, left_near * 0.88 - 0.35)
        right_wall_y = max(0.22, right_near * 0.88 - 0.35)
        r_min = max(0.01, self.scan.range_min)
        r_max = self.scan.range_max

        pts = []
        for i, raw_d in enumerate(self.scan.ranges):
            angle = self.scan.angle_min + i * self.scan.angle_increment
            angle_deg = math.degrees(angle)
            if not (-38.0 <= angle_deg <= 38.0):
                continue

            if raw_d == float("inf") or raw_d > r_max:
                dist = r_max
            elif math.isfinite(raw_d) and raw_d >= r_min:
                dist = raw_d
            else:
                continue

            lat_y = dist * math.sin(angle)
            in_corridor = (
                -20.0 <= angle_deg <= 20.0
                and dist < 5.5
                and (
                    (lat_y >= 0.0 and lat_y < left_wall_y)
                    or (lat_y < 0.0 and (-lat_y) < right_wall_y)
                )
            )
            pts.append((angle_deg, dist, in_corridor))

        if not pts:
            return float("inf"), float("inf"), float("inf")

        cand_indices = [idx for idx, (_, _, ok) in enumerate(pts) if ok]
        cand_indices.sort(key=lambda idx: pts[idx][1])
        visited = set()

        for k in cand_indices:
            if k in visited:
                continue
            d0 = pts[k][1]
            thresh = d0 + 0.40

            # 오른쪽(-angle 방향)으로 물체 끝과 배경 거리 점프 탐색
            r_idx = k
            right_opened = False
            right_jump = 0.0
            while r_idx > 0:
                nxt = r_idx - 1
                right_jump = max(right_jump, pts[nxt][1] - pts[r_idx][1])
                if pts[nxt][1] > thresh:
                    # 2~3빔 윈도우 누적 점프도 함께 확인 (모서리 기울기 대응)
                    if r_idx - 2 >= 0:
                        right_jump = max(right_jump, pts[r_idx - 2][1] - pts[r_idx][1])
                    if r_idx - 3 >= 0:
                        right_jump = max(right_jump, pts[r_idx - 3][1] - pts[r_idx][1])
                    right_opened = True
                    break
                r_idx = nxt
                visited.add(r_idx)

            # 왼쪽(+angle 방향)으로 물체 끝과 배경 거리 점프 탐색
            l_idx = k
            left_opened = False
            left_jump = 0.0
            n_pts = len(pts)
            while l_idx < n_pts - 1:
                nxt = l_idx + 1
                left_jump = max(left_jump, pts[nxt][1] - pts[l_idx][1])
                if pts[nxt][1] > thresh:
                    if l_idx + 2 < n_pts:
                        left_jump = max(left_jump, pts[l_idx + 2][1] - pts[l_idx][1])
                    if l_idx + 3 < n_pts:
                        left_jump = max(left_jump, pts[l_idx + 3][1] - pts[l_idx][1])
                    left_opened = True
                    break
                l_idx = nxt
                visited.add(l_idx)

            visited.add(k)
            # 양쪽 모두 배경(>d0+0.40m)이 열려 있고(곡선/시케인 벽은 한쪽이 -38/+38도까지 이어짐),
            # 물체 폭이 소형 차량 범위(<=0.85m)이며 최소 한쪽 경계에 급격한 거리 단차(>=0.55m)가 있어야 차량으로 인정
            if not (left_opened and right_opened):
                continue
            if max(left_jump, right_jump) < 0.55:
                continue

            span_deg = max(0.0, pts[l_idx][0] - pts[r_idx][0])
            chord_width = 2.0 * d0 * math.sin(math.radians(min(90.0, span_deg)) * 0.5)
            if chord_width > 0.85:
                continue

            obs_min = float("inf")
            left_obs_min = float("inf")
            right_obs_min = float("inf")
            for idx in range(r_idx, l_idx + 1):
                ang_deg, dist, ok = pts[idx]
                if not ok:
                    continue
                if dist < obs_min:
                    obs_min = dist
                if ang_deg >= 0.0 and dist < left_obs_min:
                    left_obs_min = dist
                elif ang_deg < 0.0 and dist < right_obs_min:
                    right_obs_min = dist
            if math.isfinite(obs_min):
                return obs_min, left_obs_min, right_obs_min

        return float("inf"), float("inf"), float("inf")

    def publish_command(self, speed, steering):

        self.speed_pub.publish(
            Float32(data=float(speed))
        )

        self.steering_pub.publish(
            Float32(data=float(steering))
        )


    # --------------------------------------------------
    # 자율주행
    # --------------------------------------------------
    def control_step(self):

        scan_fresh = False
        if self.scan_time is not None:
            try:
                scan_fresh = (
                    0.0
                    <= (self.get_clock().now() - self.scan_time).nanoseconds / 1e9
                    <= SCAN_TIMEOUT
                )
            except (TypeError, ValueError, RuntimeError):
                scan_fresh = False

        # signal 모드: race_allowed 즉시 출발 / camera 모드: 허가 + 빨간등 ON->OFF 확인 + 정상 LiDAR.
        if self.start_mode == "camera":
            start_ready = self.race_allowed and self.saw_red and self.lights_off
        else:
            start_ready = self.race_allowed

        if (
            not start_ready
            or self.scan is None
            or not scan_fresh
        ):
            self.steering = 0.0
            self.speed = 0.0
            self.publish_command(0.0, 0.0)
            return

        # 제어 타이머는 대기/타임아웃 안전용; 실제 주행 갱신은 on_scan 수신 직후 즉시 실행.
        if not self.scan_pending:
            return
        self.scan_pending = False

        sectors = {
            name: self.sector_min(*angles)
            for name, angles in SECTORS.items()
        }
        # 모든 값이 비정상이거나 영역 데이터가 없으면 안전하게 정지.
        valid_scan = (
            math.isfinite(self.scan.angle_min)
            and math.isfinite(self.scan.angle_increment)
            and self.scan.angle_increment > 0.0
            and math.isfinite(self.scan.range_min)
            and math.isfinite(self.scan.range_max)
            and self.scan.range_max > max(0.01, self.scan.range_min)
            and any(
                math.isfinite(d)
                and max(0.01, self.scan.range_min) <= d <= self.scan.range_max
                for d in self.scan.ranges
            )
            and all(math.isfinite(d) for d in sectors.values())
        )
        if not valid_scan:
            self.steering = 0.0
            self.speed = 0.0
            self.publish_command(0.0, 0.0)
            return

        # 양의 LaserScan 각도는 왼쪽, 조향 명령은 양수가 우회전.
        front = sectors["front"]
        ahead_narrow = self.sector_min(-3.5, 3.5)
        corridor_depth = self.sector_percentile(-12.0, 12.0, 0.80)
        forward_clear = self.sector_percentile(-18.0, 18.0, 0.85)
        # 벽 추종용 거리: 옆/앞 차선의 Pace Car 때문에 벽 거리가 급감하는 현상 방지 (far는 78분위수로 차량 뒤쪽 실제 벽 측정)
        left_near = self.sector_percentile(45.0, 80.0, 0.55)
        right_near = self.sector_percentile(-80.0, -45.0, 0.55)
        left_far = self.sector_percentile(20.0, 45.0, 0.78)
        right_far = self.sector_percentile(-45.0, -20.0, 0.78)
        left_far_cap = min(left_far, 8.0)
        right_far_cap = min(right_far, 8.0)

        now_sec = self.get_clock().now().nanoseconds / 1e9
        dt = (
            0.05
            if self.prev_scan_sec is None
            else max(0.025, min(0.25, now_sec - self.prev_scan_sec))
        )
        self.prev_scan_sec = now_sec

        if self.start_ready_time is None:
            self.start_ready_time = now_sec
        elapsed_since_ready = max(0.0, now_sec - self.start_ready_time)

        if self.initial_left_near is None:
            self.initial_left_near = left_near
        if self.launch_time is None and abs(left_near - self.initial_left_near) > 0.015:
            self.launch_time = now_sec
        elapsed_since_launch = (
            0.0 if self.launch_time is None else max(0.0, now_sec - self.launch_time)
        )

        # 측면 벽 위치 시간 변화율(de/dt, m/s) 계산 -> 속도 상승 시에도 감쇠비 ζ 유지
        near_error = right_near - left_near
        if self.prev_near_error is None:
            self.d_near_rate = 0.0
        else:
            raw_rate = max(-3.5, min(3.5, (near_error - self.prev_near_error) / dt))
            self.d_near_rate = 0.65 * raw_rate + 0.35 * self.d_near_rate
        self.prev_near_error = near_error

        # 속도에 따른 횡가속도(v^2/L) 보정 스케일 (ζ≈1.2 불변 유지)
        v_scale = min(1.0, (3.5 / max(3.5, self.speed)) ** 1.75)

        # 측면 벽이 아닌 차선 내부(-20~+20도, 벽에서 0.35m 이상 안쪽)의 실제 Pace Car만 검출
        obs_dist, front_left_obs, front_right_obs = self.find_corridor_obstacle(
            left_near, right_near
        )
        front_p80 = self.sector_percentile(-25.0, 25.0, 0.80)

        # 근접 충돌 방지용 최소 거리
        left_close_min = self.sector_min(30.0, 75.0)
        right_close_min = self.sector_min(-75.0, -30.0)

        # 추월 경로 탐색용 좌/우 전방 공간 (직선 벽 삼각측량 한계를 피하기 위해 6~24도 집중)
        left_pass = self.sector_percentile(6.0, 24.0, 0.70)
        right_pass = self.sector_percentile(-24.0, -6.0, 0.70)

        # 코너 회전 방향 판단용 좌/우 여유 공간
        left_corner = 0.5 * left_near + 0.5 * left_far_cap
        right_corner = 0.5 * right_near + 0.5 * right_far_cap

        # 전방 장애물(Pace Car) vs 실제 코너 판별:
        # 차선 내부에 가까운 차량(obs_dist < 3.60m)이 있고, 주변 80% 분위수 및 패싱 차선이 열려 있으면 Pace Car!
        overtake_active = (
            obs_dist < OVERTAKE_DETECT_DIST
            and front_p80 > 2.40
            and front_p80 > obs_dist + 0.85
            and max(left_pass, right_pass) > obs_dist + OVERTAKE_OPEN_MARGIN
            and abs(left_far_cap - right_far_cap) < 1.25
            and (left_near + right_near) < 3.40
        )

        # 실제 코너인지 판별 (직선에서 차체가 살짝 기울어도 -18~+18도 85% 거리는 열려 있음)
        real_corner = (not overtake_active) and (
            forward_clear < 3.40 or max(left_far_cap, right_far_cap) < 3.00 or front < 1.45
        )
        corridor_mode = False
        corridor_steer_cap = min(
            CORRIDOR_MAX_STEER,
            max(0.042, CORRIDOR_MAX_STEER * (3.5 / max(3.5, self.speed)) ** 1.6),
        )
        corridor_change_limit = min(
            CORRIDOR_MAX_STEERING_CHANGE,
            max(0.025, CORRIDOR_MAX_STEERING_CHANGE * (3.5 / max(3.5, self.speed)) ** 1.6),
        )

        if not overtake_active and front < 0.80:
            target = -0.38 if left_corner > right_corner else 0.38
        elif real_corner and front < 1.48 and ahead_narrow < 2.30:
            target = -0.28 if left_corner > right_corner else 0.28
        else:
            corridor_mode = not real_corner and not overtake_active
            # 1) 속도 비례 과감쇠 연속 PD 제어 (실제 코너 진입 시에는 2.2배 증폭해 회전력 확보)
            far_diff = max(-2.8, min(2.8, right_far_cap - left_far_cap))
            gain_mult = 2.2 if real_corner else 1.0
            target = v_scale * (
                gain_mult * (KP_NEAR * near_error + KP_FAR * far_diff)
                + KD_SEC * self.d_near_rate
            )

            # 2) 전방 Pace Car 추월 바이어스: 장애물 반대쪽 열린 차선으로 부드럽게 회피 조향 추가
            if overtake_active:
                if abs(front_left_obs - front_right_obs) > 0.25:
                    # 왼쪽 전방에 차가 있으면 우측 추월(False), 오른쪽 전방에 차가 있으면 좌측 추월(True)
                    pass_left = front_left_obs > front_right_obs
                else:
                    pass_left = left_pass > right_pass

                # 만약 선택하려는 쪽 벽이 이미 너무 가까우면 반대쪽 공간 확인
                if pass_left and left_near < OVERTAKE_WALL_SAFE and right_pass > obs_dist + 0.4:
                    pass_left = False
                elif not pass_left and right_near < OVERTAKE_WALL_SAFE and left_pass > obs_dist + 0.4:
                    pass_left = True

                urgency = max(
                    0.35,
                    min(0.95, (OVERTAKE_DETECT_DIST - obs_dist) / 1.50),
                )
                steer_bias = 0.085 * urgency * min(1.0, 3.5 / max(3.5, self.speed))
                # pass_left이면 좌회전(음수 조향) 유도, 우측 추월이면 우회전(양수 조향) 유도
                target += -steer_bias if pass_left else steer_bias

            # 3) 측면 근접 보호 (옆 차선 차량이나 벽에 너무 붙었을 때 밀어냄)
            if left_close_min < OVERTAKE_WALL_SAFE:
                target += 0.22 * (OVERTAKE_WALL_SAFE - left_close_min)
            if right_close_min < OVERTAKE_WALL_SAFE:
                target -= 0.22 * (OVERTAKE_WALL_SAFE - right_close_min)

            # 직선/완만 구간에서는 속도 반비례 포화 한계로 오버슈트 방지, 실제 코너에서는 최대 0.26 rad 허용
            steer_cap = corridor_steer_cap if corridor_mode else 0.26
            target = max(-steer_cap, min(steer_cap, target))

            # 외측 벽 침범 하드 가드: 벽과 가까우면서 그쪽 벽으로 더 꺾으려 하면 차단 (반동 방지 0.0)
            if left_near < OVERTAKE_WALL_SAFE and target < 0.0:
                target = max(0.0, target)
            elif right_near < OVERTAKE_WALL_SAFE and target > 0.0:
                target = min(0.0, target)

        # 출발 직후 급격히 중앙으로 꺾어 반대편 벽에 박는 현상 방지
        if elapsed_since_launch < LAUNCH_SOFT_DURATION and front > 1.50:
            launch_lim = 0.10 if overtake_active else LAUNCH_MAX_STEERING
            target = max(-launch_lim, min(launch_lim, target))

        # EMA → 한 프레임 변화 제한 → 차량 조향 한계 순으로 적용
        smoothed = (
            STEERING_EMA_ALPHA * target
            + (1.0 - STEERING_EMA_ALPHA) * self.steering
        )
        change_limit = (
            corridor_change_limit
            if (corridor_mode and abs(self.steering) <= corridor_steer_cap)
            else MAX_STEERING_CHANGE
        )
        change = max(
            -change_limit,
            min(change_limit, smoothed - self.steering),
        )
        self.steering = max(
            -MAX_STEERING, min(MAX_STEERING, self.steering + change)
        )

        # 연속 제동 거리 곡선 + 조향 횡가속도 한계 기반 속도 제어 (최대 7.4 m/s)
        turn = max(abs(target), abs(self.steering))
        d_brake = min(corridor_depth, ahead_narrow)
        if not overtake_active and obs_dist < 4.5:
            d_brake = min(d_brake, obs_dist)

        v_brake = math.sqrt(
            CORNER_MIN_SPEED ** 2 + 2.0 * BRAKE_DECEL * max(0.0, d_brake - BRAKE_MARGIN)
        )
        v_steer = (
            MAX_SPEED
            if turn <= 0.055
            else math.sqrt(MAX_LAT_ACCEL * 0.32 / max(0.03, turn))
        )
        self.speed = min(MAX_SPEED, v_brake, v_steer)

        if front < 0.85:
            self.speed = min(self.speed, 1.95)
        elif front < 1.25:
            self.speed = min(self.speed, 2.45)

        if overtake_active:
            if obs_dist < 0.70:
                self.speed = min(self.speed, 1.1)
            elif obs_dist < 1.15:
                self.speed = min(self.speed, 2.2)
            elif obs_dist < 1.60:
                self.speed = min(self.speed, 2.75)
            else:
                self.speed = min(self.speed, OVERTAKE_MAX_SPEED)
        elif obs_dist < 1.80:
            # 우회 불가(코너·좁은 구간) 시 느린/정지 앞차 추종 감속
            if obs_dist < 0.50:
                self.speed = 0.0
            elif obs_dist < 0.80:
                self.speed = min(self.speed, 0.75)
            elif obs_dist < 1.25:
                self.speed = min(self.speed, 1.40)
            else:
                self.speed = min(self.speed, 2.10)

        # 초근접 정면 AEB: 정지 차량·병렬 차단 시 추돌 방지 (단독 주행 min_front=0.55m 무영향)
        close_front = min(front, ahead_narrow)
        if close_front < 0.45:
            escape_open = (
                max(left_corner, right_corner) > 0.75
                and max(left_pass, right_pass) > 0.90
            )
            if close_front < 0.28 or not escape_open:
                self.speed = 0.0
            else:
                self.speed = min(self.speed, 0.75)

        if elapsed_since_launch < 0.6:
            self.speed = min(self.speed, 4.5)

        # 끼임(Stuck) 탈출 보험: 출발 3초 이후 완전히 막혀 3.5초 이상 멈춘 경우에만 후방 확인 후 짧게 후진
        if self.recovery_until is not None:
            rear_min = self.rear_clearance()
            if (
                now_sec >= self.recovery_until
                or not math.isfinite(rear_min)
                or rear_min < REAR_CLEAR_ABORT
            ):
                self.recovery_until = None
                self.stuck_since = None
                self.stuck_anchor_sig = None
                self.last_recovery_end = now_sec
                self.get_logger().info(
                    f"STUCK RECOVERY END (front={front:.2f}, rear={rear_min:.2f}) -> RESUME FORWARD"
                )
            else:
                self.speed = RECOVERY_SPEED
                self.steering = self.recovery_steer
                self.publish_command(self.speed, self.steering)
                self.get_logger().info(
                    f"RECOVERY front={front:.2f} rear={rear_min:.2f} "
                    f"speed={self.speed:.2f} steer={self.steering:.2f}"
                )
                return

        in_recovery_cooldown = (
            self.last_recovery_end is not None
            and (now_sec - self.last_recovery_end) < RECOVERY_COOLDOWN_SEC
        )
        if elapsed_since_ready > STUCK_ARM_AFTER_LAUNCH and not in_recovery_cooldown:
            # 정상 AEB 대기(좁은 구간·전면 차단으로 탈출 공간 없음: L/R_cor <= 0.85m)와 실제 끼임을 구분:
            # 빠져나갈 열린 공간(max(left_corner, right_corner) > 0.85m)이 있을 때만 복구 감시
            escape_path_exists = max(left_corner, right_corner) > 0.85
            trapped_close = (
                close_front < 0.42
                or min(sectors["front_left"], sectors["front_right"]) < 0.28
            )
            if (self.speed == 0.0 or trapped_close) and escape_path_exists:
                if self.stuck_since is None:
                    self.stuck_since = now_sec
                    self.stuck_anchor_sig = (front, left_near, right_near)
                else:
                    a_f, a_l, a_r = self.stuck_anchor_sig
                    moved = (
                        abs(front - a_f) > 0.12
                        or abs(left_near - a_l) > 0.12
                        or abs(right_near - a_r) > 0.12
                    )
                    if self.speed > 0.0 and moved:
                        self.stuck_since = now_sec
                        self.stuck_anchor_sig = (front, left_near, right_near)
                    elif now_sec - self.stuck_since >= STUCK_TRIGGER_SEC:
                        rear_min = self.rear_clearance()
                        if math.isfinite(rear_min) and rear_min >= REAR_CLEAR_START:
                            # 후진 시 steering>0이면 코가 좌측(CCW), steering<0이면 코가 우측(CW)으로 회전
                            desired_steer = (
                                RECOVERY_STEER
                                if left_corner > right_corner
                                else -RECOVERY_STEER
                            )
                            # 연속 복구 시 동일 방향 조향 누적(90도+ 역주행) 차단: 직선 후진(0.0)으로 제한
                            if (
                                self.last_recovery_end is not None
                                and (now_sec - self.last_recovery_end) < RECOVERY_SAME_DIR_BLOCK_SEC
                                and self.last_recovery_steer * desired_steer > 0.0
                            ):
                                desired_steer = 0.0
                            self.recovery_steer = desired_steer
                            if desired_steer != 0.0:
                                self.last_recovery_steer = desired_steer
                            self.recovery_until = now_sec + RECOVERY_DURATION_SEC
                            self.speed = RECOVERY_SPEED
                            self.steering = self.recovery_steer
                            self.publish_command(self.speed, self.steering)
                            self.get_logger().info(
                                f"STUCK RECOVERY START (front={front:.2f}, rear={rear_min:.2f}, "
                                f"L_cor={left_corner:.2f}, R_cor={right_corner:.2f}, steer={self.recovery_steer:.2f})"
                            )
                            return
            else:
                self.stuck_since = None
                self.stuck_anchor_sig = None
        else:
            self.stuck_since = None
            self.stuck_anchor_sig = None

        self.publish_command(self.speed, self.steering)
        self.get_logger().info(
            f"L_near={left_near:.2f} R_near={right_near:.2f} "
            f"obs={obs_dist:.2f} d_brk={d_brake:.2f} front={front:.2f} fwd85={forward_clear:.2f} "
            f"ovtk={int(overtake_active)} speed={self.speed:.2f} steer={self.steering:.2f}"
        )


def main():

    rclpy.init()

    node = RaceDriver()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:

        # Ctrl+C 때 종료 오류가 나지 않게 ROS context 확인
        try:
            if rclpy.ok():
                node.publish_command(0.0, 0.0)
        except Exception:
            pass

        try:
            node.destroy_node()
        except Exception:
            pass

        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()