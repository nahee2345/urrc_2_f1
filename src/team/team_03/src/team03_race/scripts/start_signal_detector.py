#!/usr/bin/env python3
"""
URRC Team03 F1 Start Signal Detector
- Gazebo 카메라 상단 ROI에서 5구 적색 LED 감지
- 점등 후 일제히 소등(Lights Out)되는 순간 /race/start/team03 토픽 발행
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import cv2
import numpy as np

class StartSignalDetector(Node):
    def __init__(self):
        super().__init__('start_signal_detector_03')

        self.bridge = CvBridge()
        self.start_pub = self.create_publisher(Bool, '/team03/camera_lights_out_03', 10)
        self.image_sub = self.create_subscription(
            Image,
            '/team03/camera/image_03',
            self.image_callback,
            10
        )

        self.has_seen_red = False
        self.race_started = False
        self.get_logger().info('>>> Start Signal Detector Started. Waiting for Red Lights... <<<')

    def image_callback(self, msg):
        if self.race_started:
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'CvBridge error: {e}')
            return

        # 상단 45% ROI 추출 (신호등 영역)
        h, w, _ = cv_image.shape
        roi = cv_image[0:int(h * 0.45), :]

        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        # 빨간색 HSV 대역 필터링
        mask1 = cv2.inRange(hsv, np.array([0, 100, 120]), np.array([10, 255, 255]))
        mask2 = cv2.inRange(hsv, np.array([170, 100, 120]), np.array([180, 255, 255]))
        red_mask = cv2.bitwise_or(mask1, mask2)

        red_pixels = cv2.countNonZero(red_mask)
        self.get_logger().info(
        f"RED_PIXELS={red_pixels}, "
        f"SEEN_RED={self.has_seen_red}, "
        f"STARTED={self.race_started}",
        throttle_duration_sec=1.0
        )

        # 1. 붉은 불 점등 확인
        if red_pixels > 60:
            if not self.has_seen_red:
                self.has_seen_red = True
                self.get_logger().info(f'[SIGNAL] RED LIGHTS DETECTED (Count: {red_pixels}). Standby...')

        # 2. 점등 확인 후 전구 일제히 소등 감지 (Lights Out)
        elif self.has_seen_red and red_pixels < 25:
            self.race_started = True
            self.get_logger().info('>>> [LIGHTS OUT] RACE START SIGNAL PUBLISHED! <<<')

            for _ in range(10):
                self.start_pub.publish(Bool(data=True))

def main():
    rclpy.init()
    node = StartSignalDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
