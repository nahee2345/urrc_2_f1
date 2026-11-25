#!/usr/bin/env python3

import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from rclpy.qos import qos_profile_sensor_data


class StartLightDetector(Node):

    def __init__(self):
        super().__init__("start_light_detector")

        self.saw_red = False
        self.off_count = 0
        self.finished = False

        self.create_subscription(
            Image,
            "/team_11/camera/image",
            self.image_callback,
            qos_profile_sensor_data,
        )

        self.get_logger().info("Start light detector ready")


    def image_callback(self, msg):

        if self.finished:
            return

        if msg.encoding != "rgb8":
            self.get_logger().warning(
                f"Expected rgb8, got {msg.encoding}"
            )
            return

        # ROS Image -> numpy RGB image
        image = np.frombuffer(msg.data, dtype=np.uint8)
        image = image.reshape((msg.height, msg.width, 3))

        h, w, _ = image.shape

        # 출발등은 화면 위쪽에 있으므로 위쪽 중앙만 검사
        y1 = 0
        y2 = int(h * 0.30)

        x1 = int(w * 0.10)
        x2 = int(w * 0.90)

        roi = image[y1:y2, x1:x2]

        r = roi[:, :, 0].astype(np.int16)
        g = roi[:, :, 1].astype(np.int16)
        b = roi[:, :, 2].astype(np.int16)

        # 빨간색 픽셀 판정
        red_mask = (
            (r > 150)
            & (r > g * 1.4)
            & (r > b * 1.4)
        )

        red_pixels = int(np.count_nonzero(red_mask))

        # 이 값들은 테스트용 튜닝값
        ON_THRESHOLD = 50
        OFF_THRESHOLD = 15

        if not self.saw_red:

            if red_pixels > ON_THRESHOLD:
                self.saw_red = True
                self.get_logger().info(
                    f"RED LIGHT DETECTED! pixels={red_pixels}"
                )

        else:

            if red_pixels < OFF_THRESHOLD:
                self.off_count += 1
            else:
                self.off_count = 0

            # 약 0.5초 정도 연속 소등 확인
            if self.off_count >= 5:
                self.finished = True

                self.get_logger().info(
                    "=================================="
                )
                self.get_logger().info(
                    "LIGHTS OFF -> START CONDITION DETECTED!"
                )
                self.get_logger().info(
                    "=================================="
                )


def main():

    rclpy.init()

    node = StartLightDetector()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()