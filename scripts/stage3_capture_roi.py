#!/usr/bin/env python3
"""실제 ROS 카메라 프레임을 PPM 파일로 저장해 신호등 영역을 교정한다."""
from __future__ import annotations
import argparse
from pathlib import Path
import sys

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from stage3.vision import rgb_from_ros


def main():
    p=argparse.ArgumentParser()
    p.add_argument("--topic",required=True,help="예: /model/urrc_ai_01/sensors/start_camera")
    p.add_argument("--out",type=Path,default=ROOT/"generated/stage3_start_camera.ppm")
    p.add_argument("--timeout",type=float,default=20.)
    args=p.parse_args()
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image

    class Capture(Node):
        def __init__(self):
            super().__init__("urrc_stage3_capture")
            self.done=False
            self.create_subscription(Image,args.topic,self.callback,qos_profile_sensor_data)
        def callback(self,msg):
            try:
                w,h,data=rgb_from_ros(msg)
                args.out.parent.mkdir(parents=True,exist_ok=True)
                args.out.write_bytes(f"P6\n{w} {h}\n255\n".encode()+data)
                print(f"카메라 이미지 저장: {args.out}, {w}x{h}")
                self.done=True
            except ValueError as exc:
                self.get_logger().error(str(exc))

    rclpy.init()
    node=Capture()
    import time
    end=time.monotonic()+args.timeout
    try:
        while not node.done and time.monotonic()<end:
            rclpy.spin_once(node,timeout_sec=.15)
        if not node.done:raise TimeoutError("출발 카메라 이미지 수신 실패")
    finally:
        node.destroy_node();rclpy.shutdown()

if __name__=="__main__":main()
