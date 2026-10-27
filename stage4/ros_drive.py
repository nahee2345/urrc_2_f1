"""AI-01 ROS2 Jazzy 실제 RGB 카메라 -> 물리 구동 / Ackermann 조향.

start_allowed는 stage3/ros_gate.py가 실제 카메라에서 5등 점등→전체 소등을
확인했을 때만 True로 발행하는 필수 안전 입력이다.
시뮬레이션 정답 위치·centerline·운영자 pose 토픽은 일절 구독하지 않는다.
"""
from __future__ import annotations
import argparse
from dataclasses import replace
from pathlib import Path
import math
import time
from vehicle_generator.model import Vehicle
from .lane import estimate_ros,LaneEstimate
from .safety import CameraDrive

ROOT=Path(__file__).resolve().parents[1]

def main():
    parser=argparse.ArgumentParser(description="4단계 AI-01 카메라 주행·출발 안전 인터록")
    parser.add_argument("--vehicle",default="urrc_ai_01")
    parser.add_argument("--speed-limit",type=float,default=1.10)
    args,ros_args=parser.parse_known_args()
    if args.vehicle!="urrc_ai_01":
        raise ValueError("4단계에서는 AI-01만 지원합니다")
    if not math.isfinite(args.speed_limit) or not .15<=args.speed_limit<=1.3:
        raise ValueError("실험 속도 제한은 0.15~1.3m/s")
    from rclpy.qos import qos_profile_sensor_data
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image
    from std_msgs.msg import Bool,Float64

    car=replace(Vehicle.from_file(ROOT/"config/stage1_rwd.json"),
                name=args.vehicle,drive="fwd")
    driver=CameraDrive(car,speed_limit_mps=args.speed_limit)
    rgb=f"/model/{args.vehicle}/sensors/rgb"
    permit=f"/urrc_2_f1/{args.vehicle}/start_allowed"

    class DriveNode(Node):
        def __init__(self):
            super().__init__("urrc_stage4_ai01_rgb_drive")
            self.last_error=0.
            self.last_warn=0.
            self.cmd={}
            self.gate_sub=self.create_subscription(Bool,permit,self.on_gate,10)
            self.rgb_sub=self.create_subscription(Image,rgb,self.on_image,qos_profile_sensor_data)
            for wheel in ("front_left","front_right"):
                topic=f"/model/{args.vehicle}/drive/{wheel}"
                self.cmd[wheel]=self.create_publisher(Float64,topic,10)
            for side in ("left","right"):
                self.cmd["steer_"+side]=self.create_publisher(
                    Float64,f"/model/{args.vehicle}/steer/{side}",10)
            self.timer=self.create_timer(.08,self.control)
            self.get_logger().info("AI-01 실제 카메라 차선 검출 준비, 출발 전 모든 구동 0")
        def on_gate(self,msg):
            try:driver.permission(bool(msg.data),time.monotonic())
            except ValueError:driver.started=False
        def on_image(self,msg):
            try:
                est=estimate_ros(msg,self.last_error)
                driver.camera(est,time.monotonic())
                if est.valid:self.last_error=est.error
                else:self.last_error=0.
            except (ValueError,TypeError) as err:
                driver.camera(LaneEstimate(False,reason=str(err)),time.monotonic())
        def publish(self,key,value):
            message=Float64()
            message.data=float(value)
            self.cmd[key].publish(message)
        def control(self):
            try:
                out=driver.tick(time.monotonic())
                for wheel in ("front_left","front_right"):
                    self.publish(wheel,out["drive_wheel_radps"][wheel])
                self.publish("steer_left",out["steer_left_rad"])
                self.publish("steer_right",out["steer_right_rad"])
                if not out["allowed"] and time.monotonic()-self.last_warn>6:
                    self.get_logger().warn(f"출발/주행 차단: {out['reason']}")
                    self.last_warn=time.monotonic()
            except Exception as exc:
                self.get_logger().error(f"제어 오류, 0 속도 전송: {exc}")
                for key in self.cmd:
                    self.publish(key,0.)

    rclpy.init(args=ros_args)
    node=DriveNode()
    try:rclpy.spin(node)
    finally:
        for key in node.cmd:
            node.publish(key,0.)
        node.destroy_node()
        rclpy.shutdown()

if __name__=="__main__":main()
