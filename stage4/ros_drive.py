"""AI-01 ROS2 Jazzy 실제 RGB 카메라 -> 물리 구동 / Ackermann 조향.

start_allowed는 stage3/ros_gate.py가 실제 카메라에서 5등 점등→전체 소등을
확인했을 때만 True로 발행하는 필수 안전 입력이다.
시뮬레이션 정답 위치·centerline·운영자 pose 토픽은 일절 구독하지 않는다.
"""
from __future__ import annotations
import argparse
import json
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
    parser.add_argument("--status-file",type=Path,help="실제 카메라/모터 제어 상태 JSON 로그")
    parser.add_argument("--image-file",type=Path,help="실제 RGB 카메라 첫 프레임 PPM 진단 사진")
    parser.add_argument("--lost-image-file",type=Path,help="주행 중 도로 검출 실패 직전의 실제 RGB 프레임 PPM")
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
            self.last_status=0.
            self.camera_frames=0
            self.gate_messages=0
            self.image_saved=False
            self.had_valid_road=False
            self.lost_image_saved=False
            self.last_est=LaneEstimate(False,reason="첫 RGB 프레임 미수신")
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
            if not rclpy.ok():return
            self.gate_messages+=1
            try:driver.permission(bool(msg.data),time.monotonic())
            except ValueError:driver.started=False
        def on_image(self,msg):
            if not rclpy.ok():return
            self.camera_frames+=1
            try:
                est=estimate_ros(msg,self.last_error)
                self.last_est=est
                driver.camera(est,time.monotonic())
                if est.valid:
                    self.last_error=est.error
                    self.had_valid_road=True
                else:
                    self.last_error=0.
                if (args.lost_image_file and not est.valid and
                        self.had_valid_road and driver.started and
                        not self.lost_image_saved):
                    from stage3.vision import rgb_from_ros
                    w,h,data=rgb_from_ros(msg)
                    args.lost_image_file.parent.mkdir(parents=True,exist_ok=True)
                    args.lost_image_file.write_bytes(
                        f"P6\n{w} {h}\n255\n".encode()+data)
                    self.lost_image_saved=True
                    self.get_logger().warn(
                        f"실제 도로 검출 상실 화면 저장: {args.lost_image_file} / {est.reason}")
                if args.image_file and not self.image_saved:
                    from stage3.vision import rgb_from_ros
                    w,h,rgb_bytes=rgb_from_ros(msg)
                    args.image_file.parent.mkdir(parents=True,exist_ok=True)
                    args.image_file.write_bytes(
                        f"P6\n{w} {h}\n255\n".encode()+rgb_bytes)
                    self.image_saved=True
                    self.get_logger().info(f"실제 주행 RGB 카메라 화면: {args.image_file}")
            except (ValueError,TypeError) as err:
                self.last_est=LaneEstimate(False,reason=str(err))
                driver.camera(self.last_est,time.monotonic())
        def publish(self,key,value):
            if not rclpy.ok():
                return
            message=Float64()
            message.data=float(value)
            try:
                self.cmd[key].publish(message)
            except Exception:
                if rclpy.ok():
                    raise
        def control(self):
            if not rclpy.ok():return
            try:
                out=driver.tick(time.monotonic())
                for wheel in ("front_left","front_right"):
                    self.publish(wheel,out["drive_wheel_radps"][wheel])
                self.publish("steer_left",out["steer_left_rad"])
                self.publish("steer_right",out["steer_right_rad"])
                now=time.monotonic()
                if now-self.last_status>=3:
                    self.last_status=now
                    data={"vehicle":args.vehicle,"camera_frames":self.camera_frames,
                          "gate_messages":self.gate_messages,
                          "gate_received":self.gate_messages>0,
                          "start_allowed":driver.started,
                          "lane_valid":bool(self.last_est.valid),
                          "lane_reason":self.last_est.reason,
                          "lane_samples":self.last_est.samples,
                          "lane_confidence":round(self.last_est.confidence,4),
                          "lane_error":round(self.last_est.error,4),
                          "control_allowed":out["allowed"],
                          "stop_reason":out["reason"],
                          "command_speed_mps":round(out["speed_mps"],4),
                          "command_steer_rad":round(out["race_steer_rad"],4),
                          "front_left_radps":round(out["drive_wheel_radps"]["front_left"],4),
                          "front_right_radps":round(out["drive_wheel_radps"]["front_right"],4),
                          "image_saved":self.image_saved,
                          "lost_image_saved":self.lost_image_saved}
                    if args.status_file:
                        args.status_file.parent.mkdir(parents=True,exist_ok=True)
                        tmp=args.status_file.with_suffix(".tmp")
                        tmp.write_text(json.dumps(data,ensure_ascii=False,indent=2)+"\n")
                        tmp.replace(args.status_file)
                    self.get_logger().info(
                        f"실제 RGB {self.camera_frames}프레임, 출발허가={driver.started}, "
                        f"도로검출={self.last_est.valid} "
                        f"(구간 {self.last_est.samples}, 신뢰 {self.last_est.confidence:.2f}), "
                        f"명령속도={out['speed_mps']:.3f}m/s, "
                        f"앞바퀴={data['front_left_radps']:.2f}rad/s, "
                        f"상태={out['reason']}")
                if not out["allowed"] and now-self.last_warn>6:
                    self.get_logger().warn(f"출발/주행 차단: {out['reason']}")
                    self.last_warn=now
            except Exception as exc:
                self.get_logger().error(f"제어 오류, 0 속도 전송: {exc}")
                if rclpy.ok():
                    for key in self.cmd:
                        self.publish(key,0.)

    rclpy.init(args=ros_args)
    node=DriveNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            for key in node.cmd:
                node.publish(key,0.)
        node.destroy_node()
        if rclpy.ok():
            rclpy.try_shutdown()

if __name__=="__main__":main()
