"""ROS 2 Jazzy 출발 안전 노드.

이 노드는 '출발 가능' 여부만 발행한다. 실제 AI 주행 명령은 다음 단계에서
start_allowed 신호와 차량 주행 센서만 활용하도록 별도 작성한다.
"""
from __future__ import annotations
import argparse
import json
from pathlib import Path
from .vision import LightOutGate,rgb_from_ros

def load_calibration(path):
    c=json.loads(Path(path).read_text())
    if c.get("calibrated") is not True:
        raise ValueError("시작 카메라 영역이 검증되지 않았습니다. 실제 영상으로 ROI 5개를 먼저 교정하세요.")
    if c.get("source_sensor")!="start_camera":
        raise ValueError("출발 판정은 start_camera 이미지에만 허용됩니다")
    if len(c.get("rois",[]))!=5:
        raise ValueError("빨간 신호등 ROI 5개 필요")
    return LightOutGate(c["rois"],stable_frames=int(c.get("stable_frames",3)))


def main():
    ap=argparse.ArgumentParser(description="카메라 5개 빨간등 소등 기반 안전 출발")
    ap.add_argument("--vehicle",required=True)
    ap.add_argument("--entry-id",required=True)
    ap.add_argument("--calibration",type=Path,required=True)
    ap.add_argument("--camera-topic")
    ap.add_argument("--permission-topic",help="격리 실험에서만 사용하는 독립 운영자 출발허가 토픽")
    args,ros_args=ap.parse_known_args()
    gate=load_calibration(args.calibration)
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image
    from std_msgs.msg import Bool
    camera=args.camera_topic or f"/model/{args.vehicle}/sensors/start_camera"
    out=f"/urrc_2_f1/{args.vehicle}/start_allowed"
    permit=args.permission_topic or f"/race/start/{args.entry_id}"
    if args.permission_topic and args.permission_topic.startswith("/race/"):
        raise ValueError("격리 실험의 출발 허가 토픽에 공식 /race 경로를 사용할 수 없습니다")

    class StartNode(Node):
        def __init__(self):
            super().__init__("urrc_stage3_visual_start")
            self.allowed=False
            self.last_frame=None
            self.pub=self.create_publisher(Bool,out,10)
            self.create_subscription(Bool,permit,self.on_permit,10)
            self.create_subscription(Image,camera,self.on_image,qos_profile_sensor_data)
            self.timer=self.create_timer(.05,self.publish_state)
            self.get_logger().info("카메라 출발 게이트 준비, 차단 상태 유지")
        def on_permit(self,msg):
            self.allowed=bool(msg.data)
            if not self.allowed and gate.state=="RELEASED":
                gate.reset()
            self.publish_state()
        def on_image(self,msg):
            try:
                width,height,rgb=rgb_from_ros(msg)
                stamp=float(msg.header.stamp.sec)+msg.header.stamp.nanosec/1e9
                self.last_frame=stamp
                prior_state=gate.state
                triggered=gate.update(width,height,rgb,stamp,self.allowed)
                if triggered and prior_state!="RELEASED" and rclpy.ok():
                    self.get_logger().info("5개 점등 확인 후 전체 소등을 카메라로 확인: 출발 허가")
            except Exception as exc:
                gate.reset()
                self.get_logger().error(f"카메라 오류 - 출발 차단: {exc}")
            self.publish_state()
        def publish_state(self):
            # SIGINT로 이미 ROS 컨텍스트가 종료된 경우 publish() 금지.
            if not rclpy.ok():
                return
            state=Bool()
            # 프레임 갱신이 끊기면 이미 출발 허가 상태라도 안전하게 취소.
            if self.last_frame is None:
                state.data=False
            else:
                now=self.get_clock().now().nanoseconds/1e9
                if now-self.last_frame > .6 or now < self.last_frame:
                    gate.reset()
                    state.data=False
                else:
                    state.data=bool(self.allowed and gate.state=="RELEASED")
            if rclpy.ok():
                try:
                    self.pub.publish(state)
                except Exception:
                    if rclpy.ok():
                        raise

    rclpy.init(args=ros_args)
    node=StartNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt,):
        pass
    finally:
        node.destroy_node()
        # ROS 2 Jazzy가 SIGINT 처리 중 먼저 종료했을 수 있음.
        if rclpy.ok():
            rclpy.try_shutdown()

if __name__=="__main__":
    main()
