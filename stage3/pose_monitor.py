"""운영자 전용 ROS 2 랩 감독.

/clock와 ROS-Gazebo PosePublisher TFMessage를 사용하되, AI 운전에는
어떤 트랙 좌표·운영자 계측 값도 노출하지 않는다.
"""
from __future__ import annotations
import argparse
import importlib.util
import json
from pathlib import Path
import sys
from .lap import LapSupervisor,gates_from_upstream
from .contact import ContactEvent,classify


def load_official_world_pose(root):
    """원본 공식 timing.py를 읽기 전용으로 불러온다."""
    src=Path(root)/"race/scripts/timing.py"
    spec=importlib.util.spec_from_file_location("_urrc_readonly_timing",src)
    module=importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.world_base_sample


def main():
    p=argparse.ArgumentParser(description="실제 모니터링용, AI 위치 입력 용도 금지")
    p.add_argument("--vehicle",required=True)
    p.add_argument("--track",default="monza")
    p.add_argument("--root",type=Path,default=Path(__file__).resolve().parents[1]/"upstream/urrc_f1_ws")
    p.add_argument("--output",type=Path,default=Path("generated/stage3_lap_events.jsonl"))
    p.add_argument("--target-laps",type=int,default=3)
    p.add_argument("--auto-arm",action="store_true",help="안전 출발 Bool 없이 계측 시운전할 때만")
    args=p.parse_args()
    finish,cps=gates_from_upstream(args.root,args.track)
    lap=LapSupervisor(finish,cps,target=args.target_laps)
    decode=load_official_world_pose(args.root)
    import rclpy
    from rclpy.node import Node
    from tf2_msgs.msg import TFMessage
    from std_msgs.msg import Bool
    from ros_gz_interfaces.msg import Contacts

    args.output.parent.mkdir(parents=True,exist_ok=True)
    audit=args.output.open("w",encoding="utf-8",buffering=1)
    class Monitor(Node):
        def __init__(self):
            super().__init__("urrc_stage3_official_geometry_observer")
            self.armed=False
            self.contact_subs=[]
            for ident in ("body","front_left","front_right","rear_left","rear_right"):
                topic=f"/model/{args.vehicle}/contacts/{ident}"
                sub=self.create_subscription(Contacts,topic,self.on_contacts,10)
                self.contact_subs.append(sub)
            self.create_subscription(TFMessage,f"/model/{args.vehicle}/urrc_timing",self.on_pose,200)
            self.create_subscription(Bool,f"/urrc_2_f1/{args.vehicle}/start_allowed",self.on_start,10)
        def on_contacts(self,msg):
            stamp=self.get_clock().now().nanoseconds/1e9
            for c in msg.contacts:
                item=ContactEvent(c.collision1.name,c.collision2.name,stamp)
                kind=classify(item,args.vehicle)
                if kind in ("wall","vehicle","unknown_collision"):
                    lap.contact(kind,stamp)
                    audit.write(json.dumps({"type":"contact","classification":kind,
                        "collision1":item.collision1,"collision2":item.collision2,
                        "time":stamp},ensure_ascii=False)+"\n")

        def on_start(self,msg):
            if bool(msg.data) and not self.armed:
                if lap.previous is not None:
                    try:
                        lap.arm(lap.previous[0]);self.armed=True
                        audit.write(json.dumps({"type":"arm","at":lap.previous[0]},ensure_ascii=False)+"\n")
                    except ValueError as e:self.get_logger().warning(str(e))
            elif not msg.data:
                self.armed=False
                # 안전 입력이 False면 랩 추가 중지.
                lap.active=False
        def on_pose(self,msg):
            try:
                sample=decode(msg.transforms,args.vehicle,"base_link",args.track)
                if args.auto_arm and lap.previous is not None and not self.armed:
                    lap.arm(lap.previous[0]);self.armed=True
                for event in lap.update(sample):
                    audit.write(json.dumps(event,ensure_ascii=False)+"\n")
                    self.get_logger().info(str(event))
                if lap.finished:
                    # 5개 접촉 센서가 모두 실제 ROS 브리지에 연결돼야 무충돌 PASS 가능.
                    lap.contact_monitor_ready=all(sub.get_publisher_count()>0 for sub in self.contact_subs)
                    audit.write(json.dumps({"type":"result",**lap.status()},ensure_ascii=False)+"\n")
            except Exception as exc:
                lap.active=False
                self.get_logger().error(f"계측 무효: {exc}")
    rclpy.init()
    node=Monitor()
    try:rclpy.spin(node)
    finally:
        audit.write(json.dumps({"type":"final",**lap.status()},ensure_ascii=False)+"\n")
        audit.close();node.destroy_node();rclpy.shutdown()

if __name__=="__main__":
    main()
