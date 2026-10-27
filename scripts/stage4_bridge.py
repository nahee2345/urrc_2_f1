#!/usr/bin/env python3
"""기존 3단계 브리지에 AI-01의 실제 RGB 카메라와 물리 구동 명령 추가."""
from __future__ import annotations
import argparse,json,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from scripts.stage3_bridge import mappings as stage3_mappings

def bridge_yaml(manifest):
    car=manifest["vehicle"]
    if car!="urrc_ai_01":raise ValueError("4단계는 AI-01만 연결합니다")
    base=stage3_mappings(car,"monza",manifest["topics"])
    topics=[(f"/model/{car}/sensors/rgb","sensor_msgs/msg/Image","gz.msgs.Image","GZ_TO_ROS")]
    topics += [(f"/model/{car}/drive/{wheel}","std_msgs/msg/Float64","gz.msgs.Double","ROS_TO_GZ")
               for wheel in ("front_left","front_right")]
    topics += [(f"/model/{car}/steer/{side}","std_msgs/msg/Float64","gz.msgs.Double","ROS_TO_GZ")
               for side in ("left","right")]
    used=set()
    for line in base.splitlines():
        if line.startswith("- ros_topic_name: "):used.add(line.split(": ",1)[1])
    for topic,ros,gz,direction in topics:
        if topic in used:raise ValueError("제어 토픽 중복")
        used.add(topic)
        base+=(f"- ros_topic_name: {topic}\n  gz_topic_name: {topic}\n"
               f"  ros_type_name: {ros}\n  gz_type_name: {gz}\n"
               f"  direction: {direction}\n")
    return base

def main():
    parser=argparse.ArgumentParser()
    parser.add_argument("--contacts",type=Path,default=Path("generated/stage3/AI-01/contacts.json"))
    parser.add_argument("--output",type=Path,default=Path("generated/stage4/ai01_bridge.yaml"))
    args=parser.parse_args()
    manifest=json.loads(args.contacts.read_text())
    text=bridge_yaml(manifest)
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(text)
    print(f"4단계 ROS2 <-> Gazebo 브리지: {args.output}")

if __name__=="__main__":main()
