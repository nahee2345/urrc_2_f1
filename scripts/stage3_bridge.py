#!/usr/bin/env python3
"""운영자 TF/접촉, 차량 시작 카메라 데이터의 단방향 ROS 브리지 생성."""
import argparse,json
from pathlib import Path


def mappings(vehicle,track,contact_topics):
    img=f"/model/{vehicle}/sensors/start_camera"
    data=[
      ("/clock","rosgraph_msgs/msg/Clock","gz.msgs.Clock"),
      (img,"sensor_msgs/msg/Image","gz.msgs.Image"),
      (f"/model/{vehicle}/urrc_timing","tf2_msgs/msg/TFMessage","gz.msgs.Pose_V")
    ]+[(topic,"ros_gz_interfaces/msg/Contacts","gz.msgs.Contacts")
       for topic in contact_topics]
    if len({t for t,_,_ in data})!=len(data):raise ValueError("중복 토픽")
    text=["# 원본 조정 금지. 운영자 계측·출발 이미지 전용(GZ_TO_ROS)."]
    for topic,ros,gz in data:
        text.extend([f"- ros_topic_name: {topic}",
                     f"  gz_topic_name: {topic}",f"  ros_type_name: {ros}",
                     f"  gz_type_name: {gz}","  direction: GZ_TO_ROS"])
    return "\n".join(text)+"\n"


def main():
    p=argparse.ArgumentParser()
    p.add_argument("--contacts",type=Path,required=True)
    p.add_argument("--output",type=Path,required=True)
    a=p.parse_args()
    contacts=json.loads(a.contacts.read_text(encoding="utf-8"))
    a.output.parent.mkdir(parents=True,exist_ok=True)
    a.output.write_text(mappings(contacts["vehicle"],"monza",contacts["topics"]))
    print(f"브리지 설정 생성: {a.output}")

if __name__=="__main__":main()
