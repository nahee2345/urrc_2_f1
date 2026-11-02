"""원본 몬자 월드를 변경하지 않고 생성 출력물에 관측 장치를 부착한다."""
from __future__ import annotations
import json
from pathlib import Path
import xml.etree.ElementTree as ET
from vehicle_generator.model import fmt,sub


def instrument_model(sdf:str,car_name:str):
    root=ET.fromstring(sdf)
    model=root.find("model")
    if model is None or model.get("name")!=car_name:
        raise ValueError("차량 이름 불일치")
    pp=sub(model,"plugin",filename="gz-sim-pose-publisher-system",
           name="gz::sim::systems::PosePublisher")
    for key,val in {
        "publish_link_pose":"true","publish_model_pose":"true",
        "publish_nested_model_pose":"true","publish_visual_pose":"false",
        "publish_collision_pose":"false","publish_sensor_pose":"false",
        "use_pose_vector_msg":"true","static_publisher":"false",
        "update_frequency":"200","topic":f"/model/{car_name}/urrc_timing",
    }.items():sub(pp,key,val)
    topics=[]
    for link in model.findall("link"):
        linkname=link.attrib["name"]
        for col in link.findall("collision"):
            collision=col.attrib["name"]
            label=("body" if collision=="body_collision" else
                   linkname.removesuffix("_wheel") if collision.endswith("_collision")
                   and linkname.endswith("_wheel") else collision.replace("_housing_collision","_housing"))
            sensor=sub(link,"sensor",name="stage3_contact_"+label,type="contact")
            sub(sensor,"always_on","true")
            sub(sensor,"update_rate","0")
            topic=f"/model/{car_name}/contacts/{label}"
            sub(sensor,"topic",topic)
            sub(sub(sensor,"contact"),"collision",collision)
            topics.append(topic)
    if len(set(topics))!=len(topics) or len(topics)<6:
        raise ValueError("접촉 센서 설정 부족/중복")
    ET.indent(root,space="  ")
    return ET.tostring(root,encoding="unicode",xml_declaration=True),topics


def make_monza_copy(upstream:Path,car_sdf:str,car_name:str):
    upstream=Path(upstream)
    package=(upstream/"src/urrc_track_gazebo" if (upstream/"src/urrc_track_gazebo").is_dir() else upstream)
    worldfile=package/"worlds/monza_race.sdf"
    root=ET.fromstring(worldfile.read_text())
    world=root.find("world")
    if world is None or world.attrib["name"]!="monza":
        raise ValueError("원본 Monza 월드가 아닙니다")
    if world.find("plugin[@name='gz::sim::systems::Contact']") is None:
        sub(world,"plugin",name="gz::sim::systems::Contact",
            filename="gz-sim-contact-system")
    data=json.loads((package/"tracks/processed/monza.json").read_text())
    slot=data["grid"][0]
    inserted=ET.fromstring(car_sdf).find("model")
    sub(inserted,"pose",fmt(slot["x"],slot["y"],slot["road_z"]+0.04,0,0,slot["yaw"]))
    world.append(inserted)
    ET.indent(root,space="  ")
    return ET.tostring(root,encoding="unicode",xml_declaration=True)
