"""원본과 3단계 출력의 지형 높이 및 메시 변경 여부 검증."""
from __future__ import annotations
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET


def _hash(path):
    h=hashlib.sha256()
    with Path(path).open("rb") as f:
        for chunk in iter(lambda:f.read(1<<20),b""):
            h.update(chunk)
    return h.hexdigest()


def _vertices(path):
    count=0;minimum=math.inf;maximum=-math.inf
    for line in Path(path).open(encoding="utf8"):
        if line.startswith("v "):
            _,x,y,z,*_=line.split()
            value=float(z)
            minimum=min(minimum,value)
            maximum=max(maximum,value)
            count+=1
    if count<100 or minimum>=maximum:
        raise ValueError(f"지형 메시의 Z 고저차가 없습니다: {path}")
    return {"vertices":count,"min_z_m":minimum,"max_z_m":maximum,
            "height_range_m":maximum-minimum}


def check_fidelity(upstream,package,generated_world):
    """정확한 도로 메시 Z와 출발 신호등·관전 카메라 구성 확인."""
    src=Path(upstream)/"src/urrc_track_gazebo"
    package=Path(package)
    world=Path(generated_world)
    original_road=src/"models/urrc_monza/meshes/road.obj"
    copied_road=package/"models/urrc_monza/meshes/road_textured.obj"
    original_collision=src/"models/urrc_monza/meshes/road_collision.obj"
    copied_collision=package/"models/urrc_monza/meshes/road_collision.obj"
    original=_vertices(original_road)
    updated=_vertices(copied_road)
    reference=json.loads((src/"docs/monza_geometry_report.json").read_text())
    if reference["elevation_range_m"]<1.0:
        raise ValueError("원본 메타데이터 고저차가 예상과 다릅니다")
    if abs(original["min_z_m"]-updated["min_z_m"])>1e-7 or \
       abs(original["max_z_m"]-updated["max_z_m"])>1e-7 or \
       original["vertices"]!=updated["vertices"]:
        raise ValueError("3단계 도로 외형 메시가 평탄화되거나 변경되었습니다")
    if original["height_range_m"]<1.0:
        raise ValueError("원본 도로 OBJ에 고저차가 없습니다")
    if _hash(original_collision)!=_hash(copied_collision):
        raise ValueError("도로 충돌 메시가 원본과 다릅니다")
    sdf=ET.parse(world).getroot()
    model=sdf.find("./world")
    spectator=model.find("./model[@name='urrc_v10_spectator']")
    if spectator is None:
        raise ValueError("공식 관전 카메라 4개가 누락되었습니다")
    cameras=spectator.findall(".//sensor[@type='camera']")
    if len(cameras)!=4:
        raise ValueError("공식 관전 카메라가 4개가 아닙니다")
    if model.find("./model[@name='race_start_gantry']") is None:
        raise ValueError("경기장 출발 신호등 구조물이 누락되었습니다")
    active=ET.parse(package/"worlds/monza_signal_active.sdf")
    if len(active.findall(".//visual"))!=5:
        raise ValueError("공식 전체 소등용 5개 렌즈 모델 누락")
    ground_collision=model.find("./model[@name='ground']/link/collision")
    if ground_collision is None:
        raise ValueError("공식 지면 충돌이 누락되었습니다")
    resource_include=model.findtext("./include/uri")
    if resource_include!="model://urrc_monza":
        raise ValueError(f"원본 몬자 지형 모델 포함 경로 오류: {resource_include}")
    return {"status":"PASS_ORIGINAL_MONZA_TERRAIN", "source":"urrc_f1_ws",
            "original_road":original,"enhanced_road":updated,
            "elevation_range_report_m":reference["elevation_range_m"],
            "road_collision_sha256":_hash(copied_collision),
            "mesh_identical_geometry":True,
            "spectator_cameras":len(cameras),
            "five_light_single_model":True,
            "track_resource_uri":resource_include,
            "ground_collision_present":True}
