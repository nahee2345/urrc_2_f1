"""원본 Monaco 지형·시각 효과·신호등을 보존한 독립 실험 환경.

원본 저장소 변경 금지. Monaco 공식 충돌메시와 중심선 SHA256 동일성 검사.
AI 컨트롤러는 감독용 위치·센터라인에 접근할 수 없다.
"""
from __future__ import annotations
from dataclasses import replace
import hashlib
import importlib.util
import json
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET

from vehicle_generator.model import Vehicle,sub,fmt
from vehicle_generator.sensors import build,load_presets
from stage3.instrument import instrument_model
from stage3.lap import gates_from_upstream
from stage3.terrain import _vertices

TRACK="monaco"
ROOT=Path(__file__).resolve().parents[1]

def sha256(file):
    h=hashlib.sha256()
    with Path(file).open("rb") as f:
        for block in iter(lambda:f.read(1<<20),b""):
            h.update(block)
    return h.hexdigest()

def original_package(upstream):
    p=Path(upstream)/"src/urrc_track_gazebo"
    if not (p/"urrc_tracks/v10_assets.py").is_file():
        raise FileNotFoundError("공식 urrc_f1_ws 서브모듈이 없습니다")
    return p

def prepared_package(upstream,output):
    src=original_package(upstream)
    out=Path(output)/"resources/urrc_track_gazebo"
    needed=[src/f"worlds/{TRACK}_race.sdf",
            src/f"models/urrc_{TRACK}/model.sdf",
            src/f"models/urrc_{TRACK}/meshes/road_collision.obj",
            src/f"tracks/processed/{TRACK}_centerline.csv",
            src/"urrc_tracks/v10_assets.py"]
    fingerprint=hashlib.sha256("".join(sha256(f) for f in needed).encode()).hexdigest()
    mark=out/"stage5_source_lock.json"
    if mark.exists():
        try:
            old=json.loads(mark.read_text())
            if (old["source_fingerprint"]==fingerprint and
                (out/f"worlds/{TRACK}_signal_active.sdf").is_file() and
                (out/f"models/urrc_{TRACK}/meshes/road_textured.obj").is_file()):
                return out
        except (OSError,ValueError,KeyError):pass
    if out.exists():shutil.rmtree(out)
    (out/"models").mkdir(parents=True)
    shutil.copytree(src/f"models/urrc_{TRACK}",out/f"models/urrc_{TRACK}")
    (out/"worlds").mkdir(parents=True)
    for fn in (f"{TRACK}.sdf",f"{TRACK}_race.sdf",f"{TRACK}_grid.gui.config"):
        shutil.copy2(src/"worlds"/fn,out/"worlds"/fn)
    (out/"tracks/processed").mkdir(parents=True)
    for fn in (f"{TRACK}.json",f"{TRACK}_centerline.csv"):
        shutil.copy2(src/"tracks/processed"/fn,out/"tracks/processed"/fn)
    (out/"scripts").mkdir(parents=True)
    shutil.copy2(src/"scripts/traffic_lights_v10.py",out/"scripts/traffic_lights_v10.py")
    spec=importlib.util.spec_from_file_location("_urrc_stage5_read_only_v10",src/"urrc_tracks/v10_assets.py")
    mod=importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    mod.enhance_track(out,TRACK)
    for fn in (f"models/urrc_{TRACK}/meshes/road_collision.obj",
               f"tracks/processed/{TRACK}_centerline.csv"):
        if sha256(src/fn)!=sha256(out/fn):
            raise RuntimeError("원본 Monaco 물리 메시 또는 중심선 변경을 감지했습니다: "+fn)
    mark.write_text(json.dumps({"track":TRACK,"source_fingerprint":fingerprint,
        "source_modified":False,"original_collision_equal":True,
        "original_centerline_equal":True},ensure_ascii=False,indent=2)+"\n")
    return out

def inspect_original(upstream,prepared,worldfile):
    src=original_package(upstream)
    source=src/f"models/urrc_{TRACK}/meshes/road.obj"
    textured=prepared/f"models/urrc_{TRACK}/meshes/road_textured.obj"
    a=_vertices(source);b=_vertices(textured)
    report=json.loads((src/f"docs/{TRACK}_geometry_report.json").read_text())
    if not (a["vertices"]==b["vertices"] and
            abs(a["min_z_m"]-b["min_z_m"])<1e-7 and
            abs(a["max_z_m"]-b["max_z_m"])<1e-7):
        raise RuntimeError("원본 Monaco 도로 메시가 변경되거나 평탄화되었습니다")
    if b["height_range_m"]<6 or report["elevation_range_m"]<6:
        raise RuntimeError("원본 Monaco 고저차가 사라졌습니다")
    root=ET.parse(worldfile).getroot()
    world=root.find("world")
    if world is None or world.get("name")!=TRACK:
        raise RuntimeError("잘못된 월드 이름")
    if world.findtext("include/uri")!=f"model://urrc_{TRACK}":
        raise RuntimeError("잘못된 공식 Monaco 모델 URI")
    cameras=world.findall("./model[@name='urrc_v10_spectator']//sensor[@type='camera']")
    lights=ET.parse(prepared/f"worlds/{TRACK}_signal_active.sdf").getroot().findall(".//visual")
    if len(cameras)!=4 or len(lights)!=5:
        raise RuntimeError("원본 4개 관전 카메라 또는 5개 신호등 리소스 누락")
    if sha256(src/f"models/urrc_{TRACK}/meshes/road_collision.obj")!=sha256(
            prepared/f"models/urrc_{TRACK}/meshes/road_collision.obj"):
        raise RuntimeError("도로 충돌 지형이 원본과 다릅니다")
    return {"status":"PASS_MONACO_SOURCE_FIDELITY","track":TRACK,
            "length_3d_m":report["length_3d_m"],
            "width_min_m":report["width_min_m"],
            "elevation_reference_m":report["elevation_range_m"],
            "mesh_z_range_m":b["height_range_m"],
            "mesh_vertices":b["vertices"],"original_collision_sha256":sha256(
            src/f"models/urrc_{TRACK}/meshes/road_collision.obj"),
            "spectator_cameras":len(cameras),"signal_lenses":len(lights),
            "source_files_modified":False}

def generate_monaco(upstream,output,ai="AI-01"):
    presets=load_presets()
    if ai not in presets:raise ValueError("AI-01~AI-10 중 지정")
    package=prepared_package(upstream,output)
    v=replace(Vehicle.from_file(ROOT/"config/stage1_rwd.json"),
              name="urrc_"+ai.lower().replace("-","_"),drive=presets[ai]["drive"])
    sdf,manifest=build(v,presets[ai]["driving"])
    car,contact_topics=instrument_model(sdf,v.name)
    root=ET.fromstring((package/f"worlds/{TRACK}_race.sdf").read_text())
    world=root.find("world")
    if world is None or world.get("name")!=TRACK:raise ValueError("공식 Monaco 월드 아님")
    if world.find("plugin[@name='gz::sim::systems::Contact']") is None:
        sub(world,"plugin",name="gz::sim::systems::Contact",
            filename="gz-sim-contact-system")
    grid=json.loads((package/f"tracks/processed/{TRACK}.json").read_text())["grid"][0]
    car_node=ET.fromstring(car).find("model")
    if car_node is None:raise RuntimeError("차량 모델 누락")
    sub(car_node,"pose",fmt(grid["x"],grid["y"],grid["road_z"]+.04,0,0,grid["yaw"]))
    world.append(car_node)
    ET.indent(root,space="  ")
    place=Path(output)/ai
    place.mkdir(parents=True,exist_ok=True)
    worldfile=place/"monaco_experiment.sdf"
    worldfile.write_text(ET.tostring(root,encoding="unicode",xml_declaration=True))
    (place/"model_instrumented.sdf").write_text(car)
    fidelity=inspect_original(upstream,package,worldfile)
    (place/"terrain_fidelity.json").write_text(json.dumps(fidelity,ensure_ascii=False,indent=2)+"\n")
    (place/"contacts.json").write_text(json.dumps({"vehicle":v.name,
        "topics":contact_topics},ensure_ascii=False,indent=2)+"\n")
    finish,checkpoints=gates_from_upstream(upstream,TRACK)
    (place/"gates_summary.json").write_text(json.dumps({
        "track":TRACK,"finish":finish,"checkpoints":len(checkpoints),
        "ai_has_access_to_judge_data":False},ensure_ascii=False,indent=2)+"\n")
    return {"vehicle":v.name,"world":str(worldfile),"fidelity":fidelity,
            "contact_topics":len(contact_topics),"checkpoints":len(checkpoints),
            "sensor_count":manifest["sensor_count"],"grid":grid}
