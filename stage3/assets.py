"""공식 URRC v10.8 리소스를 원본과 격리하여 준비한다.

원본 run_world.py는 실행 전에 enhance_track을 호출하여 아스팔트,
관전 카메라, 다섯 빨간 램프의 단일 모델, 배경 재질을 생성한다.
기존 3단계는 이 단계가 누락되어 맵 외관·시그널 실행이 달랐다.
원본 서브모듈 파일은 변경하지 않고 generated/stage3에만 복제/가공한다.
"""
from __future__ import annotations
import hashlib
import importlib.util
import json
from pathlib import Path
import shutil


def _digest(paths):
    h=hashlib.sha256()
    for path in paths:
        h.update(path.name.encode())
        with path.open("rb") as file:
            for chunk in iter(lambda:file.read(1<<20),b""):
                h.update(chunk)
    return h.hexdigest()


def _source_package(upstream):
    p=Path(upstream)/"src/urrc_track_gazebo"
    if not (p/"urrc_tracks/v10_assets.py").is_file():
        raise FileNotFoundError(f"원본 v10.8 에셋 생성기가 없습니다: {p}")
    return p


def _inputs(src):
    return [src/"worlds/monza.sdf",src/"worlds/monza_race.sdf",
            src/"models/urrc_monza/model.sdf",
            src/"models/urrc_monza/meshes/road.obj",
            src/"models/urrc_monza/meshes/road_collision.obj",
            src/"tracks/processed/monza.json",
            src/"tracks/processed/monza_centerline.csv",
            src/"urrc_tracks/v10_assets.py"]


def prepare_monza_assets(upstream,output):
    """공식 SDF+자원 생성의 결과를 원본 비수정 복사본으로 재현한다."""
    src=_source_package(upstream)
    output=Path(output)
    target=output/"resources/urrc_track_gazebo"
    source_key=_digest(_inputs(src))
    stamp=target/"stage3_source_lock.json"
    if stamp.is_file():
        try:
            info=json.loads(stamp.read_text())
            if info.get("source_sha256")==source_key and all(p.is_file() for p in
                    (target/"worlds/monza_race.sdf",
                     target/"worlds/monza_signal_active.sdf",
                     target/"models/urrc_monza/meshes/road_textured.obj")):
                return target,info
        except (OSError,ValueError):
            pass
    if target.exists():shutil.rmtree(target)
    for name in ("models/urrc_monza","worlds","tracks/processed","scripts"):
        (target/name).parent.mkdir(parents=True,exist_ok=True)
    shutil.copytree(src/"models/urrc_monza",target/"models/urrc_monza")
    (target/"worlds").mkdir(parents=True,exist_ok=True)
    for name in ("monza.sdf","monza_race.sdf","monza_grid.gui.config"):
        shutil.copy2(src/"worlds"/name,target/"worlds"/name)
    (target/"tracks/processed").mkdir(parents=True,exist_ok=True)
    for name in ("monza.json","monza_centerline.csv"):
        shutil.copy2(src/"tracks/processed"/name,target/"tracks/processed"/name)
    (target/"scripts").mkdir(parents=True,exist_ok=True)
    shutil.copy2(src/"scripts/traffic_lights_v10.py",target/"scripts/traffic_lights_v10.py")
    # 원본 코드를 오직 로드해 실행한다. 실행 대상 target은 격리된 복사본이다.
    genfile=src/"urrc_tracks/v10_assets.py"
    spec=importlib.util.spec_from_file_location("_official_v10_assets_readonly",genfile)
    gen=importlib.util.module_from_spec(spec)
    spec.loader.exec_module(gen)
    gen.enhance_track(target,"monza")
    # 충돌 메시와 중심선은 공식 원본 바이트와 동일해야 한다.
    for name in ("models/urrc_monza/meshes/road_collision.obj",
                 "tracks/processed/monza_centerline.csv"):
        if _digest([src/name])!=_digest([target/name]):
            raise RuntimeError(f"원본 지형 또는 좌표가 변경되었습니다: {name}")
    world=(target/"worlds/monza_race.sdf").read_text()
    if "urrc_v10_spectator" not in world or not (target/"worlds/monza_signal_active.sdf").is_file():
        raise RuntimeError("원본 v10.8 관전 카메라/단일 소등 모델 생성 실패")
    info={"source_sha256":source_key,"upstream_write":False,
          "world_source":"original v10.8 + enhance_track (isolated copy)",
          "resource_model":str(target/"models/urrc_monza"),
          "road_collision_source_identical":True,
          "centerline_source_identical":True,
          "temporary_red_signal_model":"worlds/monza_signal_active.sdf"}
    stamp.write_text(json.dumps(info,ensure_ascii=False,indent=2)+"\n")
    return target,info
