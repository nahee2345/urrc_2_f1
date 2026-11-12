"""Make a temporary Gazebo test world using the ORIGINAL official URRC meshes.

The mesh/walls/ground/gantry/plugin structure is copied from the official
<track>_race.sdf without modifications. TEAM11 car and temporary red overlay
lights are added for an independent test; no organizer files are edited.
"""
from __future__ import annotations

from copy import deepcopy
from pathlib import Path
import json
import xml.etree.ElementTree as ET

from generate_world import TRACKS,child,string_pose

TEAM_ROOT=Path(__file__).resolve().parents[1]


def build_official_world(track: str, grid: int, package: Path, output: Path):
    if track not in TRACKS or not 1<=grid<=11:
        raise ValueError("Only six tracks and grids 1..11 are supported")
    package=Path(package)
    file=package/"worlds"/f"{track}_race.sdf"
    base=ET.parse(file).getroot()
    world=base.find("world")
    if world is None or world.get("name")!=track:
        raise ValueError(f"Official world name mismatch: {file}")
    include=world.find(f"include[uri='model://urrc_{track}']")
    if include is None:
        raise ValueError("Official track mesh include missing")
    gantry=world.find("model[@name='race_start_gantry']")
    if gantry is None:
        raise ValueError("Official race gantry not found in source world")
    if world.find("model[@name='team11_test_lights']") is not None:
        raise ValueError("Test light model already present in official source")
    package_model=package/"models"/f"urrc_{track}"/"model.sdf"
    if not package_model.is_file():
        raise FileNotFoundError(package_model)
    model_root=ET.parse(package_model).getroot().find("model")
    if model_root is None or model_root.get("name")!=f"urrc_{track}":
        raise ValueError("Official mesh model doesn't match track")
    # All model:// URIs must actually exist; fail fast instead of loading
    # invisible meshes/walls into the renderer.
    for uri in model_root.findall(".//uri"):
        prefix=f"model://urrc_{track}/"
        if uri.text and uri.text.startswith(prefix):
            resource=package/"models"/f"urrc_{track}"/uri.text[len(prefix):]
            if not resource.is_file():
                raise FileNotFoundError(f"Official Gazebo mesh missing: {resource}")

    meta=json.loads((TEAM_ROOT/"maps"/track/"track.json").read_text())
    slot=meta["grid"][grid-1]
    if slot["position"]!=grid:
        raise ValueError("Grid metadata inconsistent")
    vehicle=ET.parse(TEAM_ROOT/"models"/"team11_car"/"model.sdf").getroot().find("model")
    if vehicle is None or vehicle.get("name")!="team_11":
        raise ValueError("TEAM11 model identifier mismatch")
    vehicle=deepcopy(vehicle)
    child(vehicle,"pose",string_pose(
        slot["x"],slot["y"],slot["road_z"]+.04,0.,0.,slot["yaw"]))
    world.append(vehicle)

    # The official static gantry carries five permanently dark red lenses.
    # Add five lit emissive overlays exactly at the lens positions.
    # Removing this one temp model switches all five off simultaneously.
    lamp_pose=gantry.findtext("pose")
    if not lamp_pose:
        raise ValueError("Official gantry has no pose")
    red=child(world,"model",name="team11_test_lights")
    child(red,"static","true")
    child(red,"pose",lamp_pose)
    link=child(red,"link",name="five_lamps")
    for lamp in range(5):
        visual=child(link,"visual",name=f"red_lamp_{lamp+1}")
        child(visual,"pose",string_pose(-.175,(lamp-2)*.16,1.96))
        geometry=child(visual,"geometry")
        child(child(geometry,"box"),"size",".025 .11 .12")
        material=child(visual,"material")
        child(material,"ambient","1 0.003 0.003 1")
        child(material,"diffuse","1 0.005 0.005 1")
        child(material,"emissive","1 0.003 0.003 1")

    output=Path(output).resolve()
    output.parent.mkdir(parents=True,exist_ok=True)
    ET.indent(base,space="  ")
    ET.ElementTree(base).write(output,encoding="utf-8",xml_declaration=True)
    return {
        "world":str(output),"track":track,"grid":grid,
        "source_world":str(file),"official_mesh_model":f"model://urrc_{track}",
        "spawn_xyz":[slot["x"],slot["y"],slot["road_z"]+.04],
        "start_light_model":"team11_test_lights",
        "note":"Official URRC 3D track meshes and gantry; only test vehicle/lamps added",
    }
