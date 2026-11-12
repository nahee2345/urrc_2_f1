#!/usr/bin/env python3
"""Generate runtime-only light-practice SDF and GUI from official world.
Never edits the official world, the track model, or official signals.
"""
import argparse
from pathlib import Path
import xml.etree.ElementTree as ET

def prepare(track,official,output):
    source=official/"worlds"/(track+"_race.sdf")
    gui_source=official/"worlds"/(track+"_grid.gui.config")
    output.mkdir(parents=True,exist_ok=True)
    tree=ET.parse(source)
    world=tree.getroot().find("world")
    if world is None or world.get("name")!=track:
        raise ValueError("Official world mismatch")
    # Preserve the track, physics, start gantry, and all five red lamps.
    removed=[]
    for link in world.findall(".//link"):
        for sensor in list(link.findall("sensor")):
            # Any map-mounted spectator camera is unnecessary in local practice.
            if sensor.get("type") in ("camera","depth_camera","rgbd_camera"):
                removed.append(sensor.get("name","unknown"))
                link.remove(sensor)
    for plug in list(world.findall("plugin")):
        # Preserve Physics, UserCommands, SceneBroadcaster and Sensors.
        if "spectator" in (plug.get("name","")+" "+plug.get("filename","")).lower():
            removed.append(plug.get("name","spectator_plugin"))
            world.remove(plug)
    scene=world.find("scene")
    if scene is not None:
        shadows=scene.find("shadows")
        if shadows is not None:shadows.text="false"
    for light in world.findall("light"):
        if light.get("name")=="sun":
            shadow=light.find("cast_shadows")
            if shadow is not None:shadow.text="false"
    if len(world.findall("light[@name='race_red_1']"))!=1 or len(world.findall("light[@name='race_red_5']"))!=1:
        raise ValueError("Official five-light gantry missing; refusing practice world")
    world_file=output/(track+"_practice.sdf")
    tree.write(world_file,encoding="unicode",xml_declaration=True)
    gui=ET.parse(gui_source)
    root=gui.getroot()
    keep={"MinimalScene","GzSceneManager","InteractiveViewControl","CameraTracking","WorldControl","EntityTree"}
    for plugin in list(root.findall("plugin")):
        if plugin.get("filename") not in keep:
            root.remove(plugin)
    if not any(p.get("filename")=="CameraTracking" for p in root.findall("plugin")):
        raise ValueError("CameraTracking Follow GUI plugin missing")
    gui_file=output/(track+"_practice.gui.config")
    gui.write(gui_file,encoding="unicode",xml_declaration=True)
    print("PRACTICE_WORLD="+str(world_file),flush=True)
    print("PRACTICE_GUI="+str(gui_file),flush=True)
    print("MAP_CAMERAS_REMOVED="+str(len(removed)),flush=True)
    print("MAP_CAMERA_NAMES="+",".join(removed),flush=True)
    return world_file,gui_file

if __name__=="__main__":
    p=argparse.ArgumentParser()
    p.add_argument("--track",required=True)
    p.add_argument("--official",type=Path,required=True)
    p.add_argument("--output",type=Path,required=True)
    args=p.parse_args()
    prepare(args.track,args.official,args.output)
