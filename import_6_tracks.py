#!/usr/bin/env python3
"""Copy exactly six official track assets and the original five-light script.

SOURCE is read-only. No driving logic, vehicle spawn, or start-detection logic.
"""
import argparse
import hashlib
import json
import shutil
import xml.etree.ElementTree as ET
from copy import deepcopy
from pathlib import Path

TRACKS=("monza","spa","monaco","cota","hungaroring","baku")
HERE=Path(__file__).resolve()
REPO=HERE.parent
TARGET=REPO/"src"/"urrc_track_gazebo"

def sha(path):
    h=hashlib.sha256()
    with path.open("rb") as src:
        for block in iter(lambda:src.read(1024*1024),b""):
            h.update(block)
    return h.hexdigest()

def package_root(workspace):
    root=Path(workspace).expanduser().resolve()
    candidates=(root/"src"/"urrc_track_gazebo",root)
    for pkg in candidates:
        if (pkg/"worlds"/"monza_race.sdf").is_file() and (pkg/"models"/"urrc_monza"/"model.sdf").is_file():
            return pkg
    raise FileNotFoundError("No official track package at "+str(root))

def active_signal_from_race(race_file,output):
    doc=ET.parse(race_file)
    gantry=doc.find(".//model[@name='race_start_gantry']")
    if gantry is None or not gantry.findtext("pose"):
        raise ValueError("Official race gantry/pose missing: "+str(race_file))
    root=ET.Element("sdf",version="1.9")
    model=ET.SubElement(root,"model",name="urrc_start_signal_active")
    ET.SubElement(model,"static").text="true"
    ET.SubElement(model,"pose").text=gantry.findtext("pose")
    link=ET.SubElement(model,"link",name="signal_active_visuals")
    for i in range(1,6):
        lens=gantry.find(f"link[@name='gantry_visuals']/visual[@name='red_lens_{i}']")
        if lens is None or lens.find("geometry/box/size") is None:
            raise ValueError("Official red lens missing: "+str(i))
        visual=deepcopy(lens)
        visual.set("name",f"active_red_lens_{i}")
        pose=[float(v) for v in visual.findtext("pose").split()]
        pose[0]-=.020
        visual.find("pose").text=" ".join(f"{v:.7f}" for v in pose)
        visual.find("geometry/box/size").text=".008 .110 .120"
        material=visual.find("material")
        if material is None:material=ET.SubElement(visual,"material")
        for element,value in (
            ("ambient","0.015 0.005 0.006 1"),
            ("diffuse","0.015 0.005 0.006 1"),
            ("emissive","0 0 0 1")):
            node=material.find(element)
            if node is None:node=ET.SubElement(material,element)
            node.text=value
        link.append(visual)
    output.parent.mkdir(parents=True,exist_ok=True)
    ET.ElementTree(root).write(output,encoding="utf-8",xml_declaration=True)

def main():
    parser=argparse.ArgumentParser()
    parser.add_argument("--official",default=str(Path.home()/"urrc_f1_ws"))
    parser.add_argument("--verify-only",action="store_true")
    args=parser.parse_args()
    source=package_root(args.official)
    if source==TARGET.resolve():
        raise ValueError("Source and destination must be different")
    assert str(TARGET.resolve()).startswith(str(REPO.resolve())+"/")
    copied=[]
    missing=[]
    for name in TRACKS:
        # Copy complete model directory including all real meshes and collisions.
        meshes=source/"models"/f"urrc_{name}"
        if not meshes.is_dir():missing.append(str(meshes));continue
        for src in sorted(meshes.rglob("*")):
            if src.is_file():
                copied.append((src,TARGET/src.relative_to(source)))
        for filename in (f"{name}.sdf",f"{name}_race.sdf",
                         f"{name}_grid.gui.config",f"{name}_overview.gui.config"):
            src=source/"worlds"/filename
            if src.is_file():copied.append((src,TARGET/"worlds"/filename))
            else:missing.append(str(src))
        for filename in (f"{name}.json",f"{name}_centerline.csv"):
            src=source/"tracks"/"processed"/filename
            if src.is_file():copied.append((src,TARGET/"tracks"/"processed"/filename))
            else:missing.append(str(src))
    light=source/"scripts"/"traffic_lights_v10.py"
    if light.is_file():copied.append((light,TARGET/"scripts"/light.name))
    else:missing.append(str(light))
    if missing:
        raise FileNotFoundError("Incomplete official assets; no copy performed:\n" + "\n".join(missing))
    if not args.verify_only:
        for src,dst in copied:
            dst.parent.mkdir(parents=True,exist_ok=True)
            shutil.copy2(src,dst)
        for name in TRACKS:
            active_signal_from_race(TARGET/"worlds"/f"{name}_race.sdf",
                                    TARGET/"worlds"/f"{name}_signal_active.sdf")
    mismatched=[]
    for src,dst in copied:
        if not dst.is_file() or sha(src)!=sha(dst):
            mismatched.append(str(dst))
    signals=[]
    for name in TRACKS:
        p=TARGET/"worlds"/f"{name}_signal_active.sdf"
        if not p.is_file():mismatched.append(str(p));continue
        root=ET.parse(p).getroot()
        lenses=root.findall(".//link[@name='signal_active_visuals']/visual")
        if len(lenses)!=5:mismatched.append(str(p))
        signals.append(name)
    report={"source":str(source),"target":str(TARGET),"tracks":list(TRACKS),
            "source_files_checked":len(copied),"signal_templates":signals,
            "mismatches":mismatched,"status":"PASS" if not mismatched else "FAIL"}
    if not args.verify_only:
        result=REPO/"track_import_report.json"
        result.write_text(json.dumps(report,ensure_ascii=False,indent=2)+"\n",encoding="utf-8")
    print(json.dumps(report,ensure_ascii=False,indent=2))
    if mismatched:raise SystemExit(1)

if __name__=="__main__":
    main()
