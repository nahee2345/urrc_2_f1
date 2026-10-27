#!/usr/bin/env python3
"""Stage 2: independent Gazebo RGB/depth/3D-LiDAR stream smoke test.

Uses a separate flat test world, NOT an altered URRC race track.
--run requires Gazebo Harmonic with Ogre2 headless rendering and GPU/GL driver.
"""
from __future__ import annotations
import argparse
from dataclasses import replace
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import tempfile
import time
import xml.etree.ElementTree as ET

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from vehicle_generator.model import Vehicle,sub,fmt
from vehicle_generator.sensors import build,load_presets
from scripts.stage1_smoke import make_smoke_world


def isolated_world(v,driving):
    root=ET.fromstring(make_smoke_world(v))
    world=root.find("world")
    for existing in list(world.findall("model")):
        if existing.attrib["name"]==v.name:
            world.remove(existing)
    sensors=sub(world,"plugin",filename="gz-sim-sensors-system",
                name="gz::sim::systems::Sensors")
    sub(sensors,"render_engine","ogre2")
    scene=sub(world,"scene")
    sub(scene,"ambient","0.7 0.7 0.7 1")
    sub(scene,"background","0.65 0.75 0.85 1")
    light=sub(world,"light",name="daylight",type="directional")
    sub(light,"pose","0 0 3 0 0 0")
    sub(light,"diffuse","1 1 1 1")
    sub(light,"direction","-1 0 -1")
    sub(light,"cast_shadows","false")
    box=sub(world,"model",name="sensor_obstacle")
    sub(box,"static","true")
    sub(box,"pose","2.5 0 0.2 0 0 0")
    link=sub(box,"link",name="obstacle")
    for kind in ("collision","visual"):
        item=sub(link,kind,name=kind)
        sub(sub(sub(item,"geometry"),"box"),"size","0.6 0.6 0.4")
    model=ET.fromstring(build(v,driving)[0]).find("model")
    sub(model,"pose",fmt(0,0,v.wheel_radius+0.002,0,0,0))
    world.append(model)
    ET.indent(root,space="  ")
    return ET.tostring(root,encoding="unicode",xml_declaration=True)


def has_sample(path,kind):
    if not path.is_file():return False
    if path.stat().st_size < 120:return False
    # Only examine a bounded slice; image messages can be megabytes.
    with path.open("rb") as f:prefix=f.read(900000).decode("utf-8",errors="replace")
    if kind in ("camera","depth_camera"):
        return bool(re.search(r"\bwidth:\s*160\b",prefix) and
                    re.search(r"\bheight:\s*120\b",prefix) and
                    re.search(r"\bdata:",prefix))
    if kind=="gpu_lidar":
        return bool(re.search(r"\branges:\s*[-+0-9.infINFnanNAN]",prefix)
                    and re.search(r"\bcount:\s*360\b",prefix))
    return False


def main():
    p=argparse.ArgumentParser()
    p.add_argument("--ai",default="AI-05")
    p.add_argument("--prepare",action="store_true")
    p.add_argument("--run",action="store_true")
    p.add_argument("--startup-seconds",type=float,default=12)
    p.add_argument("--capture-seconds",type=float,default=10)
    p.add_argument("--out",type=Path,default=ROOT/"generated/stage2_sensor_smoke.sdf")
    p.add_argument("--report",type=Path,default=ROOT/"generated/stage2_sensor_result.json")
    args=p.parse_args()
    if not (args.prepare or args.run):p.error("choose --prepare or --run")
    presets=load_presets()
    if args.ai not in presets:p.error("choose AI-01 ... AI-10")
    profile=presets[args.ai]
    v=replace(Vehicle.from_file(ROOT/"config/stage1_rwd.json"),
              name="urrc_"+args.ai.lower().replace("-","_"),
              drive=profile["drive"])
    xml,meta=build(v,profile["driving"])
    args.out.parent.mkdir(parents=True,exist_ok=True)
    args.out.write_text(isolated_world(v,profile["driving"]),encoding="utf-8")
    if not args.run:
        print(json.dumps({"status":"PREPARED_ONLY","ai":args.ai,
            "topics":list(meta["topics"]),"world":str(args.out)},ensure_ascii=False,indent=2))
        return 0
    result={"status":"NOT_RUN","ai":args.ai,"topics":{},"test":"stage2_gazebo_streams",
            "reason":"","driver":"Gazebo Harmonic Sensors Ogre2"}
    server=None;clients=[];log=None
    with tempfile.TemporaryDirectory(prefix="urrc_stage2_") as temp:
        tmp=Path(temp)
        log=tmp/"gazebo.log"
        try:
            if not shutil.which("gz"):
                result.update(status="UNAVAILABLE",reason="gz not installed")
                return 2
            valid=subprocess.run(["gz","sdf","-p",str(args.out)],
                capture_output=True,text=True,timeout=60)
            if valid.returncode:
                result.update(status="FAIL_SDF",reason=valid.stderr[-1800:])
                return 1
            env=dict(os.environ)
            env.setdefault("LIBGL_ALWAYS_SOFTWARE","1")
            env.setdefault("QT_QPA_PLATFORM","offscreen")
            with log.open("wb") as serverlog:
                server=subprocess.Popen(["gz","sim","-s","-r","--headless-rendering",
                    "-v","3",str(args.out)],stdout=serverlog,stderr=subprocess.STDOUT,env=env)
                time.sleep(args.startup_seconds)
                if server.poll() is not None:
                    result.update(status="FAIL_SERVER",
                        reason="Gazebo rendering server exited before sensor observation")
                    return 1
                published=subprocess.run(["gz","topic","-l"],capture_output=True,
                    text=True,timeout=20).stdout
                result["advertised_topics"]=published.splitlines()
                for name,detail in meta["topics"].items():
                    topic=detail["gazebo"]
                    outfile=tmp/(name+".log")
                    handle=outfile.open("wb")
                    proc=subprocess.Popen(["gz","topic","-e","-t",topic],
                        stdout=handle,stderr=subprocess.STDOUT)
                    clients.append((name,detail,proc,handle,outfile))
                time.sleep(args.capture_seconds)
                for name,detail,proc,handle,outfile in clients:
                    if proc.poll() is None:
                        proc.terminate()
                        try:proc.wait(timeout=3)
                        except subprocess.TimeoutExpired:
                            proc.kill();proc.wait()
                    handle.close()
                    typ=("gpu_lidar" if detail["gz_type"]=="gz.msgs.LaserScan"
                         else "depth_camera" if name=="depth" else "camera")
                    result["topics"][name]={"topic":detail["gazebo"],
                        "advertised":detail["gazebo"] in published.splitlines(),
                        "received_payload":has_sample(outfile,typ),
                        "bytes":outfile.stat().st_size if outfile.exists() else 0}
            missing=[n for n,item in result["topics"].items()
                     if not(item["advertised"] and item["received_payload"])]
            if missing:
                result.update(status="FAIL_STREAM",reason="No validated sensor messages: "+", ".join(missing))
                return 1
            result.update(status="PASS_SENSOR_STREAMS",
                reason="Real Ogre2 sensor messages received for every selected camera/lidar topic; ROS bridge and Monza not tested.")
            return 0
        except (OSError,subprocess.TimeoutExpired) as exc:
            result.update(status="ERROR",reason=str(exc))
            return 1
        finally:
            for name,detail,proc,handle,outfile in clients:
                if proc.poll() is None:
                    proc.terminate()
                    try:proc.wait(timeout=2)
                    except subprocess.TimeoutExpired:proc.kill();proc.wait()
                if not handle.closed:handle.close()
            if server is not None and server.poll() is None:
                server.terminate()
                try:server.wait(timeout=4)
                except subprocess.TimeoutExpired:server.kill();server.wait()
            if log.exists():
                result["gazebo_log_tail"]=log.read_text(errors="replace")[-4000:]
            args.report.parent.mkdir(parents=True,exist_ok=True)
            args.report.write_text(json.dumps(result,ensure_ascii=False,indent=2)+"\n",encoding="utf-8")
            print(json.dumps({k:v for k,v in result.items() if k!="gazebo_log_tail"},
                             ensure_ascii=False,indent=2))

if __name__=="__main__":raise SystemExit(main())
