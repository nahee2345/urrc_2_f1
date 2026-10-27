#!/usr/bin/env python3
"""Isolated Stage-1 physics probe; does not modify the official race world.

--prepare builds a simple flat-ground SDF with the real four-wheel car.
--run uses installed Gazebo Harmonic, reports measured body displacement.
Passing this probe proves translation only, NOT Monza laps or AI readiness.
"""
from __future__ import annotations
import argparse
import json
import math
from pathlib import Path
import re
import shutil
import subprocess
import sys
import tempfile
import time
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from vehicle_generator.model import Vehicle, generate, sub, fmt, manifest


def make_smoke_world(v: Vehicle) -> str:
    v.validate()
    sdf = ET.Element("sdf", version="1.9")
    world = sub(sdf, "world", name="urrc_stage1_smoke")
    sub(world, "gravity", "0 0 -9.81")
    physics = sub(world, "physics", name="physics", type="ignored")
    sub(physics, "max_step_size", "0.001")
    sub(physics, "real_time_factor", "1.0")
    for filename, name in (
        ("gz-sim-physics-system", "Physics"),
        ("gz-sim-user-commands-system", "UserCommands"),
        ("gz-sim-scene-broadcaster-system", "SceneBroadcaster"),
    ):
        sub(world, "plugin", filename=filename, name="gz::sim::systems::" + name)
    floor = sub(world, "model", name="stage1_ground")
    sub(floor, "static", "true")
    link = sub(floor, "link", name="ground_link")
    for kind in ("visual", "collision"):
        el = sub(link, kind, name=f"ground_{kind}")
        sub(el, "pose", "0 0 -0.05 0 0 0")
        sub(sub(sub(el, "geometry"), "box"), "size", "20 20 0.10")
    vehicle = ET.fromstring(generate(v)).find("model")
    sub(vehicle, "pose", fmt(0, 0, v.wheel_radius + 0.002, 0, 0, 0))
    world.append(vehicle)
    ET.indent(sdf, space="  ")
    return ET.tostring(sdf, encoding="unicode", xml_declaration=True)


def model_pose_stream(stream: str, model_name: str):
    """Read a Gazebo Pose_V text stream with balanced nested braces."""
    positions = []
    for match in re.finditer(r"(?m)^pose\s*\{", stream):
        depth, stop = 0, None
        for index in range(match.end()-1, len(stream)):
            if stream[index] == "{":
                depth += 1
            elif stream[index] == "}":
                depth -= 1
                if depth == 0:
                    stop = index
                    break
        if stop is None:
            continue
        body = stream[match.end():stop]
        name = re.search(r'(?m)^\s*name:\s*"([^"]+)"', body)
        if not name or name.group(1) != model_name:
            continue
        pos = re.search(r"position\s*\{([^}]*)\}", body, re.S)
        if not pos:
            continue
        def num(axis):
            value = re.search(rf"\b{axis}:\s*([-+\d.eE]+)", pos.group(1))
            return float(value.group(1)) if value else 0.0
        positions.append((num("x"), num("y"), num("z")))
    return positions


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, default=ROOT/"config/stage1_rwd.json")
    parser.add_argument("--out", type=Path, default=ROOT/"generated/stage1_smoke.sdf")
    parser.add_argument("--report", type=Path, default=ROOT/"generated/stage1_smoke_result.json")
    parser.add_argument("--prepare", action="store_true")
    parser.add_argument("--run", action="store_true")
    parser.add_argument("--startup-seconds", type=float, default=8.0)
    parser.add_argument("--motion-seconds", type=float, default=9.0)
    args = parser.parse_args()
    if not (args.prepare or args.run):
        parser.error("choose --prepare or --run")
    car = Vehicle.from_file(args.config)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(make_smoke_world(car), encoding="utf-8")
    if not args.run:
        print(f"Generated isolated test world: {args.out}")
        return 0
    result = {"test":"gazebo_headless_physics", "vehicle":car.name,
              "status":"NOT_RUN", "reason":"", "mass_kg":car.mass}
    server = observer = None
    with tempfile.TemporaryDirectory(prefix="urrc_stage1_") as temp:
        log = Path(temp)/"gazebo.log"
        pose_log = Path(temp)/"poses.log"
        try:
            if not shutil.which("gz"):
                result.update(status="UNAVAILABLE", reason="gz CLI not installed")
                return 2
            check = subprocess.run(["gz","sdf","-p",str(args.out)],
                stdout=subprocess.DEVNULL,stderr=subprocess.PIPE,text=True,timeout=30)
            if check.returncode:
                result.update(status="FAIL_SDF",reason=check.stderr[-1400:])
                return 1
            with log.open("w") as log_file, pose_log.open("w") as pose_file:
                server = subprocess.Popen(["gz","sim","-s","-r","-v","3",str(args.out)],
                    stdout=log_file,stderr=subprocess.STDOUT)
                time.sleep(args.startup_seconds)
                if server.poll() is not None:
                    result.update(status="FAIL_START",reason="Gazebo server exited")
                    return 1
                observer = subprocess.Popen(["gz","topic","-e","-t",
                    "/world/urrc_stage1_smoke/dynamic_pose/info"],
                    stdout=pose_file,stderr=subprocess.STDOUT)
                time.sleep(2)
                for side in ("rear_left","rear_right"):
                    topic = f"/model/{car.name}/drive/{side}"
                    r = subprocess.run(["gz","topic","-t",topic,"-m","gz.msgs.Double",
                                        "-p","data: -4.0"],capture_output=True,text=True,timeout=15)
                    if r.returncode:
                        result.update(status="FAIL_COMMAND",reason=f"{topic}: {r.stderr[-400:]}")
                        return 1
                time.sleep(args.motion_seconds)
                for side in ("rear_left","rear_right"):
                    subprocess.run(["gz","topic","-t",f"/model/{car.name}/drive/{side}",
                        "-m","gz.msgs.Double","-p","data: 0"],capture_output=True,
                        text=True,timeout=15)
            positions = model_pose_stream(pose_log.read_text(errors="replace"),car.name)
            if len(positions) < 2:
                result.update(status="FAIL_TELEMETRY",reason=f"pose samples: {len(positions)}")
                return 1
            start, end = positions[0],positions[-1]
            displacement = math.dist(start[:2],end[:2])
            result.update(samples=len(positions),start_xyz=start,end_xyz=end,
                          displacement_m=round(displacement,6))
            if displacement < 0.01:
                result.update(status="FAIL_MOTION",reason="No physical translation >= 0.01 m")
                return 1
            if abs(end[2] - car.wheel_radius) > 0.035:
                result.update(status="FAIL_CONTACT",reason="Vehicle height inconsistent with wheel support")
                return 1
            result.update(status="PASS_MOTION_ONLY",
                          reason="Wheel-driven translation observed. Steering, braking and Monza still unverified.")
            return 0
        except (OSError,subprocess.TimeoutExpired) as e:
            result.update(status="ERROR",reason=str(e))
            return 1
        finally:
            for proc in (observer,server):
                if proc is not None and proc.poll() is None:
                    proc.terminate()
                    try:
                        proc.wait(timeout=4)
                    except subprocess.TimeoutExpired:
                        proc.kill()
                        proc.wait()
            if log.exists():
                result["gazebo_log_tail"]=log.read_text(errors="replace")[-3000:]
            args.report.parent.mkdir(parents=True,exist_ok=True)
            args.report.write_text(json.dumps(result,ensure_ascii=False,indent=2)+"\n",encoding="utf-8")
            print(json.dumps({k:v for k,v in result.items() if k!="gazebo_log_tail"},
                             ensure_ascii=False,indent=2))


if __name__ == "__main__":
    raise SystemExit(main())
