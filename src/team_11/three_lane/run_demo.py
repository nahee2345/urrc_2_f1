#!/usr/bin/env python3
"""TEAM11 two-track Gazebo demonstration with official 5-light start."""
from __future__ import annotations
import argparse
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import time

HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE))
from prepare import TRACKS, prepare

def build(src, dest, name):
    subprocess.run(["cmake","-S",str(src),"-B",str(dest),
                    "-DCMAKE_BUILD_TYPE=Release"],check=True)
    subprocess.run(["cmake","--build",str(dest),"--parallel","2"],check=True)
    lib=dest / ("lib"+name+".so")
    if not lib.exists():
        raise RuntimeError("Missing Gazebo plugin: "+str(lib))
    return lib

def stop(proc):
    if proc is not None and proc.poll() is None:
        os.killpg(proc.pid,signal.SIGTERM)
        try:proc.wait(timeout=6)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid,signal.SIGKILL)
            proc.wait()

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument("track",choices=TRACKS)
    ap.add_argument("--source",default="~/urrc_2")
    ap.add_argument("--prepare-only",action="store_true")
    ap.add_argument("--server",action="store_true",help="headless Gazebo server")
    args=ap.parse_args()
    src=Path(args.source).expanduser().resolve()
    if not (src/"race/pace_cars/routes.py").exists():
        ap.error("Clone source first: git clone https://github.com/nahee2345/urrc_2.git ~/urrc_2")
    dest=src/"generated/team11"/args.track
    plugin_dir=src/"generated/team11"/"build"
    pace_src=src/"race/pace_cars/plugin"
    team_src=HERE/"plugin"
    pace_lib=plugin_dir/"pace"/"liburrc_pace_cars.so"
    team_lib=plugin_dir/"team"/"liburrc_team11.so"
    if not args.prepare_only:
        if not shutil.which("gz"):
            ap.error("Gazebo Harmonic 'gz' executable not found")
        pace_lib=build(pace_src,plugin_dir/"pace","urrc_pace_cars")
        team_lib=build(team_src,plugin_dir/"team","urrc_team11")
    manifest=prepare(src,dest,args.track,team_lib,pace_lib)
    print("[TEAM11] 3 lanes:",manifest["routes"],flush=True)
    print("[TEAM11] spacing:",round(manifest["lane_spacing_m"],3),
          "m, start lane:",manifest["first_lane"],flush=True)
    if args.prepare_only:
        print("[TEAM11] world prepared; not run in Gazebo",flush=True)
        return 0
    sys.path.insert(0,str(src/"race/pace_cars"))
    from run import start_signal, fire_start, confirmed_all_off
    env=os.environ.copy()
    env["GZ_PARTITION"]=f"urrc_team11_{os.getuid()}"
    env["GZ_SIM_RESOURCE_PATH"]=manifest["resource_path"]+os.pathsep+env.get("GZ_SIM_RESOURCE_PATH","")
    env["GZ_SIM_SYSTEM_PLUGIN_PATH"]=str(pace_lib.parent)+os.pathsep+str(team_lib.parent)+os.pathsep+env.get("GZ_SIM_SYSTEM_PLUGIN_PATH","")
    cmd=["gz","sim","-r","-v","3"]
    if args.server:
        cmd+=["-s","--headless-rendering"]
    else:
        cmd+=["--gui-config",str(dest/"resources/urrc_track_gazebo/worlds"/(args.track+"_grid.gui.config"))]
    cmd.append(manifest["world_file"])
    sim=lights=None
    log=dest/"team11_gazebo.log"
    try:
        with log.open("wb") as fd:
            sim=subprocess.Popen(cmd,cwd=src,env=env,stdout=fd,
                                 stderr=subprocess.STDOUT,start_new_session=True)
            ready=f"/urrc/{args.track}/pace/start/ready"
            for _ in range(100):
                if sim.poll() is not None:
                    raise RuntimeError("Gazebo quit. Inspect "+str(log))
                try:
                    r=subprocess.run(["gz","topic","-l"],env=env,text=True,
                                     capture_output=True,timeout=5)
                    if r.returncode==0 and ready in r.stdout.splitlines():
                        break
                except subprocess.TimeoutExpired:
                    pass
                time.sleep(.5)
            else:
                raise RuntimeError("Pace plugin not ready; inspect "+str(log))
            lights=start_signal(args.track,env,dest)
            fired=False
            assert lights.stdout is not None
            for line in lights.stdout:
                print("[LIGHT] "+line.rstrip(),flush=True)
                if not fired and confirmed_all_off(line):
                    fire_start(args.track,env)
                    fired=True
                    print("[TEAM11] All five lights off: start",flush=True)
            if lights.wait()!=0 or not fired:
                raise RuntimeError("Five-light signal confirmation failed; no start")
            print("[TEAM11] Steering/speed/lane transitions in "+str(log),flush=True)
            return sim.wait()
    except KeyboardInterrupt:
        return 130
    finally:
        stop(lights)
        stop(sim)

if __name__=="__main__":
    try:
        sys.exit(main())
    except (OSError,RuntimeError,subprocess.CalledProcessError) as e:
        print("[TEAM11 ERROR]",e,file=sys.stderr)
        sys.exit(2)
