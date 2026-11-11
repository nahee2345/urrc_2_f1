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
import threading

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
    ap.add_argument("--start-mode",choices=("camera","immediate"),default="camera",
                    help="camera waits for observed red->all-off images")
    ap.add_argument("--lane",choices=("auto","1","2","3"),default="auto",
                    help="lane 1=left, 2=center, 3=right; auto=nearest")
    ap.add_argument("--fixed-lane",action="store_true",
                    help="hold one exact route for QA; disables all overtaking and center-corner merge")
    ap.add_argument("--allow-lane-change",action="store_true",
                    help="legacy switch (lane changes and automatic corner-center merge now default)")
    ap.add_argument("--scenario",choices=("normal","slow-ahead","late-pass","rear-fast","rear-contact"),default="normal",
                    help="slow-ahead: 5m/s red 25m ahead; late-pass: 5m/s red 45m ahead (generated-only)")
    ap.add_argument("--physics-contact",action="store_true",
                    help="opt-in physical 50kg TEAM11 versus dynamic 1.5kg opponents; no scripted bounce")
    ap.add_argument("--practice-cars",action="store_true",
                    help="enable original moving red/blue opponents after solo baseline")
    ap.add_argument("--grid",type=int,choices=range(1,13),default=1,
                    help="odd grids -> lane 3; even grids -> lane 1")
    args=ap.parse_args()
    if args.physics_contact and not args.practice_cars:
        ap.error("--physics-contact requires --practice-cars")
    if args.scenario!="normal" and not args.practice_cars:
        ap.error("staged scenarios require --practice-cars")
    if args.fixed_lane and args.allow_lane_change:
        ap.error("--fixed-lane and --allow-lane-change cannot be combined")
    effective_fixed_lane = args.fixed_lane  # default solo still safely enters route 2 at corners
    src=Path(args.source).expanduser().resolve()
    if not (src/"race/pace_cars/routes.py").exists():
        ap.error("Clone source first: git clone https://github.com/nahee2345/urrc_2.git ~/urrc_2")
    dest=src/"generated/team11"/args.track
    plugin_dir=src/"generated/team11"/"build"
    pace_src=HERE/"pace_bounce"
    team_src=HERE/"plugin"
    pace_lib=plugin_dir/"pace_bounce"/"liburrc_pace_cars.so"
    team_lib=plugin_dir/"team"/"liburrc_team11.so"
    if not args.prepare_only:
        if not shutil.which("gz"):
            ap.error("Gazebo Harmonic 'gz' executable not found")
        pace_lib=build(pace_src,plugin_dir/"pace_bounce","urrc_pace_cars")
        team_lib=build(team_src,plugin_dir/"team","urrc_team11")
    manifest=prepare(src,dest,args.track,team_lib,pace_lib,args.start_mode,args.lane,effective_fixed_lane,args.grid,args.practice_cars,args.scenario,args.physics_contact)
    print("[TEAM11] grid:",manifest["grid_position"],
          "default lane:",manifest["selected_lane_number"],flush=True)
    print("[TEAM11] 3 lanes:",manifest["routes"],flush=True)
    print("[TEAM11] spacing:",round(manifest["lane_spacing_m"],3),
          "m, selected lane:",manifest["selected_lane_number"],
          "nearest lane:",manifest["nearest_lane_number"],
          "fixed:",effective_fixed_lane,
          "practice cars:",args.practice_cars,
          "scenario:",args.scenario,
          "physics contact:",args.physics_contact,flush=True)
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
    camera_ready=Path(manifest["camera_ready_file"])
    # Never accept a marker produced by a previous Gazebo session.
    if camera_ready.exists():
        camera_ready.unlink()
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
            if args.start_mode=="camera":
                # Do not turn on the five lights before the ego camera is advertised.
                camera_topic=manifest["start_camera_topic"]
                for _ in range(100):
                    if sim.poll() is not None:
                        raise RuntimeError("Gazebo exited while waiting for camera")
                    response=subprocess.run(["gz","topic","-l"],env=env,
                                            capture_output=True,text=True,timeout=6)
                    if camera_topic in response.stdout.splitlines():
                        print("[TEAM11 CAMERA] topic ready:",camera_topic,flush=True)
                        break
                    time.sleep(.5)
                else:
                    raise RuntimeError("TEAM11 camera topic not ready: "+camera_topic)
                # Topic listing proves advertisement, not delivery to TEAM11.
                # Wait for the plugin to decode at least one actual frame.
                for _ in range(120):
                    if sim.poll() is not None:
                        raise RuntimeError("Gazebo exited while awaiting first camera frame")
                    if camera_ready.is_file():
                        print("[TEAM11 CAMERA] real frame confirmed, signal enabled",
                              camera_ready.read_text().strip(),flush=True)
                        break
                    time.sleep(.5)
                else:
                    raise RuntimeError(
                        "TEAM11 failed to receive a valid RGB frame before signal. "
                        "Check gz-camera/system plugin and Gazebo log: "+str(log))
            # Camera mode gates only TEAM11 using actual image analysis.

            # The original pace cars may start after the regular light sequence;
            # pace-car light failures are logged and are not overridden by synthetic start.
            def run_pace_signal():
                nonlocal lights
                try:
                    lights=start_signal(args.track,env,dest)
                    confirmed=False
                    if lights.stdout is not None:
                        for line in lights.stdout:
                            print("[LIGHT] "+line.rstrip(),flush=True)
                            if not confirmed and confirmed_all_off(line):
                                fire_start(args.track,env)
                                confirmed=True
                                print("[PACE] Practice cars started after lights off",flush=True)
                    if lights.wait()!=0 or not confirmed:
                        print("[PACE WARNING] Light sequence failed; TEAM11 runs independently",flush=True)
                except Exception as exc:
                    print("[PACE WARNING] "+str(exc)+"; TEAM11 runs independently",flush=True)
            threading.Thread(target=run_pace_signal,daemon=True).start()
            print(f"[TEAM11] Start mode={args.start_mode}; camera={manifest['start_camera_topic']}",flush=True)
            print("[TEAM11] Log:",log,flush=True)
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
