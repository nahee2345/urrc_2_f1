#!/usr/bin/env python3
"""Run the *competition ROS controller* on a CSV-reconstructed Gazebo world.

Checks independent own-model pose, RGB red->dark signal, issued Twist and
measured physical distance. Never touches the official organizer workspace.
"""
from __future__ import annotations

import argparse
import datetime
import os
import re
import shutil
import signal
import subprocess
import sys
import time
from pathlib import Path

from generate_world import TRACKS, build_world

ROOT=Path(__file__).resolve().parents[2]


def launch(cmd,output,env):
    fd=open(output,"w",encoding="utf-8",buffering=1)
    try:
        return subprocess.Popen(cmd,stdout=fd,stderr=subprocess.STDOUT,
                                env=env,start_new_session=True),fd
    except BaseException:
        fd.close()
        raise


def call(cmd,env,timeout=8):
    try:
        return subprocess.run(cmd,env=env,capture_output=True,text=True,
                              timeout=timeout,check=False)
    except (OSError,subprocess.TimeoutExpired) as exc:
        return subprocess.CompletedProcess(cmd,124,"",str(exc))


def stop(process):
    if process is None or process.poll() is not None:
        return
    try:
        os.killpg(process.pid,signal.SIGINT)
        process.wait(timeout=5)
    except (OSError,subprocess.TimeoutExpired):
        try:
            os.killpg(process.pid,signal.SIGKILL)
        except ProcessLookupError:
            pass
        try:
            process.wait(timeout=3)
        except subprocess.TimeoutExpired:
            pass


def summary(path):
    source=path.read_text(encoding="utf-8",errors="replace") if path.exists() else ""
    speeds=[float(s) for s in re.findall(r"physical_distance=([0-9.]+)",source)]
    sources=re.findall(r"TEAM11 START source=([A-Z_]+)",source)
    route=re.findall(r"TEAM11 map=([a-z]+) grid=([0-9]+)",source)
    lines=[
        ("카메라 빨간불 인식","TEAM11 camera red confirmed" in source),
        ("카메라 소등 인식","TEAM11 camera LIGHTS OUT confirmed" in source),
        ("자체 차량 위치 수신","TEAM11 own pose ready" in source),
        ("맵·그리드·경로 선택",bool(route)),
        ("출발 명령 승인",bool(sources)),
        ("실제 주행 거리 1m 초과",bool(speeds) and max(speeds)>1.0),
    ]
    print("\n[TEAM11] 실제 Gazebo 시험 결과")
    for name,passed in lines:
        print("  "+("PASS" if passed else "NOT CONFIRMED")+" | "+name)
    print("  start_source:",sources[-1] if sources else "없음")
    print("  map/grid:",route[-1] if route else "없음")
    print("  measured_distance_m:",max(speeds) if speeds else 0.)
    for line in source.splitlines():
        if "TEAM11 WAIT" in line or "TEAM11 SAFE STOP" in line:
            print("  주의:",line.strip()[-330:])
    return all(passed for _,passed in lines)


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--track",choices=TRACKS,default="monza")
    ap.add_argument("--grid",type=int,choices=range(1,12),default=1)
    ap.add_argument("--seconds",type=int,default=90,
                    help="Wall-clock test duration after lights-out (default: 90)")
    ap.add_argument("--headless",action="store_true",
                    help="Run Gazebo server without GUI (requires OGRE2/EGL)")
    args=ap.parse_args()
    if args.seconds<10:
        ap.error("--seconds must be at least 10")
    for exe in ("gz","ros2"):
        if shutil.which(exe) is None:
            ap.error(exe+" not found; run sim_test/run.sh after installing ROS/Gazebo")

    stamp=datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    out=ROOT/"build"/"team11_gazebo_tests"/f"{args.track}_g{args.grid:02d}_{stamp}"
    out.mkdir(parents=True,exist_ok=True)
    world=out/"race_world.sdf"
    info=build_world(args.track,args.grid,world)
    print("[TEAM11] Generated from embedded CSV:",info)
    print("[TEAM11] Output:",out,flush=True)
    print("[TEAM11] IMPORTANT: reconstructed road, not official organizer SDF.")

    env=os.environ.copy()
    env["GZ_PARTITION"]="team11_replay_"+str(os.getpid())
    # Keep ROS isolated so user can run the full test without other ROS nodes.
    env["ROS_DOMAIN_ID"]=str(70+os.getpid()%80)

    gz_cmd=["gz","sim","-r","-v","3"]
    if args.headless:
        gz_cmd+=["-s","--headless-rendering"]
    gz_cmd.append(str(world))

    gz=bridge=clock=gate=None
    files=[]
    try:
        gz,f=launch(gz_cmd,out/"gazebo.log",env)
        files.append(f)
        ros_cmd=["ros2","launch","team11_race","race.launch.py",
                 "race_start_mode:=camera"]
        bridge,f=launch(ros_cmd,out/"team11_ros.log",env)
        files.append(f)
        clock,f=launch(["ros2","run","ros_gz_bridge","parameter_bridge",
                        "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
                       out/"clock_bridge.log",env)
        files.append(f)

        deadline=time.monotonic()+75.
        camera_seen=False
        while time.monotonic()<deadline:
            if gz.poll() is not None:
                print("[TEAM11 ERROR] Gazebo terminated; check gazebo.log",file=sys.stderr)
                return 2
            if bridge.poll() is not None:
                print("[TEAM11 ERROR] ROS launch terminated; check team11_ros.log",file=sys.stderr)
                return 2
            topics=call(["gz","topic","-l"],env,timeout=4)
            if topics.returncode==0:
                advertised=set(topics.stdout.splitlines())
                camera_seen="/team11/start_image_11" in advertised
                pose_seen="/team11/pose_11" in advertised
                if camera_seen and pose_seen:
                    print("[TEAM11] Gazebo RGB and own-pose topics advertised",flush=True)
                    break
            time.sleep(1)
        else:
            print("[TEAM11 ERROR] Camera or pose topic missing; inspect logs",
                  file=sys.stderr)
            return 2

        # Let ROS camera subscribers observe red lamps BEFORE they disappear.
        print("[TEAM11] Five red lamps ON. Holding signal for 8 seconds.",flush=True)
        time.sleep(8)
        # One Gazebo user-command remove transaction switches all five off.
        service=f"/world/{args.track}/remove"
        removed=call(["gz","service","-s",service,
                      "--reqtype","gz.msgs.Entity",
                      "--reptype","gz.msgs.Boolean","--timeout","5000",
                      "--req",'name: "team11_test_lights" type: MODEL'],
                     env,timeout=12)
        print("[TEAM11] LIGHTS OUT remove:",
              " ".join((removed.stdout+" "+removed.stderr).split())[:260],flush=True)
        if removed.returncode!=0 or "data: true" not in removed.stdout.lower():
            print("[TEAM11 ERROR] Lights-out service not confirmed; do NOT send gate",
                  file=sys.stderr)
            return 2

        # Persistent TRUE publisher prevents losing the one-time gate when the
        # ROS controller starts late. It begins only after five lights are off.
        gate,f=launch(["ros2","topic","pub","-r","10",
                       "/race/start/team_11","std_msgs/msg/Bool",
                       "{data: true}"],out/"race_gate.log",env)
        files.append(f)
        print("[TEAM11] Official start gate TRUE after confirmed lights-out.",
              flush=True)
        print("[TEAM11] ROS log:",out/"team11_ros.log",flush=True)

        end=time.monotonic()+args.seconds
        shown=0
        while time.monotonic()<end:
            if gz.poll() is not None or bridge.poll() is not None:
                print("[TEAM11 WARNING] Gazebo or ROS launch exited; inspect logs",
                      flush=True)
                break
            try:
                data=(out/"team11_ros.log").read_text(
                    encoding="utf-8",errors="replace")
                if len(data)>shown:
                    delta=data[shown:]
                    for line in delta.splitlines():
                        if any(word in line for word in (
                            "TEAM11 map=","TEAM11 START","TEAM11 WAIT",
                            "TEAM11 camera","TEAM11 own pose",
                            "TEAM11 SAFE STOP","physical_distance=",
                            "TEAM11 STRATEGY")):
                            print(line[-420:],flush=True)
                    shown=len(data)
            except OSError:
                pass
            time.sleep(1)
        return 0 if summary(out/"team11_ros.log") else 3
    except KeyboardInterrupt:
        print("\n[TEAM11] Interrupted; partial analysis:",flush=True)
        summary(out/"team11_ros.log")
        return 130
    finally:
        for p in (gate,clock,bridge,gz):
            stop(p)
        for fd in files:
            fd.close()
        print("[TEAM11] Logs:",out,flush=True)


if __name__=="__main__":
    sys.exit(main())
