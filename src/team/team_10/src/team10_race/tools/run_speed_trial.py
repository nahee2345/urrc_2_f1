#!/usr/bin/env python3
"""Original preview/camera-start trial supervisor. Pose is used ONLY to stop/score.

Waits for a local VISUAL_READY marker after Gazebo window inspection. Never
publishes a drive command, sensor substitute, start gate or pose initialisation.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shlex
import signal
import subprocess
import shutil
import pty
import threading
import time

p=argparse.ArgumentParser();p.add_argument('--output',required=True)
p.add_argument('--seconds',type=float,default=45);p.add_argument('--lap',action='store_true')
p.add_argument('--until-gate',type=int,help='Validation-only segment endpoint; never supplied to driving nodes')
a=p.parse_args();root=Path(__file__).resolve().parents[4];os.chdir(root)
out=Path(a.output).resolve()
# Latest full lap is128MB. Reserve >4x that estimate and require1GiB before launch.
if shutil.disk_usage(root).free<1024**3:raise SystemExit('Need1GiB free before recording')
if os.statvfs(root).f_favail<10000:raise SystemExit('Insufficient free inodes')
out.mkdir(parents=True,exist_ok=False)
children=[];logs=[];partition=None;stopped=False;last={}
def write(name,obj): (out/name).write_text(json.dumps(obj,indent=2))
def start(name,cmd,vehicle=False):
    f=(out/(name+'.log')).open('w');logs.append(f)
    prefix='source src/team/team_10/tools/f1_env.bash\n' if vehicle else 'source /opt/ros/jazzy/setup.bash\nexport ROS_DOMAIN_ID=42\n'
    if name=='vehicle':prefix+='export HANLA_SPEED_AUDIT_DIR='+shlex.quote(str(out/'audit'))+'\n'
    if name=='record':prefix+='export HANLA_SPEED_LIGHT_RECORDING=1\n'
    if name=='preview':
        master,slave=pty.openpty()
        c=subprocess.Popen(['bash','-c',prefix+shlex.join(cmd)],stdin=slave,
            stdout=slave,stderr=slave,start_new_session=True)
        os.close(slave)
        c.preview_input=master
        def capture():
            while True:
                try:data=os.read(master,65536)
                except OSError:break
                if not data:break
                f.write(data.decode(errors='replace'));f.flush()
        threading.Thread(target=capture,daemon=True).start()
    else:
        c=subprocess.Popen(['bash','-c',prefix+shlex.join(cmd)],stdin=subprocess.PIPE,
            stdout=f,stderr=subprocess.STDOUT,start_new_session=True)
    children.append((name,c));return c
def command(name,cmd):
    c=start(name,cmd,True);rc=c.wait(timeout=65)
    if rc:raise RuntimeError(name+' failed: '+str(rc))
def wait_until(predicate,timeout):
    deadline=time.monotonic()+timeout
    while time.monotonic()<deadline:
        if predicate():return
        time.sleep(.5)
    raise RuntimeError('Readiness timeout')
def stop_vehicle(reason):
    global stopped
    try:write('deliberate_stop.json',dict(sim=last.get('sim'),reason=reason,wall=time.time()))
    except OSError:pass
    command('false_stop',['python3','src/team/team_10/tools/permission.py','false'])
    deadline=time.monotonic()+12
    while time.monotonic()<deadline:
        try:
            d=json.loads((out/'lap_state.json').read_text())
            if d['speed_mps']<.02 and d.get('ongoing_stop_s',0) is not None and d.get('ongoing_stop_s',0)>.5:
                write('stopped.json',d);stopped=True;return
        except (OSError,ValueError):pass
        time.sleep(.5)
    raise RuntimeError('Actual stop not confirmed')
def interrupt(*_):raise KeyboardInterrupt
signal.signal(signal.SIGTERM,interrupt)
files=['team10_race/relaxed_navigation_node.py','team10_race/navigation_node.py','team10_race/speed_planner.py',
       'team10_race/relaxed_controller.py','team10_race/footprint_trajectory.py','team10_race/race_adapter.py',
       'team10_race/start_vision.py','team10_race/start_camera_node.py','team10_race/start_interlock.py',
       'team10_race/speed_audit.py','team10_race/delay_alignment.py','team10_race/path_safety.py','team10_race/wall_arc.py',
       'tools/record_lap.py','tools/record_speed_clock.py','tools/observe_f1.py',
       'tools/run_speed_trial.py','tools/permission.py','tools/speed_metrics.py','tools/speed_response.py',
       'config/navigation.yaml','models/team10_car/model.sdf']
identity={str(root/'src/team/team_10'/f):hashlib.sha256((root/'src/team/team_10'/f).read_bytes()).hexdigest() for f in files}
for f in files:
    dest=out/'source'/f;dest.parent.mkdir(parents=True,exist_ok=True)
    shutil.copy2(root/'src/team/team_10'/f,dest)
write('runtime_identity.json',dict(prefix=str(root/'install_f1/team10_race'),files=identity))
try:
    preview=start('preview',['bash','race_v24.sh','preview','qualifying','--track','monaco'])
    wait_until(lambda:'화면 확인 완료' in (out/'preview.log').read_text(errors='replace'),150)
    vehicle=start('vehicle',['ros2','launch','team10_race','race.launch.py','race_start_mode:=camera'],True)
    time.sleep(3)
    command('spawn',['python3','src/team/team_10/tools/spawn_f1.py'])
    record=start('record',['python3','src/team/team_10/tools/record_lap.py','--output',str(out)],True)
    command('ready_initial',['python3','src/team/team_10/tools/check_f1.py'])
    write('awaiting_visual_review.json',dict(wall=time.time(),instruction='Inspect actual Gazebo window; then create VISUAL_READY'))
    print('WAIT_VISUAL '+str(out/'VISUAL_READY'),flush=True)
    wait_until(lambda:(out/'VISUAL_READY').exists(),1800)
    command('permission',['python3','src/team/team_10/tools/permission.py','true'])
    command('ready_final',['python3','src/team/team_10/tools/check_f1.py'])
    required_records=['poses.jsonl','clock.jsonl','contacts.jsonl','scans.jsonl.gz',
                      'audit/navigation.jsonl','audit/adapter.jsonl']
    def check_recording():
        if record.poll() is not None:raise RuntimeError('recorder process exited')
        for name in required_records:
            path=out/name
            if not path.exists() or path.stat().st_size==0 or time.time()-path.stat().st_mtime>5.:
                raise RuntimeError('recording missing/stale: '+name)
    check_recording()
    write('recording_preflight.json',dict(expected_full_bytes=256000000,
        free_bytes=shutil.disk_usage(out).free,files=required_records))
    os.write(preview.preview_input,b'READY\n')
    write('ready_sent.json',dict(wall=time.time()))
    deadline=time.monotonic()+(1800 if a.lap or a.until_gate is not None else max(200,a.seconds*4+90))
    motion=None;reason='wall deadline'
    while time.monotonic()<deadline:
        if shutil.disk_usage(out).free<512*1024*1024:
            raise RuntimeError('recording space reserve below512MiB')
        check_recording()
        for name,c in [('vehicle',vehicle),('record',record),('preview',preview)]:
            if c.poll() is not None:raise RuntimeError(name+' exited')
        try:last=json.loads((out/'lap_state.json').read_text())
        except (OSError,ValueError):time.sleep(.5);continue
        if last.get('motion_start') is not None:motion=last['motion_start']
        if motion is not None:
            if last.get('finish_line') is not None:reason='ordered gates and finish';break
            if not a.lap and a.until_gate is not None:
                if last.get('next_gate',0)>=a.until_gate:reason='validation segment endpoint';break
            elif not a.lap and last['sim']-motion>=a.seconds:reason='planned segment duration';break
            if (last.get('ongoing_stop_s') or 0)>2.:reason='first sustained stop';break
            if (last.get('ongoing_reverse_s') or 0)>.5:reason='first sustained reverse';break
        time.sleep(.5)
    stop_vehicle(reason)
except BaseException as e:
    try:write('supervisor_error.json',dict(error=repr(e),wall=time.time(),last=last))
    except OSError:pass
    try:stop_vehicle('supervisor error: '+repr(e))
    except BaseException:pass
finally:
    for name,c in reversed(children):
        if c.poll() is None:
            try:os.killpg(c.pid,signal.SIGINT)
            except ProcessLookupError:pass
            try:c.wait(timeout=8)
            except subprocess.TimeoutExpired:
                try:os.killpg(c.pid,signal.SIGTERM)
                except ProcessLookupError:pass
                try:c.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    try:os.killpg(c.pid,signal.SIGKILL)
                    except ProcessLookupError:pass
    for f in logs:f.close()
    write('runtime_match.json',dict(matches={p:hashlib.sha256(Path(p).read_bytes()).hexdigest()==v for p,v in identity.items()}))
    write('cleanup.json',dict(actual_stopped=stopped,children=[dict(name=n,pid=c.pid,returncode=c.poll()) for n,c in children]))
print('TRIAL_DONE '+str(out),flush=True)
