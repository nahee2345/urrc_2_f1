"""Explicit local practice launcher. Competition launch never imports this module."""
import argparse
import fcntl
import hashlib
import json
import math
import os
from pathlib import Path
import re
import signal
import socket
import subprocess
import sys
import tempfile
import time
import uuid
import xml.etree.ElementTree as ET
from contextlib import contextmanager


TRACKS=('monza','monaco','hungaroring')

def state_dir(team):
    identity=hashlib.sha256((str(Path(team).resolve())+'@'+socket.gethostname()).encode()).hexdigest()[:12]
    return Path(os.environ.get('URRC_PRACTICE_STATE_DIR',str(Path(tempfile.gettempdir())/('urrc_team01_'+identity))))

def write_json(path,data):
    temporary=path.with_suffix('.tmp');temporary.write_text(json.dumps(data,ensure_ascii=False,indent=2));temporary.replace(path)

def process_start(pid):
    try:return Path('/proc',str(pid),'stat').read_text().split(') ',1)[1].split()[19]
    except (OSError,IndexError):return None

def alive(pid,start):return start is not None and process_start(pid)==start

@contextmanager
def role_lock(directory,role):
    directory.mkdir(parents=True,exist_ok=True)
    with (directory/(str(role)+'.lock')).open('a+') as lock:
        try:fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        except BlockingIOError:raise RuntimeError(str(role)+'번이 이미 실행 중입니다. 같은 번호를 중복 실행하지 마세요.')
        yield

def run(command,env,timeout=8):
    return subprocess.run(command,env=env,capture_output=True,text=True,timeout=timeout)

def services(env):
    result=run(['gz','service','-l'],env,4)
    return result.stdout.splitlines() if result.returncode==0 else []

def request(session,service,kind,payload,optional=False):
    try:
        result=run(['gz','service','-s','/world/'+session['world']+'/'+service,'--reqtype',kind,
                    '--reptype','gz.msgs.Boolean','--timeout','3000','--req',payload],session['env'],5)
        ok=result.returncode==0 and re.search(r'\bdata:\s*true',result.stdout) is not None
    except (OSError,subprocess.TimeoutExpired):
        if not optional:raise
        return False
    if not ok and not optional:raise RuntimeError(service+' 요청 실패: '+result.stdout[-200:]+result.stderr[-200:])
    return ok

def wait_world(session,timeout=60):
    deadline=time.monotonic()+timeout
    target='/world/'+session['world']+'/scene/info'
    while time.monotonic()<deadline:
        if not alive(session['owner_pid'],session['owner_start']):raise RuntimeError('1번 경기장 실행이 종료됐습니다.')
        if target in services(session['env']):return
        time.sleep(.25)
    raise RuntimeError('경기장 서비스 준비 시간 초과: '+target)

def map_package(team):
    candidates=[]
    if os.environ.get('URRC_MAP_PACKAGE'):candidates.append(Path(os.environ['URRC_MAP_PACKAGE']))
    try:
        from ament_index_python.packages import get_package_share_directory
        candidates.append(Path(get_package_share_directory('urrc_track_gazebo')))
    except LookupError:pass
    for parent in Path(team).resolve().parents:candidates.append(parent/'src/urrc_track_gazebo')
    for p in candidates:
        if (p/'worlds').is_dir() and (p/'models').is_dir():return p.resolve()
    raise RuntimeError('공통 urrc_track_gazebo 맵 패키지를 찾지 못했습니다. 공통 workspace를 source하거나 1번에 --map-file을 지정하세요.')

def map_spec(team,track,map_file=None,grid_file=None,spawn=None):
    package=None
    if map_file:
        world=Path(map_file).expanduser().resolve()
        for parent in world.parents:
            if (parent/'models').is_dir():package=parent;break
    else:
        package=map_package(team)
        if track=='random':
            import random
            available=[name for name in TRACKS if (package/'worlds'/(name+'_race.sdf')).is_file()]
            if not available:raise RuntimeError('실행 가능한 공통 맵이 없습니다.')
            track=random.choice(available)
        world=package/'worlds'/(track+'_race.sdf')
    node=ET.parse(world).getroot().find('world')
    if node is None:raise ValueError('SDF world가 없는 맵 파일입니다.')
    name=node.get('name')
    if not name or '/' in name:raise ValueError('유효하지 않은 월드 이름')
    slot=None
    metadata=Path(grid_file).expanduser().resolve() if grid_file else (package/'tracks/processed'/(name+'.json') if package else None)
    if spawn is not None:
        x,y,z,yaw=spawn;slot=dict(x=x,y=y,road_z=z,yaw=yaw)
    elif metadata is not None and metadata.is_file():
        grids=json.loads(metadata.read_text()).get('grid',[])
        if grids:slot=min(grids,key=lambda p:int(p.get('position',1)))
    if slot is None:raise RuntimeError('이 맵의 출발 위치가 없습니다. 1번에 --spawn X Y ROAD_Z YAW 또는 --grid-file을 지정하세요.')
    if not all(math.isfinite(float(slot[k])) for k in ('x','y','road_z','yaw')):raise ValueError('유효하지 않은 출발 위치')
    gui=world.parent/(name+'_grid.gui.config')
    return dict(world=name,map_file=str(world),map_package=str(package) if package else '',
                gui=str(gui) if gui.is_file() else '',slot=slot,has_lights=node.find("model[@name='race_start_gantry']") is not None)

def owned_pids(token,role=None):
    result=[]
    for p in Path('/proc').iterdir():
        if not p.name.isdigit() or int(p.name)==os.getpid():continue
        try:
            fields=(p/'environ').read_bytes().split(b'\0')
            if ('URRC_PRACTICE_SESSION='+token).encode() not in fields:continue
            if role is not None and ('URRC_PRACTICE_ROLE='+str(role)).encode() not in fields:continue
            start=process_start(int(p.name))
            if start is not None:result.append((int(p.name),start))
        except OSError:pass
    return result

def stop_owned(token,role=None):
    for sig,delay in ((signal.SIGINT,2.),(signal.SIGTERM,1.),(signal.SIGKILL,.2)):
        pids=owned_pids(token,role)
        if not pids:return
        for pid,start in pids:
            try:
                if alive(pid,start):os.kill(pid,sig)
            except ProcessLookupError:pass
        deadline=time.monotonic()+delay
        while time.monotonic()<deadline and any(alive(pid,start) for pid,start in pids):time.sleep(.05)

def stop_child(process):
    if process is None or process.poll() is not None:return
    for sig,delay in ((signal.SIGINT,4.),(signal.SIGTERM,2.),(signal.SIGKILL,1.)):
        try:os.killpg(process.pid,sig);process.wait(timeout=delay);return
        except ProcessLookupError:return
        except subprocess.TimeoutExpired:pass

def load_session(directory):
    path=directory/'session.json'
    if not path.is_file():raise RuntimeError('먼저 다른 터미널에서 bash run.sh 1을 실행하세요.')
    session=json.loads(path.read_text())
    if session.get('closing') or not alive(session['owner_pid'],session['owner_start']):raise RuntimeError('경기장이 종료됐습니다. 1번부터 다시 실행하세요.')
    env=os.environ.copy();env.update(session['connection']);session['env']=env
    return session

def persist(directory,session):
    write_json(directory/'session.json',{k:v for k,v in session.items() if k!='env'})

def pause_on_exit(directory,session):
    try:
        current=json.loads((directory/'session.json').read_text())
        if current.get('closing') or current.get('token')!=session['token']:return
        request(session,'control','gz.msgs.WorldControl','pause: true',optional=True)
        for side in ('left','right'):
            try:run(['gz','topic','-t','/team_01/actuator/rear_'+side+'_velocity','-m','gz.msgs.Double','-p','data: 0.0'],session['env'],3)
            except (OSError,subprocess.TimeoutExpired):pass
        print('[PRACTICE] 제어/센서 종료: 안전을 위해 경기장을 일시정지했습니다.',flush=True)
    except (OSError,ValueError):pass

def set_light(session,index,on):
    color='r: 1 g: .008 b: .008 a: 1' if on else 'r: 0 g: 0 b: 0 a: 1'
    payload=(f'name: "red_lens_{index}" parent_name: "gantry_visuals" '
             'material { ambient { r: .12 g: .006 b: .008 a: 1 } diffuse { r: .12 g: .006 b: .008 a: 1 } '
             'specular { r: .02 g: .02 b: .02 a: 1 } emissive { '+color+' } }')
    request(session,'visual_config','gz.msgs.Visual',payload)

def wait_camera(session):
    os.environ.update(session['connection'])
    import rclpy
    from sensor_msgs.msg import Image
    from std_msgs.msg import Bool,String
    from rclpy.qos import qos_profile_sensor_data
    rclpy.init();node=rclpy.create_node('team_01_practice_start_observer');seen=dict(images=0,traffic=0,control=False)
    node.create_subscription(Image,'/team_01/camera/image_raw',lambda m:seen.update(images=seen['images']+1),qos_profile_sensor_data)
    node.create_subscription(String,'/team_01/traffic_light_state',lambda m:seen.update(traffic=seen['traffic']+1),10)
    node.create_subscription(Bool,'/team_01/internal/controller_ready',lambda m:seen.update(control=m.data),10)
    try:
        deadline=time.monotonic()+60
        while time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.05)
            if seen['images']>=3 and seen['traffic']>=1 and seen['control']:return
        raise RuntimeError('2번 실제 카메라/3번 인식·제어 준비 미완료. 2번과 3번 로그를 확인하세요: '+str(seen))
    finally:node.destroy_node();rclpy.shutdown()

def map_role(args,team,directory):
    with role_lock(directory,1):
        spec=map_spec(team,args.track,args.map_file,args.grid_file,args.spawn)
        suffix=hashlib.sha256((str(team)+socket.gethostname()).encode()).hexdigest()[:12]
        env=os.environ.copy();env.update(GZ_PARTITION='team_01_practice_'+suffix,
            ROS_DOMAIN_ID=os.environ.get('ROS_DOMAIN_ID','71'),GZ_IP='127.0.0.1',
            ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST',URRC_PRACTICE_SESSION=uuid.uuid4().hex,URRC_PRACTICE_ROLE='1')
        if any(s.startswith('/world/') for s in services(env)):raise RuntimeError('이 연습 연결에 기존 월드가 있습니다. 이전 1번 실행을 종료하세요.')
        resources=[str(Path(spec['map_package'])/'models')] if spec['map_package'] else []
        if env.get('GZ_SIM_RESOURCE_PATH'):resources.append(env['GZ_SIM_RESOURCE_PATH'])
        env['GZ_SIM_RESOURCE_PATH']=os.pathsep.join(resources)
        keys=('GZ_PARTITION','ROS_DOMAIN_ID','GZ_IP','ROS_AUTOMATIC_DISCOVERY_RANGE','URRC_PRACTICE_SESSION','GZ_SIM_RESOURCE_PATH')
        session=dict(spec,owner_pid=os.getpid(),owner_start=process_start(os.getpid()),token=env['URRC_PRACTICE_SESSION'],
                     connection={k:env[k] for k in keys},closing=False)
        persist(directory,session);session['env']=env;world=None
        try:
            command=['gz','sim','-r']
            if args.server:command+=['-s','--headless-rendering']
            elif spec['gui']:command+=['--gui-config',spec['gui']]
            command.append(spec['map_file'])
            world=subprocess.Popen(command,env=env,start_new_session=True)
            wait_world(session)
            if session['has_lights']:
                for i in range(1,6):set_light(session,i,False)
            print('[PRACTICE 1] '+spec['world']+' 경기장 준비. 2번 차량/센서 → 3번 자율주행을 실행하세요.',flush=True)
            print('[PRACTICE 1] Ctrl+C로 이 세션의 경기장·차량·팀 실행을 종료합니다.',flush=True)
            if session['has_lights']:
                while input('[PRACTICE 1] 2·3번 준비 후 READY 입력: ').strip().upper()!='READY':pass
                print('[PRACTICE 1] 실제 카메라·인식·제어 준비 확인 중',flush=True)
                wait_camera(session)
                for i in range(1,6):set_light(session,i,True);print('[PRACTICE 1] RED '+str(i)+'/5',flush=True);time.sleep(1.)
                time.sleep(1.)
                for i in range(1,6):set_light(session,i,False)
                print('[PRACTICE 1] 빨간등 전체 소등. 차량 카메라가 인식한 뒤 출발합니다.',flush=True)
            else:print('[PRACTICE 1] 이 외부 맵에는 기본 5등이 없습니다. 맵 제공자의 신호가 필요합니다.',flush=True)
            return world.wait()
        finally:
            session['closing']=True;persist(directory,session)
            for role in (2,3):
                p=directory/(str(role)+'.json')
                try:
                    child=json.loads(p.read_text())
                    if child.get('token')==session['token'] and alive(child['pid'],child['start']):os.kill(child['pid'],signal.SIGINT)
                except (OSError,ValueError,ProcessLookupError):pass
            stop_owned(session['token']);stop_child(world)
            print('[PRACTICE 1] 이 연습 세션 종료',flush=True)

def car_exists(session):
    from google.protobuf.text_format import Parse
    from gz.msgs10.scene_pb2 import Scene
    result=run(['gz','service','-s','/world/'+session['world']+'/scene/info','--reqtype','gz.msgs.Empty',
                '--reptype','gz.msgs.Scene','--timeout','3000','--req',''],session['env'],5)
    if result.returncode:raise RuntimeError('차량 존재 여부를 확인하지 못했습니다.')
    scene=Parse(result.stdout,Scene())
    return any(m.name=='team_01' for m in scene.model)

def connected_role(role,args,directory):
    with role_lock(directory,role):
        session=load_session(directory);env=session['env'];env['URRC_PRACTICE_ROLE']=str(role)
        for argument in args.launch_args:
            if argument.startswith('gz_partition:=') and argument.split(':=',1)[1]!=env['GZ_PARTITION']:
                raise RuntimeError('숫자 1/2/3은 연결을 자동 공유합니다. 외부 맵 연결은 sensors/autonomy 모드를 사용하세요.')
            if argument.startswith('world_name:=') and argument.split(':=',1)[1]!=session['world']:
                raise RuntimeError('1번과 다른 월드로 연결할 수 없습니다.')
        os.environ.update(session['connection']);wait_world(session)
        # ROS launch wrappers belong only to this practice token, never an
        # operator competition. Store identity to guard against PID reuse.
        write_json(directory/(str(role)+'.json'),dict(pid=os.getpid(),start=process_start(os.getpid()),token=session['token']))
        child=None
        try:
            if role==2:
                if car_exists(session):print('[PRACTICE 2] 기존 team_01 차량 재사용',flush=True)
                else:
                    from ament_index_python.packages import get_package_share_directory
                    share=Path(get_package_share_directory('team_01_race'));slot=session['slot']
                    print('[PRACTICE 2] team_01 차량 생성',flush=True)
                    subprocess.run(['ros2','run','ros_gz_sim','create','-world',session['world'],'-file',str(share/'models/team_01_car/model.sdf'),
                        '-name','team_01','-x',str(slot['x']),'-y',str(slot['y']),'-z',str(float(slot['road_z'])+.04),'-Y',str(slot['yaw'])],env=env,check=True,timeout=45)
                    deadline=time.monotonic()+10
                    while not car_exists(session):
                        if time.monotonic()>deadline:raise RuntimeError('차량 생성 확인 시간 초과')
                        time.sleep(.1)
                launch='sensors.launch.py';extra=['world_name:='+session['world'],'gz_partition:='+env['GZ_PARTITION']]
            else:
                launch='race.launch.py';extra=['bridge_sensors:=false','require_official_start:=false','gz_partition:='+env['GZ_PARTITION']]
            request(session,'control','gz.msgs.WorldControl','pause: false')
            print('[PRACTICE '+str(role)+'] '+('차량·센서·TF 연결' if role==2 else '자율주행 연결')+' / '+session['world'],flush=True)
            child=subprocess.Popen(['ros2','launch','team_01_race',launch,*extra,*args.launch_args],env=env,start_new_session=True)
            return child.wait()
        finally:
            pause_on_exit(directory,session);stop_child(child);stop_owned(session['token'],role)

def watch_role(role,args,directory):
    topics={4:'corner_control',5:'path_manager',6:'control_status',7:'speed_control',8:'depth_corner',9:'drive_stability'}
    env=os.environ.copy()
    try:
        session=load_session(directory);env.update(session['connection']);env['URRC_PRACTICE_ROLE']=str(role)
    except (OSError,ValueError,RuntimeError):pass
    os.execvpe('ros2',['ros2','topic','echo','/team_01/debug/'+topics[role],'--field','data',*args.launch_args],env)


def main(argv=None):
    parser=argparse.ArgumentParser(description='team_01 three-terminal local practice; no competition spawning')
    parser.add_argument('role',type=int,choices=tuple(range(1,10)));parser.add_argument('--team-root',required=True,type=Path)
    parser.add_argument('--track',choices=(*TRACKS,'random'),default='monza');parser.add_argument('--map-file',type=Path)
    parser.add_argument('--grid-file',type=Path);parser.add_argument('--spawn',nargs=4,type=float,metavar=('X','Y','ROAD_Z','YAW'))
    parser.add_argument('--server',action='store_true');args,extra=parser.parse_known_args(argv)
    if args.role==1 and extra:
        if len(extra)==1 and extra[0] in (*TRACKS,'random'):args.track=extra[0]
        else:parser.error('알 수 없는 경기장 인자: '+' '.join(extra))
    args.launch_args=extra
    team=args.team_root.resolve();directory=state_dir(team)
    def interrupted(*_):raise KeyboardInterrupt
    signal.signal(signal.SIGINT,interrupted);signal.signal(signal.SIGTERM,interrupted)
    try:
        if args.role>=4:return watch_role(args.role,args,directory)
        return map_role(args,team,directory) if args.role==1 else connected_role(args.role,args,directory)
    except KeyboardInterrupt:return 130
    except (OSError,ValueError,RuntimeError,subprocess.SubprocessError,EOFError) as error:
        print('[PRACTICE ERROR] '+str(error),file=sys.stderr,flush=True);return 2

if __name__=='__main__':sys.exit(main())
