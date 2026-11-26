"""Read-only native Gazebo pose receiver and token-protected LAN WebSocket."""
import asyncio,csv,json,math,secrets,socket,threading,time,re,os
from pathlib import Path
from collections import deque
import numpy as np
import xml.etree.ElementTree as ET
from aiohttp import web, WSMsgType
from signal_state import parse_signal
from gz.transport13 import Node
from gz.msgs10.pose_v_pb2 import Pose_V
ROOT=Path(__file__).resolve().parents[2]; WEB=Path(__file__).resolve().parent/'web'
# Persistent access key: keep permissions private and never expose via static routes.
keyfile=Path(__file__).resolve().parent/'.access_key'
try:
    fd=os.open(keyfile,os.O_WRONLY|os.O_CREAT|os.O_EXCL,0o600)
    with os.fdopen(fd,'w') as f:f.write(secrets.token_urlsafe(24))
except FileExistsError:pass
TOKEN=keyfile.read_text().strip()
if len(TOKEN)<24:raise RuntimeError('Invalid spectator access key file')
lock=threading.Lock(); cars={}; history={}; pattern=re.compile(r'team_(?:0[1-9]|1[0-2])$')
track=os.environ['URRC_SPECTATOR_TRACK']
if track not in ('monza','spa','monaco','cota','hungaroring','baku'):raise ValueError('Unsupported official track')
with (ROOT/'src/urrc_track_gazebo/tracks/processed'/f'{track}_centerline.csv').open() as f:
    route=np.array([[float(r[k]) for k in ('x_m','y_m','z_m')] for r in csv.DictReader(f)])
if np.linalg.norm(route[0]-route[-1])<.001: route=route[:-1]
ends=np.roll(route,-1,axis=0); delta=ends-route; lengths=np.linalg.norm(delta[:,:2],axis=1); cumulative=np.r_[0,np.cumsum(lengths[:-1])]; lap=float(lengths.sum())
STAGES={"qualifying":2,"semi_a":3,"semi_b":3,"final":5}
stage=os.environ["URRC_SPECTATOR_STAGE"]
if stage not in STAGES:raise ValueError("Invalid race stage")
lap_count=STAGES[stage]
base_offsets={}
for team_dir in (ROOT/"src/team").glob("team_*"):
    for model_file in team_dir.rglob("model.sdf"):
        try:
            model=ET.parse(model_file).getroot().find("model")
            link=model.find("link[@name='base_link']")
            if link is None:continue
            pose=link.find("pose")
            if pose is not None and pose.get("relative_to") not in (None,"__model__"):continue
            base_offsets[team_dir.name]=list(map(float,(link.findtext("pose") or "0 0 0 0 0 0").split()))[:3];break
        except (OSError,ValueError,ET.ParseError,AttributeError):continue
rank_state={}; pending={}; arrows={}; previous_set=set()
signal={"phase":"waiting","count":0,"started_ms":None,"sim_start":None}
best_laps={}
start_roster=set(); crossed_start=set()
start_cleared=False
retired_cars={}  # Last authoritative pose and track progress, frozen on DNF.
fastest={"team":None,"time":None}
def refresh_signal(now):
    # The race process writes telemetry AFTER each confirmed red lamp change.
    # Partition matching prevents a stale or unrelated race being displayed.
    state_file = ROOT/'race/results/spectator_signal_state.json'
    try:
        info=json.loads(state_file.read_text(encoding='utf-8'))
        if info.get('track') != track or info.get('partition') != os.environ.get('GZ_PARTITION'):
            return
        if not isinstance(info.get('timestamp'),(int,float)) or time.time()-info['timestamp']>3600:
            return
        phase=info.get('phase')
        count=info.get('count')
        if phase not in ('waiting','countdown','started','error') or not isinstance(count,int) or not 0<=count<=5:
            return
    except (OSError,ValueError,TypeError):
        return
    if phase=='started' and signal['phase']!='started':
        signal['started_ms']=now*1000
        signal['sim_start']=max((r['t'] for r in cars.values()),default=None)
        best_laps.clear();fastest.update(team=None,time=None)
        start_roster.clear();start_roster.update(n for n,r in cars.items() if now-r['seen']<2)
        crossed_start.clear()
        retired_cars.clear()
        for r in cars.values():
            r['lap_start']=None;r['lap_gate']=-1;r['last_lap']=None;r['laps']=0;r['finished']=False
    signal.update(phase=phase,count=count)
def base_position(p,q,offset):
    v=np.array(offset);u=np.array(q[:3]);w=q[3];return (np.array(p)+2*np.cross(u,np.cross(u,v)+w*v)).tolist()
def project_progress(p, previous_s=None, maximum_step=None):
    """Project to a nearby centerline segment, preserving longitudinal continuity.

    Monaco has adjacent/overlapping arms of track. Picking the closest segment
    globally for every frame can jump a car half a lap; the previous code then
    RESET its cumulative distance, causing false overtakes. Once acquired,
    limit the search to physically reachable along-track positions instead.
    """
    off=np.asarray(p[:2],dtype=float)-route[:,:2]
    d=delta[:,:2]
    frac=np.clip(np.sum(off*d,axis=1)/np.maximum(lengths**2,1e-10),0,1)
    projected=cumulative+frac*lengths
    squared=np.sum((off-d*frac[:,None])**2,axis=1)
    if previous_s is not None:
        # Circular track coordinates, including wrap at start/finish.
        circular=((projected-previous_s+lap/2)%lap)-lap/2
        reachable=np.abs(circular)<=maximum_step
        if not np.any(reachable):
            return None
        squared=np.where(reachable,squared,np.inf)
    i=int(np.argmin(squared))
    # Out-of-track measurements may be temporarily corrupt. Do not overwrite
    # a valid race history with a point from another part of the circuit.
    if previous_s is not None and squared[i]>25.0:
        return None
    return float(projected[i]%lap)

def progress(p):
    return project_progress(p)
start_finish=json.loads((ROOT/'src/urrc_track_gazebo/tracks/processed'/f'{track}.json').read_text())['start_finish']
start_s=progress([start_finish['x'],start_finish['y'],start_finish['z']])
def relative_progress(s):return (s-start_s)%lap
def receive(msg):
    stamp=msg.header.stamp.sec+msg.header.stamp.nsec*1e-9
    if stamp<=0:return
    now=time.monotonic()
    with lock:
        for pose in msg.pose:
            name=pose.name
            if not pattern.fullmatch(name):continue
            p=[pose.position.x,pose.position.y,pose.position.z];q=[pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w]
            if not all(math.isfinite(x) for x in p+q):continue
            if name in retired_cars:continue  # Never advance a retired car's track progress.
            old=cars.get(name)
            # The spectator process is isolated per race. Old/out-of-order
            # packets must never erase a vehicle's accumulated lap progress.
            if old and stamp<=old['t']:
                continue
            base=base_position(p,q,base_offsets[name]) if name in base_offsets else None
            if base is None:continue  # no guessed base_link measurement
            dt=stamp-old['t'] if old else 0
            if old:
                # Track projection stays near the last accepted longitudinal
                # position, rather than jumping across Monaco's close roads.
                step_cap=min(lap*.45,max(4.0,dt*35.0+2.0))
                projected=project_progress(base,(old['s']+start_s)%lap,step_cap)
                if projected is None:
                    continue
                s=relative_progress(projected)
                ds=(s-old['s']+lap/2)%lap-lap/2
                # Reject impossible progress *without resetting* total/laps.
                if abs(ds)>step_cap or math.dist(base,old['base'])>max(5.0,dt*50.0):
                    continue
            else:
                projected=project_progress(base)
                s=relative_progress(projected)
                ds=0.0
            speed=math.dist(base,old['base'])/dt if dt>0 else 0.0
            along=ds/dt if dt>0 else 0.0
            total=old['total']+ds if old else (s+lap/2)%lap-lap/2
            start=old['start'] if old else None
            origin=0.0
            if start is None and speed>.2:start=stamp
            peak=max(old['peak'] if old else 0,speed)
            laps=max(old['laps'] if old else 0,max(0,int(total//lap)))
            lap_start=old['lap_start'] if old else None
            last_lap=old['last_lap'] if old else None
            lap_gate=old.get('lap_gate',-1) if old else -1
            if old and ds>0 and math.floor(total/lap)>math.floor(old['total']/lap):
                gate=math.floor(total/lap)*lap
                if gate>=0 and signal["phase"]=="started" and not (old and old["finished"]):
                    crossed_start.add(name)
                    cross=old['t']+dt*(gate-old['total'])/ds
                    if old.get('lap_start') is not None and gate>old.get('lap_gate',-1):
                        last_lap=max(0.0,cross-old['lap_start'])
                        best=best_laps.get(name)
                        if best is None or last_lap<best:
                            best_laps[name]=last_lap
                        if fastest['time'] is None or last_lap<fastest['time']:
                            fastest.update(team=name,time=last_lap)
                    if gate>lap_gate:lap_start=cross;lap_gate=gate
            finished=laps>=lap_count
            hist=history.setdefault(name,deque(maxlen=36000));hist.append((total,stamp))
            cars[name]={'team':name,'p':p,'base':base,'along':along,'q':q,'t':stamp,'seen':now,'s':s,'total':total,'origin':origin,'lap_start':lap_start,'lap_gate':lap_gate,'last_lap':last_lap,'lap_time':stamp-lap_start if lap_start is not None else None,'speed':speed,'peak':peak,'start':start,'laps':laps,'finished':finished}
def ttc(distance,closing):
    return distance/closing if closing>.05 else None

def gap(leader,follower):
    """Historical crossing time; estimate from measured pace when no crossing exists.

    A distant/initial-grid follower may be behind the first leader sample.
    This must not be displayed as an exact crossing-based interval.
    """
    target=follower['total']; h=history.get(leader['team'])
    if h and h[0][0]<=target<=h[-1][0]:
        prev=None
        for b,tb in reversed(h):
            if prev is not None:
                a,ta=prev
                if b<=target<=a and a>b:
                    return max(0.,follower['t']-(tb+(target-b)/(a-b)*(ta-tb))),False
            prev=(b,tb)
    distance=max(0.,leader['total']-follower['total'])
    if distance<.01:return 0.,True
    # Estimate from the leader's actual recent pace, not instantaneous
    # car speed; prefer recent 15s track displacement to 1-frame noise.
    pace=None
    if h and len(h)>1:
        end_s,end_t=h[-1]
        for old_s,old_t in reversed(h):
            if end_t-old_t>=15.:
                pace=(end_s-old_s)/(end_t-old_t)
                break
        if pace is None:
            start_s,start_t=h[0]
            if end_t>start_t+2:
                pace=(end_s-start_s)/(end_t-start_t)
    if pace is None or pace<=.25:
        # DNF leader has no moving pace: use the pursuing car's pace
        # only as an estimate, never as an exact historical time.
        pace=max(.0,follower.get('along',0.))
    if pace<=.25:return None,True
    return distance/pace,True

def update_operator_status():
    """The local operator owns DNF decisions. The spectator only observes."""
    path=ROOT/'race/results/operator_v26_active.json'
    try:
        info=json.loads(path.read_text(encoding='utf-8'))
        if info.get('track')!=track or info.get('stage')!=stage:
            return
        # The session is active only while the owning operator process exists.
        if not info.get('pid'):
            return
        os.kill(int(info['pid']),0)
        states=info.get('status',{})
        for name,status in states.items():
            if status=='DNF' and name not in retired_cars and name in cars:
                # Copy BEFORE the operator removes the physical Gazebo entity.
                state=dict(cars[name]);state['speed']=0.;state['along']=0.
                state['retired']=True;state['status']='DNF'
                retired_cars[name]=state
            elif status=='FINISHED' and name in cars:
                cars[name]['finished']=True
    except (OSError,ValueError,TypeError,KeyError):
        return

def snapshot(include_hud=True):
    global previous_set, start_cleared
    with lock:
        now=time.monotonic();
        if include_hud:
            refresh_signal(now)
            update_operator_status()
        rows=[dict(x) for name,x in cars.items() if name not in retired_cars and now-x['seen']<2]
        rows.extend(dict(x) for x in retired_cars.values())
        # Absolute unwrapped progress establishes order: completed full laps
        # outrank any car still on an earlier lap. Never rank by modulo s.
        rows.sort(key=lambda x:-x['total'])
        names={r['team'] for r in rows}
        if signal['phase']=='started' and not start_cleared:
            start_roster.update(names)
            start_cleared=bool(start_roster) and start_roster.issubset(crossed_start)
        if names!=previous_set:
            rank_state.clear();pending.clear();arrows.clear();previous_set=names
        if include_hud:
            for i,r in enumerate(rows):
                name=r['team']; rank=i+1; old=rank_state.setdefault(name,rank)
                if rank==old:pending.pop(name,None)
                else:
                    candidate=pending.get(name)
                    if not candidate or candidate[0]!=rank:pending[name]=(rank,now)
                    elif now-candidate[1]>=.5:
                        arrows[name]=(1 if rank<old else -1,now+3);rank_state[name]=rank;pending.pop(name,None)
        for i,r in enumerate(rows):
            r['rank']=i+1;r['ahead']=rows[i-1]['team'] if i else None;r['behind']=rows[i+1]['team'] if i+1<len(rows) else None
            r['status']='DNF' if r.get('retired') else ('FINISHED' if r['finished'] else 'RUNNING')
            r['arrow']=arrows.get(r['team'],(0,0))[0] if arrows.get(r['team'],(0,0))[1]>now else 0
            r['ahead_gap']=None;r['behind_gap']=None;r['ahead_gap_estimated']=False;r['behind_gap_estimated']=False
            if include_hud and i and not r.get('retired'):
                r['ahead_gap'],r['ahead_gap_estimated']=gap(rows[i-1],r)
            if include_hud and i+1<len(rows) and not r.get('retired'):
                r['behind_gap'],r['behind_gap_estimated']=gap(r,rows[i+1])
            # HUD neighbour times follow current race POSITION, not the
            # cyclic geometry of the circuit. Leader has no car ahead,
            # last-place has no car behind. DNF has no TTC prediction.
            for side,other in [('front',rows[i-1] if i else None),
                               ('rear',rows[i+1] if i+1<len(rows) else None)]:
                if other is None or r.get('retired') or r['finished']:
                    r[side]=None
                else:
                    diff=(other['total']-r['total']) if side=='front' else (r['total']-other['total'])
                    closing=(r['along']-other['along']) if side=='front' else (other['along']-r['along'])
                    dist=max(0.,diff)
                    r[side]={'team':other['team'],'distance':dist,
                             'closing':closing,'ttc':ttc(dist,closing)}
            r['sample_ms']=r.pop('seen')*1000;r['elapsed']=max(0,r['t']-signal['sim_start']) if signal['sim_start'] is not None else None
            r['best_lap']=best_laps.get(r['team'])
            r['is_fastest']=fastest['team']==r['team'] and fastest['time'] is not None
            if signal['phase']!='started':
                r['lap_time']=None
                r['last_lap']=None
            elif r['laps']==0 and signal['sim_start'] is not None:
                r['lap_time']=r['elapsed']
        return {'signal':dict(signal, start_cleared=start_cleared),'teams':rows,'lap_length':lap,'stage':stage,'track':track,'provisional':True,'lap_count':lap_count,'running_count':sum(not r['finished'] and not r.get('retired') for r in rows),
                'fastest_lap':{'team':fastest['team'],'time':fastest['time']} if fastest['time'] is not None else None}
async def auth(request,handler):
    # Query key is needed for initial page and WebSocket; static assets contain no live data.
    if request.path in ('/','/ws') and not secrets.compare_digest(request.query.get('key',''),TOKEN):raise web.HTTPForbidden()
    return await handler(request)
@web.middleware
async def middleware(request,handler):return await auth(request,handler)
async def index(request):
    page=(WEB/'index.html').read_text(encoding='utf-8').replace('MONZA',track.upper())
    return web.Response(text=page,content_type='text/html',headers={'Referrer-Policy':'no-referrer','Cache-Control':'no-store'})
# One shared producer, at most one queued packet per viewer. Slow clients do
# not delay the producer or other viewers. HUD calculations run at 10 Hz.
clients=set(); latest_packet=None
async def producer(app):
    global latest_packet
    hud={}; next_hud=0.; deadline=time.monotonic(); previous=None
    while True:
        now=time.monotonic(); full=now>=next_hud
        if full:next_hud=now+.1
        data=snapshot(full)
        if full:hud={r['team']:{k:r[k] for k in ('ahead_gap','behind_gap','ahead_gap_estimated','behind_gap_estimated')} for r in data['teams']}
        else:
            for r in data['teams']:r.update(hud.get(r['team'],{}))
        signature=tuple((r['team'],r['t'],r['rank']) for r in data['teams'])
        if signature!=previous or full:
            previous=signature;data['sent_ms']=time.monotonic()*1000
            latest_packet=json.dumps(data,separators=(',',':'),allow_nan=False)
            for queue in tuple(clients):
                if queue.full():queue.get_nowait()
                queue.put_nowait(latest_packet)
        deadline+=1/60
        if deadline<time.monotonic():deadline=time.monotonic()
        await asyncio.sleep(max(0,deadline-time.monotonic()))
async def lifecycle(app):
    task=asyncio.create_task(producer(app))
    yield
    task.cancel()
    await asyncio.gather(task,return_exceptions=True)
async def ws(request):
    sock=web.WebSocketResponse(heartbeat=15,max_msg_size=256);await sock.prepare(request)
    queue=asyncio.Queue(maxsize=1);clients.add(queue)
    if latest_packet is not None:queue.put_nowait(latest_packet)
    async def send():
        while not sock.closed:
            await asyncio.wait_for(sock.send_str(await queue.get()),timeout=1)
    task=asyncio.create_task(send())
    try:
        async for message in sock:
            # Viewer is read-only; no control messages accepted.
            if message.type in (WSMsgType.TEXT,WSMsgType.BINARY):break
            if task.done():break
    finally:
        clients.discard(queue);task.cancel();await asyncio.gather(task,return_exceptions=True);await sock.close()
    return sock
async def manifest(request):return web.FileResponse(WEB/'manifest.json')
node=Node()
if not node.subscribe(Pose_V,f'/world/{track}/dynamic_pose/info',receive):raise RuntimeError('Gazebo pose subscription failed')
app=web.Application(middlewares=[middleware]);app.cleanup_ctx.append(lifecycle);app.router.add_get('/',index);app.router.add_get('/ws',ws);app.router.add_get('/manifest.json',manifest);app.router.add_get('/track.json',lambda r:web.json_response(route[::max(1,len(route)//1500)].tolist()));app.router.add_static('/assets',WEB/'assets');app.router.add_static('/three',WEB/'node_modules/three');app.router.add_get('/app.js',lambda r:web.FileResponse(WEB/'app.js'))
def lan_ip():
    try:
        with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as s:
            s.connect(('192.0.2.1',9));return s.getsockname()[0]
    except OSError:return '127.0.0.1'
print(f'공식 대회 관전: {stage}/{track} / 읽기 전용',flush=True)
print('관전 주소: http://'+lan_ip()+':8765/?key='+TOKEN,flush=True)
print('고정 주소 (mDNS 설정 시): http://urrc2.local:8765/?key='+TOKEN,flush=True)
print('운영 PC LAN IP 확인: hostname -I (위 주소가 127.x이면 실제 LAN IP로 변경)',flush=True)
web.run_app(app,host='0.0.0.0',port=8765,access_log=None)
