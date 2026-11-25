#!/usr/bin/env python3
"""Read-only sim-time speed/limiter analysis. Ground truth is evaluation only."""
import argparse
import bisect
from collections import defaultdict
import json
import gzip
import math
from pathlib import Path
import numpy as np


def read(path):
    if not path.exists() and Path(str(path)+'.gz').exists():
        path=Path(str(path)+'.gz')
    if not path.exists():
        return []
    rows = []
    with (gzip.open(path,'rt') if path.suffix=='.gz' else path.open()) as f:
        for line in f:
            try:
                rows.append(json.loads(line))
            except ValueError:
                pass  # A live writer may not yet have finished its final line.
    return rows


def quantiles(values):
    v = [x for x in values if x is not None and math.isfinite(x)]
    return dict(zip(('p50','p95','max'), map(float,np.quantile(v,[.5,.95,1])))) if v else None


def external_clock(folder, poses=None):
    for name,label in [('clock.jsonl','independent ROS /clock receipt'),
                       ('clock_partial.jsonl','partial independent ROS /clock receipt')]:
        rows=read(folder/name)
        if len(rows)>1:return rows,label
    return (poses if poses is not None else read(folder/'poses.jsonl')),'validation pose timestamp/receipt mapping (transport skew included)'


def calibrate(rows, clock):
    """Keep local ROS clock values, align event completion to independent clock.

    A busy single executor cannot process /clock during planning. Its local
    clock must not move plan completions earlier in the applied-time ledger.
    """
    if len(clock)<2:return
    wall=np.array([r['wall'] for r in clock]);sim=np.array([r['sim'] for r in clock])
    for r in rows:
        if 'sim' in r and 'wall' in r and wall[0]<=r['wall']<=wall[-1]:
            r['sim_local']=r['sim'];r['sim']=float(np.interp(r['wall'],wall,sim))


def analyze(folder):
    plans = read(folder/'audit/navigation.jsonl')
    all_poses=read(folder/'poses.jsonl')
    external,clock_source=external_clock(folder,all_poses)
    calibrate(plans,external)
    outputs = [r for r in plans if r.get('event') == 'output']
    completed = [r for r in plans if r.get('event') == 'plan']
    chain = read(folder/'chain.jsonl')
    if not chain:
        chain = read(folder.parents[1]/'chain.jsonl')
    release = next((json.loads(r['values']['/team_10/start_status_10'])['stamp'] for r in chain
        if r.get('values',{}).get('/team_10/camera_gate_10') and '/team_10/start_status_10' in r['values']),None)
    if release is None:
        release = next((r['sim'] for r in completed if r.get('started')),None)
    if release is None:
        raise ValueError('No optical release in evidence')
    poses = all_poses
    crossings = read(folder/'crossings.jsonl')
    lines = [r['time'] for r in crossings if r['gate']==0]
    finish = lines[1] if len(lines)>1 else None
    stopfile = folder/'deliberate_stop.json'
    stop = json.loads(stopfile.read_text()) if stopfile.exists() else {}
    adapter_rows=[r for r in read(folder/'audit/adapter.jsonl') if r.get('event')=='output']
    calibrate(adapter_rows,external)
    def adapter_reason(row):
        reason=row.get('output_reason')
        if reason=='SPEED_CLAMP' and abs(row['requested'])-abs(row.get('limited_request_mps',row['speed']))<=1e-5:
            return 'FLOAT32_BOUNDARY'
        return reason or ('STOP_GATE' if not row['started'] else
            ('OUTPUT_REDUCTION_UNCLASSIFIED' if abs(row['requested'])>abs(row['speed'])+.01 else 'COMMAND'))
    # Temporary gate closures are part of the trial, never an excuse to trim
    # slow time. End only at finish or the supervisor's deliberate final stop.
    last_started=max((i for i,r in enumerate(adapter_rows) if r['started']),default=-1)
    gate_close=next((r['sim'] for i,r in enumerate(adapter_rows) if i>last_started and not r['started']
        and r['wall']>=stop.get('wall',math.inf) and r['sim']>release),None)
    end = finish or gate_close or stop.get('sim') or stop.get('last_state',{}).get('sim') or poses[-1]['sim']
    poses = sorted({r['sim']:r for r in poses if release <= r['sim'] <= end}.values(),key=lambda r:r['sim'])
    t = np.array([r['sim'] for r in poses]);xyz=np.array([r['xyz'] for r in poses]);yaw=np.array([r['yaw'] for r in poses])
    dt = np.diff(t);delta=np.diff(xyz,axis=0);ds=np.linalg.norm(delta,axis=1);speed=ds/dt
    distance=np.r_[0,np.cumsum(ds)];duration=float(t[-1]-t[0]);above=speed>=10.
    continuous=longest=total=0.
    for d,ok in zip(dt,above):
        continuous=continuous+float(d) if ok else 0.
        longest=max(longest,continuous)
        if ok:total+=float(d)
    smooth=[];signed=[]
    for i in range(1,len(t)):
        j=int(np.searchsorted(t,t[i]-.2))
        if i>j and t[i]-t[j]>.1:
            smooth.append((t[i],float((distance[i]-distance[j])/(t[i]-t[j]))))
            d=xyz[i]-xyz[j]
            signed.append((t[i],float((d[0]*math.cos(yaw[i])+d[1]*math.sin(yaw[i]))/(t[i]-t[j]))))
    def episodes(series,predicate,min_time):
        result=[];start=None
        for tm,v in series:
            if predicate(v):
                if start is None:start=tm
            elif start is not None:
                if tm-start>=min_time:result.append([start,tm])
                start=None
        if start is not None and series[-1][0]-start>=min_time:result.append([start,series[-1][0]])
        return result
    smooth_total=smooth_longest=continuous=0.
    for (previous,_),(tm,v) in zip(smooth,smooth[1:]):
        continuous=continuous+tm-previous if v>=10. else 0.
        if v>=10.:smooth_total+=tm-previous
        smooth_longest=max(smooth_longest,continuous)
    # Rows are applied from completion to next completion in the SAME sim clock.
    completed=sorted({r['sim']:r for r in completed if r['sim']<=end}.values(),key=lambda r:r['sim'])
    pts=[r['sim'] for r in completed]
    limits=defaultdict(float);states=defaultdict(float);joint=defaultdict(float);lost=0.;state_only=0.
    def reasons(row):
        p=row.get('plan')
        if not p or not row.get('started') or not row.get('valid'):
            return [row.get('state','NO_PLAN')]
        caps=dict(p['candidates']);caps['RACE']=p['race_cap']
        minimum=min(caps.values());tol=1e-5
        return sorted(k for k,v in caps.items() if v<=minimum+tol)
    for i,r in enumerate(completed):
        a=max(release,r['sim']);b=min(end,completed[i+1]['sim'] if i+1<len(completed) else end)
        weight=max(0.,b-a)
        if not weight:continue
        why=reasons(r);joint['+'.join(why)]+=weight
        for k in why:limits[k]+=weight
        p=r.get('plan');states[p['state'] if p else r['state']]+=weight
        if p:
            others=[v for k,v in p['candidates'].items() if k!='STATE']+[p['race_cap']]
            if p['candidates']['STATE']<min(others)-1e-5:
                state_only+=weight
                lost+=(min(others)-p['target'])*weight
    under=defaultdict(float);under_joint=defaultdict(float)
    adapter_times=[r['sim'] for r in adapter_rows]
    for st,d,v in zip(t[1:],dt,speed):
        if v>=10:continue
        idx=bisect.bisect_right(pts,float(st))-1
        why=reasons(completed[idx]) if idx>=0 else ['NO_PLAN']
        if idx>=0 and completed[idx].get('plan'):
            p=completed[idx]['plan']
            if p['final']<p['target']-1e-4:why=why+['ACCEL_RATE']
            elif p['final']>p['target']+1e-4:why=why+['DECEL_RATE']
        under['+'.join(why)]+=float(d)
        ai=bisect.bisect_right(adapter_times,float(st))-1
        ar=adapter_reason(adapter_rows[ai]) if ai>=0 else 'NO_ADAPTER'
        both=why+(['ADAPTER_'+ar] if ar not in ('COMMAND','FLOAT32_BOUNDARY') else [])
        under_joint['+'.join(both)]+=float(d)
    valid=[r for r in completed if r.get('plan') and release<=r['sim']<=end]
    ext_wall=np.array([r['wall'] for r in external]);ext_sim=np.array([r['sim'] for r in external])
    external_age=[];external_queue=[];executor_lag=[]
    if len(external)>1:
        for r in valid:
            if ext_wall[0]<=r['wall']-r['processing_wall_s'] and r['wall']<=ext_wall[-1]:
                done=float(np.interp(r['wall'],ext_wall,ext_sim))
                receipt=float(np.interp(r['wall']-r['processing_wall_s'],ext_wall,ext_sim))
                external_age.append(done-r['scan_stamp']);external_queue.append(receipt-r['scan_stamp'])
                executor_lag.append(done-r.get('sim_local',r['sim']))
    output_stats=quantiles([r.get('plan_age_wall') for r in outputs if release<=r['sim']<=end])
    output_joint=defaultdict(float)
    ordered_outputs=sorted({r['sim']:r for r in outputs if r['sim']<=end}.values(),key=lambda r:r['sim'])
    plan_mono=[r['mono'] for r in completed]
    for i,r in enumerate(ordered_outputs):
        a=max(release,r['sim']);b=min(end,ordered_outputs[i+1]['sim'] if i+1<len(ordered_outputs) else end)
        if b<=a:continue
        k=bisect.bisect_right(plan_mono,r['mono'])-1
        why=(reasons(completed[k]) if k>=0 else ['NO_PLAN'])
        if not r['state'].startswith('DRIVE'):why=[r['state']]
        output_joint['+'.join(why)]+=b-a
    contacts=read(folder/'contacts.jsonl');joints=[r for r in contacts if 'joint_angles' in r and release<=r['sim']<=end]
    adapters=[r for r in adapter_rows if release<=r['sim']<=end]
    adapter_reasons=defaultdict(float)
    for i,r in enumerate(adapters):
        stop=min(end,adapters[i+1]['sim'] if i+1<len(adapters) else end)
        why=adapter_reason(r)
        adapter_reasons[why]+=max(0.,stop-r['sim'])
    # Plugin axes: negative competition steering gives positive joint angle.
    at=[r['sim'] for r in adapters];errors=[]
    for r in joints:
        i=bisect.bisect_right(at,r['sim'])-1
        if i>=0 and r['sim']-at[i]<.25:
            left,right=r['joint_angles']
            kl=math.tan(left)/(.340+.084*math.tan(left))
            kr=math.tan(right)/(.340-.084*math.tan(right))
            errors.append(math.atan(.340*(kl+kr)/2)+adapters[i]['steering'])
    res=dict(clock='simulation seconds',window='optical release through finish or deliberate final adapter gate close; includes acceleration, temporary safety stops and permission delivery wait',
        camera_release_sim=release,end_sim=end,observed_seconds=duration,distance_m=float(distance[-1]),
        mean_actual_mps=float(distance[-1]/duration),max_actual_sample_mps=float(max(speed)),
        max_actual_0_2s_mps=max(v for _,v in smooth),above_10_sim_s=total,above_10_ratio=total/duration,
        above_10_longest_sim_s=longest,threshold_definition='3D pose interval distance / sim interval >= 10.0, no tolerance',
        above_10_smoothed_sim_s=smooth_total,above_10_smoothed_ratio=smooth_total/duration,
        above_10_smoothed_longest_sim_s=smooth_longest,
        smoothed_threshold_definition='Trailing approximately0.2sim-second distance/time >=10.0, right-end sample weighted by sim interval; no tolerance',
        sample_gap_max_sim_s=float(max(dt)),first_line=lines[0] if lines else None,finish_line=finish,
        lap_seconds=finish-lines[0] if finish else None,gate_sequence=[r['gate'] for r in crossings],
        limiter_joint_sim_s=dict(joint),limiter_inclusive_sim_s=dict(limits),state_sim_s=dict(states),
        navigation_output_joint_sim_s=dict(output_joint),
        solely_state_limited_sim_s=state_only,counterfactual_target_gap_m=lost,
        below_10_reasons_sim_s=dict(under),processing_wall_s=quantiles([r['processing_wall_s'] for r in valid]),
        below_10_joint_navigation_adapter_sim_s=dict(under_joint),
        sensor_age_done_sim_s=quantiles([r['sensor_age_done_sim'] for r in valid]),plan_age_at_output_wall_s=output_stats,
        external_age_clock_source=clock_source,external_age_plan_samples=len(external_age),
        limiter_time_axis='event wall receipt/completion mapped to independent clock; local ROS sim values retained in raw audit',
        external_sensor_age_done_sim_s=quantiles(external_age),
        external_sensor_age_receipt_sim_s=quantiles(external_queue),executor_clock_lag_sim_s=quantiles(executor_lag),
        external_clock_max_receipt_gap_wall_s=float(max(np.diff(ext_wall))) if len(ext_wall)>1 else None,
        preview_m=quantiles([r['plan']['preview'] for r in valid]),
        steering_tracking_abs_error_rad=quantiles([abs(x) for x in errors]),
        steering_saturation_sim_s=sum(max(0,min(end,valid[i+1]['sim'] if i+1<len(valid) else end)-max(release,r['sim'])) for i,r in enumerate(valid) if abs(r['plan']['filtered_steering'])>=.95*.488692),
        stop_episodes_0_3s=episodes(smooth,lambda v:v<.05,.3),reverse_episodes_0_5s=episodes(signed,lambda v:v<-.2,.5),
        diagnostic_plan_count=len(valid),
        adapter_output_reason_sim_s=dict(adapter_reasons),
        adapter_output_reduction_samples=sum(abs(r['requested'])>abs(r['speed'])+.01 and r['started'] for r in adapters),
        adapter_speed_clamp_samples=sum(adapter_reason(r)=='SPEED_CLAMP' for r in adapters),
        audit_end=[r for r in plans if r.get('event')=='audit_end'])
    if finish:
        idx=int(np.searchsorted(t,lines[0]));res['lap_mean_actual_mps']=float((distance[-1]-distance[idx])/(finish-lines[0]))
    (folder/'speed_metrics.json').write_text(json.dumps(res,indent=2))
    return res


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--run',required=True);a=p.parse_args()
    print(json.dumps(analyze(Path(a.run)),indent=2))
