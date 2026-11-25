#!/usr/bin/env python3
"""Align recorded outputs, pose-derived velocity and joints for evaluation only."""
import argparse,bisect,csv,gzip,json,math
from pathlib import Path
import numpy as np
from speed_metrics import read,quantiles,external_clock,calibrate

p=argparse.ArgumentParser();p.add_argument('--run',required=True);a=p.parse_args();out=Path(a.run)
nav=read(out/'audit/navigation.jsonl');clock,_=external_clock(out);calibrate(nav,clock)
plans=[r for r in nav if r.get('plan')]
outputs=[r for r in nav if r.get('event')=='output']
ad=sorted({r['sim']:r for r in read(out/'audit/adapter.jsonl') if r.get('event')=='output'}.values(),key=lambda r:r['sim'])
calibrate(ad,clock)
poses=read(out/'poses.jsonl');joints=[r for r in read(out/'contacts.jsonl') if 'joint_angles' in r]
def series(rows):return np.array([r['sim'] for r in rows])
pt=series(poses);xyz=np.array([r['xyz'] for r in poses]);distance=np.r_[0,np.cumsum(np.linalg.norm(np.diff(xyz,axis=0),axis=1))]
times=np.arange(pt[0]+.2,pt[-1],.05)
actual=(np.interp(times,pt,distance)-np.interp(times-.2,pt,distance))/.2
acceleration=np.gradient(actual,.05)
at=series(ad);requested=np.array([r['speed'] for r in ad])
jt=series(joints);pn=series(plans);ot=series(outputs)
def latest(rows,times,tm):
    i=bisect.bisect_right(times,tm)-1
    return rows[i] if i>=0 else None
trace=[]
for tm,v,accel in zip(times,actual,acceleration):
    r=latest(plans,pn,tm);d=latest(ad,at,tm);j=latest(joints,jt,tm)
    output=latest(outputs,ot,tm)
    if not r or not d:continue
    q=r['plan'];angles=j['joint_angles'] if j else None
    measured=None
    if angles:
        kl=math.tan(angles[0])/(.340+.084*math.tan(angles[0]))
        kr=math.tan(angles[1])/(.340-.084*math.tan(angles[1]))
        measured=math.atan(.340*(kl+kr)/2)
    row=dict(sim=float(tm),actual_mps=float(v),actual_accel_mps2=float(accel),navigation_desired=r['navigation_desired'],
        navigation_output=None if output is None else output['speed'],
        navigation_output_sim=None if output is None else output['sim'],
        adapter_mps=d['speed'],adapter_steering_contract=d['steering'],actual_steering_left=measured,
        adapter_requested_mps=d['requested'],adapter_output_sim=d['sim'],
        adapter_output_reason=d.get('output_reason'),
        raw_left=q['raw_steering'],filtered_left=q['filtered_steering'],target=q['target'],final=q['final'],
        unconstrained=q['unconstrained'],state=q['state'],source=q['source'],
        preview=q['preview'],free_distance=q['free_distance'],curvature=q['curvature'],
        clearance=q['clearance'],footprint_clearance=q['footprint_clearance'],
        lookahead=q.get('lookahead'),target_distance=math.hypot(*q['target_xy']),
        body_reason=q.get('body_reason'),extension_curvature=q.get('extension_curvature'),
        tangent_distance=q.get('extension_diagnostics',{}).get('tangent_distance'),
        body_checks=q.get('extension_diagnostics',{}).get('body_checks'),
        scan_stamp=r['scan_stamp'],plan_done_sim=r['sim'],
        external_sensor_age_done_sim=r['sim']-r['scan_stamp'],
        sensor_age_done_sim=r['sensor_age_done_sim'],processing_wall=r['processing_wall_s'],
        plan_age_sim=float(tm-r['sim']),adapter_command_age_wall=d['command_age_wall'])
    row.update({'cap_'+k:v for k,v in q['candidates'].items()});row['cap_RACE']=q['race_cap']
    trace.append(row)
with gzip.open(out/'speed_timeline.csv.gz','wt') as f:
    w=csv.DictWriter(f,fieldnames=list(trace[0]));w.writeheader();w.writerows(trace)
lag_stats=[]
for lag in np.arange(0,.401,.01):
    # actual is a trailing .2s mean, so compare with equally smoothed commands.
    cmd=np.interp(times-lag,at,requested);cmd_old=np.interp(times-.2-lag,at,requested)
    mean=(cmd+cmd_old)/2
    mask=(times>=pn[0])&(times<=pn[-1])
    lag_stats.append((float(np.mean((mean[mask]-actual[mask])**2)),float(lag)))
best=min(lag_stats)
chain=[];om=[r['mono'] for r in outputs];pm=[r['mono'] for r in plans]
for d in ad:
    if not d['started'] or d.get('output_reason','COMMAND') not in ('COMMAND','SPEED_CLAMP'):continue
    age=d.get('command_age_wall')
    if age is None:continue
    receipt=d['mono']-age
    oi=bisect.bisect_right(om,receipt+.001)-1
    if oi<0:continue
    o=outputs[oi]
    if np.float32(o['speed'])!=np.float32(d['requested']):continue
    pi=bisect.bisect_right(pm,o['mono'])-1
    if pi<0:continue
    r=plans[pi]
    if o['mono']-r['mono']>.25:continue
    chain.append(dict(sensor_to_adapter_output_sim_s=d['sim']-r['scan_stamp'],
        sensor_to_navigation_output_sim_s=o['sim']-r['scan_stamp'],
        navigation_to_adapter_receipt_wall_s=max(0.,receipt-o['mono']),
        adapter_receipt_to_output_wall_s=age))
cmd=np.interp(times,at,requested);derivative=np.gradient(cmd,.05)
# Only sustained near-full commanded braking can test the planner's braking
# assumption. Gentle voluntary deceleration is not a capability measurement.
active=derivative< -2.6;episodes=[];start=None
for i,on in enumerate(active):
    if on and start is None:start=i
    if start is not None and (not on or i==len(active)-1):
        stop=i
        if times[stop]-times[start]>=.45 and actual[start]>.8:
            lags=best[1];mask=(times>=times[start]+lags+.15)&(times<=times[stop]+lags)
            if sum(mask)>=4:
                slope=float(np.polyfit(times[mask]-times[mask][0],actual[mask],1)[0])
                episodes.append(dict(start_sim=float(times[start]),end_sim=float(times[stop]),
                    initial_actual_mps=float(actual[start]),commanded_drop=float(cmd[start]-cmd[stop]),
                    measured_decel_mps2=-slope))
        start=None
result=dict(clock='simulation seconds',actual_velocity_window_s=.2,timeline_step_sim_s=.05,
    best_command_velocity_lag_sim_s=best[1],lag_fit_rmse_mps=math.sqrt(best[0]),
    warning='Lag is an offline fit including smoothing; not a worst-case latency guarantee. Braking fits include road grade.',
    braking_episodes=episodes,deceleration_mps2=quantiles([e['measured_decel_mps2'] for e in episodes]))
result['command_chain_latency']=dict(matched_samples=len(chain),
    sensor_to_adapter_output_sim_s=quantiles([r['sensor_to_adapter_output_sim_s'] for r in chain]),
    sensor_to_navigation_output_sim_s=quantiles([r['sensor_to_navigation_output_sim_s'] for r in chain]),
    navigation_to_adapter_receipt_wall_s=quantiles([r['navigation_to_adapter_receipt_wall_s'] for r in chain]),
    adapter_receipt_to_output_wall_s=quantiles([r['adapter_receipt_to_output_wall_s'] for r in chain]),
    method='Latest publication before receipt reconstructed from adapter monotonic age; float32 speed matched. No sequence IDs: observed association, not a hard latency bound.')
false_stops=[];gate_closures=[]
last_started=max((i for i,r in enumerate(ad) if r['started']),default=-1)
for ai,(prev,r) in enumerate(zip(ad,ad[1:]),1):
    if prev['started'] and not r['started']:
        tm=r['sim'];v0=float((np.interp(tm,pt,distance)-np.interp(tm-.1,pt,distance))/.1)
        # A navigation gate may already have sent zero while the car is
        # still braking. Eligibility follows actual velocity, not last command.
        if v0<=.5:continue
        indices=np.flatnonzero((times>=tm)&(times<=tm+6)&(actual<.05))
        settled=next((int(i) for i in indices if i+5<len(actual) and max(actual[i:i+6])<.05),None)
        if settled is not None:
            stop=float(times[settled]);ds=float(np.interp(stop,pt,distance)-np.interp(tm,pt,distance))
            measurement=dict(adapter_zero_sim=tm,initial_actual_mps=v0,settled_sim=stop,
                stopping_sim_s=stop-tm,stopping_distance_m=ds,
                terminal_gate_close=ai>last_started,
                effective_decel_including_response_mps2=v0*v0/(2*ds) if ds>0 else None)
            gate_closures.append(measurement)
            if ai>last_started:false_stops.append(measurement)
result['false_stop_measurements']=false_stops
result['gate_close_measurements_including_temporary']=gate_closures
(out/'speed_response.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
