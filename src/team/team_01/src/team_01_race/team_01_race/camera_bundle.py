"""Atomic Camera geometry transport; public Path topics remain available."""
from dataclasses import dataclass
import json,math
import numpy as np
from std_msgs.msg import Header
from builtin_interfaces.msg import Time
from diagnostic_msgs.msg import DiagnosticArray,DiagnosticStatus,KeyValue

@dataclass
class ArrayPath:
    header: Header
    points: np.ndarray

def encode_bundle(result,header):
    return json.dumps(dict(frame_id='team_01/base_link',sec=header.stamp.sec,nanosec=header.stamp.nanosec,
        observed_lengths=[float(max(p[0] for p in result.get(key,[]))-min(p[0] for p in result.get(key,[]))) if result.get(key) else 0. for key in ('left_samples','right_samples')],path_source=result.get('path_source','CAMERA_PATH'),track_width=result.get('track_width'),track_width_age=result.get('track_width_age'),confidence=result['confidence'],observed=[bool(x) for x in result['observed']],traffic_light=result['light'],reason=result['reason'],
        **{name:result[key].tolist() if result[key] is not None else [] for name,key in [('center_path','path'),('left_boundary','left'),('right_boundary','right')]}),allow_nan=False,separators=(',',':'))

def decode_bundle(data,ros_now,timeout=.25):
    d=json.loads(data);sec=d['sec'];ns=d['nanosec'];conf=float(d['confidence'])
    if (d['frame_id']!='team_01/base_link' or type(sec) is not int or type(ns) is not int or
        not 0<=ns<1000000000 or not -.05<=ros_now-(sec+ns*1e-9)<=timeout or
        not math.isfinite(conf) or not 0<=conf<=1 or len(d['observed'])!=2 or not all(type(x) is bool for x in d['observed']) or
        d['traffic_light'] not in ('RED','GREEN','YELLOW','UNKNOWN','LIGHTS_OUT')):raise ValueError('invalid_camera_bundle')
    header=Header(frame_id='team_01/base_link',stamp=Time(sec=sec,nanosec=ns))
    result={}
    for name in ('center_path','left_boundary','right_boundary'):
        points=np.asarray(d[name],dtype=float)
        if points.size==0:points=np.empty((0,2))
        if points.ndim!=2 or points.shape[1]!=2:raise ValueError('invalid_camera_array')
        result[name]=ArrayPath(header,points)
    values=dict(confidence=str(conf),left_observed=str(int(d['observed'][0])),right_observed=str(int(d['observed'][1])),traffic_light=d['traffic_light'],path_reason=str(d.get('reason','')))
    values['path_source']=str(d.get('path_source','CAMERA_PATH'))
    for key in ('track_width','track_width_age'):
        if d.get(key) is not None:values[key]=str(d[key])
    result['status']=DiagnosticArray(header=header,status=[DiagnosticStatus(name='camera_path',message=d['traffic_light'],values=[KeyValue(key=k,value=v) for k,v in values.items()])])
    return result


def encode_lidar_frame(result,header,points,indices,wall_indices,scan_valid):
    """One capture: measured corridor, transformed scan and its wall labels."""
    data=json.loads(encode_bundle(result,header))
    data['lidar_frame']=dict(version=1,valid=bool(scan_valid),points=points.tolist(),
        indices=indices.tolist(),wall_indices=[int(i) for i in wall_indices])
    return json.dumps(data,allow_nan=False,separators=(',',':'))


def decode_lidar_frame(data,ros_now,timeout=.25):
    parts=decode_bundle(data,ros_now,timeout)
    frame=json.loads(data)['lidar_frame']
    if type(frame['version']) is not int or frame['version']!=1 or type(frame['valid']) is not bool:raise ValueError('invalid_lidar_frame')
    points=np.asarray(frame['points'],dtype=float)
    if points.size==0:points=np.empty((0,2))
    if points.ndim!=2 or points.shape[1]!=2:raise ValueError('invalid_lidar_frame_points')
    indices=frame['indices'];walls=frame['wall_indices'];index_set=set(indices)
    if (len(points)>10000 or len(points)!=len(indices) or not np.isfinite(points).all() or
        any(type(i) is not int or not 0<=i<100000 for i in [*indices,*walls]) or
        len(indices)!=len(index_set) or any(i not in index_set for i in walls)):
        raise ValueError('invalid_lidar_frame_points')
    return parts,points,np.asarray(indices,dtype=int),set(walls),frame['valid']


def encode_lidar_observation(frame,obstacles,valid):
    data=json.loads(frame)
    data.pop('lidar_frame')
    data['lidar_observation']=dict(version=1,valid=bool(valid),obstacles=[
        dict(id=str(o.id),x=o.x,y=o.y,width=o.width,depth=o.depth,
             confidence=o.confidence,closing_speed=o.closing_speed) for o in obstacles])
    return json.dumps(data,allow_nan=False,separators=(',',':'))


def decode_lidar_observation(data,ros_now,timeout=.25):
    parts=decode_bundle(data,ros_now,timeout)
    observation=json.loads(data)['lidar_observation']
    if type(observation['version']) is not int or observation['version']!=1 or type(observation['valid']) is not bool:raise ValueError('invalid_lidar_observation')
    obstacles=observation['obstacles']
    if not isinstance(obstacles,list) or len(obstacles)>1000:raise ValueError('invalid_lidar_obstacles')
    ids=set()
    for obstacle in obstacles:
        values=[obstacle[k] for k in ('x','y','width','depth','confidence','closing_speed')]
        if (not isinstance(obstacle['id'],str) or obstacle['id'] in ids or
            not all(type(v) in (int,float) and math.isfinite(v) for v in values) or min(values[2:4])<=0 or
            not 0<=values[4]<=1 or values[5]<0):raise ValueError('invalid_lidar_obstacle')
        ids.add(obstacle['id'])
    if not observation['valid'] and obstacles:raise ValueError('invalid_lidar_observation')
    return parts,obstacles,observation['valid']
