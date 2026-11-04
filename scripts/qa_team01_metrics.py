#!/usr/bin/env python3
"""Read-only ROS2 Gazebo QA; no vehicle command publisher and no map feedback to controller."""
import argparse,csv,json,math,statistics,time
from collections import Counter,deque
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool,Float32,String
from team01_race.lap_validation import LapVerifier,load_reference

def p95(a):return round(float(np.percentile(a,95)),4) if a else None
def path_error(p,ref):
    a=ref[:-1];v=ref[1:]-a
    if not len(v):return None
    t=np.clip(np.sum((p-a)*v,axis=1)/np.maximum(np.sum(v*v,axis=1),1e-8),0,1)
    return float(np.min(np.linalg.norm(p-(a+t[:,None]*v),axis=1)))

class Metrics(Node):
    def __init__(self,reference,out):
        super().__init__('team01_independent_metrics')
        self.out=Path(out);self.out.mkdir(parents=True,exist_ok=True)
        self.verifier=LapVerifier(*load_reference(reference))
        self.log=(self.out/'qa_events.jsonl').open('w',encoding='utf-8')
        self.f=(self.out/'trajectory.csv').open('w',newline='',encoding='utf-8')
        self.csv=csv.writer(self.f)
        self.csv.writerow(['t_mono_s','x_m','y_m','yaw_rad','v_mps','signed_center_offset_m',
                           'road_clearance_m','planned_path_error_m','cmd_v_mps','cmd_steer_rad','mode','reason'])
        self.lastpose=None;self.odom_pose=None;self.lastsample=0.;self.distance=0.
        self.plans=deque(maxlen=80);self.seq=0;self.last_used=-1
        self.offset=[];self.clear=[];self.err=[];self.speed=[];self.steering=[]
        self.states=Counter();self.reasons=Counter();self.transitions=Counter()
        self.mode=None;self.debug={};self.start={};self.adapter={}
        self.ready=self.lights=self.permission=self.sent=False
        self.sign=0;self.sign_at=0.;self.reversals=0
        self.prev_wheel_rad=None;self.steer_change_deg=0.;self.measured_v=0.
        self.straight_abs_steer=[];self.straight_actual_speed=[];self.straight_speed_ratio=[]
        self.create_subscription(Odometry,'/team01/gz_odom',self.odom,qos_profile_sensor_data)
        self.create_subscription(String,'/team01/debug/path',self.path,20)
        self.create_subscription(String,'/team01/debug/start',self.start_cb,20)
        self.create_subscription(String,'/team01/debug/adapter',self.adapter_cb,20)
        self.create_subscription(Bool,'/team01/ready_for_lights',self.ready_cb,20)
        self.create_subscription(Float32,'/team01/steering',self.steer_cb,20)
        self.create_timer(4.,self.save)
    def event(self,kind,details):
        self.log.write(json.dumps({'t':round(time.monotonic(),3),'event':kind,**details},
                                  ensure_ascii=False,separators=(',',':'))+'\n')
    def ready_cb(self,m):
        if m.data and not self.ready:self.ready=True;self.event('camera_ready',{'ok':True})
    def start_cb(self,m):
        try:self.start=json.loads(m.data)
        except (ValueError,TypeError):return
        if self.start.get('lights_out') and not self.lights:
            self.lights=True;self.event('lights_out',{'ok':True})
        if self.start.get('permission') and not self.permission:
            self.permission=True;self.event('permission',{'ok':True})
    def adapter_cb(self,m):
        try:self.adapter=json.loads(m.data)
        except (ValueError,TypeError):return
        if float(self.adapter.get('sent_linear_mps') or 0)>.05 and not self.sent:
            self.sent=True;self.event('adapter_sent',{'v':self.adapter['sent_linear_mps']})
    def steer_cb(self,m):
        v=float(m.data)
        if not math.isfinite(v):return
        self.steering.append(v)
        if self.prev_wheel_rad is not None and self.measured_v>.5:
            self.steer_change_deg+=abs(math.degrees(v-self.prev_wheel_rad))
        self.prev_wheel_rad=v
        s=1 if v>math.radians(2) else -1 if v<-math.radians(2) else 0
        t=time.monotonic()
        if s and self.sign and s!=self.sign and t-self.sign_at>.20:
            self.reversals+=1;self.sign_at=t
        if s:self.sign=s
    def path(self,m):
        try:d=json.loads(m.data)
        except (ValueError,TypeError):return
        self.debug=d
        mode=str(d.get('mode') or 'UNKNOWN');reason=str(d.get('reason') or '')
        self.states[mode]+=1
        if mode in ('STOP','BRAKE','RECOVER'):self.reasons[reason]+=1
        if self.mode is not None and mode!=self.mode:
            self.transitions[self.mode+'->'+mode]+=1
            self.event('mode_change',{'old':self.mode,'new':mode,'reason':reason})
        self.mode=mode
        if not d.get('reference_valid') or self.odom_pose is None:return
        try:
            ref=np.asarray(d.get('reference_xy') or [],dtype=float).reshape((-1,2))
            if len(ref)<3 or not np.isfinite(ref).all():return
            x,y,yaw=self.odom_pose;c=math.cos(yaw);s=math.sin(yaw)
            transformed=np.column_stack((x+c*ref[:,0]-s*ref[:,1],
                                         y+s*ref[:,0]+c*ref[:,1]))
            self.seq+=1;self.plans.append((self.seq,time.monotonic(),(x,y),transformed))
        except (ValueError,TypeError):return
    def odom(self,m):
        p=m.pose.pose.position;q=m.pose.pose.orientation
        x,y=float(p.x),float(p.y)
        yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        if not all(math.isfinite(z) for z in (x,y,yaw)):return
        if self.lastpose is not None:
            dist=math.hypot(x-self.lastpose[0],y-self.lastpose[1])
            if dist<3:self.distance+=dist
        self.lastpose=(x,y);self.odom_pose=(x,y,yaw)
        now=time.monotonic()
        if now-self.lastsample<.10:return
        self.lastsample=now
        v=float(m.twist.twist.linear.x)
        if not math.isfinite(v):v=0.
        self.speed.append(abs(v))
        self.measured_v=abs(v)
        _,lat,width,_=self.verifier.project((x,y))
        clearance=width/2-abs(lat)-self.verifier.edge_margin
        self.offset.append(abs(lat));self.clear.append(clearance)
        d=self.debug
        if d.get('mode')=='STRAIGHT' and d.get('reference_valid') and clearance>.10 and abs(v)>.4:
            self.straight_actual_speed.append(abs(v))
            if self.prev_wheel_rad is not None:
                self.straight_abs_steer.append(abs(math.degrees(self.prev_wheel_rad)))
            try:
                cmd=float(d.get('cmd_speed') or 0.)
                if cmd>.5:self.straight_speed_ratio.append(min(1.5,abs(v)/cmd))
            except (TypeError,ValueError):pass
        error=None
        # Don't use current base_link path starting at (0,0): compare an old,
        # frozen plan (0.6-1.5s old) with later actual Gazebo position instead.
        for i,ts,origin,ref in reversed(self.plans):
            if .6<=now-ts<=1.5 and i!=self.last_used:
                if math.hypot(x-origin[0],y-origin[1])>.25:
                    error=path_error(np.asarray((x,y)),ref)
                    self.last_used=i
                    if error is not None:self.err.append(error)
                break
        d=self.debug
        self.csv.writerow([round(now,4),round(x,5),round(y,5),round(yaw,5),
                           round(v,4),round(lat,4),round(clearance,4),
                           '' if error is None else round(error,4),
                           d.get('cmd_speed'),d.get('cmd_steer'),d.get('mode'),d.get('reason')])
    def save(self):
        self.f.flush();self.log.flush()
        valid=self.distance>=5 and len(self.err)>=15
        centered=self.distance>=5 and len(self.offset)>=15
        data={'actual_distance_m':round(self.distance,3),
              'odom_sample_count':len(self.speed),
              'planned_path_tracking_evaluable':bool(valid),
              'planned_path_error_mean_m':round(statistics.mean(self.err),4) if valid else None,
              'planned_path_error_p95_m':p95(self.err) if valid else None,
              'planned_path_error_sample_count':len(self.err),
              'centerline_evaluable':bool(centered),
              'centerline_abs_offset_mean_m':round(statistics.mean(self.offset),4) if centered else None,
              'centerline_abs_offset_p95_m':p95(self.offset) if centered else None,
              'road_clearance_min_m':round(min(self.clear),4) if self.clear else None,
              'negative_clearance_fraction':round(sum(v<0 for v in self.clear)/len(self.clear),4) if centered else None,
              'actual_speed_max_mps':round(max(self.speed),3) if self.speed else None,
              'actual_speed_p95_mps':p95(self.speed),
              'steering_reversals_over_2deg':self.reversals,
              'steering_variation_deg_per_m':round(self.steer_change_deg/self.distance,3) if self.distance>=5 else None,
              'safe_straight_samples':len(self.straight_actual_speed),
              'safe_straight_speed_mean_mps':round(statistics.mean(self.straight_actual_speed),3) if len(self.straight_actual_speed)>=15 else None,
              'safe_straight_abs_steer_mean_deg':round(statistics.mean(self.straight_abs_steer),3) if len(self.straight_abs_steer)>=15 else None,
              'safe_straight_abs_steer_p95_deg':p95(self.straight_abs_steer) if len(self.straight_abs_steer)>=15 else None,
              'safe_straight_actual_over_cmd_ratio':round(statistics.mean(self.straight_speed_ratio),3) if len(self.straight_speed_ratio)>=15 else None,
              'modes':dict(self.states),'fault_reasons':dict(self.reasons),
              'mode_transitions':dict(self.transitions),
              'camera_ready':self.ready,'camera_lights_out':self.lights,
              'permission':self.permission,'adapter_nonzero':self.sent,
              'last_start_status':self.start,'last_adapter_status':self.adapter,
              'disclaimer':'QA only. Centerline offset is not tracking failure if driving off-center safely. Path error uses a 0.6-1.5s frozen planned path transformed into Gazebo odom world frame; needs >=5m travel and >=15 samples. No track coordinates fed to controller.'}
        f=self.out/'drive_metrics.json';tmp=f.with_suffix('.tmp')
        tmp.write_text(json.dumps(data,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
        tmp.replace(f)
    def close(self):self.save();self.f.close();self.log.close()

def main():
    p=argparse.ArgumentParser()
    p.add_argument('--reference',required=True);p.add_argument('--out',required=True)
    a=p.parse_args();rclpy.init();n=Metrics(a.reference,a.out)
    try:rclpy.spin(n)
    except KeyboardInterrupt:pass
    finally:n.close();n.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
