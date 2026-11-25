"""Full-footprint gaps and normal-offset trajectories; one downstream PP."""
from dataclasses import dataclass
from copy import copy
import math
import numpy as np
from .corridor_geometry import basis,progress,normal_hit,project_point,resample_x,footprint_distance,supported_prefix,observed_support,drive_prefix
from .lidar_filter import polyline_distance

@dataclass
class ObstaclePassConfig:
    enabled: bool=True
    obstacle_pass_speed_mps: float=2.
    pass_side_hysteresis: bool=True
    merge_center_enabled: bool=True
    clear_confirm_frames: int=3
    merge_confirm_frames: int=3

def footprint(obstacle):
    return np.array([[obstacle.x+sx*obstacle.depth/2,obstacle.y+sy*obstacle.width/2] for sx,sy in [(-1,-1),(-1,1),(1,1),(1,-1)]])

def inside_corridor(path,left,right,clearance):
    if path is None or len(path)<3 or not np.isfinite(path).all() or np.any(np.diff(path[:,0])<=0):return False
    if not np.all(observed_support(path,left,right)):return False
    return bool(np.min(polyline_distance(path,left))>=clearance-1e-5 and np.min(polyline_distance(path,right))>=clearance-1e-5)

class ObstaclePassPlanner:
    def __init__(self,width,wheelbase,margin=.15,long_margin=.3,rear_overhang=.21,config=None):
        self.width=width;self.wheelbase=wheelbase;self.margin=margin;self.long_margin=long_margin;self.rear=rear_overhang
        self.c=config or ObstaclePassConfig()
        if not 0<self.c.obstacle_pass_speed_mps<=2.5 or min(self.c.clear_confirm_frames,self.c.merge_confirm_frames)<2:raise ValueError('invalid obstacle pass policy')
        self.steering_limit_deg=30.;self.candidate_debug={}
        self.reset()
    def reset(self):
        self.side='CENTER';self.state='CENTER';self.observation=None;self.observation_key=None;self.offset=0.;self.clear_count=self.merge_count=0;self.last_clear_key=None;self.motion_time=None;self.last_motion=None;self.safe_path=None;self.corners=None;self.last_measurement_key=None;self.debug={};self.transitions=[]
    def integrate(self,now,twist):
        self.last_motion=None
        if self.motion_time is not None and twist is not None:
            dt=max(0.,min(.25,now-self.motion_time));vx,vy,w=twist;angle=w*dt
            dx=vx*dt if abs(w)<1e-6 else (vx*math.sin(angle)+vy*(math.cos(angle)-1))/w
            dy=vy*dt if abs(w)<1e-6 else (vx*(1-math.cos(angle))+vy*math.sin(angle))/w
            c,s=math.cos(angle),math.sin(angle);rotation=np.array([[c,-s],[s,c]]);self.last_motion=(np.array([dx,dy]),rotation)
            if self.observation is not None:
                q=((self.corners if self.corners is not None else footprint(self.observation))-[dx,dy])@rotation
                self.corners=q
                lo,hi=q.min(axis=0),q.max(axis=0);self.observation.x,self.observation.y=(lo+hi)/2;self.observation.depth,self.observation.width=hi-lo
            if self.safe_path is not None:self.safe_path=drive_prefix(resample_x((self.safe_path-[dx,dy])@rotation))
        self.motion_time=now
    def _set_state(self,state):
        if state!=self.state:self.transitions.append(state)
        self.state=state
    def candidate(self,center,left,right,obstacle,side,others=()):
        from .navigation import valid_path,path_geometry
        detail={};self.candidate_debug[side]=detail
        t,n=basis(center);s=progress(center)
        if self.side==side and obstacle.min_x<=center[0,0]+self.long_margin:
            # Alongside/behind the car, an obstacle cross-section can fall
            # behind finite scan support. Keep the side corridor only if its
            # complete path still clears both live walls and every footprint.
            q=drive_prefix(supported_prefix(resample_x(center+self.offset*n),left,right))
            clearance=self.width/2+self.margin
            if valid_path(q) and inside_corridor(q,left,right,clearance) and all(footprint_distance(q,o)>=clearance-1e-5 for o in (obstacle,*others)):
                steering=math.degrees(math.atan(self.wheelbase*float(np.max(abs(path_geometry(q)[1])))))
                if steering<=min(30.,self.steering_limit_deg):
                    detail.update(reason='side_clearance_hold',goal=self.offset,required_steering_deg=steering)
                    return q,0.,self.offset
        projected=[project_point(center,q)[:2] for q in footprint(obstacle)]
        if np.any((footprint(obstacle)-center[-1])@t[-1]>.02):
            detail['reason']='footprint_beyond_observed_path';return None,0.,0.
        ss,ds=np.asarray(projected).T;lo=float(ss.min());hi=float(ss.max());edge=float(ds.max() if side=='LEFT_PASS' else ds.min());sign=1 if side=='LEFT_PASS' else -1
        # Evaluate the entire obstacle longitudinal interval, including its edges.
        queries=np.unique(np.r_[lo,hi,s[(s>=lo)&(s<=hi)]])
        available=[];goals=[]
        for si in queries:
            point=np.array([np.interp(si,s,center[:,0]),np.interp(si,s,center[:,1])]);normal=np.array([np.interp(si,s,n[:,0]),np.interp(si,s,n[:,1])]);normal/=np.linalg.norm(normal)
            wall=normal_hit(point,normal,left if sign==1 else right,sign)
            if wall is None:return None,0.,0.
            available.append(sign*(wall-edge));goals.append(.5*(wall+edge))
        gap=min(available);clearance=self.width/2+self.margin
        if gap<2*clearance:return None,gap,0.
        goal=float(min(goals) if sign==1 else max(goals))
        # Reach the gap midpoint gradually. Search entry lengths inside the
        # observed prefix; every candidate must clear the entire footprint.
        # This permits completing the centring after the obstacle's front
        # edge when clearance is already adequate, without exceeding steering.
        start=project_point(center,np.zeros(2))[1] if self.side in ('LEFT_PASS','RIGHT_PASS') else 0.
        slope=-t[0,1]/max(t[0,0],.2) if self.side in ('LEFT_PASS','RIGHT_PASS') else 0.
        detail.update(center_start=center[0].tolist(),obstacle=[obstacle.x,obstacle.y,obstacle.depth,obstacle.width],start_offset=start,goal=goal)
        first=max(.3,hi+self.long_margin)
        def evaluate(entry):
            u=np.clip(s/entry,0,1);delta=goal-start;initial_derivative=slope*entry
            offset=start+initial_derivative*u+(10*delta-6*initial_derivative)*u**3+(-15*delta+8*initial_derivative)*u**4+(6*delta-3*initial_derivative)*u**5
            if obstacle.x-obstacle.depth/2<=center[0,0]+self.long_margin and abs(start-goal)<.1:offset=np.full(len(s),goal)
            q=resample_x(center+offset[:,None]*n)
            if q is not None:
                q=supported_prefix(q,left,right)
            if q is not None:
                # A normal offset changes X support and can reach a vertical
                # tangent sooner. Keep only the measured, PP-valid prefix.
                steep=np.flatnonzero(abs(np.diff(q[:,1]))>=.35)
                if len(steep):q=q[:steep[0]+1]
            detail['entry_length']=float(entry)
            if not valid_path(q):detail['reason']='invalid_forward_path';return None
            if not inside_corridor(q,left,right,clearance):detail['reason']='wall_clearance';return None
            if any(footprint_distance(q,o)<clearance-1e-5 for o in (obstacle,*others)):detail['reason']='footprint_clearance';return None
            detail['required_steering_deg']=math.degrees(math.atan(self.wheelbase*float(np.max(abs(path_geometry(q)[1])))))
            if detail['required_steering_deg']>min(30.,self.steering_limit_deg):detail['reason']='steering_limit';return None
            detail['reason']='valid'
            return q
        # Prefer the gap midpoint. When its steering demand is infeasible,
        # search the safe interior of the same gap; all original wall/object
        # margins remain mandatory. Retain an already safe pass offset.
        reserve=min(.05,max(0.,(gap-2*clearance)/4))
        near=edge+sign*(clearance+reserve)
        targets=list(np.linspace(goal,near,5))
        if self.side==side and sign*self.offset>0:targets.insert(0,self.offset)
        for goal in targets:
            detail['goal']=float(goal)
            previous_entry=None;previous_reason=None
            for entry in np.unique(np.clip(first+np.arange(0.,2.501,.1),.3,max(.3,s[-1]))):
                q=evaluate(entry);reason=detail.get('reason')
                if q is not None:return q,gap,goal
                if previous_reason=='steering_limit' and reason=='footprint_clearance':
                    for fine in np.linspace(previous_entry,entry,11)[1:-1]:
                        q=evaluate(fine)
                        if q is not None:return q,gap,goal
                previous_entry=entry;previous_reason=reason
        return None,gap,goal
    def plan(self,center,left,right,obstacles,blocking,key,now):
        self.debug=dict(obstacle_count=len(obstacles),blocking_obstacle=blocking[0].id if blocking else None,required_width=self.width+2*self.margin,left_gap=0.,right_gap=0.,left_pass_valid=False,right_pass_valid=False,selected_pass_side=self.side,mode=self.state,pass_speed_limit=self.c.obstacle_pass_speed_mps)
        if not self.c.enabled and blocking:self._set_state('STOP_BLOCKED');return None,self.state,'obstacle_pass_disabled'
        observed=next((o for o in obstacles if self.observation is not None and o.id==self.observation.id),None)
        if observed is None and self.observation is not None and self.side in ('LEFT_PASS','RIGHT_PASS'):
            # Geometry can disappear long enough for the detector's short ID
            # timeout. Associate a reacquired footprint with the ego-projected
            # active object before deciding that it is a new pass.
            matches=[o for o in obstacles if math.hypot(o.x-self.observation.x,o.y-self.observation.y)<=.8 and .4<=o.width/max(self.observation.width,.01)<=2.5 and .25<=o.depth/max(self.observation.depth,.01)<=4.]
            if matches:observed=min(matches,key=lambda o:math.hypot(o.x-self.observation.x,o.y-self.observation.y))
        if observed is not None and key!=self.last_measurement_key:
            self.observation=copy(observed);self.corners=footprint(observed);self.last_measurement_key=key
        if self.observation is not None and self.side in ('LEFT_PASS','RIGHT_PASS'):
            rear_clear=self.observation.x+self.observation.depth/2 < -self.rear-self.long_margin
            if key!=self.last_clear_key:
                self.clear_count=self.clear_count+1 if rear_clear and not blocking else 0;self.last_clear_key=key
            if self.clear_count>=self.c.clear_confirm_frames:
                self._set_state('MERGE_CENTER' if self.c.merge_center_enabled else 'CENTER')
                if not self.c.merge_center_enabled:self.side='CENTER';self.observation=None
                return center.copy(),self.state,'confirmed_obstacle_rear_clearance'
        if self.state=='MERGE_CENTER' and not blocking:return center.copy(),self.state,'merging_to_center'
        if self.side in ('LEFT_PASS','RIGHT_PASS') and self.observation is not None:
            live=[o for o in obstacles if o.id!=self.observation.id]+[self.observation]
            warm=self.validated_previous(left,right,live)
            if warm is not None:
                self._set_state(self.side)
                lg,rg=self.measure_gaps(center,left,right,self.observation)
                self.debug.update(mode=self.state,selected_pass_side=self.side,validated_previous_pass=True,left_gap=lg,right_gap=rg,gap_recomputed=True,left_pass_valid=self.side=='LEFT_PASS',right_pass_valid=self.side=='RIGHT_PASS')
                return warm,self.side,'live_validated_previous_pass_geometry'
        obstacle=self.observation if self.observation is not None and self.side in ('LEFT_PASS','RIGHT_PASS') else blocking[0] if blocking else None
        if obstacle is None:self._set_state('CENTER');return center.copy(),'CENTER','camera_center_path'
        if self.observation is None or obstacle.id!=self.observation.id:
            self._set_state('PASS_SELECT');self.observation=copy(obstacle);self.corners=footprint(obstacle);self.last_measurement_key=key;self.side='CENTER';self.clear_count=0
        other=[o for o in obstacles if o.id!=obstacle.id]
        lp,lg,ld=self.candidate(center,left,right,obstacle,'LEFT_PASS',other);rp,rg,rd=self.candidate(center,left,right,obstacle,'RIGHT_PASS',other)
        self.debug.update(blocking_obstacle=obstacle.id,left_gap=lg,right_gap=rg,left_pass_valid=lp is not None,right_pass_valid=rp is not None,candidates=self.candidate_debug)
        choices=[(side,path,gap,goal) for side,path,gap,goal in [('LEFT_PASS',lp,lg,ld),('RIGHT_PASS',rp,rg,rd)] if path is not None]
        chosen=next((c for c in choices if c[0]==self.side),None) if self.c.pass_side_hysteresis else None
        if chosen is None and self.side in ('LEFT_PASS','RIGHT_PASS'):
            warm=self.validated_previous(left,right,[obstacle,*other])
            if warm is not None:
                self._set_state(self.side);self.debug.update(mode=self.state,selected_pass_side=self.side,validated_previous_pass=True)
                return warm,self.side,'live_validated_previous_pass_geometry'
        if chosen is None and self.side in ('LEFT_PASS','RIGHT_PASS') and self.c.pass_side_hysteresis:
            self._set_state('STOP_BLOCKED');return None,self.state,'selected_pass_side_no_longer_safe'
        if chosen is None and choices:
            from .navigation import path_geometry
            def score(c):
                _,path,gap,goal=c;curvature=float(np.max(abs(path_geometry(path)[1])));clear=min(float(np.min(polyline_distance(path,left))),float(np.min(polyline_distance(path,right))))
                return curvature+.3*abs(goal)+.2*abs(path[0,1])-.2*gap-.3*clear
            chosen=min(choices,key=score)
        if chosen is None:self._set_state('STOP_BLOCKED');return None,self.state,'no_vehicle_width_gap' if lg<2*(self.width/2+self.margin) and rg<2*(self.width/2+self.margin) else 'no_safe_pass_trajectory'
        self.side,path,_,self.offset=chosen;self._set_state(self.side);self.debug.update(mode=self.state,selected_pass_side=self.side)
        return path,self.state,'wall_obstacle_normal_gap' if blocking else 'tracked_obstacle_rear_clearance_hold'
    def measure_gaps(self,center,left,right,obstacle):
        _,normals=basis(center);s=progress(center)
        ss,ds=np.asarray([project_point(center,q)[:2] for q in footprint(obstacle)]).T
        queries=np.unique(np.r_[ss.min(),ss.max(),s[(s>=ss.min())&(s<=ss.max())]])
        result=[]
        for side,wall in ((1,left),(-1,right)):
            edge=float(ds.max() if side==1 else ds.min());gaps=[]
            for si in queries:
                point=np.array([np.interp(si,s,center[:,0]),np.interp(si,s,center[:,1])]);normal=np.array([np.interp(si,s,normals[:,0]),np.interp(si,s,normals[:,1])]);normal/=np.linalg.norm(normal)
                hit=normal_hit(point,normal,wall,side)
                if hit is None:gaps=[];break
                gaps.append(side*(hit-edge))
            result.append(float(min(gaps)) if gaps else 0.)
        return result

    def validated_previous(self,left,right,obstacles):
        from .navigation import valid_path,path_geometry
        if self.safe_path is None or self.side not in ('LEFT_PASS','RIGHT_PASS'):return None
        warm=drive_prefix(supported_prefix(self.safe_path,left,right));clearance=self.width/2+self.margin
        if not valid_path(warm) or not inside_corridor(warm,left,right,clearance):return None
        tangent,_=basis(warm)
        for obstacle in obstacles:
            if footprint_distance(warm,obstacle)<clearance-1e-5:return None
            if obstacle.max_x>=0 and np.any((footprint(obstacle)-warm[-1])@tangent[-1]>.02):return None
        steering=math.degrees(math.atan(self.wheelbase*float(np.max(abs(path_geometry(warm)[1])))))
        return warm if steering<=min(30.,self.steering_limit_deg) else None

    def note_selected(self,selected,center,key):
        self.safe_path=selected.copy()
        if self.state!='MERGE_CENTER':return
        difference=float(np.max(abs(selected[:,1]-np.interp(selected[:,0],center[:,0],center[:,1]))))
        if key!=self.observation_key:
            self.merge_count=self.merge_count+1 if difference<.05 else 0;self.observation_key=key
        if self.merge_count>=self.c.merge_confirm_frames:
            self.side='CENTER';self.observation=None;self._set_state('CENTER')
