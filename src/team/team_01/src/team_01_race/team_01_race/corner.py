"""Camera center-path geometry is the sole source of corner authority."""
from dataclasses import dataclass
import math
import numpy as np
from .navigation import path_geometry, valid_path

SEVERITIES=('STRAIGHT','MILD_CORNER','MEDIUM_CORNER','SHARP_CORNER')


@dataclass
class CornerConfig:
    enabled: bool = True
    preview_distance_m: float = 5.0
    near_distance_m: float = 1.0
    mild_curvature_enter: float = .08
    medium_curvature_enter: float = .18
    sharp_curvature_enter: float = .30
    mild_curvature_exit: float = .06
    medium_curvature_exit: float = .14
    sharp_curvature_exit: float = .24
    mild_heading_enter_deg: float = 8.0
    medium_heading_enter_deg: float = 18.0
    sharp_heading_enter_deg: float = 30.0
    mild_heading_exit_deg: float = 6.0
    medium_heading_exit_deg: float = 14.0
    sharp_heading_exit_deg: float = 24.0
    straight_max_speed_mps: float = 7.0
    mild_max_speed_mps: float = 5.0
    medium_max_speed_mps: float = 3.5
    sharp_max_speed_mps: float = 2.5
    straight_max_steering_deg: float = 8.0
    mild_max_steering_deg: float = 14.0
    medium_max_steering_deg: float = 20.0
    sharp_max_steering_deg: float = 25.0
    physical_max_steering_deg: float = 30.0
    sharp_fast_curvature: float = .45
    sharp_fast_heading_deg: float = 45.0
    corner_enter_confirm_frames: int = 2
    corner_exit_confirm_frames: int = 3
    corner_approach_decel_mps2: float = 4.0
    corner_brake_margin_m: float = .75
    reaction_time_sec: float = .10
    corner_entry_distance_m: float = .75
    exit_path_stability_m: float = .12
    exit_min_confidence: float = .75
    curvature_median_points: int = 5
    camera_history_sec: float = .25

    def validate(self, normal_speed):
        numeric=[v for v in vars(self).values() if not isinstance(v,bool)]
        enters=[self.mild_curvature_enter,self.medium_curvature_enter,self.sharp_curvature_enter]
        exits=[self.mild_curvature_exit,self.medium_curvature_exit,self.sharp_curvature_exit]
        headings=[self.mild_heading_enter_deg,self.medium_heading_enter_deg,self.sharp_heading_enter_deg]
        heading_exits=[self.mild_heading_exit_deg,self.medium_heading_exit_deg,self.sharp_heading_exit_deg]
        speeds=self.speeds(normal_speed);steers=self.steer_limits()
        if (not all(math.isfinite(v) and v>0 for v in numeric) or not 0<self.near_distance_m<self.preview_distance_m<=5 or
            enters!=sorted(set(enters)) or headings!=sorted(set(headings)) or
            not all(a<b for a,b in zip(exits,enters)) or not all(a<b for a,b in zip(heading_exits,headings)) or
            not speeds[0]>=speeds[1]>=speeds[2]>=speeds[3] or steers!=sorted(steers) or
            steers[-1]>self.physical_max_steering_deg or self.physical_max_steering_deg>30 or
            min(self.corner_enter_confirm_frames,self.corner_exit_confirm_frames)<2 or
            self.curvature_median_points%2!=1 or not 0<self.exit_min_confidence<=1):
            raise ValueError('invalid corner severity configuration')

    def speeds(self,normal_speed):
        return [min(normal_speed,v) for v in (self.straight_max_speed_mps,self.mild_max_speed_mps,self.medium_max_speed_mps,self.sharp_max_speed_mps)]

    def steer_limits(self):
        return [self.straight_max_steering_deg,self.mild_max_steering_deg,self.medium_max_steering_deg,self.sharp_max_steering_deg]

    def curvature_enters(self):
        return [self.mild_curvature_enter,self.medium_curvature_enter,self.sharp_curvature_enter]

    def heading_enters(self):
        return [self.mild_heading_enter_deg,self.medium_heading_enter_deg,self.sharp_heading_enter_deg]


def preview_geometry(path,config,horizon=None):
    horizon=config.preview_distance_m if horizon is None else horizon
    if not valid_path(path,horizon):raise ValueError('invalid Camera center path')
    distance=np.r_[path[0,0],path[0,0]+np.cumsum(np.linalg.norm(np.diff(path,axis=0),axis=1))]
    heading,curvature=path_geometry(path)  # Reuse the existing geometry calculator.
    heading=np.unwrap(heading);half=config.curvature_median_points//2
    curvature=np.array([np.median(curvature[max(0,i-half):min(len(path),i+half+1)]) for i in range(len(path))])
    visible=distance<=horizon
    if not visible.any():visible[0]=True
    near=distance<=config.near_distance_m
    if not near.any():near[0]=True
    reference=float(np.median(heading[near]))
    change=np.abs(heading-reference)
    # Median filtering plus a spatial 90th percentile requires a supported
    # bend, rather than granting sharp authority from a few derivative kinks.
    values=np.abs(curvature[visible])
    robust=float(np.percentile(values,90))
    future=float(np.degrees(np.max(change[visible])))
    klevel=sum(robust>=v for v in config.curvature_enters())
    forward_coverage=float(path[-1,0])
    confirmation_heading=max(future,abs(math.degrees(reference))) if forward_coverage<2*config.near_distance_m else future
    hlevel=sum(confirmation_heading>=v for v in config.heading_enters())
    level=min(klevel,hlevel)
    conflict=abs(klevel-hlevel)>=2
    upcoming=visible & (np.abs(curvature)>=config.curvature_enters()[max(0,level-1)])
    first=np.flatnonzero(upcoming) if level else []
    ref_y=float(np.median(path[near,1]-math.tan(reference)*path[near,0]))
    return dict(forward_coverage_m=forward_coverage,confirmation_heading_deg=confirmation_heading,path_heading_body_deg=math.degrees(reference),current_curvature=float(np.max(np.abs(curvature[near]))),max_future_curvature=float(np.max(values)),
                robust_future_curvature=robust,average_future_curvature=float(np.mean(values)),
                future_heading_change=math.radians(future),future_heading_change_deg=future,
                lateral_path_change=float(np.max(np.abs(path[visible,1]-(ref_y+math.tan(reference)*path[visible,0])))),
                distance_to_corner=float(distance[first[0]]) if len(first) else math.inf,
                available_preview_distance_m=float(min(distance[-1],horizon)),
                candidate_level=level,geometry_conflict=conflict,upcoming=level>0,
                hard=robust>=config.sharp_fast_curvature and future>=config.sharp_fast_heading_deg)


class CornerDetector:
    def __init__(self,config=None,normal_speed=7.):
        self.c=config or CornerConfig();self.c.validate(normal_speed);self.normal_speed=normal_speed
        self.state='STRAIGHT';self.level=0;self.enter_count=self.exit_count=0
        self.pending_level=0;self.last_key=None;self.last_geometry_key=None;self.last_path=None;self.last_time=-math.inf
        self.metrics=preview_geometry(np.column_stack((np.arange(.1,5.01,.1),np.zeros(50))),self.c)
        self.geometry_confidence=0.;self.straight_count=0
        self.finish_count=0

    @property
    def severity(self):return SEVERITIES[self.level]

    def reject(self,key,transient=False):
        if key!=self.last_key:
            self.last_key=key
            if not transient:self.enter_count=self.exit_count=0

    def update(self,path,key,now,*,speed,steering_demand=0.,steering_fresh=True,confidence=1.,path_age=0.):
        # steering_demand/fresh are accepted for old callers; neither entry nor
        # exit/severity classification reads steering, IMU or LiDAR.
        if not self.c.enabled or key==self.last_key:return False
        self.last_key=key
        try:metrics=preview_geometry(path,self.c)
        except (ValueError,FloatingPointError):
            self.enter_count=self.exit_count=0;return False
        if not all(math.isfinite(v) for v in (now,speed,confidence,path_age)) or not 0<=path_age<=self.c.camera_history_sec:return False
        if confidence<.4:
            self.enter_count=self.exit_count=0;return False
        uninterrupted=now-self.last_time<=self.c.camera_history_sec
        stable=(uninterrupted and self.last_path is not None and
                np.max(abs(path[:,1]-np.interp(path[:,0],self.last_path[:,0],self.last_path[:,1])))<=self.c.exit_path_stability_m)
        if not uninterrupted:self.enter_count=self.exit_count=0
        self.last_geometry_key=key
        self.metrics=metrics;self.last_path=path.copy();self.last_time=now-path_age
        self.geometry_confidence=confidence*(.65 if metrics['geometry_conflict'] else 1.)
        previous_level=self.level
        candidate=metrics['candidate_level']
        # A shortened view cannot disprove the heading of an already confirmed
        # bend. Current high-confidence curvature must still support its level.
        if self.level>0 and metrics['forward_coverage_m']<2*self.c.near_distance_m:
            self.geometry_confidence=confidence
        heading_aligned=abs(metrics['path_heading_body_deg'])<=self.c.mild_heading_exit_deg
        fully_low=(heading_aligned and metrics['robust_future_curvature']<self.c.mild_curvature_exit and metrics['future_heading_change_deg']<self.c.mild_heading_exit_deg)
        self.finish_count=self.finish_count+1 if fully_low and stable and confidence>=self.c.exit_min_confidence else 0
        if candidate>self.level:
            self.enter_count=self.enter_count+1 if candidate==self.pending_level else 1
            self.pending_level=candidate;self.exit_count=0
            if (metrics['hard'] and confidence>=self.c.exit_min_confidence) or self.enter_count>=self.c.corner_enter_confirm_frames:
                self.level=candidate;self.enter_count=0
        else:
            self.enter_count=0
            if self.level:
                kexit=[self.c.mild_curvature_exit,self.c.medium_curvature_exit,self.c.sharp_curvature_exit][self.level-1]
                hexit=[self.c.mild_heading_exit_deg,self.c.medium_heading_exit_deg,self.c.sharp_heading_exit_deg][self.level-1]
                # Entry requires both signals. Release the higher level when
                # either signal falls below its exit threshold, while the
                # lower candidate and three stable frames corroborate it.
                below_k=metrics['robust_future_curvature']<kexit
                below_h=metrics['future_heading_change_deg']<hexit
                enough_preview=metrics['forward_coverage_m']>=min(self.c.preview_distance_m,2*self.c.near_distance_m)
                low=(enough_preview and heading_aligned and candidate<self.level and (below_k and below_h if self.level==1 else below_k or below_h))
                self.exit_count=self.exit_count+1 if low and stable and confidence>=self.c.exit_min_confidence else 0
                if self.exit_count>=self.c.corner_exit_confirm_frames:
                    self.level=max(candidate,self.level-1);self.exit_count=0
        if self.level==0:self.state='CORNER_EXIT' if previous_level>0 and self.finish_count>=self.c.corner_exit_confirm_frames else 'CORNER_APPROACH' if candidate>0 else 'STRAIGHT'
        elif self.finish_count>=self.c.corner_exit_confirm_frames:self.state='CORNER_EXIT'
        elif metrics['distance_to_corner']>self.c.corner_entry_distance_m and speed>self.c.speeds(self.normal_speed)[self.level]+.15:self.state='CORNER_APPROACH'
        else:self.state='CORNER_COMMIT'
        return True

    def policy(self,now,speed):
        level=self.level if self.c.enabled else 0
        limit=self.c.speeds(self.normal_speed)[level]
        if self.metrics['geometry_conflict'] and self.c.enabled:limit=min(limit,self.c.mild_max_speed_mps)
        distance=self.metrics['distance_to_corner']
        if math.isfinite(distance):distance=max(0.,distance-max(0.,speed)*max(0.,now-self.last_time))
        required=max(0.,speed*speed-limit*limit)/(2*self.c.corner_approach_decel_mps2)
        margin=self.c.corner_brake_margin_m+max(0.,speed)*self.c.reaction_time_sec
        feasible=speed<=limit+.05 or distance>=required+margin
        entry=level>0 and distance<=self.c.corner_entry_distance_m
        return dict(self.metrics,corner_state=self.state,corner_severity=SEVERITIES[level],corner_level=level,
                    corner_speed_limit=limit,corner_steering_limit_deg=self.c.steer_limits()[level],
                    current_speed=speed,current_speed_source='control_command_estimate',
                    distance_to_corner=distance,required_braking_distance=required,braking_margin=margin,
                    braking_feasible=feasible,entry_overspeed=entry and speed>limit+.15,
                    preview_braking_insufficient=required+margin>self.metrics['available_preview_distance_m'] and speed>limit+.05,
                    corner_enter_count=self.enter_count,corner_exit_count=self.exit_count,
                    camera_source_stamp=self.last_geometry_key if isinstance(self.last_geometry_key,(int,float)) else 0.,
                    camera_geometry_confidence=self.geometry_confidence,
                    camera_geometry_age=max(0.,now-self.last_time))

    def constrain(self,result,now,speed):
        out=result.copy();out.update(self.policy(now,speed))
        # A partially visible sharp bend may not yet expose 30 degrees. Brake
        # conservatively using supported curvature plus the existing mild
        # heading threshold; steering authority still requires full severity.
        preview_sharp=(self.c.enabled and 0<=now-self.last_time<=self.c.camera_history_sec and
            self.geometry_confidence>=self.c.exit_min_confidence and
            self.metrics['robust_future_curvature']>=self.c.sharp_curvature_enter and
            self.metrics['confirmation_heading_deg']>=self.c.mild_heading_enter_deg)
        preview_limit=self.c.sharp_max_speed_mps if preview_sharp else out['corner_speed_limit']
        out['corner_preview_speed_limit']=preview_limit
        if out['enabled']:
            if preview_limit<out['speed']:out['active_limiter']='CORNER_PREVIEW' if preview_sharp else 'CORNER_SEVERITY'
            out['speed']=min(out['speed'],preview_limit)
        # Preview limiting starts on confirmed geometry, even before braking
        # becomes mandatory by distance. This never waits for a PP turn.
        out['corner_braking']=bool(out['enabled'] and self.c.enabled and (self.level>0 or preview_sharp) and speed>out['speed']+.01)
        return out
