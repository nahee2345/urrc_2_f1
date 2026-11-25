"""Distinct-depth-frame corner latch and relative odometry arc-length tracking."""
import math
from .depth_geometry import DepthConfig
from .corner import SEVERITIES,CornerConfig

class CornerTravel:
    def __init__(self,timeout=.25):
        self.timeout=timeout;self.distance=0.;self.position=None;self.stamp=-math.inf;self.time=-math.inf;self.accepted=False;self.samples=[];self.epoch=0
    def update(self,stamp,now,age,position,speed):
        if (not all(math.isfinite(v) for v in (*position,stamp,now,age,speed)) or stamp<=self.stamp or not -.05<=age<=self.timeout):return False
        if self.position is not None:
            dt=stamp-self.stamp;step=math.dist(position,self.position)
            # A reset/teleport is not distance travelled toward a known corner.
            if dt>self.timeout or step>max(.5,(abs(speed)+2.)*dt):
                self.accepted=False;self.position=tuple(position);self.stamp=stamp;self.time=now;self.samples=[];self.epoch+=1;return False
            self.distance+=step
        self.position=tuple(position);self.stamp=stamp;self.time=now-max(0.,age);self.accepted=True
        self.samples.append((stamp,self.distance));self.samples=self.samples[-128:]
        return True
    def distance_at(self,stamp):
        if len(self.samples)<2 or not self.samples[0][0]<=stamp<=self.samples[-1][0]:return None
        for (a,x),(b,y) in zip(self.samples,self.samples[1:]):
            if a<=stamp<=b:return x+(y-x)*(stamp-a)/(b-a)
        return None
    def fresh(self,now):return self.accepted and 0<=now-self.time<=self.timeout

class DepthCornerTrack:
    def __init__(self,c=None,corner=None):
        self.corner=corner or CornerConfig()
        self.c=c or DepthConfig();self.c.validate();self.last_stamp=-math.inf;self.time=-math.inf;self.data={}
        self.latched=None;self.pending=None;self.count=0;self.straight_count=0;self.straight=False
        self.state='DEPTH_UNAVAILABLE';self.correction=0.;self.last_measurement=None;self.exit_time=None
    def fresh(self,now):return bool(self.data.get('depth_valid')) and 0<=now-self.time<=self.c.timeout_sec
    def remaining(self,travel):
        return max(0.,self.latched['initial']+self.latched.get('offset',0.)-(travel-self.latched['odom'])) if self.latched else None
    def update(self,data,ros_now,now,travel,odom_fresh,lidar_straight):
        try:
            stamp=float(data['stamp']);age=ros_now-stamp
            if not math.isfinite(stamp) or stamp<=self.last_stamp or not -.05<=age<=self.c.timeout_sec:return False
            continuous=stamp-self.last_stamp<=self.c.timeout_sec
            self.last_stamp=stamp
            if not data.get('depth_valid') or float(data.get('confidence',0))<self.c.min_confidence:
                self.count=0;self.straight_count=0
                # A geometry rejection is not a new valid observation. Retain
                # the last actual measurement only until its original deadline.
                if not self.fresh(now):
                    self.straight=False;self.data=dict(data,depth_valid=False)
                return False
            # Confirmation continuity follows source frames, not transport age
            # added a second time. Freshness still gates every speed policy.
            if not continuous:self.count=0;self.straight_count=0;self.straight=False
            self.data=data;self.time=now-max(0.,age);self.correction=0.
            if data.get('depth_corner_candidate') and data.get('distance') is not None:
                level=int(data['level']);direction=int(data['direction']);distance=float(data['distance'])
                if level not in (1,2,3) or direction not in (-1,1) or not math.isfinite(distance) or not 0<=distance<=self.c.horizon_m:raise ValueError('invalid corner')
                self.straight_count=0;self.last_measurement=distance
                pending=self.pending
                agrees=(pending is not None and direction==pending['direction'] and abs(level-pending['level'])<=1 and
                        abs(distance-max(0.,pending['distance']-(travel-pending['odom'])))<=self.c.association_distance_m and now-pending['time']<=self.c.timeout_sec)
                self.count=self.count+1 if agrees else 1
                self.pending=dict(direction=direction,level=level,distance=distance,odom=travel,time=now)
                self.state='CORNER_CANDIDATE' if self.latched is None else 'CORNER_CONFIRMED'
                if self.count>=self.c.corner_confirm_frames:
                    # Geometry confirmation must revoke straight authority even
                    # when odometry cannot establish a distance-tracking latch.
                    self.straight=False
                if self.count>=self.c.corner_confirm_frames and odom_fresh:
                    if self.latched is None:
                        self.straight=False
                        self.latched=dict(initial=distance,odom=travel,level=level,direction=direction,time=now,detected=now)
                    else:
                        old=self.latched;prediction=self.remaining(travel)
                        same=(direction==old['direction'] and abs(level-old['level'])<=1 and abs(distance-prediction)<=self.c.association_distance_m and (now-old['time']<=self.c.latch_timeout_sec or self.count>=self.c.corner_confirm_frames))
                        if same:
                            self.correction=self.c.correction_gain*(distance-prediction)
                            # Preserve original detection anchor for diagnostics.
                            old['offset']=old.get('offset',0.)+self.correction;old['level']=max(old['level'],level);old['time']=now
                        elif prediction<=.75 and lidar_straight and distance>=3.:
                            # The previous corner has been passed by odometry,
                            # and the local LiDAR corridor is straight; repeated
                            # fresh geometry may now bind the following corner.
                            self.latched=dict(initial=distance,odom=travel,level=level,direction=direction,time=now,detected=now)
                        # Farther corners cannot replace an unpassed latch.
                    self.state='CORNER_CONFIRMED'
            else:
                self.count=0;self.pending=None
                low=bool(data.get('straight_geometry'))
                # Exit thresholds confirm a new straight; the existing entry
                # thresholds retain an already confirmed straight in between.
                # This is hysteresis, not newly relaxed curve thresholds.
                segments=data.get('segments',[])
                stable_band=(self.straight and self.latched is None and lidar_straight and bool(segments) and
                    float(data.get('heading_change_deg',math.inf))<self.corner.mild_heading_enter_deg and
                    max(float(s['p90_curvature']) for s in segments)<self.corner.mild_curvature_enter)
                if not low and not stable_band:self.straight=False;self.state='DEPTH_GEOMETRY_UNCERTAIN'
                self.straight_count=self.straight_count+1 if low else 0
                if self.latched:
                    if odom_fresh and self.remaining(travel)<=.75 and lidar_straight and self.straight_count>=self.c.straight_confirm_frames:
                        self.latched=None;self.state='CORNER_EXIT';self.exit_time=now;self.straight=False
                elif self.straight_count>=self.c.straight_confirm_frames:
                    self.straight=True;self.state='STRAIGHT_CONFIRMED'
            return True
        except (KeyError,TypeError,ValueError,OverflowError):return False
    def policy(self,now,travel,odom_fresh,speed,speeds,decel,margin,reaction):
        fresh=self.fresh(now);remaining=self.remaining(travel)
        usable_latch=self.latched is not None and odom_fresh and now-self.latched['time']<=self.c.latch_timeout_sec
        level=self.latched['level'] if usable_latch else 0
        valid_straight=fresh and self.straight and self.latched is None
        unavailable=('NONE' if usable_latch or valid_straight else
            'SENSOR_TIMEOUT' if not fresh and bool(self.data.get('depth_valid')) else
            'INSUFFICIENT_SUPPORT' if not fresh else 'ODOM_TIMEOUT' if not odom_fresh and (self.latched or self.count>=self.c.corner_confirm_frames and self.data.get('depth_corner_candidate')) else
            'CORNER_TRACK_EXPIRED' if self.latched else 'CORNER_CANDIDATE' if self.data.get('depth_corner_candidate') else
            'STRAIGHT_CONFIRMATION_PENDING' if self.data.get('straight_geometry') else 'GEOMETRY_AMBIGUOUS')
        effective_state=('STRAIGHT_CONFIRMED' if valid_straight else 'CORNER_CONFIRMED' if usable_latch else
            'DEPTH_UNAVAILABLE' if not fresh else 'CORNER_TRACK_UNAVAILABLE' if self.latched else
            'CORNER_CANDIDATE' if self.data.get('depth_corner_candidate') else
            'STRAIGHT_CANDIDATE' if self.data.get('straight_geometry') else 'DEPTH_GEOMETRY_UNCERTAIN')
        corner_limit=speeds[level] if usable_latch else speeds[0] if valid_straight else self.c.fallback_speed_mps
        target=corner_limit
        # Existing deceleration and reaction/margin; coverage is measured, never polynomial reach.
        safety_margin=margin+max(0.,speed)*reaction
        support=float(self.data.get('depth_curvature_supported_distance_m',0.)) if fresh else 0.
        coverage_limit=math.sqrt(speeds[3]**2+2*decel*max(0.,support-safety_margin)) if fresh else self.c.fallback_speed_mps
        braking_limit=coverage_limit
        if usable_latch:
            braking_limit=min(braking_limit,math.sqrt(speeds[level]**2+2*decel*max(0.,remaining-safety_margin)))
            # Treat the policy's severity cap as a ceiling immediately after confirmation.
        if not fresh:braking_limit=min(braking_limit,self.c.fallback_speed_mps)
        required=max(0.,speed**2-speeds[level]**2)/(2*decel)
        return dict(depth_valid=fresh,depth_age=now-self.time if math.isfinite(self.time) else None,
            depth_corner_severity=SEVERITIES[level] if usable_latch or valid_straight else 'UNKNOWN',
            depth_corner_confirmed=usable_latch,depth_corner_candidate=bool(self.data.get('depth_corner_candidate')),
            depth_corner_direction=self.latched['direction'] if self.latched else self.data.get('direction',0),
            depth_corner_speed_limit=corner_limit,depth_remaining_corner_distance=remaining,
            depth_distance_to_corner_m=self.data.get('distance'),depth_confidence=self.data.get('confidence',0.),
            corner_latched=self.latched is not None,depth_state=effective_state,depth_unavailable_reason=unavailable,depth_level=level,
            depth_supported_distance_m=self.data.get('depth_supported_distance_m',0.),
            depth_curvature_supported_distance_m=self.data.get('depth_curvature_supported_distance_m',0.),
            corner_preview_braking_limit=min(speeds[0],braking_limit),required_braking_distance=required,
            braking_margin=safety_margin,braking_feasible=remaining is None or remaining>=required+safety_margin,
            depth_target_speed=min(target,braking_limit),segment_support=self.data.get('segments',[]),
            initial_depth_corner_distance=self.latched['initial'] if self.latched else None,
            odom_distance_at_detection=self.latched['odom'] if self.latched else None,
            current_cumulative_odom_distance=travel,travelled_since_detection=travel-self.latched['odom'] if self.latched else None,
            remaining_corner_distance=remaining,latest_depth_measurement=self.last_measurement,distance_correction=self.correction,
            corner_age=now-self.latched['detected'] if self.latched else None)
