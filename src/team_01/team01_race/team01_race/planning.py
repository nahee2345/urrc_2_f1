"""Conservative F1-like local trajectory sampling; no external track state."""
from dataclasses import dataclass
import math
import numpy as np
from team01_race.race_strategy import RacingLine, LapLearner
from team01_race.curved_path import adaptive_pursuit_candidate, preview_curvature


@dataclass
class Motion:
    speed: float
    steering: float  # race contract: +right, -left, radians
    mode: str
    reason: str


class TrafficGate:
    """Arm only on real red-light observations; launch on stable all-off.

    A partial camera field of view is permitted: 4 visible red LEDs may be
    the peak of a physical 5-LED sequence. Count-only mode is for unit tests;
    runtime mode also checks the change in intensity from a dark baseline.
    Before any light is observed, a blank image can never authorize movement.
    """
    def __init__(self, required=4, on_frames=2, off_frames=3,
                 min_sequence_s=3.0):
        self.required=int(required)
        self.on_frames=int(on_frames)
        self.off_frames=int(off_frames)
        self.min_sequence_s=float(min_sequence_s)
        self.active_frames=0
        self.dark_frames=0
        self.saw_full_red=False
        self.lights_out=False
        self.frame_count=0
        self.baseline_samples=[]
        self.baseline=None
        self.peak_strength=0.0
        self.first_bright_time=None
        self.last_blobs=0
        self.last_strength=0.0
        self.first_bright_blobs=0
        self.armed_at=None
        self.maximum_blobs=0
        self.status='WAIT_RED'

    def observe(self, blobs, strength=None, timestamp=None):
        import time
        t=time.monotonic() if timestamp is None else float(timestamp)
        blobs=max(0,int(blobs))
        self.last_blobs=blobs
        self.maximum_blobs=max(blobs,self.maximum_blobs)
        if self.lights_out:
            return True
        if strength is None:
            # Test/diagnostic mode: no illumination measurements available.
            if blobs>=self.required:
                self.active_frames+=1
                self.dark_frames=0
                if self.active_frames>=self.on_frames:
                    self.saw_full_red=True
                    self.status='ARMED'
            elif self.saw_full_red and blobs==0:
                self.dark_frames+=1
                self.status='VERIFY_OUT'
                if self.dark_frames>=self.off_frames:
                    self.lights_out=True
                    self.status='LIGHTS_OUT'
            else:
                if not self.saw_full_red:self.active_frames=0
                self.dark_frames=0
            return self.lights_out

        strength=max(0.,float(strength))
        if not math.isfinite(strength):return False
        self.last_strength=strength
        self.frame_count+=1
        if self.baseline is None:
            # The launcher holds the lamps off until all ROS streams have
            # initialized. Median suppresses an occasional noisy first frame.
            self.baseline_samples.append(strength)
            if len(self.baseline_samples)>=8:
                self.baseline=float(np.median(self.baseline_samples))
            return False
        excess=strength-self.baseline
        # Emissive overlays have a substantial energy difference relative
        # to the unlit gantry. 4/5 visible lamps are sufficient to arm.
        delta_threshold=max(120.,self.baseline*.18)
        illuminated=excess>delta_threshold
        if illuminated:
            if self.first_bright_time is None:
                self.first_bright_time=t
                self.first_bright_blobs=blobs
            self.peak_strength=max(self.peak_strength,strength)
        if not self.saw_full_red:
            if illuminated and blobs>=self.required:
                self.active_frames+=1
                if self.active_frames>=self.on_frames:
                    self.saw_full_red=True
                    self.armed_at=t
                    self.status='ARMED'
            else:
                self.active_frames=0
            return False
        # No movement before the physical red sequence can have ended.
        # Total time measured from *first visible red* is >=3.0 s; thus
        # a temporary black frame after lamp #4 cannot trigger departure.
        if self.first_bright_time is None:
            return False
        # If the first three lamps were outside the FOV, the first clearly
        # detected event may already be 4 reds. The sequence is still valid
        # if four reds were held, then disappear stably after an arm delay.
        elapsed=t-self.first_bright_time
        last_four_only=(self.first_bright_blobs>=self.required and
                        elapsed>=.65 and self.armed_at is not None and
                        t-self.armed_at>=.45)
        if elapsed<self.min_sequence_s-1e-6 and not last_four_only:
            return False
        observed_gain=max(0.,self.peak_strength-self.baseline)
        returned_to_dark=strength<=self.baseline+max(delta_threshold*.5,observed_gain*.30)
        # Use count=0 when visible lamps vanish. The fallback energy drop
        # handles permanent red lens appearance after emissive model removal.
        all_off=(blobs==0 or returned_to_dark)
        if all_off:
            self.dark_frames+=1
            self.status='VERIFY_OUT'
            if self.dark_frames>=self.off_frames:
                self.lights_out=True
                self.status='LIGHTS_OUT'
        else:
            self.dark_frames=0
            self.status='ARMED'
        return self.lights_out


@dataclass
class RoadCorridor:
    centers: np.ndarray
    half_width: float
    valid: bool
    heading_deg: float
    bounds: np.ndarray = None   # optional local (x, right, left) from RGB+Depth


def estimate_road_corridor(points, *, min_width=.65, max_width=5.5):
    """Extract left/right guardrail clearance from local 2-D obstacles.

    No global track/map coordinates are used. This is a geometric corridor,
    not semantic road-surface segmentation: if walls are invisible it is
    explicitly invalid. Vehicles inside the roadway are excluded from the
    rail bands by multi-bin continuity checks.
    """
    pts=np.asarray(points,dtype=np.float64).reshape(-1,2)
    pts=pts[np.all(np.isfinite(pts),axis=1)]
    centers=[]
    for x in np.arange(.75,7.0,.55):
        p=pts[np.abs(pts[:,0]-x)<.40]
        l=p[(p[:,1]>.32)&(p[:,1]<4.0),1]
        r=p[(p[:,1]<-.32)&(p[:,1]>-4.0),1]
        if len(l)<2 or len(r)<2:
            continue
        # The inside faces of the two walls are near y=0.
        left=float(np.percentile(l,15));right=float(np.percentile(r,85))
        width=left-right
        if min_width<=width<=max_width:
            centers.append((x,.5*(left+right),width))
    if len(centers)<3:
        return RoadCorridor(np.empty((0,2)),0.,False,0.)
    a=np.asarray(centers)
    # Require continuity. A nearby opponent is not a road boundary.
    widths=a[:,2]
    median=float(np.median(widths))
    keep=np.abs(widths-median)<max(.35,median*.35)
    a=a[keep]
    if len(a)<3 or (a[-1,0]-a[0,0])<1.2:
        return RoadCorridor(np.empty((0,2)),0.,False,0.)
    centers=a[:,:2]
    slopes=np.diff(centers[:,1])/np.maximum(.1,np.diff(centers[:,0]))
    heading=math.degrees(math.atan(float(np.median(slopes))))
    return RoadCorridor(centers,float(np.median(a[:,2])/2),True,heading)


def fuse_verified_corridors(vision, lidar, *, max_center_error_m=.30,
                            max_width_error_m=.65):
    """Extend short aligned RGB/depth asphalt with independently sensed rails.

    NEVER read a preloaded track, odometry pose, or map. Corridor extension is
    permitted only where observed LiDAR and visual centres/widths agree across
    a useful overlapping interval; fused boundary is deliberately shrunken
    6 cm beyond vision to account for uncertain rail distance.
    """
    if vision is None or lidar is None or not vision.valid or not lidar.valid:
        return None
    if vision.bounds is None or len(vision.bounds)<4 or len(lidar.centers)<4:
        return None
    v=np.asarray(vision.bounds,dtype=float)
    lc=np.asarray(lidar.centers,dtype=float)
    lo=max(float(v[0,0]),float(lc[0,0]))
    hi=min(float(v[-1,0]),float(lc[-1,0]))
    if hi-lo<.75 or float(lc[-1,0])-float(v[-1,0])<.75:
        return None
    probes=np.linspace(lo+.10,hi-.10,5)
    vr=np.interp(probes,v[:,0],v[:,1]);vl=np.interp(probes,v[:,0],v[:,2])
    vc=.5*(vr+vl)
    ll=np.interp(probes,lc[:,0],lc[:,1])
    if (np.max(np.abs(vc-ll))>max_center_error_m or
            np.max(np.abs(vl-vr-2*float(lidar.half_width)))>max_width_error_m):
        return None
    extra=lc[lc[:,0]>v[-1,0]+.08]
    extra=extra[extra[:,0]<=min(11.,v[-1,0]+8.)]
    if len(extra)<2:return None
    half=max(.30,float(lidar.half_width)-.06)
    additions=np.column_stack((extra[:,0],extra[:,1]-half,extra[:,1]+half))
    fused=np.vstack((v,additions))
    if np.any(np.diff(fused[:,0])<=0):return None
    centers=np.column_stack((fused[:,0],(fused[:,1]+fused[:,2])*.5))
    slope=np.diff(centers[:,1])/np.maximum(.05,np.diff(centers[:,0]))
    heading=math.degrees(math.atan(float(np.median(slope))))
    return RoadCorridor(centers,float(np.median((fused[:,2]-fused[:,1])/2)),
                        True,heading,fused)


class SafetyPlanner:
    def __init__(self, cfg):
        self.cfg=cfg
        self.speed=0.0
        self.prev_steer=0.0
        self.last_lead=None
        self.lead_speed=None
        self.overtake_side=0  # 0 idle, +1 left, -1 right
        self.overtake_hold=0.0
        self.defend=False
        self.road=RoadCorridor(np.empty((0,2)),0.,False,0.)
        self.road_source='none'
        self.cruise_side=1
        self.cruise_elapsed=0.
        self.overtake_start_distance_m=None
        self.planned_pass_distance_m=None
        self.lead_distance=None
        self.last_overtake_seen=0.
        self.target_lane_offset=0.
        self.racing_line=RacingLine(cfg)
        self.lap_learner=LapLearner(cfg)
        self.last_mode="STOP"
        self.preview_curve=0.
        self.planner_type="constant_arc"
        self.measured_speed=None
        self.near_edge_m=None  # observed curb clearance from base_link
        self.edge_speed_cap_mps=None
        self.edge_alert='NONE'
        self.startup_steer_limit_active=False
        self.startup_right_error_m=None

    @staticmethod
    def _visual_corridor(bounds):
        if bounds is None:
            return None
        a=np.asarray(bounds,dtype=float).reshape(-1,3)
        a=a[np.all(np.isfinite(a),axis=1)]
        if len(a)<4:
            return None
        a=a[np.argsort(a[:,0])]
        if a[-1,0]-a[0,0]<1.0 or np.any(a[:,2]-a[:,1]<.5):
            return None
        centers=np.column_stack((a[:,0],.5*(a[:,1]+a[:,2])))
        delta=np.diff(centers,axis=0)
        slope=np.median(delta[:,1]/np.maximum(.05,delta[:,0]))
        return RoadCorridor(centers,float(np.median((a[:,2]-a[:,1])/2)),
                            True,math.degrees(math.atan(float(slope))),a)

    def _within_road(self,steer,horizon):
        if not self.road.valid:
            return True,0.
        ds,arc=self._path_samples(steer,min(5.,horizon))
        centers=self.road.centers
        applicable=(arc[:,0]>=centers[0,0])&(arc[:,0]<=centers[-1,0])
        if np.count_nonzero(applicable)<3:
            return False,1e6
        x,y=arc[applicable,0],arc[applicable,1]
        if self.road.bounds is not None:
            right=np.interp(x,self.road.bounds[:,0],self.road.bounds[:,1])
            left=np.interp(x,self.road.bounds[:,0],self.road.bounds[:,2])
            margin=self.cfg['path_half_width_m']+.055
            over=np.maximum(np.maximum(right+margin-y, y-(left-margin)),0.)
            return bool(np.max(over)<.001),float(np.mean(over))
        expected=np.interp(x,centers[:,0],centers[:,1])
        deviation=np.abs(y-expected)
        avail=self.road.half_width-self.cfg['path_half_width_m']-.08
        return bool(avail>.05 and np.all(deviation<=avail)),float(np.mean(deviation))

    def _path_samples(self, angle, horizon=8.0):
        wheel_base=self.cfg['wheel_base_m']
        curvature=math.tan(angle)/wheel_base
        # Restrict predicted path to a single forward arc, not loops.
        if abs(curvature) > 1e-6:
            horizon=min(horizon,2.1/abs(curvature))
        dist=np.arange(.25,max(.26,horizon)+.01,.20)
        if abs(curvature)<1e-6:
            x=dist
            y=np.zeros_like(dist)
        else:
            x=np.sin(curvature*dist)/curvature
            y=(1-np.cos(curvature*dist))/curvature
        return dist, np.stack((x,y),axis=1)

    def clearance(self, obstacles, angle, horizon=8.0):
        """Distance to first projected collision, including full vehicle width + margin."""
        if len(obstacles)==0:return horizon
        ds, xy=self._path_samples(angle, horizon)
        r=self.cfg['path_half_width_m']
        # Locally choose every sample, not a straight ray, to avoid wall clipping.
        for i in range(len(ds)):
            candidates=obstacles[(np.abs(obstacles[:,0]-xy[i,0]) < r+.2)
                                 & (np.abs(obstacles[:,1]-xy[i,1]) < r+.2)]
            if len(candidates) and np.any(np.sum((candidates-xy[i])**2,axis=1)<r*r):
                return float(ds[i])
        return min(horizon,float(ds[-1]))

    def _lead(self, depth_obstacles, dt):
        # Candidate moving car in own corridor. A stationary wall can look similar;
        # only label as possible lead, never use estimate as a clearance guarantee.
        if len(depth_obstacles)==0:
            self.last_lead=None
            self.lead_speed=None
            return None
        pts=depth_obstacles
        m=(pts[:,0]>.5)&(pts[:,0]<9)&(np.abs(pts[:,1])<.29)
        # On bends, the inner curb sweeps across the vehicle's forward
        # camera centerline. A curb is NOT a lead car: exclude any depth
        # point adjacent to the measured left/right road boundary. It remains
        # in the *obstacle cloud* and must still pass collision checks.
        if self.road.valid and len(self.road.centers)>2:
            xs=pts[:,0]
            if self.road.bounds is not None:
                b=np.asarray(self.road.bounds)
                right=np.interp(xs,b[:,0],b[:,1])
                left=np.interp(xs,b[:,0],b[:,2])
            else:
                c=np.interp(xs,self.road.centers[:,0],self.road.centers[:,1])
                right=c-self.road.half_width
                left=c+self.road.half_width
            half=self.cfg['path_half_width_m']+.06
            m &= (xs>=self.road.centers[0,0]) & (xs<=self.road.centers[-1,0])
            m &= (pts[:,1]>right+half) & (pts[:,1]<left-half)
        d=float(np.min(pts[m,0])) if np.any(m) else None
        if d is None:
            self.last_lead=None;self.lead_speed=None
            return None
        if self.last_lead is not None and .015<dt<.45 and abs(d-self.last_lead)<1.8:
            est=max(0.,min(15., self.speed+(d-self.last_lead)/dt))
            self.lead_speed=est if self.lead_speed is None else .65*self.lead_speed+.35*est
        self.last_lead=d
        return d

    def step(self, depth_obstacles, lidar_obstacles, rear_gap, dt, enabled,
             depth_valid, front_lidar_valid=True, road_bounds=None, actual_speed=None):
        if not enabled or not depth_valid or (not front_lidar_valid and
             not self.cfg.get('unsafe_allow_without_front_lidar',False)):
            self.speed=0.
            self.prev_steer=0.
            self.last_mode='STOP'
            return Motion(0.,0.,'STOP','gate or stale/missing forward sensor')
        dt=max(.01,min(dt,.25))
        depth_obstacles=np.asarray(depth_obstacles).reshape((-1,2))
        lidar_obstacles=np.asarray(lidar_obstacles).reshape((-1,2))
        pts=np.vstack((depth_obstacles,lidar_obstacles))
        # Reject self-returns near bumper and points far behind front axle.
        pts=pts[(pts[:,0]>.28) & (pts[:,0]<12.) & (np.abs(pts[:,1])<5.)]
        lidar_road=estimate_road_corridor(lidar_obstacles)
        vision_road=self._visual_corridor(road_bounds)
        # F1TENTH-style adaptive local lookahead: RGB/Depth proves nearby
        # asphalt; continuous, cross-checked LiDAR barriers extend the *same*
        # drivable corridor for high-speed braking and corner anticipation.
        # A disagreement is NOT fused into a permissive drivable polygon.
        fused=fuse_verified_corridors(vision_road, lidar_road)
        self.road=fused if fused is not None else (vision_road if vision_road is not None else lidar_road)
        self.road_source=('rgb_depth+lidar' if fused is not None else
                          'rgb_depth' if vision_road is not None else
                          'lidar' if lidar_road.valid else 'none')
        # Optional Gazebo wheel odometry is a diagnostic and corrects the
        # commanded-distance dead reckoning; never use world pose or map XY.
        self.measured_speed=(float(actual_speed) if actual_speed is not None and
                             math.isfinite(float(actual_speed)) and float(actual_speed)>=0 else None)
        observed=self.measured_speed if self.measured_speed is not None else self.speed
        self.lap_learner.integrate(observed,self.prev_steer,dt,
                                   self.cfg['wheel_base_m'],self.road,self.last_mode)
        self.racing_line.update(self.road,observed,dt,self.overtake_side,
            exploration_lap=(self.cfg.get('disable_first_lap_racing_line',False)
                             and self.lap_learner.lap==1))
        info=self.racing_line.last_distances
        # A visible road can still have the CAR outside its boundaries. v7
        # checked future arcs, but not the immediate base_link curb clearance.
        # Begin slowing while there is still room to make a correction.
        self.near_edge_m=(min(info.left_m,info.right_m) if info.valid else None)
        self.edge_speed_cap_mps=None
        self.edge_alert='NONE'
        if self.near_edge_m is not None:
            margin=float(self.cfg['path_half_width_m'])+.055
            caution=margin+float(self.cfg.get('edge_caution_extra_m',.20))
            if self.near_edge_m <= margin:
                self.edge_alert='CRITICAL'
                # No safe lateral envelope is confirmed; never accelerate.
                self.speed=0.
                self.prev_steer=0.
                self.last_mode='BRAKE'
                return Motion(0.,0.,'BRAKE','base_link too close to curb / off road')
            if self.near_edge_m < caution:
                self.edge_alert='CAUTION'
                fraction=max(0.,min(1.,(self.near_edge_m-margin)/(caution-margin)))
                self.edge_speed_cap_mps=(float(self.cfg.get('edge_caution_min_speed_mps',.45))
                    + fraction*float(self.cfg.get('edge_caution_extra_speed_mps',1.15)))
        # Only fallback to LiDAR when BOTH continuous roadside barriers are
        # measured; do not treat missing/ambiguous image as free asphalt.
        verified_fallback=(vision_road is None and lidar_road.valid and
            self.cfg.get('permit_lidar_verified_fallback',False))
        if self.cfg.get('require_visual_road',False) and vision_road is None and not verified_fallback:
            self.speed=0.;self.prev_steer=0.
            self.last_mode='STOP'
            return Motion(0.,0.,'STOP','no verified black asphalt + depth corridor')
        # Prior v5 alternated left/right every 12 s. First-lap lane holding
        # must instead follow measured center and corner geometry.
        horizon=min(10.0,max(3.6,2.8+self.speed*.95))
        lead=self._lead(depth_obstacles,dt)
        self.lead_distance=lead
        self.overtake_start_distance_m=None
        self.planned_pass_distance_m=None
        safety_side_clear=(rear_gap is None or rear_gap>2.5)
        passing_enabled=bool(self.cfg.get('enable_overtake',True))
        if not passing_enabled:
            self.overtake_side=0
            self.overtake_hold=0.
        self.target_lane_offset=self.racing_line.offset_m
        if self.road.valid:
            width=self.road.half_width
            max_offset=max(0.,width-self.cfg['path_half_width_m']-.11)
            if passing_enabled and lead is not None and self.lead_speed is not None:
                closing=max(0.,self.speed-self.lead_speed)
                safe_deceleration=max(.5,self.cfg['max_braking_mps2'])
                # TTC / braking-envelope aware initiation point. All dimensions m.
                trigger=max(2.8,self.speed*self.cfg['reaction_delay_s']+
                            closing*closing/(2*safe_deceleration)+2.0)
                self.overtake_start_distance_m=trigger
                # Distance car must travel to establish an ahead-of-lead gap.
                gap_to_clear=lead+.55+1.2
                desired_speed=self.cfg['straight_speed_mps']*1.15
                rel=max(.05,desired_speed-self.lead_speed)
                self.planned_pass_distance_m=desired_speed*gap_to_clear/rel
            if (passing_enabled and not self.racing_line.grid and self.overtake_side==0 and lead is not None and
                1.3<lead<max(4.0,self.overtake_start_distance_m or 4.) and
                safety_side_clear and max_offset>=.42 and
                (self.lead_speed is None or self.lead_speed<self.cfg['straight_speed_mps']*1.15-.35)):
                # Select the side with more verified road width and fewer returns.
                options=[]
                xlook=min(float(self.road.centers[-1,0]),lead+.4)
                if xlook>=self.road.centers[0,0]:
                    mid=float(np.interp(xlook,self.road.centers[:,0],self.road.centers[:,1]))
                    for side in (1,-1):
                        if self.road.bounds is not None:
                            bound=(np.interp(xlook,self.road.bounds[:,0],self.road.bounds[:,2]) if side>0 else
                                   np.interp(xlook,self.road.bounds[:,0],self.road.bounds[:,1]))
                            space=side*(bound-mid)
                        else:
                            space=width
                        # A pass is forbidden when a side cannot fit the whole car.
                        safe_space=space-self.cfg['path_half_width_m']-.12
                        if safe_space<.42:continue
                        nearby=pts[(pts[:,0]>.8)&(pts[:,0]<min(lead+2,8.)) &
                                   (pts[:,1]*side>.28)]
                        blocking=int(np.count_nonzero(nearby))
                        options.append((safe_space-.015*blocking+.01*(side==self.cruise_side),side))
                if options:
                    self.overtake_side=max(options)[1]
                    self.overtake_hold=0.
            if self.overtake_side:
                self.target_lane_offset=self.overtake_side*min(.62,max_offset*.93)
        # Smooth the commanded route through a local look-ahead point.
        desired=0.
        if self.road.valid:
            xgoal=float(np.clip(1.2+min(1.4,self.speed*.24),
                                self.road.centers[0,0],self.road.centers[-1,0]))
            ycenter=float(np.interp(xgoal,self.road.centers[:,0],self.road.centers[:,1]))
            ygoal=ycenter+self.target_lane_offset
            if self.racing_line.grid:
                # v10: The start straight is NOT a distant-centerline tracking
                # problem.  Matching the right-wall gap means ZERO steer.  v9
                # evaluated the right boundary 2-3m ahead and incorrectly
                # steered toward the next bend while still in the grid phase.
                # Compare the SAME near-field curb station for reference and
                # current gap, so distant curvature does not bias the heading.
                ref=self.racing_line.right_reference_m
                self.startup_right_error_m=None
                if ref is None or not info.valid:
                    desired=0.
                else:
                    error=float(ref-info.right_m) # + : too near right -> left
                    self.startup_right_error_m=error
                    deadband=float(self.cfg.get('startup_gap_deadband_m',.07))
                    if abs(error)<deadband:
                        desired=0.
                    else:
                        effective=math.copysign(abs(error)-deadband,error)
                        # Low-gain right-curb distance control, not pursuit of
                        # an arbitrary far lookahead target.
                        desired=math.radians(float(np.clip(
                            effective*float(self.cfg.get('startup_gap_kp_deg_per_m',2.0)),
                            -float(self.cfg.get('startup_max_steer_deg',.8)),
                            float(self.cfg.get('startup_max_steer_deg',.8)))))
            else:
                desired=math.atan2(2*self.cfg['wheel_base_m']*ygoal,
                                   xgoal*xgoal+ygoal*ygoal)
                desired=float(np.clip(desired,-math.radians(self.cfg['max_center_steer_deg']),
                                               math.radians(self.cfg['max_center_steer_deg'])))
        angles=list(np.linspace(-self.cfg['max_center_steer_deg'],
                                self.cfg['max_center_steer_deg'],23)*math.pi/180.)
        if self.road.valid:
            # Pure-pursuit candidate adds sub-degree steering on gentle bends;
            # a coarse fixed steering grid otherwise treats them as straight.
            path=self.road.centers
            look_x=float(np.clip(min(path[-1,0],3.0),path[0,0],3.0))
            look_y=float(np.interp(look_x,path[:,0],path[:,1]))
            pure=math.atan2(2*self.cfg['wheel_base_m']*look_y,
                            look_x*look_x+look_y*look_y)
            steering_max=math.radians(self.cfg['max_center_steer_deg'])
            for delta in (-.025,0.,.025):
                angles.append(float(np.clip(pure+delta,-steering_max,steering_max)))
        self.preview_curve=preview_curvature(self.road)
        startup_hold=(self.racing_line.grid and self.cfg.get('startup_right_hold_s',0.)>0)
        startup_limit=math.radians(float(self.cfg.get('startup_max_steer_deg',3.)))
        self.startup_steer_limit_active=bool(startup_hold)
        if startup_hold:
            horizon=min(horizon,float(self.cfg.get('startup_verified_horizon_m',2.0)))
            # Do not offer a broad sample of steering candidates.  On the
            # straight, clearance maximizing may pick a wallward turn even
            # though the requested right curb gap is already perfect.
            desired=float(np.clip(desired,-startup_limit,startup_limit))
            angles=[desired]
        records=[]
        for steer in angles:
            ok,err=self._within_road(steer,horizon)
            if not ok:continue
            free=self.clearance(pts,steer,horizon)
            # Follow selected lateral racing line, not permanent track centre.
            score=(free-1.0*abs(steer-desired)-.25*abs(steer-self.prev_steer)
                   -.12*abs(steer))
            records.append((score,free,steer))
        # A constant-steering arc cannot negotiate a changing-curvature bend.
        # Generate local trajectories by *recalculating* pure pursuit along the
        # observed RGB+Depth road at every predicted 14cm of progress.
        adaptive=[]
        if self.road.valid and not startup_hold:
            for shift in (0., -.11, +.11):
                c=adaptive_pursuit_candidate(
                    self.road, pts, self.cfg,
                    self.measured_speed if self.measured_speed is not None else self.speed,
                    desired_offset=self.target_lane_offset,
                    relative_offset=shift,
                    horizon=min(5.0,horizon))
                if c is not None and (not startup_hold or
                                      abs(c.steer_rad)<=startup_limit):
                    adaptive.append(c)
        use_adaptive=bool(adaptive) and (
            not records or abs(self.road.heading_deg)>=3. or
            self.preview_curve>=.04 or self.racing_line.corner)
        if use_adaptive:
            choice=max(adaptive,key=lambda c:c.score)
            free,steer=choice.first_collision_m,choice.steer_rad
            self.planner_type='adaptive_pursuit'
        else:
            self.planner_type='constant_arc'
            if not records:
                # Neither adaptive path nor constant arc fits the visible road.
                self.speed=0.
                self.prev_steer=0.
                self.last_mode='BRAKE'
                return Motion(0.,0.,'BRAKE','no verified road-safe candidate')
            records.sort(reverse=True)
            _,free,steer=records[0]
            viable=[r for r in records if r[1]>=max(2.0,free-1.25)]
            if viable:
                _,free,steer=min(viable,key=lambda r:abs(r[2]-desired))
        if (self.racing_line.grid and not self.racing_line.last_distances.valid
                and abs(steer)>math.radians(4.)):
            self.speed=0.
            self.prev_steer=0.
            self.last_mode='STOP'
            return Motion(0.,0.,'STOP','grid straight path is not verified safe')
        # Clamp initial steering, with a stop if the clamped arc would not fit
        # the curb corridor. No unchecked steering override after validation.
        if startup_hold and abs(steer)>startup_limit+1e-8:
            self.speed=0.;self.prev_steer=0.;self.last_mode='BRAKE'
            return Motion(0.,0.,'BRAKE','startup steering exceeds bounded safe corridor')
        actual_v=max(self.speed,self.measured_speed or 0.)
        stopping=(actual_v*self.cfg['reaction_delay_s']+
                  actual_v**2/(2*self.cfg['max_braking_mps2']))
        hard_gap=free-self.cfg['emergency_margin_m']
        if hard_gap<=.18 or (self.speed>0 and free<stopping+.15):
            self.speed=max(0.,self.speed-self.cfg['max_braking_mps2']*dt)
            if hard_gap<=.18:self.speed=0.
            self.last_mode='BRAKE'
            return Motion(self.speed,-steer,'BRAKE','obstacle in stopping envelope')
        if self.overtake_side:
            self.overtake_hold+=dt
            # Do not abruptly return to centre after the opponent disappears.
            if lead is not None:self.last_overtake_seen=0.
            else:self.last_overtake_seen+=dt
            if self.last_overtake_seen>2.0 and (rear_gap is None or rear_gap>4.):
                self.cruise_side=self.overtake_side
                self.overtake_side=0
                self.overtake_hold=0.
                self.last_overtake_seen=0.
            elif self.overtake_hold>10.:
                self.overtake_side=0
                self.overtake_hold=0.
        # Defensive strategy is *only* maintaining speed and a steady safe line.
        # Never swerve to block another car, especially in an adjacent lane.
        self.defend=(passing_enabled and self.overtake_side==0 and rear_gap is not None and
                     rear_gap >= 0.0 and rear_gap<self.cfg['rear_defend_min_gap_m'])
        lap_boost=self.lap_learner.speed_multiplier(self.road.valid,self.last_mode,
                                                    self.speed>=0.)
        # Explicit phase goal: first lap explores, second aims higher ONLY
        # after on-board confirmation of lap closure. These are target speeds,
        # never bypass braking-distance / boundary / curvature limits.
        lap_target=(float(self.cfg.get('second_lap_straight_speed_mps',12.))
                    if self.lap_learner.lap>1 else
                    float(self.cfg['straight_speed_mps']))
        if not self.cfg.get('enable_overtake',True):
            lap_boost=1.0  # no extra speed via opponent logic
        v=lap_target*lap_boost*(1.15 if self.overtake_side else 1.)
        if self.racing_line.grid:
            # Start along the right curb, low speed and very small steering.
            v=min(v,self.cfg.get('grid_discovery_speed_mps',.65))
        if self.edge_speed_cap_mps is not None:
            v=min(v,self.edge_speed_cap_mps)
        if verified_fallback:
            # Guardrails are measured, but asphalt segmentation failed.
            v=min(v,self.cfg.get('unmapped_speed_mps',.75))
        if not self.road.valid:
            # Depth plane != proof of an asphalt lane. Move only at a
            # cautious exploration speed where unbounded open space occurs.
            v=min(v,self.cfg.get('unmapped_speed_mps',.75))
        if (abs(steer)>=math.radians(self.cfg['corner_threshold_deg'])
                or self.racing_line.corner or self.lap_learner.learned_corner_ahead()
                or self.preview_curve >= self.cfg.get('preview_curvature_threshold',.08)):
            v=min(v,lap_target*lap_boost*
                  self.cfg['corner_speed_factor'])
        curvature=max(abs(math.tan(steer))/self.cfg['wheel_base_m'],
                      self.preview_curve)
        if curvature>1e-5:
            v=min(v,math.sqrt(self.cfg['max_lateral_accel_mps2']/curvature))
        # Speed bound from nearest forward collision and sensor/actuator delay.
        # Do not apply a *constant-arc* clearance estimate to an adaptive
        # multi-steer candidate: it can see a curb where the predicted path turns.
        if self.planner_type != 'adaptive_pursuit':
            free=self.clearance(pts,steer,horizon)
        d=max(0.,free-self.cfg['emergency_margin_m'])
        decel=self.cfg['max_braking_mps2']
        delay=self.cfg['reaction_delay_s']
        v_safe=max(0.,math.sqrt((decel*delay)**2+2*decel*d)-decel*delay)
        v=min(v,v_safe)
        if self.road.valid:
            # Beyond the last observed paired guardrail, drivable road is
            # unknown. Limit speed to the distance we can verify.
            verified=max(0.,float(self.road.centers[-1,0])-.6)
            if verified<.6:
                v=0.
            else:
                v=min(v,max(0.,math.sqrt((decel*delay)**2 +
                                  2*decel*verified)-decel*delay))
        if lead is not None and self.overtake_side==0:
            headway=max(.9,self.speed*.65)
            if lead < headway+1.0:
                v=min(v,max(0.,(lead-1.0)/.65))
            if self.lead_speed is not None and lead<4.5:
                v=min(v,self.lead_speed+.3)
        # Passing boost is always bounded by lateral acceleration, distance,
        # stopping envelope, and the commanded acceleration rate above.
        if self.defend:
            # Hold established racing line; following vehicle does not authorize
            # closing an unsafe gap or obstructing the adjacent passing car.
            steer=.65*self.prev_steer+.35*steer
        # Rate-limit acceleration; allow emergency braking at configured rate.
        v=np.clip(v,self.speed-decel*dt,self.speed+self.cfg['accel_mps2']*dt)
        self.speed=float(max(0.,v))
        self.prev_steer=float(steer)
        mode=('OVERTAKE' if self.overtake_side else 'GRID' if self.racing_line.grid
              else 'CORNER' if self.racing_line.corner or
                   abs(steer)>=math.radians(self.cfg['corner_threshold_deg'])
              else 'DEFEND' if self.defend else 'STRAIGHT')
        self.last_mode=mode
        return Motion(self.speed,-float(steer),mode,
                      'sensor-bounded '+self.planner_type)
