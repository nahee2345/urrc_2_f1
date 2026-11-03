"""Sensor geometry and safety helpers shared with the controller."""
import math
import numpy as np
from team01_race.planning_road import RoadCorridor
from team01_race.race_strategy import RacingLine, LapLearner

class PlannerGeometry:
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
        self.static_wall_distance_m=None
        self.front_wall_speed_mps=None
        self._prev_wall_distance_m=None
        self._wall_static_frames=0
        self.pre_corner_active=False
        self.recenter_reason='none'
        from team01_race.rejoin import RejoinPlanner
        from team01_race.steering import SteeringLimiter
        from team01_race.yaw_control import YawFeedback
        from team01_race.reference_path import GradualReference
        from team01_race.reference_scheduler import ReferenceScheduler
        from team01_race.mode_recovery import CaptureMode
        self.rejoin=RejoinPlanner(cfg)
        self.steer_limiter=SteeringLimiter(cfg)
        self.yaw_feedback=YawFeedback(cfg)
        self.reference_path=GradualReference(cfg)
        self.reference_scheduler=ReferenceScheduler(self.reference_path,cfg)
        self.reference_geometry=None
        self.capture_mode=CaptureMode()
        self.map_ready=False

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

    def _front_wall(self, points, measured_speed, dt):
        """Detect a wide, stationary front LiDAR wall; not a bend by itself."""
        cloud=np.asarray(points,dtype=float).reshape((-1,2))
        self.static_wall_distance_m=None
        self.front_wall_speed_mps=None
        if len(cloud)<10:
            self._prev_wall_distance_m=None
            self._wall_static_frames=0
            return False
        front=cloud[(cloud[:,0]>1.0)&(cloud[:,0]<10.)&(abs(cloud[:,1])<1.8)]
        distances=[]
        for x in np.arange(1.0,10.,.35):
            band=front[abs(front[:,0]-x)<.22]
            if len(band)>=7 and np.ptp(band[:,1])>=.65:
                distances.append(float(np.median(band[:,0])))
        if not distances:
            self._prev_wall_distance_m=None
            self._wall_static_frames=0
            return False
        distance=min(distances)
        self.static_wall_distance_m=distance
        if (self._prev_wall_distance_m is not None and .015<dt<.25 and
                measured_speed is not None):
            world_speed=measured_speed+(distance-self._prev_wall_distance_m)/dt
            if abs(world_speed)<1.5:
                self._wall_static_frames+=1
            else:
                self._wall_static_frames=0
            self.front_wall_speed_mps=world_speed
        self._prev_wall_distance_m=distance
        return self._wall_static_frames>=2

    def _lane_obstacle_ahead(self, points, corridor, lane_offset, horizon):
        if not corridor.valid:
            return False
        cloud=np.asarray(points,dtype=float).reshape((-1,2))
        if len(cloud)==0:return False
        cloud=cloud[(cloud[:,0]>.9)&(cloud[:,0]<horizon)]
        if len(cloud)==0:return False
        xs=np.clip(cloud[:,0],corridor.centers[0,0],corridor.centers[-1,0])
        center=np.interp(xs,corridor.centers[:,0],corridor.centers[:,1])+lane_offset
        return bool(np.any(abs(cloud[:,1]-center)<self.cfg['path_half_width_m']+.18))

