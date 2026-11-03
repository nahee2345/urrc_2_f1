"""Local curb-centric first lap, bounded out-in-out and cautious lap learning.

Uses only on-vehicle sensing and *command-integrated* distance/yaw (not world pose).
Distance / lap verification is approximate, never competition timing authority.
"""
from dataclasses import dataclass
import math
import numpy as np


@dataclass
class RoadDistances:
    valid: bool
    left_m: float = 0.0
    right_m: float = 0.0
    midpoint_m: float = 0.0
    width_m: float = 0.0
    ahead_m: float = 0.0


def base_link_road_distances(corridor, at_x=0.75):
    """Distances from base_link center to the left/right road boundaries.

    Returned +left/-right in base frame. Only extrapolate near the front bumper;
    reject corridor estimates that start too far ahead to measure the vehicle.
    """
    if not corridor.valid or len(corridor.centers)<3:
        return RoadDistances(False)
    c=np.asarray(corridor.centers)
    if c[0,0]>1.25 or c[-1,0]<1.5:
        return RoadDistances(False)
    x=min(max(at_x,float(c[0,0])),float(c[-1,0]))
    center=float(np.interp(x,c[:,0],c[:,1]))
    if corridor.bounds is not None:
        b=np.asarray(corridor.bounds)
        right=float(np.interp(x,b[:,0],b[:,1]))
        left=float(np.interp(x,b[:,0],b[:,2]))
    else:
        left=center+float(corridor.half_width)
        right=center-float(corridor.half_width)
    w=left-right
    if not (np.isfinite(w) and .5<=w<=5.5 and left>right):
        return RoadDistances(False)
    return RoadDistances(True,left,-right,center,w,float(c[-1,0]))


class RacingLine:
    """Start grid -> road center -> outside/apex/outside without world map."""
    def __init__(self, cfg):
        self.cfg=cfg
        self.phase='GRID'
        self.traveled_m=0.0
        self.confirmed_frames=0
        self.lost_frames=0
        self.corner_sign=0
        self.corner_distance=0.0
        self.straight_frames=0
        self.last_distances=RoadDistances(False)
        self.offset_m=0.0
        self.corner_seen=False
        # Time begins only after the camera has authorized motion and the
        # planner calls update(). Do not infer road pose from a stored track.
        self.startup_elapsed_s=0.0
        self.right_reference_m=None
        self.center_confirmed=False
        self.center_transition_from_m=0.
        self.center_transition_elapsed_s=0.

    def update(self, road, speed, dt, overtake_side=0, exploration_lap=False):
        dt=max(0.,min(.25,float(dt)))
        d=max(0.,min(10.,float(speed)))*dt
        self.traveled_m+=d
        if speed>.08:
            self.startup_elapsed_s+=dt
        info=base_link_road_distances(road)
        self.last_distances=info
        if (self.right_reference_m is None and info.valid and
                0.28 <= info.right_m <= 1.50):
            # Measure the starting curb gap rather than a hardcoded lane.
            self.right_reference_m=info.right_m
        usable=(info.valid and info.ahead_m >= self.cfg.get('grid_min_ahead_m',1.65)
                and info.width_m>=self.cfg.get('grid_min_width_m',.68))
        self.confirmed_frames=self.confirmed_frames+1 if usable else 0
        self.lost_frames=0 if usable else self.lost_frames+1
        if self.phase=='GRID':
            # Emergency early release: if a genuine close bend arrives before
            # 3 seconds, holding a nearly straight steering angle until the
            # curb is crossed is unsafe. Allow live curve tracking only when
            # the observed right-side gap has actually collapsed and both
            # boundaries are continuously measured. The normal starting
            # straight remains in GRID for the requested three seconds.
            emergency_bend=(usable and self.right_reference_m is not None
                            and self.startup_elapsed_s>=1.0
                            and self.traveled_m>=1.0
                            and self.right_reference_m-info.right_m>=.10
                            and abs(float(road.heading_deg))>=6.0)
            held_enough=(self.startup_elapsed_s >=
                         self.cfg.get('startup_right_hold_s',0.0)) or emergency_bend
            if (held_enough and usable and
                    self.traveled_m>=self.cfg.get('grid_straight_m',.65)
                    and self.confirmed_frames>=self.cfg.get('road_confirm_frames',6)):
                self.phase='CENTER'
                self.center_confirmed=True
                if emergency_bend:
                    # Safest close bend: go for the sensed road centre now,
                    # instead of preserving a far-right initial line.
                    self.center_transition_from_m=0.
                    self.center_transition_elapsed_s=float(self.cfg.get('center_transition_s',2.))
                if self.right_reference_m is not None and not emergency_bend:
                    half_width=info.width_m/2
                    max_offset=max(0.,half_width-self.cfg['path_half_width_m']-.09)
                    self.center_transition_from_m=float(np.clip(
                        self.right_reference_m-half_width,-max_offset,max_offset))
                if not emergency_bend:self.center_transition_elapsed_s=0.
            self.offset_m=0.
            return
        if self.center_confirmed:
            self.center_transition_elapsed_s+=dt
        duration=max(.2,float(self.cfg.get('center_transition_s',2.0)))
        fraction=max(0.,min(1.,self.center_transition_elapsed_s/duration))
        transition_offset=self.center_transition_from_m*(1.-fraction)
        if exploration_lap:
            # No preplanned racing line / overtaking during first lap.
            self.phase='CENTER'
            self.offset_m=transition_offset
            return
        if not usable:
            # No speculative lane change when the sides have disappeared.
            self.offset_m=0.
            return
        heading=abs(road.heading_deg)
        entering=heading>=self.cfg.get('corner_prelook_heading_deg',7.5)
        straight=heading<self.cfg.get('corner_end_heading_deg',3.5)
        # The old exit state could persist when a new opposite bend appeared.
        # That kept an outside-line request active through the following turn.
        sign=1 if road.heading_deg>0 else -1
        if self.phase.startswith('CORNER') and entering and sign!=self.corner_sign:
            self.corner_sign=sign
            self.phase='CORNER_ENTRY'
            self.corner_distance=0.
            self.straight_frames=0
        if self.phase=='CENTER' and entering and overtake_side==0:
            self.corner_sign=1 if road.heading_deg>0 else -1
            self.phase='CORNER_ENTRY'
            self.corner_distance=0.
            self.straight_frames=0
            self.corner_seen=True
        if self.phase.startswith('CORNER'):
            self.corner_distance+=d
            if straight:
                self.straight_frames+=1
            else:
                self.straight_frames=0
            if self.phase=='CORNER_ENTRY' and self.corner_distance>=self.cfg.get('corner_entry_length_m',1.2):
                self.phase='CORNER_APEX'
            if self.phase=='CORNER_APEX' and self.corner_distance>=self.cfg.get('corner_apex_length_m',2.9):
                self.phase='CORNER_EXIT'
            # Road straightens again; return to center rather than weaving.
            if (self.phase=='CORNER_EXIT' and self.straight_frames>=6) or (self.straight_frames>=12):
                self.phase='CENTER'
                self.corner_distance=0.
                self.straight_frames=0
        # A reduced-width envelope keeps side offsets inside verified edges.
        max_side=max(0.,info.width_m/2-self.cfg['path_half_width_m']-.16)
        lateral=min(self.cfg.get('corner_lateral_offset_m',.23),max_side*.55)
        if self.phase=='CORNER_ENTRY': target=-self.corner_sign*lateral
        elif self.phase=='CORNER_APEX':target=self.corner_sign*lateral
        elif self.phase=='CORNER_EXIT':
            # Never move to the outside while the turn is still steep.
            # v7 entered CORNER_EXIT after a fixed distance and then stayed
            # there throughout the still-sharpening 30-degree field bend.
            heading_limit=max(10.,1.5*self.cfg.get('corner_prelook_heading_deg',7.5))
            target=0. if heading>=heading_limit else -self.corner_sign*lateral
        else:target=transition_offset
        if overtake_side:target=0.  # Safety planner owns passing lane.
        # Race-line optimization must give way to curb clearance. Side offsets
        # are zero when either measured side is close to the vehicle envelope.
        if min(info.left_m,info.right_m)<self.cfg['path_half_width_m']+.20:
            target=0.
        # Avoid instantaneous lateral target flips at the apex.
        limit=self.cfg.get('max_line_offset_change_mps',.55)*max(.01,dt)
        self.offset_m=float(np.clip(target,self.offset_m-limit,self.offset_m+limit))

    @property
    def grid(self): return self.phase=='GRID'

    @property
    def corner(self):return self.phase.startswith('CORNER')


class LapLearner:
    """Conservative lap estimate: heading loop + distance + unique sensor stripe.

    Learning is provisional: integrated command is not actual travelled distance.
    Speed hints NEVER override live curb/obstacle safety limits.
    """
    def __init__(self, cfg):
        self.cfg=cfg
        self.distance_m=0.
        self.yaw_rad=0.
        self.elapsed_s=0.
        self.lap=1
        self.first_lap_distance=None
        self.first_lap_records=[]
        self.safe_record_by_bin={}
        self.reference=None
        self._last_record_at=-1.
        self._line_previous=False
        self._last_line_at=-1000.
        self.brakes=0
        self.first_lap_brakes=0
        self.first_lap_features=[]
        self._last_feature_at=-1.0
        self.pos_x_m=0.0
        self.pos_y_m=0.0
        self.last_confirmation='NONE'

    @staticmethod
    def signature(road):
        if not road.valid or len(road.centers)<3:return None
        c=np.asarray(road.centers)
        if c[0,0]>1.0 or c[-1,0]<3.5:return None
        vals=[float(np.interp(x,c[:,0],c[:,1])) for x in (1.,2.,3.)]
        return np.asarray(vals+[float(road.half_width)],dtype=float)

    def integrate(self, speed, wheel_steer, dt, wheel_base, road, mode):
        dt=max(0.,min(.25,float(dt)))
        v=max(0.,min(12.,float(speed)))
        ds=v*dt
        self.elapsed_s+=dt
        self.distance_m+=ds
        delta_yaw=ds*math.tan(float(wheel_steer))/max(.1,float(wheel_base))
        midpoint_yaw=self.yaw_rad+0.5*delta_yaw
        self.pos_x_m+=ds*math.cos(midpoint_yaw)
        self.pos_y_m+=ds*math.sin(midpoint_yaw)
        self.yaw_rad+=delta_yaw
        if (self.lap==1 and road.valid and ds>0 and
                self.distance_m-self._last_feature_at>=1.0):
            near=base_link_road_distances(road)
            if near.valid:
                # Observed features only; local dead-reckoning may drift.
                self.first_lap_features.append({
                    'distance_m_est':round(self.distance_m,3),
                    'local_x_m_est':round(self.pos_x_m,3),
                    'local_y_m_est':round(self.pos_y_m,3),
                    'yaw_rad_est':round(self.yaw_rad,4),
                    'right_curb_m':round(near.right_m,3),
                    'left_curb_m':round(near.left_m,3),
                    'road_width_m':round(near.width_m,3),
                    'road_heading_deg':round(float(road.heading_deg),3),
                    'visible_ahead_m':round(near.ahead_m,3),
                    'speed_mps_est':round(v,3),
                    'mode':str(mode),
                })
                self._last_feature_at=self.distance_m
        if mode in ('BRAKE','STOP') and v<.25:self.brakes+=1
        if self.lap==1 and road.valid and ds>0 and self.distance_m-self._last_record_at>=.9:
            self.first_lap_records.append((self.distance_m,float(road.heading_deg),v,mode))
            self._last_record_at=self.distance_m
        if self.reference is None and self.lap==1 and self.distance_m<4.0:
            self.reference=self.signature(road)

    def observe_line(self, crossline_seen, road):
        """Call once per camera perception update. Rising edge only."""
        detected=bool(crossline_seen)
        rise=detected and not self._line_previous
        self._line_previous=detected
        if not rise or self.lap!=1:return False
        if self.distance_m-self._last_line_at<10.:return False
        self._last_line_at=self.distance_m
        # Reject accidental grid markings and early repetitive striping.
        min_distance=self.cfg.get('lap_min_distance_m',250.)
        min_time=self.cfg.get('lap_min_time_s',90.)
        if self.distance_m<min_distance or self.elapsed_s<min_time:
            self.last_confirmation='TOO_EARLY'
            return False
        if not (self.cfg.get('lap_min_abs_yaw_rad',5.2)<=abs(self.yaw_rad)<=self.cfg.get('lap_max_abs_yaw_rad',8.5)):
            self.last_confirmation='YAW_NOT_CLOSED'
            return False
        sig=self.signature(road)
        if self.reference is None or sig is None or np.max(np.abs(sig-self.reference))>.32:
            self.last_confirmation='NO_CORRIDOR_MATCH'
            return False
        self.lap=2
        self.first_lap_distance=self.distance_m
        self.first_lap_brakes=self.brakes
        self.last_confirmation='SENSOR_CONFIRMED_PROVISIONAL'
        # Align first-lap commanded speeds into metre-indexed hint data.
        for dist,curvature,v,mode in self.first_lap_records:
            self.safe_record_by_bin[int(dist//2)]=(v,curvature,mode)
        return True

    def speed_multiplier(self, current_road_valid, current_mode, safe_now):
        if self.lap==1 or not current_road_valid or not safe_now or current_mode in ('BRAKE','STOP'):
            return 1.
        # Only modest speed increase after provisional lap closure. Safety planner
        # still applies live corner ratio, obstacle envelope and lateral accel.
        if self.first_lap_distance:
            travelled=(self.distance_m-self.first_lap_distance)%max(1.,self.first_lap_distance)
            previous=self.safe_record_by_bin.get(int(travelled//2))
            if previous and (previous[2] in ('BRAKE','STOP') or previous[0]<.35):
                return 1. # Recorded difficulty: no local speed increase.
        return min(1.12,float(self.cfg.get('learned_lap_speed_factor',1.10)))

    def learned_corner_ahead(self):
        """Only use recorded curvature for proactive slowing, never blind speeding."""
        if self.lap==1 or self.first_lap_distance is None:return False
        offset=(self.distance_m-self.first_lap_distance)%max(1.,self.first_lap_distance)
        for ahead in (1.,2.,3.):
            info=self.safe_record_by_bin.get(int((offset+ahead)//2))
            if info and abs(info[1])>self.cfg.get('corner_prelook_heading_deg',7.5):return True
        return False
