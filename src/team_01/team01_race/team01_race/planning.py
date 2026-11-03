"""Safety-first controller; perception never bypasses collision checks."""
import math
import numpy as np
from team01_race.planning_gate import Motion, TrafficGate
from team01_race.planning_road import RoadCorridor, estimate_road_corridor, fuse_verified_corridors
from team01_race.planning_geometry import PlannerGeometry
from team01_race.curved_path import adaptive_pursuit_candidate, preview_curvature


class SafetyPlanner(PlannerGeometry):
    def step(self, depth_obstacles, lidar_obstacles, rear_gap, dt, enabled,
             depth_valid, front_lidar_valid=True, road_bounds=None, actual_speed=None, map_target=None, map_ready=False, actual_yaw_rate=None):
        if not enabled or not depth_valid or (not front_lidar_valid and
             not self.cfg.get('unsafe_allow_without_front_lidar',False)):
            self.speed=0.
            self.prev_steer=0.
            self.steer_limiter.reset()
            self.last_mode='STOP'
            return Motion(0.,0.,'STOP','gate or stale/missing forward sensor')
        dt=max(.01,min(dt,.25))
        self.map_ready=bool(map_ready)
        depth_obstacles=np.asarray(depth_obstacles).reshape((-1,2))
        lidar_obstacles=np.asarray(lidar_obstacles).reshape((-1,2))
        pts=np.vstack((depth_obstacles,lidar_obstacles))
        # Reject self-returns near bumper and points far behind front axle.
        pts=pts[(pts[:,0]>.28) & (pts[:,0]<15.7) & (np.abs(pts[:,1])<5.)]
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
        # A stationary car is not evidence of a corner: require independently
        # observed curvature in addition to a broad stationary front wall.
        early_curve=preview_curvature(self.road)
        bending=bool(self.road.valid and
                     (abs(self.road.heading_deg)>=self.cfg.get('corner_prelook_heading_deg',7.5)
                      or early_curve>=self.cfg.get('preview_curvature_threshold',.08)))
        wall_static=self._front_wall(lidar_obstacles,self.measured_speed,dt)
        wall_bend=bool(wall_static and bending and
                       self.static_wall_distance_m is not None and
                       self.static_wall_distance_m<self.cfg.get('precorner_wall_range_m',10.))
        current_lane=self.racing_line.offset_m
        risk_distance=max(4.,min(15.,observed*self.cfg['reaction_delay_s']+
                       observed*observed/(2*self.cfg['max_braking_mps2'])+3.0))
        obstruction=self._lane_obstacle_ahead(pts,self.road,current_lane,risk_distance)
        self.pre_corner_active=bending or wall_bend
        self.recenter_reason=('corner_wall' if wall_bend else
                              'corner' if bending else
                              'obstacle' if obstruction else 'none')
        self.racing_line.update(self.road,observed,dt,self.overtake_side,
            exploration_lap=(self.cfg.get('disable_first_lap_racing_line',False)
                             and self.lap_learner.lap==1),
            center_requested=(bending or wall_bend or obstruction))
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
        horizon=min(15.0,max(4.0,3.+self.speed*1.3))
        lead=self._lead(depth_obstacles,dt)
        self.lead_distance=lead
        self.overtake_start_distance_m=None
        self.planned_pass_distance_m=None
        passing_enabled=bool(self.cfg.get('enable_overtake',True) and self.map_ready)
        self.target_lane_offset,self.overtake_side=(
            self.rejoin.update(self.road,pts,lead,self.lead_speed,
                               rear_gap,dt,observed,passing_enabled,
                               base_offset=self.racing_line.offset_m))
        self.overtake_start_distance_m=lead
        self.planned_pass_distance_m=self.rejoin.remaining_m
        # Acquire the selected lane over several metres, preserving vehicle
        # heading at x=0. A distant straight line must not cause a snap turn.
        mapped_goal=(None if not self.map_ready or map_target is None else
                     (map_target[0],map_target[1]+self.target_lane_offset))
        self.reference_geometry=self.reference_scheduler.update(
            self.road,self.target_lane_offset,observed,actual_yaw_rate,
            self.prev_steer,dt,mapped_goal)
        desired=0.
        if self.road.valid:
            pursuit_speed=(self.measured_speed if self.measured_speed is not None
                           else self.speed)
            xgoal=float(np.clip(1.8+min(4.8,pursuit_speed*.44),
                                self.road.centers[0,0],self.road.centers[-1,0]))
            if self.reference_geometry is not None:
                ygoal=float(np.interp(xgoal,self.reference_geometry[:,0],
                                      self.reference_geometry[:,1]))
            else:
                ygoal=float(np.interp(xgoal,self.road.centers[:,0],
                                      self.road.centers[:,1])+self.target_lane_offset)
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
            path=(self.reference_geometry if self.reference_geometry is not None
                  else self.road.centers)
            look_x=float(np.clip(min(path[-1,0],3.0),max(.7,path[0,0]),3.0))
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
            # One continuous path reference: eliminate per-tick left/right
            # candidate switching that previously caused zigzag steering.
            for shift in (0.,):
                # If the far end of a newly visible curve is out of the safe
                # envelope, attempt a SHORTER fully checked trajectory.
                # Its short free distance is then used by the braking
                # supervisor: no assertion that the unseen road is clear.
                for trial in (min(14.,horizon),min(4.,horizon),
                              min(2.3,horizon)):
                    c=adaptive_pursuit_candidate(
                        self.road,pts,self.cfg,
                        self.measured_speed if self.measured_speed is not None else self.speed,
                        desired_offset=self.target_lane_offset,
                        relative_offset=shift,horizon=trial,
                        reference=self.reference_geometry)
                    if c is not None:
                        adaptive.append(c)
                        break
        # Keep a single controller for straight and bend when the full
        # collision-checked adaptive rollout is available.
        use_adaptive=bool(adaptive) and not startup_hold
        if use_adaptive:
            choice=max(adaptive,key=lambda c:c.score-.40*abs(c.steer_rad-self.prev_steer))
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
        shallow=self.capture_mode.update(
            self.road.heading_deg,self.preview_curve,
            hazard=(not self.road.valid or self.edge_alert!='NONE'
                    or bool(self.overtake_side) or lead is not None))
        if not startup_hold and shallow:
            # Straight-line acquisition is intentionally shallow. Allow full
            # steering on actual bends, hazards and when already near an edge.
            capture_limit=math.radians(float(
                self.cfg.get('straight_capture_max_steer_deg',6.)))
            steer=float(np.clip(steer,-capture_limit,capture_limit))
        if not startup_hold:
            steer=self.yaw_feedback.correct(steer,observed,actual_yaw_rate)
            # The chosen adaptive trajectory was checked for edges; smoothing
            # must not require a constant arc to reproduce a varying curve.
            steer=self.steer_limiter.apply(steer,dt,observed)
            # The selected adaptive path has already passed a full rollout
            # against the road and obstacles. A CONSTANT-arc projection of
            # the *rate-limited first wheel command* over 2.5 m is not that
            # trajectory: it falsely rejects bends where the wheel will keep
            # turning on subsequent controller ticks. Check the bounded
            # near-term response, and re-plan with fresh sensing every tick.
            # Keep the conservative 2.5 m check for constant-arc control.
            near_horizon=(min(2.5,horizon)
                          if self.planner_type!='adaptive_pursuit'
                          else min(horizon,max(1.25,min(2.5,observed*.25))))
            safe,_=self._within_road(steer,near_horizon)
            if not safe:
                self.speed=0.;self.last_mode='BRAKE'
                return Motion(0.,-self.prev_steer,'BRAKE','limited steering outside visible road')
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
        # Defensive strategy is *only* maintaining speed and a steady safe line.
        # Never swerve to block another car, especially in an adjacent lane.
        # DEFEND is only a situational flag for lane holding and diagnostics.
        # A nearby rear car is relevant even before SLAM map closure.
        # It NEVER authorizes a swerve or overtaking without a valid map.
        self.defend=(self.overtake_side==0 and rear_gap is not None and
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
                or self.pre_corner_active or self.lap_learner.learned_corner_ahead()
                or self.pre_corner_active
                or self.preview_curve >= self.cfg.get('preview_curvature_threshold',.08)):
            v=min(v,lap_target*lap_boost*
                  self.cfg['corner_speed_factor'])
        curvature=max(abs(math.tan(steer))/self.cfg['wheel_base_m'],
                      self.preview_curve)
        # Nav2 RPP-inspired curvature regulation is a speed cap, not a boost.
        regulate_radius=float(self.cfg.get('regulated_min_turn_radius_m',2.0))
        if curvature>1./max(.1,regulate_radius):
            v=min(v, max(.60,lap_target * (1./(curvature*regulate_radius))))
        if curvature>1e-5:
            v=min(v,math.sqrt(self.cfg['max_lateral_accel_mps2']/curvature))
        # Speed bound from nearest forward collision and sensor/actuator delay.
        # Do not apply a *constant-arc* clearance estimate to an adaptive
        # multi-steer candidate: it can see a curb where the predicted path turns.
        if self.planner_type != 'adaptive_pursuit':
            free=self.clearance(pts,steer,horizon)
        d=max(0.,free-self.cfg['emergency_margin_m'])
        # Limit commanded speed by forward time-to-collision as well as the
        # independent braking-distance and lateral-acceleration envelopes.
        v=min(v,d/max(.35,float(self.cfg.get('regulated_min_ttc_s',.8))))
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
        # Rate-limit acceleration; allow emergency braking at configured rate.
        # Never accelerate beyond what the currently verified forward
        # clearance permits us to stop within, including reaction delay.
        verified=max(0.,float(free)-float(self.cfg['emergency_margin_m']))
        delay=max(0.,float(self.cfg['reaction_delay_s']))
        v_visible=max(0.,math.sqrt((decel*delay)**2+2*decel*verified)-decel*delay)
        v=min(v,v_visible,self.speed_stage.cap(self.map_ready))
        if self.reference_geometry is None and self.road.valid:
            # A fallback arc is not proof of a stable racing trajectory.
            v=min(v,float(self.cfg.get('missing_reference_speed_mps',1.25)))
        v=np.clip(v,self.speed-decel*dt,self.speed+self.cfg['accel_mps2']*dt)
        self.speed=float(max(0.,v))
        self.prev_steer=float(steer)
        mode=('OVERTAKE' if self.overtake_side else 'GRID' if self.racing_line.grid
              else 'CORNER' if self.pre_corner_active or
                   abs(steer)>=math.radians(self.cfg['corner_threshold_deg'])
              else 'DEFEND' if self.defend else 'STRAIGHT')
        self.last_mode=mode
        return Motion(self.speed,-float(steer),mode,
                      'sensor-bounded '+self.planner_type)
