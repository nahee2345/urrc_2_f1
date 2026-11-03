"""v9 new specifications, independent of actual ROS/Gazebo runtime."""
from pathlib import Path
import sys, unittest, math, numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from team01_race.race_strategy import RacingLine,LapLearner
from team01_race.planning import SafetyPlanner, RoadCorridor, TrafficGate

CFG=dict(
 straight_speed_mps=7.,second_lap_straight_speed_mps=12.,
 corner_speed_factor=.30, corner_threshold_deg=10.,
 target_straight_speed_mps=20., preview_curvature_threshold=.08,
 max_center_steer_deg=29.8,wheel_base_m=.34,max_lateral_accel_mps2=5.,
 max_braking_mps2=5.,accel_mps2=2.,reaction_delay_s=.3,
 emergency_margin_m=.55,path_half_width_m=.18,
 overtake_offset_m=.48,overtake_advantage_mps=.8,
 grid_straight_m=.65,grid_discovery_speed_mps=1.25,
 grid_min_ahead_m=1.65,grid_min_width_m=.68,road_confirm_frames=6,
 corner_prelook_heading_deg=7.5,corner_end_heading_deg=3.5,
 corner_entry_length_m=1.2,corner_apex_length_m=2.9,
 corner_lateral_offset_m=.23,max_line_offset_change_mps=.55,
 lap_min_distance_m=65.,lap_min_time_s=25.,
 lap_min_abs_yaw_rad=5.2,lap_max_abs_yaw_rad=8.5,
 learned_lap_speed_factor=1.0,edge_caution_extra_m=.12,
 edge_caution_min_speed_mps=.45,edge_caution_extra_speed_mps=1.15,
 require_visual_road=True,permit_lidar_verified_fallback=False,
 unmapped_speed_mps=.75,unsafe_allow_without_front_lidar=False,
 startup_right_hold_s=3.,startup_max_steer_deg=.8,startup_gap_deadband_m=.07,
 startup_gap_kp_deg_per_m=2.,startup_verified_horizon_m=2.,center_transition_s=2.,
 disable_first_lap_racing_line=True,enable_overtake=False,
 rear_defend_min_gap_m=1.5,
)

def road(width=.75,right=.59,curve=0.,xmax=7.):
    x=np.arange(.6,xmax,.16)
    mid=width-right+curve*x*x
    bounds=np.column_stack((x,mid-width,mid+width))
    return RoadCorridor(np.column_stack((x,mid)),width,True,
                        math.degrees(math.atan(curve*3)),bounds)

class V9(unittest.TestCase):
    def test_startup_follows_right_gap_for_three_seconds(self):
        p=SafetyPlanner(CFG)
        # Initial wider left side and 0.59m right gap, as in the real log.
        prev=None
        for i in range(29):
            cmd=p.step(np.empty((0,2)),np.empty((0,2)),4.,.1,True,True,True,
                       road().bounds,actual_speed=1.)
            self.assertEqual(p.racing_line.phase,'GRID')
            self.assertLessEqual(abs(math.degrees(cmd.steering)),0.80001)
            self.assertNotEqual(cmd.mode,'OVERTAKE')
            prev=cmd
        self.assertAlmostEqual(p.racing_line.right_reference_m,.59,places=2)
        for i in range(5):
            cmd=p.step(np.empty((0,2)),np.empty((0,2)),4.,.1,True,True,True,
                       road().bounds,actual_speed=1.)
        self.assertEqual(p.racing_line.phase,'CENTER')
        self.assertLess(p.racing_line.offset_m,0.) # measured right-lane position begins off center

    def test_no_center_jump_when_road_narrows(self):
        p=SafetyPlanner(CFG)
        for i in range(10):
            p.step(np.empty((0,2)),np.empty((0,2)),4.,.1,True,True,True,
                   road().bounds,actual_speed=1.)
        vals=[]
        for i in range(25):
            cmd=p.step(np.empty((0,2)),np.empty((0,2)),4.,.1,True,True,True,
                       road(width=.40,right=.36).bounds,actual_speed=1.)
            vals.append(abs(math.degrees(cmd.steering)))
        self.assertLessEqual(max(vals[:19]),0.8001)
        self.assertEqual(p.racing_line.phase,'CENTER')
        self.assertLessEqual(abs(p.racing_line.offset_m),.13)
        for i in range(30):
            p.step(np.empty((0,2)),np.empty((0,2)),4.,.1,True,True,True,
                   road(width=.40,right=.36).bounds,actual_speed=1.)
        self.assertAlmostEqual(p.racing_line.offset_m,0.,delta=.005)

    def test_stationary_time_does_not_burn_startup(self):
        s=RacingLine(CFG)
        for _ in range(100):s.update(road(),0.,.1,exploration_lap=True)
        self.assertEqual(s.phase,'GRID')
        self.assertLess(s.startup_elapsed_s,.1)

    def test_no_overtake_even_with_slow_car(self):
        p=SafetyPlanner(CFG)
        # Bumper is visible: do not pass; never create an overtake_side.
        obstacle=np.array([[2.,0.],[2.1,.1],[2.2,-.1]])
        modes=[]
        for _ in range(50):
            m=p.step(obstacle,np.empty((0,2)),3.,.1,
                     True,True,True,road(width=1.,right=1.).bounds,actual_speed=.8)
            modes.append(m.mode)
        self.assertEqual(p.overtake_side,0)
        self.assertNotIn('OVERTAKE',modes)

    def test_first_lap_logs_locally_observed_widths_not_map(self):
        learner=LapLearner(CFG)
        self.assertFalse(learner.first_lap_features)
        for _ in range(40):
            learner.integrate(.8,0.02,.1,.34,road(width=.55,right=.4),'STRAIGHT')
        self.assertGreater(len(learner.first_lap_features),1)
        item=learner.first_lap_features[0]
        self.assertAlmostEqual(item['road_width_m'],1.1,delta=.02)
        self.assertIn('yaw_rad_est',item)
        self.assertEqual(learner.lap,1)

    def test_second_lap_speed_target_only_after_sensor_loop(self):
        p=SafetyPlanner(CFG)
        for _ in range(38):
            p.step(np.empty((0,2)),np.empty((0,2)),4.,.1,
                   True,True,True,road(width=1.2,right=1.2).bounds,actual_speed=1.)
        self.assertEqual(p.lap_learner.lap,1)
        self.assertEqual(p.racing_line.phase,'CENTER')
        self.assertLessEqual(p.speed,7.)
        p.lap_learner.lap=2 # simulate separate *verified* sensor loop
        for _ in range(40):
            p.step(np.empty((0,2)),np.empty((0,2)),4.,.1,
                   True,True,True,road(width=1.2,right=1.2).bounds,actual_speed=2.)
        self.assertEqual(p.lap_learner.lap,2)
        # Live visibility <= 7m still enforces a stricter cap than 12m/s.
        self.assertLess(p.speed,12.)

    def test_synthetic_curved_track_without_prior_map(self):
        # Closed-loop kinematic *surrogate*, not Gazebo real physics.
        p=SafetyPlanner(CFG)
        x=y=yaw=0.
        dt=.08
        modes=[];errors=[]
        def f(t):return .027*t*t if t<11 else .027*121+.594*(t-11)
        def slope(t):return .054*min(max(t,0.),11.)
        for _ in range(280):
            co,si=math.cos(yaw),math.sin(yaw)
            bounds=[]
            for v in np.arange(x+.4,x+7.1,.11):
                dx=v-x;dy=f(v)-y
                px=co*dx+si*dy;py=-si*dx+co*dy
                half=.58*math.sqrt(1+slope(v)**2)
                if px>.4:bounds.append((px,py-half,py+half))
            if len(bounds)<6:break
            b=np.asarray(sorted(bounds,key=lambda t:t[0]),float)
            b=b[np.r_[True,np.diff(b[:,0])>.025]]
            m=p.step(np.empty((0,2)),np.empty((0,2)),5.,dt,
                     True,True,True,b)
            yaw+=m.speed*math.tan(-m.steering)/.34*dt
            x+=m.speed*math.cos(yaw)*dt
            y+=m.speed*math.sin(yaw)*dt
            modes.append(m.mode)
            errors.append(abs(y-f(x)))
            if x>17:break
        self.assertGreater(x,17.)
        self.assertLess(max(errors),.24)
        self.assertNotIn('BRAKE',modes)
        self.assertNotIn('OVERTAKE',modes)

    def test_camera_four_to_zero_still_required(self):
        t=TrafficGate(required=4,on_frames=2,off_frames=3)
        for n in (1,2,3,4,4):t.observe(n)
        self.assertTrue(t.saw_full_red)
        for n in (0,0,0):t.observe(n)
        self.assertTrue(t.lights_out)

if __name__=='__main__':unittest.main()
