"""v7 regression: short curb horizons and curved local trajectories.
No test here claims Gazebo/physics validation.
"""
import math
from pathlib import Path
import sys
import unittest
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from team01_race.planning import SafetyPlanner
from team01_race.curved_path import adaptive_pursuit_candidate,preview_curvature
from team01_race.race_strategy import RacingLine
from test_offline import CFG

CONFIG={**CFG,'grid_discovery_speed_mps':1.25,'grid_straight_m':.65,
        'grid_min_ahead_m':1.65,'grid_min_width_m':.68,'road_confirm_frames':6,
        'require_visual_road':True,'preview_curvature_threshold':.08,
        'corner_prelook_heading_deg':7.5,'corner_end_heading_deg':3.5,
        'corner_entry_length_m':1.2,'corner_apex_length_m':2.9,
        'max_line_offset_change_mps':.55,'corner_lateral_offset_m':.23}


def bounds(xmax=2.3,curve=0.,width=.40,shift=0.):
    x=np.arange(.6,xmax,.18)
    y=curve*x*x+shift
    return np.column_stack((x,y-width,y+width))


class V7Tests(unittest.TestCase):
    def test_logged_grid_never_exits_fixed(self):
        """Regression from log: 333 m stuck in GRID despite two valid curbs."""
        p=SafetyPlanner(CONFIG)
        modes=[]
        for _ in range(50):
            m=p.step(np.empty((0,2)),np.empty((0,2)),4.,.1,
                     True,True,True,bounds())
            modes.append(m.mode)
        self.assertIn('STRAIGHT',modes)
        self.assertEqual(p.racing_line.phase,'CENTER')
        self.assertGreater(p.speed,.65)

    def test_grid_discovery_speed_can_be_125_cms(self):
        p=SafetyPlanner(CONFIG)
        vals=[]
        for _ in range(16):
            m=p.step(np.empty((0,2)),np.empty((0,2)),4.,.1,
                     True,True,True,bounds())
            if m.mode=='GRID':vals.append(m.speed)
        self.assertGreater(max(vals),.65)
        self.assertLessEqual(max(vals),1.25+1e-6)

    def test_missing_both_road_edges_no_grid_escape(self):
        s=RacingLine(CONFIG)
        for _ in range(60):
            bad=SafetyPlanner._visual_corridor(bounds(xmax=1.25))
            s.update(bad if bad is not None else SafetyPlanner(CONFIG).road,1.25,.1)
        self.assertEqual(s.phase,'GRID')

    def test_curved_depth_corridor_has_receding_trajectory(self):
        road=SafetyPlanner._visual_corridor(bounds(xmax=5.,curve=.12,width=.58))
        self.assertIsNotNone(road)
        cand=adaptive_pursuit_candidate(road,np.empty((0,2)),CONFIG,
                                       2.,horizon=4.5)
        self.assertIsNotNone(cand)
        self.assertGreater(abs(cand.steer_rad),math.radians(1.))
        self.assertGreater(len(cand.points),6)
        self.assertGreater(preview_curvature(road),.08)

    def test_corner_selects_adaptive_not_constant_arc(self):
        p=SafetyPlanner(CONFIG)
        for _ in range(30):
            p.step(np.empty((0,2)),np.empty((0,2)),4.,.1,
                   True,True,True,bounds(xmax=5.))
        m=p.step(np.empty((0,2)),np.empty((0,2)),4.,.1,
                   True,True,True,bounds(xmax=5.,curve=.12,width=.6))
        self.assertEqual(p.planner_type,'adaptive_pursuit')
        self.assertEqual(m.mode,'CORNER')
        self.assertNotEqual(m.mode,'BRAKE')

    def test_adaptive_rejects_curb_collision(self):
        road=SafetyPlanner._visual_corridor(bounds(xmax=4.,curve=.04,width=.43))
        wall=np.array([[1.3,y] for y in np.arange(-.35,.36,.05)])
        cand=adaptive_pursuit_candidate(road,wall,CONFIG,2.,horizon=3.)
        self.assertIsNone(cand)

    def test_bending_curb_not_mislabelled_as_lead_vehicle(self):
        p=SafetyPlanner(CONFIG)
        b=bounds(xmax=5.,curve=.12,width=.62)
        wall=np.vstack((b[:,(0,1)],b[:,(0,2)]))
        for _ in range(3):
            m=p.step(wall,wall,4.,.1,True,True,True,b)
        self.assertIsNone(p.lead_distance,
                          'nearby roadside curb must not look like opponent')
        self.assertEqual(p.planner_type,'adaptive_pursuit')
        self.assertNotEqual(m.mode,'BRAKE')

    def test_odom_integrates_distance_not_commanded_speed(self):
        p=SafetyPlanner(CONFIG)
        for _ in range(35):
            p.step(np.empty((0,2)),np.empty((0,2)),4.,.1,
                   True,True,True,bounds(xmax=5.),actual_speed=.2)
        self.assertLess(p.lap_learner.distance_m,.85)
        self.assertAlmostEqual(p.measured_speed,.2)

    def test_target_20_is_not_enabled_during_initial_test(self):
        import ast
        src=(Path(__file__).resolve().parents[1]/'team01_race/node.py').read_text()
        defaults=ast.literal_eval(next(n.value for n in ast.parse(src).body
                                   if isinstance(n,ast.Assign)
                                   and any(isinstance(t,ast.Name) and t.id=='DEFAULTS'
                                           for t in n.targets)))
        self.assertEqual(defaults['target_straight_speed_mps'],20.)
        self.assertEqual(defaults['straight_speed_mps'],7.)
        self.assertEqual(defaults['grid_discovery_speed_mps'],1.25)

    def test_closed_loop_27_degree_sweep_stays_centered(self):
        """Synthetic kinematic curved corridor: measured track, no world hints to planner."""
        p=SafetyPlanner(CONFIG)
        xx=yy=yaw=0.;dt=.08
        errors=[];modes=[]
        def curve(x):
            return .027*x*x if x<11 else .027*121+.594*(x-11)
        def tangent(x):return .054*min(max(x,0.),11.)
        for _ in range(230):
            co,si=math.cos(yaw),math.sin(yaw)
            b=[]
            for v in np.arange(xx+.4,xx+7.1,.11):
                dx=v-xx;dy=curve(v)-yy
                px=co*dx+si*dy;py=-si*dx+co*dy
                half=.58*math.sqrt(1+tangent(v)**2)
                if px>.4:b.append((px,py-half,py+half))
            if len(b)<6:break
            b=np.asarray(sorted(b,key=lambda q:q[0]),float)
            b=b[np.r_[True,np.diff(b[:,0])>.025]]
            cmd=p.step(np.empty((0,2)),np.empty((0,2)),4.,dt,
                       True,True,True,b)
            steer=-cmd.steering
            yaw+=cmd.speed*math.tan(steer)/.34*dt
            xx+=cmd.speed*math.cos(yaw)*dt
            yy+=cmd.speed*math.sin(yaw)*dt
            errors.append(abs(yy-curve(xx)))
            modes.append(cmd.mode)
            if xx>17.:break
        self.assertGreater(xx,16.,'failed to traverse synthetic curved corridor')
        self.assertLess(max(errors),.35,'drifted out of usable corridor')
        self.assertIn('CORNER',modes)
        self.assertNotIn('BRAKE',modes)

    def test_stale_lidar_stops_even_on_adaptive_curve(self):
        p=SafetyPlanner(CONFIG)
        m=p.step(np.empty((0,2)),np.empty((0,2)),4.,.1,
                 True,True,False,bounds(xmax=5.,curve=.10,width=.8))
        self.assertEqual(m.speed,0.)

    def test_road_edges_remain_mandatory(self):
        p=SafetyPlanner(CONFIG)
        m=p.step(np.empty((0,2)),np.empty((0,2)),4.,.1,
                 True,True,True,None)
        self.assertEqual(m.speed,0.)

if __name__=='__main__':unittest.main()
