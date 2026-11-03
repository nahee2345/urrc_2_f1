"""Lane hold, recenter, static wall, steering and sensor regression tests."""
import sys, math, unittest, numpy as np
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from team01_race.planning import SafetyPlanner
from test_v9_startup_lap_exploration import CFG, road
C={**CFG,'hold_safe_lane_mode':True,'max_steer_rate_deg_s':35.,
   'steer_smoothing_tau_s':.16,'center_reached_tolerance_m':.12,
   'max_line_offset_change_mps':.28,'precorner_wall_range_m':10.}
E=np.empty((0,2))
class V13(unittest.TestCase):
    def make(self):
        p=SafetyPlanner(C)
        for _ in range(50):
            p.step(E,E,4.,.1,True,True,True,road(width=1.,right=.6).bounds,actual_speed=1.)
        self.assertEqual(p.racing_line.phase,'CENTER')
        return p
    def test_safe_lane_is_held(self):
        p=self.make()
        a=[]
        for _ in range(40):
            p.step(E,E,4.,.1,True,True,True,road(width=1.,right=.6).bounds,actual_speed=2.)
            a.append(p.racing_line.offset_m)
        self.assertLess(a[-1],-.2)
        self.assertAlmostEqual(a[-1],a[-15],delta=.04)
        self.assertEqual(p.recenter_reason,'none')
    def test_corner_gradually_recenter(self):
        p=self.make()
        old=p.racing_line.offset_m
        p.step(E,E,4.,.05,True,True,True,road(width=1.,right=.6,curve=.10).bounds,actual_speed=2.)
        self.assertEqual(p.recenter_reason,'corner')
        self.assertGreater(p.racing_line.offset_m,old)
        self.assertLessEqual(p.racing_line.offset_m-old,.01401)
    def test_obstacle_only_on_held_lane(self):
        p=self.make()
        b=road(width=1.,right=.6).bounds
        p.step(np.array([[2.,.65]]),E,4.,.1,True,True,True,b,actual_speed=1.)
        self.assertEqual(p.recenter_reason,'none')
        p.step(np.array([[2.,0.],[2.05,.04]]),E,4.,.1,True,True,True,b,actual_speed=1.)
        self.assertEqual(p.recenter_reason,'obstacle')
        self.assertTrue(p.racing_line.recenter_active)
    def test_steer_rate_and_lowpass(self):
        p=SafetyPlanner(C)
        old=0.
        for _ in range(16):
            v=p._soft_steer(math.radians(27),.05,6.)
            self.assertGreaterEqual(v,old)
            self.assertLessEqual(v-old,math.radians(27)*.05+1e-6)
            p.prev_steer=old=v
        self.assertGreater(old,0.)
    def test_stationary_wall_not_always_a_corner(self):
        p=SafetyPlanner(C)
        w=lambda x:np.array([[x,y] for y in np.linspace(-.6,.6,25)])
        self.assertFalse(p._front_wall(w(4.),1.,.1))
        self.assertFalse(p._front_wall(w(3.9),1.,.1))
        self.assertTrue(p._front_wall(w(3.8),1.,.1))
        self.assertLess(abs(p.front_wall_speed_mps),1.5)
        q=SafetyPlanner(C)
        for _ in range(35):
            q.step(E,w(4.),4.,.1,True,True,True,road(width=1.,right=.6).bounds,actual_speed=0.)
        self.assertFalse(q.pre_corner_active)
    def test_missing_sensor_still_stops(self):
        p=self.make()
        m=p.step(E,E,4.,.1,True,False,True,road(width=1.,right=.6).bounds)
        self.assertEqual(m.mode,'STOP')
    def test_smooth_curved_trajectory(self):
        p=SafetyPlanner(C)
        x=y=yaw=0.
        def f(t):return .027*t*t if t<11 else .027*121+.594*(t-11)
        def slope(t):return .054*min(max(t,0.),11.)
        errors=[];modes=[]
        for _ in range(250):
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
            m=p.step(E,E,4.,.08,True,True,True,b)
            yaw+=m.speed*math.tan(-m.steering)/.34*.08
            x+=m.speed*math.cos(yaw)*.08
            y+=m.speed*math.sin(yaw)*.08
            errors.append(abs(y-f(x)));modes.append(m.mode)
            if x>17:break
        self.assertGreater(x,17.)
        self.assertLess(max(errors),.25)
        self.assertNotIn('BRAKE',modes)
if __name__=='__main__':unittest.main()
