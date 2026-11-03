from pathlib import Path
import sys, unittest, math
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from team01_race.planning import SafetyPlanner
from team01_race.race_strategy import RacingLine
from team01_race.lap_validation import LapVerifier
from test_v9_startup_lap_exploration import CFG

def bnds(right=-.65,left=.65,ahead=6.,curve=0.):
    xs=np.arange(.65,ahead,.16)
    ys=curve*xs*xs
    return np.column_stack((xs,ys+right,ys+left))

def qacircle(radius=20.,n=160,width=3.):
    angles=np.linspace(0,2*math.pi,n,endpoint=False)
    return np.stack((radius*np.cos(angles),radius*np.sin(angles)),axis=1),np.full(n,width)

class V10Start(unittest.TestCase):
    def test_no_steer_on_verified_straight_for_3s(self):
        p=SafetyPlanner(CFG)
        for i in range(31):
            m=p.step(np.empty((0,2)),np.empty((0,2)),None,.10,True,True,True,bnds(),actual_speed=1.25)
            if p.racing_line.grid:self.assertAlmostEqual(m.steering,0,delta=math.radians(.02))
        self.assertEqual(p.racing_line.phase,'CENTER')
        self.assertEqual(p.overtake_side,0)

    def test_prevent_preview_left_wiggle_at_grid(self):
        p=SafetyPlanner(CFG)
        # The distant curb turns LEFT, but the right curb gap is the same
        # near the bumper; v9 sampled a future bend and steered at launch.
        near_straight=bnds(curve=.008)
        for i in range(8):
            m=p.step(np.empty((0,2)),np.empty((0,2)),None,.1,True,True,True,near_straight,actual_speed=.9)
            self.assertLessEqual(abs(math.degrees(m.steering)),.8+1e-6)
        self.assertEqual(p.racing_line.phase,'GRID')

    def test_right_gap_feedback_direction(self):
        p=SafetyPlanner(CFG)
        for _ in range(5):p.step(np.empty((0,2)),np.empty((0,2)),None,.1,True,True,True,bnds(),actual_speed=1.)
        # right wall moved closer, should command LEFT (-race steering).
        m=p.step(np.empty((0,2)),np.empty((0,2)),None,.1,True,True,True,bnds(right=-.40,left=.90),actual_speed=1.)
        self.assertLess(m.steering,0.)
        self.assertLessEqual(abs(math.degrees(m.steering)),.8+1e-6)

    def test_return_to_curb_correct_heading_not_spurious_future_curve(self):
        p=SafetyPlanner(CFG)
        for _ in range(5):p.step(np.empty((0,2)),np.empty((0,2)),None,.1,True,True,True,bnds(),actual_speed=1.)
        m=p.step(np.empty((0,2)),np.empty((0,2)),None,.1,True,True,True,bnds(right=-.90,left=.40),actual_speed=1.)
        self.assertGreater(m.steering,0.) # race +right

    def test_hard_corner_can_emergency_leave_grid(self):
        cfg=dict(CFG)
        r=RacingLine(cfg)
        import types
        for _ in range(15):r.update(types.SimpleNamespace(valid=True,centers=np.column_stack((np.arange(.6,6,.2),np.zeros(27))),
                  bounds=bnds(right=-.60,left=.60),heading_deg=0.,half_width=.6),1.,.1,exploration_lap=True)
        x=np.arange(.65,6.,.16)
        bent=np.column_stack((x,.16*x*x-.47,.16*x*x+.73))
        road=types.SimpleNamespace(valid=True,centers=np.column_stack((x,.16*x*x+.13)),bounds=bent,heading_deg=15.,half_width=.6)
        r.update(road,1.,.1,exploration_lap=True)
        self.assertEqual(r.phase,'CENTER')

class V10LapQA(unittest.TestCase):
    def test_full_circle_120_bins_pass(self):
        xy,width=qacircle()
        v=LapVerifier(xy,width)
        states=[]
        for angle in np.linspace(0,2*math.pi,1100):
            states.append(v.observe(20*math.cos(angle),20*math.sin(angle)))
            if states[-1]=='GEOMETRIC_LAP_PASS':break
        self.assertEqual(v.status,'GEOMETRIC_LAP_PASS')
        self.assertGreaterEqual(len(v.coverage),100)

    def test_no_false_lap_on_part_loop(self):
        xy,width=qacircle()
        v=LapVerifier(xy,width)
        for angle in np.linspace(0,math.pi,450):v.observe(20*math.cos(angle),20*math.sin(angle))
        self.assertNotEqual(v.status,'GEOMETRIC_LAP_PASS')

    def test_off_track_is_failure(self):
        xy,width=qacircle()
        v=LapVerifier(xy,width)
        for angle in np.linspace(0,.2,20):v.observe(20*math.cos(angle),20*math.sin(angle))
        for radius in (20.5,21.,21.5,21.9):
            v.observe(radius*math.cos(.2),radius*math.sin(.2))
        for _ in range(10):v.observe(21.9*math.cos(.2),21.9*math.sin(.2))
        self.assertEqual(v.status,'OFF_TRACK')

    def test_pose_jump_rejected(self):
        xy,width=qacircle()
        v=LapVerifier(xy,width)
        v.observe(20.,0.)
        self.assertEqual(v.observe(30.,4.),'POSE_JUMP')

if __name__=='__main__':unittest.main()
