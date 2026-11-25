import unittest
import math
import sys
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from team09_awd.core import corridor_state,tracking_curvature,plan_local,speed_limit


def walls(offset=0.,heading=0.,noise=0.):
    x=np.linspace(-2,55,2200)
    rng=np.random.default_rng(42)
    return np.vstack([np.column_stack([x,offset+np.tan(heading)*x+y+rng.normal(0,noise,len(x))]) for y in (-1.5,1.5)])

class StraightControl(unittest.TestCase):
    def test_heading_separate_from_offset(self):
        e,h,q=corridor_state(walls(.35,.10,.015))
        self.assertAlmostEqual(e,.35,delta=.03);self.assertAlmostEqual(h,.10,delta=.09);self.assertGreater(q,.7)
    def test_both_error_signs(self):
        for sign in (-1,1):
            self.assertGreater(sign*tracking_curvature(sign*.4,sign*.12,8.),0)
    def test_fine_correction_not_coarse_candidate(self):
        k=plan_local(walls(.1),speed=8.).curvature
        self.assertGreater(k,0);self.assertLess(k,.015)
    def test_heading_corrected_even_when_lookahead_midpoint_is_zero(self):
        self.assertGreater(tracking_curvature(-.15,.10,10.),0)
    def test_gyro_damps_excess_turn(self):
        self.assertLess(tracking_curvature(.3,.05,8.,.4),tracking_curvature(.3,.05,8.,0.))
    def test_closed_loop_straight_converges(self):
        # Kinematic bicycle, 8m/s, 50ms updates; initial 0.6m offset and 0.12rad heading.
        y=.6;heading=.12;rate=0.;history=[]
        for _ in range(200):
            offset=-y/math.cos(heading)
            k=tracking_curvature(offset,-heading,8.,rate)
            desired=8.*k
            rate+=.3*(desired-rate)
            heading+=rate*.05;y+=8.*math.sin(heading)*.05;history.append(y)
        self.assertLess(abs(y),.45);self.assertLess(abs(heading),.09)
        # v04 deliberately settles slowly toward a +/-15cm central band.
        for _ in range(600):
            k=tracking_curvature(-y/math.cos(heading),-heading,8.,rate)
            rate+=.3*(8.*k-rate);heading+=rate*.05;y+=8.*math.sin(heading)*.05
        self.assertLess(abs(y),.17)
        self.assertLess(max(abs(v) for v in history),1.05)
    def test_continuous_command_under_noise(self):
        ks=[plan_local(walls(.1,noise=n),speed=8.).curvature for n in (0.,.005,.09,.015)]
        self.assertLess(max(ks)-min(ks),.005)
    def test_corridor_path_still_checks_obstacle(self):
        p=np.vstack([walls(),np.column_stack([np.full(100,3.),np.linspace(-1.5,1.5,100)])])
        self.assertLess(plan_local(p,speed=8.).free,3.1)
    def test_high_speed_needs_long_free_distance(self):
        self.assertLess(speed_limit(0,10.,1,18.),9.)
        self.assertEqual(speed_limit(0,48.,1,18.),18.)
    def test_parallel_wall_rejection(self):
        p=walls();p[p[:,1]>0,1]+=.4*np.maximum(0,p[p[:,1]>0,0])
        self.assertIsNone(corridor_state(p))

if __name__=='__main__':unittest.main()
