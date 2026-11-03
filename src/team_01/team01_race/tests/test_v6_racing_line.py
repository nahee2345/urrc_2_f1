"""v6 sensor-only regression tests; no physical Gazebo assertions."""
import math
from pathlib import Path
import sys
import unittest
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from team01_race.planning import SafetyPlanner, RoadCorridor
from team01_race.race_strategy import RacingLine,LapLearner,base_link_road_distances
from team01_race.road_surface import transverse_start_marking
from test_offline import CFG


def road(curve=0.,width=.95,offset=.0):
    x=np.arange(.55,5.5,.25)
    center=curve*x*x+offset
    return RoadCorridor(np.column_stack((x,center)),width,True,
                        math.degrees(math.atan(3*curve)),
                        np.column_stack((x,center-width,center+width)))


def road_bounds(curve=0.,width=.95,offset=.0):
    return road(curve,width,offset).bounds


class RacingV6Tests(unittest.TestCase):
    def test_center_distances_are_from_base_link(self):
        d=base_link_road_distances(road(offset=.13,width=.9))
        self.assertTrue(d.valid)
        self.assertAlmostEqual(d.left_m,1.03,places=3)
        self.assertAlmostEqual(d.right_m,.77,places=3)
        self.assertAlmostEqual(d.midpoint_m,.13,places=3)

    def test_curb_missing_does_not_claim_center(self):
        c=RoadCorridor(np.array([[3.,0],[4.,0],[5.,0]]),1.,True,0.)
        self.assertFalse(base_link_road_distances(c).valid)

    def test_grid_straight_requires_distance_and_repeated_curb_confirmation(self):
        s=RacingLine({**CFG,'path_half_width_m':.18,'road_confirm_frames':6})
        for _ in range(4):s.update(road(),.65,.1)
        self.assertEqual(s.phase,'GRID')
        for _ in range(20):s.update(road(),.65,.1)
        self.assertEqual(s.phase,'CENTER')
        self.assertAlmostEqual(s.offset_m,0.)

    def test_disappearing_road_prevents_corner_race_line(self):
        s=RacingLine({**CFG,'path_half_width_m':.18})
        for _ in range(20):s.update(road(),.65,.1)
        s.update(road(curve=.085),1.5,.1)
        missing=RoadCorridor(np.empty((0,2)),0.,False,0.)
        s.update(missing,1.,.1)
        self.assertEqual(s.offset_m,0.)

    def test_out_in_out_phase_respects_width_and_slew(self):
        cfg={**CFG,'path_half_width_m':.18,'road_confirm_frames':6}
        s=RacingLine(cfg)
        for _ in range(25):s.update(road(),.65,.1)
        self.assertEqual(s.phase,'CENTER')
        offsets=[];phases=[]
        for _ in range(38):
            s.update(road(curve=.085),1.5,.1)
            offsets.append(s.offset_m); phases.append(s.phase)
        self.assertIn('CORNER_ENTRY',phases)
        self.assertIn('CORNER_APEX',phases)
        self.assertIn('CORNER_EXIT',phases)
        self.assertLess(min(offsets),-.06)
        self.assertGreater(max(offsets),.06)
        self.assertLessEqual(max(abs(v) for v in offsets),.24)
        self.assertLessEqual(max(abs(b-a) for a,b in zip(offsets,offsets[1:])),.0551)

    def test_no_unsafe_offset_when_road_narrow(self):
        s=RacingLine({**CFG,'path_half_width_m':.18})
        for _ in range(20):s.update(road(width=.55),.65,.1)
        s.update(road(curve=.085,width=.55),1.,.1)
        self.assertLess(abs(s.offset_m),.20)

    def test_initial_planner_grid_is_not_old_left_right_weave(self):
        p=SafetyPlanner({**CFG,'require_visual_road':True})
        p.cruise_side=-1
        cmd=p.step(np.empty((0,2)),np.empty((0,2)),5.,.1,True,True,True,road_bounds())
        self.assertEqual(cmd.mode,'GRID')
        self.assertLessEqual(cmd.speed,.21)
        self.assertAlmostEqual(p.target_lane_offset,0.)
        self.assertAlmostEqual(cmd.steering,0.,delta=.02)

    def test_center_tracking_after_discovery_uses_both_curbs(self):
        p=SafetyPlanner({**CFG,'require_visual_road':True})
        for _ in range(32):p.step(np.empty((0,2)),np.empty((0,2)),5.,.1,True,True,True,road_bounds())
        self.assertEqual(p.racing_line.phase,'CENTER')
        cmd=p.step(np.empty((0,2)),np.empty((0,2)),5.,.1,True,True,True,road_bounds(offset=.2))
        self.assertLess(cmd.steering,0.)
        self.assertGreater(p.racing_line.last_distances.left_m,p.racing_line.last_distances.right_m)

    def test_corner_speed_is_thirty_percent_even_if_steer_shallow(self):
        p=SafetyPlanner({**CFG,'require_visual_road':True})
        for _ in range(32):p.step(np.empty((0,2)),np.empty((0,2)),5.,.1,True,True,True,road_bounds())
        cmd=p.step(np.empty((0,2)),np.empty((0,2)),5.,.1,True,True,True,road_bounds(curve=.085))
        self.assertTrue(p.racing_line.corner)
        # Physical braking is bounded at 5 m/s²; do not instantaneously jump
        # from 4m/s to 1.5m/s in a single 0.1 s update.
        for _ in range(15):
            cmd=p.step(np.empty((0,2)),np.empty((0,2)),5.,.1,True,True,True,road_bounds(curve=.085))
        self.assertLessEqual(cmd.speed,1.5+1e-6)

    def test_lap_not_confirmed_by_distance_or_time_alone(self):
        l=LapLearner(CFG)
        ref=road()
        l.integrate(.1,0.,.1,.34,ref,'STRAIGHT')
        l.distance_m=400.;l.elapsed_s=400.;l.yaw_rad=0.
        self.assertFalse(l.observe_line(True,ref))
        self.assertEqual(l.lap,1)

    def test_first_lap_requires_line_edge_and_signature_and_turn(self):
        l=LapLearner(CFG)
        l.integrate(.1,0.,.1,.34,road(),'STRAIGHT')
        l.distance_m=500.;l.elapsed_s=180.;l.yaw_rad=math.tau
        self.assertTrue(l.observe_line(True,road()))
        self.assertEqual(l.lap,2)
        l.distance_m+=5. # Skip start-up segment's low-speed first-lap record.
        self.assertAlmostEqual(l.speed_multiplier(True,'STRAIGHT',True),1.10)
        self.assertFalse(l.observe_line(True,road()))

    def test_lap_rejects_mismatched_curb_signature(self):
        l=LapLearner(CFG)
        l.integrate(.1,0.,.1,.34,road(),'STRAIGHT')
        l.distance_m=550.;l.elapsed_s=180.;l.yaw_rad=math.tau
        self.assertFalse(l.observe_line(True,road(offset=.7)))
        self.assertEqual(l.lap,1)

    def test_learned_difficult_sections_do_not_speedup(self):
        l=LapLearner(CFG)
        l.lap=2;l.first_lap_distance=500.;l.distance_m=501.
        l.safe_record_by_bin[0]=(.0,0.,'BRAKE')
        self.assertEqual(l.speed_multiplier(True,'STRAIGHT',True),1.)

    def test_startline_is_not_all_white_frame(self):
        uv=[];xy=[]
        img=np.full((200,200,3),[44,46,49],dtype=np.uint8)
        k=0
        for x in np.arange(.5,2.8,.04):
            for y in np.arange(-.75,.75,.09):
                row,col=divmod(k,200)
                if 1.12<x<1.23:img[row,col]=[215,215,215]
                xy.append([x,y]);uv.append([row,col]);k+=1
        # Use a wide enough image to accommodate every synthetic sample.
        img=np.full((200,1200,3),[44,46,49],dtype=np.uint8)
        for i,(x,y) in enumerate(xy):
            row,col=divmod(i,1200)
            uv[i]=[row,col]
            if 1.12<x<1.23:img[row,col]=[220,220,220]
        bounds=np.column_stack((np.arange(.5,2.8,.20),
                   np.full(12,-.85),np.full(12,.85)))
        self.assertTrue(transverse_start_marking(xy,uv,img,bounds))
        img[:]=[220,220,220]
        self.assertFalse(transverse_start_marking(xy,uv,img,bounds))

if __name__=='__main__':unittest.main()
