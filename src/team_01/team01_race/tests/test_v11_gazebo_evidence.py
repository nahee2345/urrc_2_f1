import math
from pathlib import Path
import sys
import unittest
import xml.etree.ElementTree as ET
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from team01_race.planning import RoadCorridor, fuse_verified_corridors
from team01_race.curved_path import adaptive_pursuit_candidate
from team01_race.lap_validation import LapVerifier
from team01_race.race_strategy import base_link_road_distances
from test_v9_startup_lap_exploration import CFG
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from safe_grid_spawn import adjusted_grid


def corridors(vend=2.35, lend=7.7, bend=0., visual_center=0.):
    x=np.arange(.65,vend,.2)
    cy=bend*x*x+visual_center
    bounds=np.column_stack((x,cy-1.,cy+1.))
    v=RoadCorridor(np.column_stack((x,cy)),1.,True,0.,bounds)
    lxs=np.arange(.75,lend,.4)
    lcy=bend*lxs*lxs
    lidar=RoadCorridor(np.column_stack((lxs,lcy)),1.,True,0.)
    return v,lidar

class V11Evidence(unittest.TestCase):
    def test_sim8_negative_angular_velocity_not_clipped(self):
        model=Path(__file__).resolve().parents[1]/'models/team01_car/model.sdf'
        root=ET.parse(model).getroot()
        plugin=root.find(".//plugin[@name='gz::sim::systems::AckermannSteering']")
        self.assertIsNotNone(plugin)
        self.assertLess(float(plugin.findtext('min_velocity')),0)
        self.assertGreater(float(plugin.findtext('max_velocity')),0)

    def test_visual_and_lidar_continuity_extends_visibility(self):
        v,l=corridors()
        fuse=fuse_verified_corridors(v,l)
        self.assertIsNotNone(fuse)
        self.assertGreater(fuse.centers[-1,0],5.)
        self.assertLessEqual(fuse.half_width,1.)

    def test_fusion_rejects_misaligned_lidar(self):
        v,l=corridors(visual_center=.8)
        self.assertIsNone(fuse_verified_corridors(v,l))

    def test_fusion_rejects_single_unverified_wall(self):
        v,l=corridors()
        l.valid=False
        self.assertIsNone(fuse_verified_corridors(v,l))

    def test_fusion_does_not_invent_extra_road_without_reach(self):
        v,l=corridors(lend=2.8)
        self.assertIsNone(fuse_verified_corridors(v,l))

    def test_extended_depth_lidar_corridor_still_checks_footprint(self):
        v,l=corridors()
        f=fuse_verified_corridors(v,l)
        self.assertIsNotNone(f)
        good=adaptive_pursuit_candidate(f,np.empty((0,2)),CFG,5.,horizon=6.)
        self.assertIsNotNone(good)
        self.assertGreater(good.lengths[-1],3.)
        bad=adaptive_pursuit_candidate(f,np.array([[1.,0.]]),CFG,5.,horizon=6.)
        self.assertIsNone(bad)

    def test_grid_adjustment_is_small_and_measured(self):
        angles=np.linspace(0,2*np.pi,200,endpoint=False)
        pts=np.column_stack((20*np.cos(angles),20*np.sin(angles)))
        widths=np.full(len(pts),2.4)
        x,y=21.10,0.
        a=adjusted_grid(pts,widths,x,y,math.pi/2)
        self.assertGreater(a['shift_m'],0.0)
        self.assertLess(a['shift_m'],.45)
        self.assertLessEqual(abs(a['new_lateral_m']),.87+1e-6)
        self.assertAlmostEqual(a['yaw'],math.pi/2,places=5)

    def test_grid_already_safe_remains_unchanged(self):
        ang=np.linspace(0,2*np.pi,200,endpoint=False)
        pts=np.column_stack((20*np.cos(ang),20*np.sin(ang)))
        a=adjusted_grid(pts,np.full(len(pts),2.4),20.0,0.0,0.0)
        self.assertAlmostEqual(a['shift_m'],0.,places=2)

    def test_lap_verifier_remains_strict(self):
        ang=np.linspace(0,2*np.pi,200,endpoint=False)
        pts=np.column_stack((20*np.cos(ang),20*np.sin(ang)))
        v=LapVerifier(pts,np.full(len(pts),2.4))
        for _ in range(9):v.observe(21.4,0.)
        self.assertEqual(v.status,'OFF_TRACK')

if __name__=='__main__':unittest.main()
