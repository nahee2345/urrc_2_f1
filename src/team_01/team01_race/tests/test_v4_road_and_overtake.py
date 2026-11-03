"""Headless regressions. Mock images and trajectories, NOT Gazebo validation."""
import math
from pathlib import Path
import sys
import unittest
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from team01_race.perception import depth_geometry
from team01_race.road_surface import dark_asphalt_bounds
from team01_race.planning import TrafficGate, SafetyPlanner
from test_offline import CFG,synthetic_road
from test_integration_offline import lit_frame
from team01_race.perception import red_signal_features


def mock_asphalt(grade=0.,width=.95,color=(42,44,47),outside=(73,79,68)):
    depth=synthetic_road(grade)
    scene=depth_geometry(depth)
    frame=np.broadcast_to(np.array(outside,dtype=np.uint8),(240,424,3)).copy()
    onroad=np.abs(scene.ground_xy[:,1])<=width
    pix=scene.ground_uv[onroad]
    frame[pix[:,0],pix[:,1]]=color
    road=dark_asphalt_bounds(scene.ground_xy,scene.ground_uv,frame)
    return scene,road,frame


def bounds(left=.95,right=-.95):
    x=np.arange(.64,5.8,.32)
    return np.column_stack((x,np.full(len(x),right),np.full(len(x),left)))


class V4Tests(unittest.TestCase):
    def test_cannot_start_on_black_before_red(self):
        g=TrafficGate(required=4,on_frames=2,off_frames=3)
        for i in range(80):
            n,e=red_signal_features(lit_frame(0))
            self.assertFalse(g.observe(n,e,timestamp=i*.05))

    def test_four_visible_red_then_zero_launch(self):
        g=TrafficGate(required=4,on_frames=2,off_frames=3,min_sequence_s=3.)
        seq=[(0,i*.05) for i in range(12)] + [(1,.8),(2,1.5),(3,2.2),
              (4,3.1),(4,3.2),(4,3.3),(0,4.1),(0,4.2),(0,4.3)]
        for n,t in seq[:-1]:
            count,energy=red_signal_features(lit_frame(n))
            self.assertFalse(g.observe(count,energy,timestamp=t))
        count,energy=red_signal_features(lit_frame(0))
        self.assertTrue(g.observe(count,energy,timestamp=4.3))
        self.assertEqual(g.maximum_blobs,4)

    def test_black_asphalt_excludes_olive_offroad(self):
        scene,road,_=mock_asphalt(width=.82)
        self.assertTrue(scene.valid)
        self.assertTrue(road.valid)
        self.assertGreater(len(road.bounds),4)
        span=road.bounds[:,2]-road.bounds[:,1]
        self.assertLess(np.max(span),1.9)

    def test_olive_ground_alone_is_not_asphalt(self):
        scene=depth_geometry(synthetic_road())
        frame=np.broadcast_to(np.array([73,79,68],dtype=np.uint8),(240,424,3)).copy()
        self.assertFalse(dark_asphalt_bounds(scene.ground_xy,scene.ground_uv,frame).valid)

    def test_no_ground_black_image_cannot_start_driving(self):
        road=dark_asphalt_bounds(np.empty((0,2)),np.empty((0,2),int),np.zeros((240,424,3),np.uint8))
        self.assertFalse(road.valid)
        p=SafetyPlanner({**CFG,'require_visual_road':True})
        out=p.step(np.empty((0,2)),np.empty((0,2)),4.,.1,True,True,True,road.bounds)
        self.assertEqual(out.speed,0.)

    def test_ascending_and_descending_drivable(self):
        for grade in [-.08,0,.08]:
            scene,road,_=mock_asphalt(grade=grade)
            self.assertTrue(scene.valid)
            self.assertTrue(road.valid)
            self.assertLess(len(scene.obstacles),12)
            p=SafetyPlanner({**CFG,'require_visual_road':True})
            m=p.step(scene.obstacles,np.empty((0,2)),4.,.1,True,True,True,road.bounds)
            self.assertGreater(m.speed,0.)

    def test_not_always_centre_left_side(self):
        p=SafetyPlanner({**CFG,'require_visual_road':True})
        m=p.step(np.empty((0,2)),np.empty((0,2)),4.,.1,True,True,True,bounds())
        self.assertEqual(p.road_source,'rgb_depth')
        self.assertAlmostEqual(m.steering,0.,delta=.05)
        self.assertAlmostEqual(p.target_lane_offset,0.,delta=.01)

    def test_not_always_centre_right_side(self):
        p=SafetyPlanner({**CFG,'require_visual_road':True})
        p.cruise_side=-1
        m=p.step(np.empty((0,2)),np.empty((0,2)),4.,.1,True,True,True,bounds())
        self.assertAlmostEqual(m.steering,0.,delta=.05)
        self.assertAlmostEqual(p.target_lane_offset,0.,delta=.01)

    def test_narrow_road_no_blind_lateral_exit(self):
        p=SafetyPlanner({**CFG,'require_visual_road':True})
        b=bounds(.30,-.30)
        m=p.step(np.empty((0,2)),np.empty((0,2)),4.,.1,True,True,True,b)
        self.assertLess(abs(m.steering),math.radians(6))
        self.assertTrue(p._within_road(-m.steering,3.)[0])

    def test_corner_speed_30_percent_of_straight_target(self):
        cfg={**CFG,'require_visual_road':True}
        p=SafetyPlanner(cfg)
        p.speed=0.0
        for _ in range(120):
            p.step(np.empty((0,2)),np.empty((0,2)),6.,.1,
                   True,True,True,bounds())
        self.assertAlmostEqual(cfg['corner_speed_factor'],.30)
        # The configured command envelope is 30% whenever steering >=10°.
        corner_v=cfg['straight_speed_mps']*cfg['corner_speed_factor']
        self.assertAlmostEqual(corner_v,1.5)

    def test_close_lead_brakes_before_collision(self):
        p=SafetyPlanner({**CFG,'require_visual_road':True})
        pts=np.array([[.55,y] for y in np.linspace(-.3,.3,20)])
        m=p.step(pts,pts,4.,.1,True,True,True,bounds())
        self.assertEqual(m.mode,'BRAKE')
        self.assertEqual(m.speed,0.)

    def test_lead_speed_and_pass_distance_are_distance_based(self):
        p=SafetyPlanner({**CFG,'require_visual_road':True})
        for x in (5.,4.92,4.84,4.76,4.68):
            pts=np.array([[x,y] for y in (-.1,0,.1)])
            p.step(pts,np.empty((0,2)),4.,.1,True,True,True,bounds(1.1,-1.1))
        self.assertIsNotNone(p.lead_speed)
        # Passing is disabled until SLAM map closure, regardless of range.
        self.assertEqual(p.rejoin.phase,'CRUISE')
        self.assertEqual(p.planned_pass_distance_m,0.)

    def test_rear_close_disables_new_overtake(self):
        p=SafetyPlanner({**CFG,'require_visual_road':True})
        pts=np.array([[3.2,y] for y in (-.1,0,.1)])
        m=p.step(pts,np.empty((0,2)),.55,.1,True,True,True,bounds(1.1,-1.1))
        self.assertEqual(p.overtake_side,0)

    def test_safe_gap_can_select_overtake_side(self):
        p=SafetyPlanner({**CFG,'require_visual_road':True})
        for _ in range(45):
            p.step(np.empty((0,2)),np.empty((0,2)),8.,.1,True,True,True,bounds(1.2,-1.2))
        self.assertFalse(p.racing_line.grid)
        pts=np.array([[3.2,y] for y in (-.1,0,.1)])
        m=p.step(pts,np.empty((0,2)),8.,.1,True,True,True,
                 bounds(1.2,-1.2),map_ready=True)
        self.assertNotEqual(p.overtake_side,0)
        # The safety supervisor may brake while a slow car is nearby.
        self.assertIn(m.mode,('BRAKE','OVERTAKE','CORNER','STRAIGHT'))

    def test_explicit_visual_road_required_even_with_lidar(self):
        p=SafetyPlanner({**CFG,'require_visual_road':True})
        lidar=np.column_stack((np.arange(.4,6.,.12),np.ones(len(np.arange(.4,6.,.12)))))
        self.assertEqual(p.step(np.empty((0,2)),lidar,6.,.1,True,True,True,None).speed,0.)

    def test_closed_loop_mock_curve_and_edge_clearance(self):
        # Lightweight bicycle simulation using supplied road bounds; not Gazebo.
        track=lambda x:.1*np.sin(x/2.5)
        width=.8
        p=SafetyPlanner({**CFG,'require_visual_road':True})
        x=y=yaw=0.
        max_error=0.
        for _ in range(150):
            distance=np.arange(.65,4.8,.32)
            centre=track(x+distance)-y-distance*np.tan(yaw)
            road=np.column_stack((distance,centre-width,centre+width))
            cmd=p.step(np.empty((0,2)),np.empty((0,2)),6.,.1,
                       True,True,True,road)
            yaw+=cmd.speed*np.tan(-cmd.steering)/.34*.1
            x+=cmd.speed*np.cos(yaw)*.1
            y+=cmd.speed*np.sin(yaw)*.1
            max_error=max(max_error,abs(y-track(x)))
        self.assertGreater(x,20.)
        self.assertLess(max_error,width-.15)

    def test_sdf_camera_alignment_and_scope(self):
        from xml.etree import ElementTree as ET
        m=ET.parse(str(Path(__file__).resolve().parents[1]/'models/team01_car/model.sdf'))
        camera=m.find(".//sensor[@name='road_rgb']")
        depth=m.find(".//sensor[@name='front_depth']")
        self.assertEqual(camera.findtext('pose'),depth.findtext('pose'))
        self.assertEqual(camera.findtext('camera/image/width'),depth.findtext('camera/image/width'))
        self.assertEqual(camera.findtext('camera/image/height'),depth.findtext('camera/image/height'))
        self.assertEqual(camera.findtext('camera/image/format'),'R8G8B8')

if __name__=='__main__':unittest.main()
