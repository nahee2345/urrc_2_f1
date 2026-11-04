"""105 degree RGB aligned geometrically with unchanged 92 degree Depth."""
import sys
from pathlib import Path
import unittest
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from team01_race.rgb_depth_projection import depth_uv_in_rgb,red_white_wall_points
from team01_race.perception import depth_geometry
from team01_race.road_surface import dark_asphalt_bounds
from test_v4_road_and_overtake import mock_asphalt


class V24(unittest.TestCase):
    def test_center_and_outer_depth_pixels_project_inside_wide_rgb(self):
        uv=np.array([[120,212],[120,0],[120,423],[0,212]],int)
        mapped,valid=depth_uv_in_rgb(uv,(240,424),(240,424),92.,105.)
        self.assertTrue(all(valid))
        self.assertLess(abs(mapped[0,1]-212),2)
        self.assertGreater(mapped[1,1],0)
        self.assertLess(mapped[2,1],423)

    def test_legacy_equal_fov_is_identity(self):
        uv=np.array([[3,20],[200,402],[120,212]])
        mapped,valid=depth_uv_in_rgb(uv,(240,424),(240,424),92.,92.)
        np.testing.assert_array_equal(mapped,uv)
        self.assertTrue(np.all(valid))

    def test_asphalt_requires_reprojected_pixels(self):
        scene,road,orig=mock_asphalt(width=.82)
        self.assertTrue(road.valid)
        broad=np.broadcast_to(np.array((73,79,68),dtype=np.uint8),
                              (240,424,3)).copy()
        new_uv,inside=depth_uv_in_rgb(scene.ground_uv,orig.shape,
                                       broad.shape,92.,105.)
        target=np.abs(scene.ground_xy[:,1])<=.82
        selected=new_uv[target&inside]
        broad[selected[:,0],selected[:,1]]=(42,44,47)
        fitted=dark_asphalt_bounds(scene.ground_xy,scene.ground_uv,broad,
                                  depth_shape=(240,424),depth_hfov_deg=92.,
                                  rgb_hfov_deg=105.)
        self.assertTrue(fitted.valid)
        self.assertLess(float(np.median(fitted.bounds[:,2]-
                                         fitted.bounds[:,1])),1.95)

    def test_colored_wall_must_also_be_raised(self):
        frame=np.zeros((240,424,3),dtype=np.uint8)
        depth_uv=np.array([[120,150],[120,250],[120,350]])
        mapped,_=depth_uv_in_rgb(depth_uv,frame.shape,frame.shape,92.,105.)
        frame[mapped[:,0],mapped[:,1]]=(220,24,24)
        pts=np.array([[3.,-1.],[3.,1.],[4.,1.]])
        out=red_white_wall_points(pts,depth_uv,np.array([.2,.03,.2]),
                                   frame,frame.shape)
        self.assertEqual(len(out),2)


if __name__=='__main__':unittest.main()
