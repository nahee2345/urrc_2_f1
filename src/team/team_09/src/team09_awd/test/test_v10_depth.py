import unittest,math
from unittest.mock import Mock
import numpy as np
from team09_awd.depth import depth_meters,project,rgbd_preview,fit_depth_route
from team09_awd.vision import CAMERA_PITCH
from team09_awd.safe_log import BestEffortLog

class DepthTests(unittest.TestCase):
    def test_units_invalid(self):
        d=depth_meters(np.array([[1000,0,65535]]),'16UC1')
        self.assertEqual(d[0,0],1);self.assertTrue(np.isnan(d[0,1:]).all())
        with self.assertRaises(ValueError):depth_meters(np.ones((2,2)),'rgb8')
    def test_projection(self):
        p=project([[320,240]],[2],(400,400,320,240))[0]
        np.testing.assert_allclose(p,[.18+2*math.cos(CAMERA_PITCH),0,.30-2*math.sin(CAMERA_PITCH)])
    def test_synthetic_road(self):
        h,w=480,640;k=(400,400,320,240);v,u=np.mgrid[:h,:w]
        dz=-math.sin(CAMERA_PITCH)-math.cos(CAMERA_PITCH)*(v-240)/400
        with np.errstate(divide='ignore',invalid='ignore'):d=-.365/dz
        left=-(u-320)*d/400
        valid=(d>.25)&(d<30)&(abs(left)<1.2)
        rgb=np.zeros((h,w,3),np.uint8);rgb[valid]=100
        depth=np.where(valid,d,np.nan)
        route,obstacles,slope=rgbd_preview(rgb,depth,k)
        fit=fit_depth_route(route)
        self.assertIsNotNone(fit);self.assertLess(abs(slope),1e-5)
        self.assertLess(abs(np.polyval(fit[0],5)),.03)
        self.assertEqual(len(obstacles),0)
    def test_unknown_depth(self):
        r,obs,s=rgbd_preview(np.zeros((48,64,3),np.uint8),np.full((48,64),np.nan),(40,40,32,24))
        self.assertIsNone(r);self.assertEqual(len(obs),0)
    def test_bad_route(self):
        self.assertIsNone(fit_depth_route(np.array([[1,0,2]]*8)))
    def test_disk_failure(self):
        path=Mock();path.open.side_effect=OSError(28,'No space')
        log=BestEffortLog(path);log.write('test');log.flush();log.close()
        self.assertTrue(log.failed)

if __name__=='__main__':unittest.main()
