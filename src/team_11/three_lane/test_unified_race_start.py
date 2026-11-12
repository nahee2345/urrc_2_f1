"""Official start and 1..11 starting-grid camera field-of-view regression."""
import json, math, pathlib, sys, unittest
import xml.etree.ElementTree as ET
ROOT=pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from team11_race.start_policy import choose_start
from team11_race.camera_start import LightTransition


class StartPolicyTests(unittest.TestCase):
    def test_no_start_before_official_gate_even_if_neighbor_moves(self):
        for camera in (False,True):
            for moving in (False,True):
                self.assertIsNone(choose_start("camera",False,camera,moving,True,99.))

    def test_primary_camera_then_neighbor_then_gate(self):
        self.assertEqual(choose_start("camera",True,True,True,True,.01),
                         "CAMERA_LIGHTS_OUT")
        self.assertEqual(choose_start("camera",True,False,True,True,.01),
                         "NEIGHBOR_MOVED_AFTER_GATE")
        self.assertIsNone(choose_start("camera",True,False,False,True,.15))
        self.assertEqual(choose_start("camera",True,False,False,False,.80),
                         "OFFICIAL_GATE_SOLO")
        self.assertEqual(choose_start("camera",True,False,False,True,2.05),
                         "OFFICIAL_GATE_TIMEOUT")
        self.assertEqual(choose_start("signal",True,False,False,True,0.),
                         "OFFICIAL_SIGNAL")

    def test_competition_uses_advanced_straight_speed_limit(self):
        source=(ROOT/"team11_race/control.py").read_text()
        self.assertIn('("max_speed_mps",15.0)',source)
        self.assertIn("RaceDirector()",source)

    def test_four_or_five_red_then_all_off(self):
        model=LightTransition()
        for frame in (16,24):
            self.assertFalse(model.update(frame))
        self.assertTrue(model.saw_red)
        for _ in range(3):self.assertFalse(model.update(0))
        self.assertTrue(model.update(0))

    def test_cam_view_approx_for_first_eleven_grids_each_track(self):
        # The stored start-finish waypoint approximates the gantry XY, as in
        # the practice route generator. Confirm basic optical coverage only;
        # real Gazebo rendering still needs independent confirmation.
        model=ET.parse(ROOT/"models/team11_car/model.sdf").getroot().find("model")
        sensor=model.find("link[@name='base_link']/sensor[@name='front_rgb']")
        self.assertIsNotNone(sensor)
        camera=sensor.find("camera")
        hfov=float(camera.findtext("horizontal_fov"))
        w=float(camera.findtext("image/width"))
        h=float(camera.findtext("image/height"))
        vhalf=math.atan(math.tan(hfov/2)*h/w)
        pos=[float(v) for v in sensor.findtext("pose").split()]
        z_model=float(model.find("link[@name='base_link']").findtext("pose").split()[2])
        for track in ("monza","spa","monaco","cota","hungaroring","baku"):
            map_data=json.loads((ROOT/"maps"/track/"track.json").read_text())
            finish=map_data["start_finish"]
            for grid in map_data["grid"][:11]:
                with self.subTest(track=track,grid=grid["position"]):
                    yaw=float(grid["yaw"])
                    cx=grid["x"]+pos[0]*math.cos(yaw)
                    cy=grid["y"]+pos[0]*math.sin(yaw)
                    dx=finish["x"]-cx
                    dy=finish["y"]-cy
                    heading=math.atan2(dy,dx)-yaw
                    heading=math.atan2(math.sin(heading),math.cos(heading))
                    dz=(finish["z"]+1.96)-(grid["road_z"]+z_model+pos[2])
                    elevation=math.atan2(dz,math.hypot(dx,dy))
                    self.assertLess(abs(heading),hfov/2)
                    self.assertLess(abs(elevation+pos[4]),vhalf)


if __name__=="__main__":
    unittest.main()
