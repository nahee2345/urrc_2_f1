import math
import tempfile
import unittest
from pathlib import Path
from strategy import Track, Planner, Vehicle

def ring():
    import csv
    from routes import generate
    points=[]
    for i in range(160):
        angle=2*math.pi*i/160
        points.append(dict(x_m=22*math.cos(angle),y_m=22*math.sin(angle),
            z_m=0.,yaw_rad=angle+math.pi/2,road_width_m=3.0))
    return generate(points)[0]

class StrategyTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory()
        import csv
        for i, data in ring().items():
            with (Path(self.temp.name)/f"lane_{i}.csv").open("w",newline="") as f:
                w=csv.DictWriter(f,fieldnames=list(data[0]))
                w.writeheader();w.writerows(data)
        self.track=Track(self.temp.name)
        self.planner=Planner(self.track,1)
    def tearDown(self): self.temp.cleanup()

    def car(self,index,lane=1,speed=4,stamp=1,name="ego"):
        p=self.track.lanes[lane][index]
        return Vehicle(p["x_m"],p["y_m"],p["yaw_rad"],speed,stamp,name)

    def test_15ms_straight_cap_and_corner_braking(self):
        self.track.curvature=[0.0]*self.track.n
        commands=[]
        for i in range(90):
            commands.append(self.planner.step(self.car(5,speed=15),[],1.0+i*.2))
        self.assertAlmostEqual(commands[-1].speed,15.0,places=5)
        self.assertTrue(all(0 <= c.speed <= 15.0 for c in commands))
        self.track.curvature=[.2]*self.track.n
        reduced=self.planner.step(self.car(5,speed=15),[],19.2)
        self.assertLess(reduced.speed,15.0)
    def test_wait_gate(self):
        cmd=self.planner.step(self.car(5),[],1,False)
        self.assertEqual(cmd.speed,0)
        self.assertEqual(cmd.mode,"WAIT_SIGNAL")

    def test_brake_approaching_car(self):
        ego=self.car(5,1,5,1)
        front=self.car(10,1,1,1,"front")
        cmd=self.planner.step(ego,[front],1)
        self.assertEqual(cmd.mode,"PASS")
        self.assertEqual(cmd.lane,2)

    def test_only_defend_if_no_front(self):
        ego=self.car(20,1,4,1)
        rear=self.car(10,2,10,1,"rear")
        cmd=self.planner.step(ego,[rear],1)
        self.assertEqual(cmd.mode,"DEFEND")
        self.assertEqual(cmd.lane,2)

    def test_sharp_corner_disables_maneuver(self):
        self.track.curvature=[.2]*self.track.n
        cmd=self.planner.step(self.car(8),[self.car(10,1,1,1,"front")],1)
        self.assertNotEqual(cmd.mode,"PASS")
        self.assertLess(cmd.speed,10)

    def test_stale_pose_filtered(self):
        cmd=self.planner.step(self.car(8),[self.car(9,1,1,0,"old")],1)
        self.assertNotEqual(cmd.mode,"PASS")

if __name__=="__main__":
    unittest.main()
