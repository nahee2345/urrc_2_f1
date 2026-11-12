"""Pure Python front/rear sensor and 3-route strategy priority tests."""
from math import sin,cos,atan2,hypot
import pathlib,sys,unittest
ROOT=pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from team11_race.perception import LaserTracker,Actor,extract_clusters
from team11_race.strategy import RaceDirector
from team11_race.planner import Planner


class FakeRoute:
    length=1000.
    meta={"route_spacing_m":.765}
    center=[(float(i),0.,0.,0.,2.) for i in range(100)]
    last_index=30

    def sample(self,lane,s):
        return (float(s)%self.length, (.765,0.,-.765)[lane],0.)
    def curvature(self,s):
        return getattr(self,"curve",0.)


def actor(uid,x,y,speed,source="front",hits=4):
    return Actor(uid,x,y,speed,hits,1.,x,y,source)


def planner(lane=1,speed=10.,travel=35.):
    p=Planner(FakeRoute(),lane,external_strategy=True)
    p.last_s=30.
    p.speed=speed
    p.travel=travel
    return p


def laser_frame(x,y,source="front"):
    amin=-2.18 if source=="front" else -1.57
    step=(4.36 if source=="front" else 3.14)/360
    r=[float("inf")]*361
    sx=x-.187 if source=="front" else -(x+.208)
    sy=y if source=="front" else -y
    angle=atan2(sy,sx)
    center=round((angle-amin)/step)
    length=hypot(sx,sy)
    for i in range(center-2,center+3):
        if 0<=i<len(r):r[i]=length
    return r,amin,step


class StrategyTests(unittest.TestCase):
    def test_priority_front_before_corner_and_rear(self):
        p=planner()
        p.route.curve=.035
        targets=[actor(1,42.,0.,5.),
                 actor(2,18.,-.765,18.,"rear")]
        d=RaceDirector().choose(p,(30.,0.,0.,0.),targets,1.)
        self.assertEqual(d.reason,"FRONT_PASS")
        self.assertEqual(d.lane,0)
        self.assertLess(d.ttc,3.)

    def test_collision_risk_blocks_front_overtake_even_with_ttc_below_three(self):
        # The rear car would collide with the leftward change during the
        # maneuver, so safety must outrank the front-passing preference.
        p=planner()
        d=RaceDirector().choose(
            p,(30.,0.,0.,0.),
            [actor(1,42.,0.,5.),actor(2,25.,-.765,18.,"rear")],1.)
        self.assertEqual(d.reason,"FRONT_FOLLOW")
        self.assertIsNone(d.lane)

    def test_no_early_overtake_or_distant_car(self):
        p=planner()
        targets=[actor(1,45.5,0.,8.)]
        d=RaceDirector().choose(p,(30.,0.,0.,0.),targets,1.)
        self.assertEqual(d.reason,"KEEP_ROUTE")
        self.assertIsNone(d.lane)

    def test_corner_takes_center_priority(self):
        p=planner(lane=0)
        p.route.curve=.05
        d=RaceDirector().choose(p,(30.,.765,0.,0.),[],1.)
        self.assertEqual(d.reason,"CORNER_CENTER")
        self.assertEqual(d.lane,1)

    def test_rear_defense_after_front_checks(self):
        p=planner()
        d=RaceDirector().choose(p,(30.,0.,0.,0.),
                                [actor(2,25.,-.765,17.,"rear")],1.)
        self.assertEqual(d.reason,"REAR_DEFENSE")
        self.assertIn(d.lane,(0,2))
        self.assertLess(d.ttc,5.)

    def test_rear_defense_forbidden_during_corner(self):
        p=planner()
        p.route.curve=.05
        d=RaceDirector().choose(p,(30.,0.,0.,0.),
                                [actor(2,25.,-.765,17.,"rear")],1.)
        self.assertEqual(d.reason,"KEEP_ROUTE")

    def test_lane_change_is_committed(self):
        p=planner()
        self.assertTrue(p.request_lane(0,"PASS"))
        self.assertFalse(p.request_lane(2,"SECOND"))
        self.assertTrue(p.changing)
        self.assertEqual(p.change_reason,"PASS")

    def test_side_vehicle_motion_measured_by_real_laser_frames(self):
        tracker=LaserTracker()
        ego=(0.,0.,0.,0.)
        for step,x in enumerate((1.5,1.65,1.86,2.05)):
            data,amin,dang=laser_frame(x,1.25)
            tracker.update(data,amin,dang,.1,16.,"front",ego,0.,
                           1.+step*.10)
        self.assertTrue(tracker.neighbor_present(ego,1.3))
        self.assertTrue(tracker.neighbor_motion(ego,1.3))
        self.assertGreater(len(tracker.visible(1.3)),0)

    def test_unconfirmed_single_laser_return_is_not_neighbor(self):
        tracker=LaserTracker()
        data,amin,dang=laser_frame(1.5,1.25)
        tracker.update(data,amin,dang,.1,16.,"front",(0.,0.,0.,0.),0.,1.)
        self.assertFalse(tracker.neighbor_motion((0.,0.,0.,0.),1.))


if __name__=="__main__":
    unittest.main()
