"""Sensor boundary and identity lifetime contracts, without pose truth input."""
import math
from pathlib import Path
import xml.etree.ElementTree as ET

from team10_race.geometry import Point2
from team10_race.lidar_processing import ScanPoint
from team10_race.opponent import OpponentCandidate, OpponentDetector, OpponentTracker
from team10_race.wall_tracker import PolynomialWall, WallObservation


def observation(x,y,stamp,source):
    return OpponentCandidate(x,y,math.hypot(x,y),.25,math.atan2(y,x),
        OpponentTracker._position(x,y),(10,11,12),timestamp=stamp,sources=(source,),extent=.2,point_count=3)


def test_rear_fov_boundary_is_sensor_origin_boundary_not_base_side_classification():
    root=ET.parse(Path(__file__).parents[1]/'models/team10_car/model.sdf').getroot()
    sensor=root.find("model/link[@name='lidar_link']/sensor[@name='rear_opponent_lidar']")
    horizontal=sensor.find('lidar/scan/horizontal')
    lower,upper=float(horizontal.findtext('min_angle')),float(horizontal.findtext('max_angle'))
    for y in [-.5,.5]:
        bearings=[math.atan2(math.sin(math.atan2(y,x+.245)-math.pi),
                             math.cos(math.atan2(y,x+.245)-math.pi)) for x in [-.35,-.245,-.20]]
        assert lower<=bearings[0]<=upper
        assert abs(abs(bearings[1])-math.pi/2)<1e-6
        assert not lower<=bearings[2]<=upper
    # The recorded C gap is well inside this field of view.
    bearing=math.atan2(-1.983,-4.675+.245)-math.pi
    bearing=math.atan2(math.sin(bearing),math.cos(bearing))
    assert lower<bearing<upper


def test_rear_side_front_continuous_detector_observation_keeps_one_id():
    detector=OpponentDetector(minimum_points=2);tracker=OpponentTracker()
    walls=WallObservation(PolynomialWall('left',(1.2,0,0),-1,12,60,.01),
                          PolynomialWall('right',(-1.2,0,0),-1,12,60,.01),2.4)
    centreline=[Point2(i*.1,0) for i in range(121)]
    positions=[]
    for i,x in enumerate([-1.,-.8,-.6,-.4,-.25,-.1,.1,.25,.4,.6,.8,1.],1):
        source='rear' if x<=-.245 else 'primary'
        points=[ScanPoint(10+j,math.hypot(x,.50+j*.03),Point2(x,.50+j*.03)) for j in range(3)]
        found=detector.detect(points,walls,centreline,dedicated_rear_sensor=source=='rear')
        assert len(found)==1
        candidate=observation(found[0].x,found[0].y,i*.1,source)
        tracks=tracker.update([candidate],.1,timestamp=i*.1)
        if i>=3:
            assert [t.track_id for t in tracks]==[1]
            positions.append(tracks[0].position)
    assert set(positions)=={'REAR','LEFT_SIDE','FRONT'}


def test_multisensor_handoff_is_independent_of_source_and_classification():
    tracker=OpponentTracker()
    for i,(x,source) in enumerate([(-.8,'rear'),(-.7,'rear'),(-.6,'rear'),
            (-.4,'secondary'),(-.2,'primary'),(0.,'secondary'),(.2,'primary'),(.4,'secondary'),(.6,'primary')],1):
        tracks=tracker.update([observation(x,.1,i*.1,source)],.1,timestamp=i*.1)
        if i>=3:assert [t.track_id for t in tracks]==[1]
    assert tracker._tracks[1].sources==('primary','rear','secondary')


def test_temporary_rear_sensor_loss_uses_visible_primary_without_new_id():
    tracker=OpponentTracker()
    for i in range(1,16):
        source='primary' if 5<=i<=11 else 'rear'
        tracks=tracker.update([observation(-2+i*.05,.1,i*.1,source)],.1,timestamp=i*.1)
        if i>=3:assert len(tracks)==1 and tracks[0].track_id==1 and tracks[0].missed==0


def test_true_all_sensor_loss_has_no_fake_candidate_or_fresh_threat():
    tracker=OpponentTracker()
    for i in range(1,5):tracker.update([observation(-3+i*.1,.1,i*.1,'rear')],.1,timestamp=i*.1)
    for i in range(5,10):
        tracks=tracker.update([],.1,timestamp=i*.1,raw_observations=[])
        assert len(tracks)==1 and tracks[0].missed==i-4 and tracks[0].closing_rate==0
        assert tracker.diagnostics['candidates']==[] and tracker._next_id==2


def test_genuinely_absent_expired_vehicle_returns_with_new_id():
    tracker=OpponentTracker()
    for i in range(1,5):tracker.update([observation(-3,.1,i*.1,'rear')],.1,timestamp=i*.1)
    for i in range(5,21):tracker.update([],.1,timestamp=i*.1)
    assert tracker._tracks=={}
    for i in range(21,25):tracks=tracker.update([observation(-3,.1,i*.1,'rear')],.1,timestamp=i*.1)
    assert [t.track_id for t in tracks]==[2]


def test_single_measured_beam_preserves_confirmed_id_through_partial_occlusion():
    tracker=OpponentTracker()
    for i in range(1,5):tracker.update([observation(-6,.1,i*.1,'rear'),observation(-3,.1,i*.1,'rear')],.1,timestamp=i*.1)
    rear_id=next(t.track_id for t in tracker._tracks.values() if t.x<-5)
    for i in range(5,31):
        points=[] if 10<=i<=15 else [ScanPoint(179,3.,Point2(-3,.1)),ScanPoint(180,6.,Point2(-6,.1))]
        tracks=tracker.update([observation(-3,.1,i*.1,'rear')],.1,timestamp=i*.1,raw_observations=[('rear',i*.1,points)])
        assert {t.track_id for t in tracks}=={1,2}
        rear=next(t for t in tracks if t.track_id==rear_id)
        assert rear.missed==(i-9 if 10<=i<=15 else 0)
    assert len(tracker._tracks)==2


def test_single_beam_cannot_birth_or_confirm_a_ghost_vehicle():
    tracker=OpponentTracker()
    for i in range(1,20):
        assert tracker.update([],.1,timestamp=i*.1,
            raw_observations=[('rear',i*.1,[ScanPoint(180,6.,Point2(-6,.1))])])==[]
    assert tracker._tracks=={}


def test_single_beam_outside_physical_footprint_cannot_reacquire_confirmed_id():
    tracker=OpponentTracker()
    for i in range(1,5):tracker.update([observation(-6,.1,i*.1,'rear')],.1,timestamp=i*.1)
    tracks=tracker.update([],.1,timestamp=.5,
        raw_observations=[('rear',.5,[ScanPoint(180,5.3,Point2(-5.3,.1))])])
    assert len(tracks)==1 and tracks[0].missed==1 and tracks[0].closing_rate==0


def test_single_beam_does_not_merge_two_nearby_distinct_vehicles():
    tracker=OpponentTracker()
    for i in range(1,5):tracker.update([observation(-6,0,i*.1,'rear'),observation(-6,.48,i*.1,'rear'),observation(-3,.24,i*.1,'rear')],.1,timestamp=i*.1)
    rear_ids={t.track_id for t in tracker._tracks.values() if t.x<-5}
    points=[ScanPoint(179,3.,Point2(-3,0)),ScanPoint(180,6.,Point2(-6,0)),
            ScanPoint(199,3.,Point2(-3,.48)),ScanPoint(200,6.,Point2(-6,.48))]
    tracks=tracker.update([],.1,timestamp=.5,raw_observations=[('rear',.5,points)])
    assert {t.track_id for t in tracks}=={1,2,3}
    assert all(t.missed==0 for t in tracks if t.track_id in rear_ids)


def test_isolated_single_return_cannot_keep_a_real_track_alive_indefinitely():
    tracker=OpponentTracker()
    for i in range(1,5):tracker.update([observation(-6,.1,i*.1,'rear')],.1,timestamp=i*.1)
    for i in range(5,21):
        tracks=tracker.update([],.1,timestamp=i*.1,
            raw_observations=[('rear',i*.1,[ScanPoint(180,6.,Point2(-6,.1))])])
        assert all(t.missed>0 and t.closing_rate==0 for t in tracks)
    assert tracker._tracks=={}


def test_forward_mount_raw_rear_flank_cannot_steal_an_existing_id():
    tracker=OpponentTracker()
    for i in range(1,5):tracker.update([observation(-1.3,-1.3,i*.1,'rear')],.1,timestamp=i*.1)
    points=[ScanPoint(200+j,math.hypot(-1.3,-1.3+j*.03),Point2(-1.3,-1.3+j*.03)) for j in range(3)]
    tracks=tracker.update([],.1,timestamp=.5,raw_observations=[('primary',.5,points)])
    assert len(tracks)==1 and tracks[0].track_id==1 and tracks[0].missed==1


def test_single_wall_endpoint_inside_footprint_cannot_fake_rear_observation():
    tracker=OpponentTracker()
    for i in range(1,5):tracker.update([observation(-6,.1,i*.1,'rear')],.1,timestamp=i*.1)
    points=[ScanPoint(180,6.,Point2(-6,.1))]
    points.extend(ScanPoint(200+i,math.hypot(-6+i*.1,.20),Point2(-6+i*.1,.20)) for i in range(12))
    tracks=tracker.update([],.1,timestamp=.5,raw_observations=[('rear',.5,points)])
    assert len(tracks)==1 and tracks[0].missed==1 and tracks[0].closing_rate==0
