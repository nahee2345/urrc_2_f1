import json,math
from pathlib import Path
from types import SimpleNamespace
from team10_race.footprint_trajectory import plan_footprint_trajectory
from team10_race.lidar_processing import scan_to_points,continuous_segments
from team10_race.wall_tracker import WallObservation
from team10_race.geometry import Point2
from test_speed_planner import make_planner

def fixture(name='corridor_failure_20261119'):
 r=json.loads((Path(__file__).parent/'data'/f'{name}.json').read_text());s=SimpleNamespace(**r['scan']);s.ranges=[v if v is not None else math.inf for v in s.ranges]
 points=scan_to_points(s.ranges,s.angle_min,s.angle_increment,s.range_min,s.range_max,270,.25,24)
 return r,s,continuous_segments(points,.3,2,8)

def test_actual_failure_reproduces_backwards_path_and_rear_side_collision():
 r,s,seg=fixture();g=r['geometry'];path=[Point2(**p) for p in g['speed_path']];obs=[Point2(**p) for p in g['speed_obstacles']];p=make_planner();pts,d=p._preview(path)
 assert pts[0].x<0
 assert p._corridor_limit(pts,d,obs)==0
 assert r['corridor_hits'][0]['obstacle']['x']<0
 assert r['corridor_hits'][0]['long']<.55

def test_same_actual_scan_has_observed_forward_full_envelope_trajectory():
 r,s,seg=fixture();t=plan_footprint_trajectory(s,seg,WallObservation(None,None,1.72),r['geometry']['command']['steering_angle'],1.)
 assert t is not None and t.free_distance>=2.4
 assert t.minimum_clearance>=.08 and len(t.centerline.points)>=5
 assert t.centerline.raw_point_count>=8
 assert 0<t.steering<=.488692
 p=make_planner();plan=p.plan(t.centerline.points,[],t.steering,t.steering,.1,swept_free_distance_m=t.free_distance)
 assert plan.speed_limit_from_corridor>0
 stopped=p.plan(t.centerline.points,[],0,0,.1,swept_free_distance_m=0)
 assert stopped.speed_limit_from_corridor==0

def test_actual_later_stopped_pose_has_no_safe_forward_path():
 r,s,seg=fixture('stopped_no_wall_20261119')
 assert plan_footprint_trajectory(s,seg,WallObservation(None,None,1.72),0,0) is None
 assert plan_footprint_trajectory(s,[],WallObservation(None,None,1.72),0,0) is None


def test_actual_no_return_scan_is_observed_free_range_not_invalid_data():
    q=json.loads((Path(__file__).parent/'data/no_return_20261122.json').read_text())
    s=SimpleNamespace(**q);s.ranges=[v if v is not None else math.nan for v in q['ranges']]
    assert len(q['positive_infinity_indices'])==9 and not q['nan_indices']
    for i in q['positive_infinity_indices']:s.ranges[i]=math.inf
    points=scan_to_points(s.ranges,s.angle_min,s.angle_increment,s.range_min,s.range_max,270,.25,24)
    seg=continuous_segments(points,.3,2,8)
    path=plan_footprint_trajectory(s,seg,WallObservation(None,None,2.),0,0)
    assert path is not None and path.free_distance>=1.2
    # A missing/invalid observation is still blocked, unlike +Inf no-hit rays.
    for i in q['positive_infinity_indices']:s.ranges[i]=math.nan
    assert plan_footprint_trajectory(s,seg,WallObservation(None,None,2.),0,0) is None
