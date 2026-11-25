import json,math
from pathlib import Path
from team10_race.lidar_processing import scan_to_points,continuous_segments
from team10_race.wall_tracker import PolynomialWall,WallObservation
from team10_race.wall_arc import build_wall_arc_centerline
from team10_race.centerline import build_centerline
from team10_race.safety import perception_is_valid

def fixture():
    q=json.loads((Path(__file__).parent/'data/tight_bend_scan.json').read_text());r=q['scan'];w=q['observation']
    pts=scan_to_points([x if x is not None else math.inf for x in r['ranges']],r['angle_min'],r['angle_increment'],r['range_min'],r['range_max'],270,.25,24)
    seg=continuous_segments(pts,.3,2,8)
    return seg,WallObservation(None,PolynomialWall(**w['right']),w['track_width_m'])

def test_recorded_bend_has_short_x_fit_but_observed_continuous_arc():
    seg,obs=fixture();legacy=build_centerline(obs,20,.25)
    assert len(legacy.points)<5
    arc=build_wall_arc_centerline(seg,obs)
    assert arc.source=='RAW_RIGHT_ARC_FALLBACK'
    assert len(arc.points)>=5 and arc.points[-1].y>1.2
    assert perception_is_valid(len(seg),1,len(arc.points),5)
    obstacles=[v.point for s in seg for v in s]
    assert min(p.distance(q) for p in arc.points for q in obstacles)>=.35

def test_no_geometry_no_arc_and_no_relaxed_safety():
    seg,obs=fixture()
    assert not build_wall_arc_centerline([],obs).points
    raw=build_wall_arc_centerline(seg,WallObservation(None,None,obs.track_width_m))
    assert len(raw.points)>=5 and raw.raw_point_count>=8
    assert not build_wall_arc_centerline(seg,obs,minimum_clearance=2.).points

def test_recorded_no_polynomial_fit_still_requires_long_measured_wall():
    r=json.loads((Path(__file__).parent/'data/tight_bend_no_fit_scan.json').read_text())
    pts=scan_to_points([x if x is not None else math.inf for x in r['ranges']],r['angle_min'],r['angle_increment'],r['range_min'],r['range_max'],270,.25,24)
    segments=continuous_segments(pts,.3,2,8)
    obs=WallObservation(None,None,1.75)
    arc=build_wall_arc_centerline(segments,obs)
    assert len(arc.points)>=5 and arc.raw_point_count>=8
    assert arc.points[-1].y>1.2
    assert not build_wall_arc_centerline([s[:7] for s in segments],obs).points

def test_single_wall_current_scan_proves_clearer_turn_than_shallow_fit():
    r=json.loads((Path(__file__).parent/'data/single_wall_obstructed_fit_scan.json').read_text())
    pts=scan_to_points([x if x is not None else math.inf for x in r['ranges']],r['angle_min'],r['angle_increment'],r['range_min'],r['range_max'],270,.25,24)
    segments=continuous_segments(pts,.3,2,8)
    arc=build_wall_arc_centerline(segments,WallObservation(None,None,1.75))
    assert len(arc.points)>=5 and arc.points[3].y<-.5
    assert min(p.distance(q.point) for p in arc.points for s in segments for q in s)>=.35
