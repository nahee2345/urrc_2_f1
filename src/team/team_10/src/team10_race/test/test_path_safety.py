import math
from types import SimpleNamespace
from team10_race.geometry import Point2
from team10_race.path_safety import verify_path, extend_straight, near_path_agreement, extend_visible_path


def scan(front=25.):
    inc=2*math.pi/719;angles=[-math.pi+i*inc for i in range(720)]
    ranges=[]
    for a in angles:
        wall=1./abs(math.sin(a)) if abs(math.sin(a))>1e-9 else math.inf
        obstacle=front/math.cos(a) if math.cos(a)>0 else math.inf
        ranges.append(min(25.,wall,obstacle))
    return SimpleNamespace(ranges=ranges,angle_min=-math.pi,angle_increment=inc,range_min=.08,range_max=25.)


def test_tangent_extension_uses_only_verified_visible_distance():
    path=[Point2(x*.25,0.) for x in range(1,21)]
    extended=extend_straight(path,22.)
    assert extended[-1].x>21.
    body=verify_path(scan(),extended)
    assert body.distance>21.
    blocked=verify_path(scan(front=8.),extended)
    assert 7.<blocked.distance<8.


def test_body_corner_collision_cannot_hide_outside_centre_tube():
    path=[Point2(x*.25,.82) for x in range(1,21)]
    assert verify_path(scan(),path).distance<1.


def test_observed_curve_is_not_replaced_by_straight_extension():
    path=[Point2(5*math.sin(i*.05),5*(1-math.cos(i*.05))) for i in range(1,15)]
    assert len(extend_straight(path,22.))==len(path)


def test_invalid_rays_are_not_free_space():
    s=scan();s.ranges=[math.nan]*720
    assert verify_path(s,[Point2(i*.25,0.) for i in range(1,21)]).distance==0.


def test_smooth_wall_fit_cannot_replace_a_different_near_control_path():
    path=[Point2(i*.25,0.) for i in range(1,21)]
    assert near_path_agreement(path,[Point2(p.x,.04) for p in path])
    assert not near_path_agreement(path,[Point2(p.x,.15) for p in path])
    assert not near_path_agreement(path,list(reversed(path)))


def test_visible_bending_corridor_can_extend_without_changing_near_path():
    s=scan();ranges=[]
    for i in range(len(s.ranges)):
        angle=s.angle_min+i*s.angle_increment
        roots=[]
        for radius in (99.,101.):
            b=200.*math.sin(angle);c=10000.-radius*radius;disc=b*b-4*c
            if disc>=0:
                roots.extend(t for t in ((-b-math.sqrt(disc))/2,(-b+math.sqrt(disc))/2) if t>.08)
        ranges.append(min([25.]+roots))
    s.ranges=ranges
    path=tuple(Point2(i*.25,0.) for i in range(1,21))
    tangent=extend_straight(path,22.)
    selected,safety,curvature=extend_visible_path(s,path,22.)
    assert safety.distance>verify_path(s,tangent).distance+3.
    assert [p for p in selected if p.x<=3.]==[p for p in tangent if p.x<=3.]
    assert curvature is not None and abs(curvature)<=.02
    assert safety==verify_path(s,selected)
    # A larger speed-dependent controller target must remain on precisely the
    # same near path, even when a different distant continuation is selected.
    farther,checked,_=extend_visible_path(s,path,22.,near_distance=4.7)
    assert checked.distance>verify_path(s,tangent).distance
    assert [p for p in farther if p.x<=4.7]==[p for p in tangent if p.x<=4.7]


def test_visible_extension_cannot_bypass_a_front_wall_or_unknown_scan():
    path=tuple(Point2(i*.25,0.) for i in range(1,21))
    _,blocked,_=extend_visible_path(scan(front=8.),path,22.)
    assert blocked.distance<8.
    s=scan();s.ranges=[math.nan]*len(s.ranges)
    _,unknown,_=extend_visible_path(s,path,22.)
    assert unknown.distance==0.


def test_gentle_tail_search_does_not_extend_a_sharp_observed_curve():
    path=tuple(Point2(5*math.sin(i*.05),5*(1-math.cos(i*.05))) for i in range(1,15))
    chosen,_,curvature=extend_visible_path(scan(),path,22.)
    assert chosen==path and curvature is None
