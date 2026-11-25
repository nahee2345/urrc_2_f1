import math
from dataclasses import replace

import pytest

from team10_race.geometry import Point2
from team10_race.lidar_processing import ScanPoint, continuous_segments
from team10_race.opponent import (
    OpponentCandidate,
    OpponentDetector,
    OpponentTracker,
    RaceStrategy,
    TrackedOpponent,
    filter_selected_pass_obstacle,
    fuse_opponent_candidates,
    mask_confirmed_opponents,
    offset_path,
    translate_scan_points,
    transform_scan_points,
    estimate_tracking_motion,
    translate_wall_observation,
)
from team10_race.soft_centering import CorridorMeasurement
from team10_race.wall_tracker import PolynomialWall, WallObservation, WallTracker


def walls(left: float = 1.10, right: float = -1.10) -> WallObservation:
    return WallObservation(
        PolynomialWall("left", (left, 0.0, 0.0), -0.1, 12.0, 60, 0.01),
        PolynomialWall("right", (right, 0.0, 0.0), -0.1, 12.0, 60, 0.01),
        left - right,
    )


def test_rear_sensor_rigid_transform_preserves_beam_and_range():
    points = [ScanPoint(17, 3.0, Point2(3.0, 0.3))]
    transformed = transform_scan_points(points, -0.245, math.pi)
    assert transformed[0].point.x == pytest.approx(-3.245)
    assert transformed[0].point.y == pytest.approx(-0.3)
    assert transformed[0].beam_index == 17
    assert transformed[0].range == 3.0


def test_unoccluded_rear_mount_accepts_real_flank_without_changing_legacy_gate():
    points = [ScanPoint(i, math.hypot(-3.0, y), Point2(-3.0, y))
              for i, y in enumerate((0.50, 0.53, 0.56, 0.59))]
    detector = OpponentDetector(minimum_points=2)
    assert detector.detect(points, walls()) == []
    rear = detector.detect(points, walls(), dedicated_rear_sensor=True)
    assert len(rear) == 1 and rear[0].position == 'REAR'
    outside = [ScanPoint(i, math.hypot(-3.0, y), Point2(-3.0, y))
               for i, y in enumerate((1.0, 1.03, 1.06))]
    assert detector.detect(outside, walls(), dedicated_rear_sensor=True) == []


def test_three_sensor_overlap_fuses_one_physical_vehicle():
    def observation(x, y, beams):
        return OpponentCandidate(x, y, math.hypot(x,y), .25,
                                 math.atan2(y,x), 'REAR', beams)
    primary = [observation(-.5, -.5, (1,2,3))]
    secondary = [observation(-.55, -.52, (4,5,6))]
    rear = [observation(-.65, -.50, (7,8,9,10))]
    fused = fuse_opponent_candidates(fuse_opponent_candidates(primary, secondary), rear)
    tracker = OpponentTracker()
    for _ in range(3):tracks = tracker.update(fused, .1)
    assert len(fused) == len(tracks) == 1


def test_rear_wall_chain_is_not_made_compact_by_corridor_clipping():
    chain = [ScanPoint(i, math.hypot(-7.0+i*.1, .5+i*.1),
                       Point2(-7.0+i*.1, .5+i*.1)) for i in range(12)]
    detector = OpponentDetector(minimum_points=2)
    assert detector.detect(chain, walls(), dedicated_rear_sensor=True) == []
    vehicle = [ScanPoint(100+i, math.hypot(-3., .1+i*.02),
                         Point2(-3., .1+i*.02)) for i in range(4)]
    result = detector.detect(chain+vehicle, walls(), dedicated_rear_sensor=True)
    assert len(result) == 1 and result[0].x == pytest.approx(-3.)


def tracking_observation(x,y,stamp,source='rear',wall_distance=None):
    return OpponentCandidate(x,y,math.hypot(x,y),.25,math.atan2(y,x),
                             OpponentTracker._position(x,y),(10,11,12),
                             timestamp=stamp,sources=(source,),extent=.2,
                             point_count=3,wall_distance=wall_distance)


def test_same_rear_vehicle_keeps_id_for_continuous_observations():
    tracker=OpponentTracker()
    for i in range(1,31):
        tracks=tracker.update([tracking_observation(-6+i*.1,.1,i*.1)],.1,timestamp=i*.1)
        if i>=3:assert len(tracks)==1 and tracks[0].track_id==1


def test_rear_side_rear_classification_changes_do_not_create_identity():
    tracker=OpponentTracker()
    for i,x in enumerate([-.6,-.5,-.4,-.2,0.,-.2,-.4,-.6],1):
        tracks=tracker.update([tracking_observation(x,.6,i*.1)],.1,timestamp=i*.1)
        if i>=3:assert len(tracks)==1 and tracks[0].track_id==1


def test_sensor_handoff_does_not_own_identity():
    tracker=OpponentTracker()
    sources=['rear']*3+['primary']*3+['secondary']*3+['rear']*3
    for i,source in enumerate(sources,1):
        tracks=tracker.update([tracking_observation(-2+i*.03,.2,i*.1,source)],.1,timestamp=i*.1)
        if i>=3:assert len(tracks)==1 and tracks[0].track_id==1
    assert tracker._tracks[1].sources==('primary','rear','secondary')


def test_recently_missed_track_recovers_without_filter_lag_velocity_bias():
    tracker=OpponentTracker()
    for i in range(1,11):
        tracks=tracker.update([tracking_observation(-6+i*.1,.1,i*.1)],.1,timestamp=i*.1)
    assert tracks[0].relative_vx==pytest.approx(1.,abs=.06)
    for i in range(11,14):tracker.update([],.1,timestamp=i*.1)
    tracks=tracker.update([tracking_observation(-4.6,.1,1.4)],.1,timestamp=1.4)
    assert len(tracks)==1 and tracks[0].track_id==1


def test_duplicate_sensor_and_split_cluster_observations_make_one_track():
    tracker=OpponentTracker()
    for i in range(1,5):
        observations=[tracking_observation(-3,.1,i*.1,'rear'),
                      tracking_observation(-2.85,.13,i*.1,'secondary'),
                      tracking_observation(-3.15,.12,i*.1,'primary')]
        tracks=tracker.update(observations,.1,timestamp=i*.1)
    assert len(tracks)==1 and tracks[0].track_id==1


def test_close_distinct_abreast_vehicles_never_merge():
    tracker=OpponentTracker()
    for i in range(1,8):
        observations=[tracking_observation(-3+i*.1,0,i*.1),
                      tracking_observation(-3+i*.09,.48,i*.1)]
        tracks=tracker.update(observations,.1,timestamp=i*.1)
    assert {t.track_id for t in tracks}=={1,2}


def test_stationary_ghost_compact_cluster_cannot_confirm_rear_threat():
    tracker=OpponentTracker()
    for i in range(1,10):
        tracks=tracker.update([tracking_observation(-5-i*.1,.1,i*.1)],.1,
                              timestamp=i*.1,ego_motion=(-.1,0.,0.))
        assert tracks==[]
    strategy=RaceStrategy()
    decision=strategy.update(tracks,corridor(),2.,.1)
    assert decision.rear_closing_rate==0.


def test_ego_rotation_and_translation_preserve_rear_identity():
    tracker=OpponentTracker();x,y=-6.,.6
    for i in range(1,15):
        if i>1:
            cosine,sine=math.cos(-.15),math.sin(-.15)
            x,y=cosine*(x+.2)-sine*y-.1,sine*(x+.2)+cosine*y
        tracks=tracker.update([tracking_observation(x,y,i*.1)],.1,
                              timestamp=i*.1,ego_motion=(-.1,0.,-.15) if i>1 else (0.,0.,0.))
        if i>=4:assert len(tracks)==1 and tracks[0].track_id==1


def test_confirmed_track_reassociates_raw_evidence_outside_detector_corridor():
    tracker=OpponentTracker()
    for i in range(1,7):
        tracker.update([tracking_observation(-8+i*.3,1.,i*.1)],.1,
                       timestamp=i*.1,ego_motion=(-.1,0.,0.))
    points=[ScanPoint(20+i,math.hypot(-5.9,1.+i*.02),Point2(-5.9,1.+i*.02)) for i in range(3)]
    tracks=tracker.update([],.1,timestamp=.7,ego_motion=(-.1,0.,0.),
                          raw_observations=[('rear',.7,points)])
    assert len(tracks)==1 and tracks[0].track_id==1 and tracks[0].missed==0


def test_measured_wall_proximity_prevents_ghost_birth_without_hiding_rear():
    tracker=OpponentTracker()
    for i in range(1,5):
        tracks=tracker.update([tracking_observation(-4,.1,i*.1,wall_distance=.05),
                              tracking_observation(-8+i*.3,.4,i*.1,wall_distance=.40)],
                             .1,timestamp=i*.1,ego_motion=(-.1,0.,0.))
    assert len(tracks)==1 and tracks[0].x < -5


def test_tracking_registration_uses_lidar_frame_motion_without_world_poses():
    previous=[Point2(i*.1,1.+.03*(i*.1)**2) for i in range(70)]
    previous += [Point2(i*.1,-1.-.02*(i*.1)**2) for i in range(70)]
    angle=-.025;c,s=math.cos(angle),math.sin(angle)
    current=[Point2(c*p.x-s*p.y-.1,s*p.x+c*p.y+.02) for p in previous]
    tx,ty,yaw=estimate_tracking_motion(previous,current,.1,speed=1.,yaw_rate=.25)
    assert tx==pytest.approx(-.1,abs=.02)
    assert ty==pytest.approx(.02,abs=.02)
    assert yaw==pytest.approx(angle,abs=.005)


def test_sliding_radial_wall_sliver_never_confirms_as_rear_car():
    tracker=OpponentTracker()
    for i in range(1,12):
        ghost=replace(tracking_observation(-5+i*.1,.2,i*.1),extent=.40,
                      radial_span=.399,point_count=2)
        assert tracker.update([ghost],.1,timestamp=i*.1,ego_motion=(0.,0.,0.))==[]
        assert tracker.diagnostics['candidates'][0]['reason']=='radial_wall_sliver_no_birth'


def test_rear_front_facets_over_validated_distance_lateral_yaw_are_not_hidden():
    for distance in (1.5,2.,3.,4.,6.,8.):
        for lateral in (0.,-.3,.3,-.6,.6):
            for degrees in (0,-10,10,-20,20,-30,30):
                angle=math.radians(degrees);c,s=math.cos(angle),math.sin(angle)
                ends=[Point2(-distance+c*.095-s*y,lateral+s*.095+c*y)
                      for y in (-.0425,.0425)]
                extent=ends[0].distance(ends[1])
                span=abs(math.hypot(ends[0].x+.245,ends[0].y)-math.hypot(ends[1].x+.245,ends[1].y))
                assert span<.90*extent


def corridor(left: float = 1.10, right: float = -1.10) -> CorridorMeasurement:
    return CorridorMeasurement(
        left_wall_y=left,
        right_wall_y=right,
        left_clearance=left - 0.116,
        right_clearance=-right - 0.116,
        lower_center_bound_y=right + 0.466,
        upper_center_bound_y=left - 0.466,
        corridor_center_y=0.5 * (left + right),
        vehicle_lateral_error=0.5 * (left + right),
        wall_safety_state="NORMAL",
        has_two_walls=True,
    )


def scan_line(x: float, first_beam: int, center_y: float = 0.0):
    values = []
    for index, y in enumerate((-0.14, -0.09, -0.04, 0.01, 0.06, 0.11, 0.14)):
        point = Point2(x, center_y + y)
        values.append(ScanPoint(first_beam + index, point.norm, point))
    return values


def wall_points():
    values = []
    for side, first in ((1.10, 100), (-1.10, 500)):
        for index in range(40):
            point = Point2(0.25 + 0.20 * index, side)
            values.append(ScanPoint(first + index, point.norm, point))
    return values


def wall_tracker():
    return WallTracker(
        minimum_points=8,
        bin_size_m=0.20,
        maximum_residual_m=0.28,
        anchor_x_m=0.30,
        initial_track_width_m=2.20,
        minimum_track_width_m=1.70,
        maximum_track_width_m=2.70,
        width_filter_alpha=0.12,
    )


def candidate(
    x: float,
    y: float = 0.0,
    track_id: int = 1,
    closing: float = 0.0,
    lateral_velocity: float = 0.0,
):
    position = "FRONT" if x > 0.3 else ("REAR" if x < -0.3 else ("LEFT_SIDE" if y >= 0 else "RIGHT_SIDE"))
    return TrackedOpponent(
        track_id, x, y, math.hypot(x, y), 0.24, math.atan2(y, x),
        -closing, lateral_velocity, closing, position, 5,
    )


def test_walls_only_have_no_opponent_and_wall_points_are_not_vehicles():
    assert OpponentDetector().detect(wall_points(), walls()) == []


def test_front_and_rear_compact_clusters_are_detected():
    detector = OpponentDetector()
    front = detector.detect(scan_line(4.0, 300), walls(), (Point2(4.0, 0.0),))
    rear = detector.detect(scan_line(-3.0, 680), walls())
    assert len(front) == 1 and front[0].position == "FRONT"
    assert front[0].x == pytest.approx(4.0)
    assert 0.20 < front[0].width < 0.35
    assert len(rear) == 1 and rear[0].position == "REAR"
    assert rear[0].bearing == pytest.approx(math.pi, abs=0.06)


def test_temporal_tracker_associates_front_and_estimates_closing_rate():
    tracker = OpponentTracker()
    first = OpponentCandidate(4.0, 0.0, 4.0, 0.24, 0.0, "FRONT", (1, 2, 3))
    second = OpponentCandidate(3.8, 0.0, 3.8, 0.24, 0.0, "FRONT", (1, 2, 3))
    assert tracker.update([first], 0.10) == []
    assert tracker.update([second], 0.10) == []
    third = OpponentCandidate(3.6, 0.0, 3.6, 0.24, 0.0, "FRONT", (1, 2, 3))
    tracks = tracker.update([third], 0.10)
    assert len(tracks) == 1
    assert tracks[0].track_id == 1
    assert tracks[0].position == "FRONT"
    assert tracks[0].closing_rate > 0.0
    assert tracker.update([], 0.10)[0].track_id == 1
    assert tracker.update([], 0.10)[0].track_id == 1


def test_tracker_keeps_multiple_unique_tracks_with_velocity_class_and_timeout():
    tracker = OpponentTracker()
    observations = [
        OpponentCandidate(3.0, -0.2, 3.01, 0.24, -0.07, "FRONT", (10, 11)),
        OpponentCandidate(-2.0, 0.1, 2.00, 0.24, 3.09, "REAR", (20, 21)),
    ]
    assert tracker.update(observations, 0.10) == []
    assert tracker.update(observations, 0.10) == []
    tracks = tracker.update([
        OpponentCandidate(2.8, -0.2, 2.81, 0.24, -0.07, "FRONT", (10, 11)),
        OpponentCandidate(-1.8, 0.1, 1.80, 0.24, 3.09, "REAR", (20, 21)),
    ], 0.10)
    assert {item.track_id for item in tracks} == {1, 2}
    assert {item.position for item in tracks} == {"FRONT", "REAR"}
    assert all(item.relative_vx != 0.0 for item in tracks)
    for _ in range(16):
        tracks = tracker.update([], 0.10)
    assert tracks == []


def test_sensor_points_and_wall_polynomials_translate_to_base_frame():
    point = ScanPoint(10, 2.0, Point2(2.0, -0.1))
    translated = translate_scan_points([point], 0.20)
    assert translated[0].point == Point2(2.20, -0.1)
    assert translated[0].range == 2.0

    observation = WallObservation(
        PolynomialWall("left", (1.0, 0.2, 0.01), 0.0, 10.0, 20, 0.02),
        PolynomialWall("right", (-1.0, 0.2, 0.01), 0.0, 10.0, 20, 0.02),
        2.0,
    )
    shifted = translate_wall_observation(observation, 0.20)
    for base_x in (0.20, 2.0, 8.0):
        assert shifted.left.y(base_x) == pytest.approx(
            observation.left.y(base_x - 0.20)
        )
        assert shifted.right.y(base_x) == pytest.approx(
            observation.right.y(base_x - 0.20)
        )


def test_dual_lidar_candidates_fuse_to_one_physical_opponent():
    primary = OpponentCandidate(
        5.00, 0.10, 5.001, 0.22, 0.02, "FRONT", (350, 351)
    )
    secondary = OpponentCandidate(
        5.04, 0.08, 5.041, 0.25, 0.016, "FRONT", (348, 349, 350, 351)
    )
    fused = fuse_opponent_candidates([primary], [secondary])
    assert len(fused) == 1
    assert primary.x < fused[0].x < secondary.x
    assert fused[0].width == pytest.approx(0.25)

    tracker = OpponentTracker()
    assert tracker.update(fused, 0.10) == []
    assert tracker.update(fused, 0.10) == []
    assert len(tracker.update(fused, 0.10)) == 1


def test_split_body_fragments_do_not_create_two_physical_opponents():
    primary = [
        OpponentCandidate(3.80, -0.16, 3.803, 0.12, -0.04, "FRONT", (340, 341)),
        OpponentCandidate(4.20, -0.14, 4.202, 0.12, -0.03, "FRONT", (344, 345)),
    ]
    secondary = [
        OpponentCandidate(3.82, -0.15, 3.823, 0.25, -0.04, "FRONT", (338, 339, 340)),
    ]
    assert len(fuse_opponent_candidates(primary, secondary)) == 1


def test_distinct_non_overlapping_opponents_remain_separate():
    first = OpponentCandidate(3.0, 0.0, 3.0, 0.24, 0.0, "FRONT", (350, 351))
    second = OpponentCandidate(3.8, 0.0, 3.8, 0.24, 0.0, "FRONT", (350, 351))
    assert len(fuse_opponent_candidates([first, second], [])) == 2


def test_abreast_cars_inside_length_radius_are_not_one_body_fragment():
    front = OpponentCandidate(5.0, -0.30, 5.01, 0.24, -0.06, "FRONT", (350, 351))
    side = OpponentCandidate(5.02, 0.18, 5.02, 0.24, 0.04, "FRONT", (356, 357))
    assert len(fuse_opponent_candidates([front, side], [])) == 2
    assert len(fuse_opponent_candidates([front], [side])) == 2


def test_unconfirmed_abreast_candidate_blocks_pass_before_second_track_confirms():
    strategy = RaceStrategy()
    lead = candidate(5.0, -0.30)
    side = OpponentCandidate(5.02, 0.18, 5.02, 0.24, 0.04, "FRONT", (356, 357))
    for _ in range(6):
        decision = strategy.update([lead], corridor(), 1.5, 0.033,
                                   pass_candidates=[side])
        assert decision.race_state in {"FOLLOW", "PREPARE_PASS"}


def test_unmatched_secondary_candidate_can_update_track_without_hallucination():
    secondary = OpponentCandidate(
        6.0, -0.2, 6.003, 0.24, -0.033, "FRONT", (355, 356, 357)
    )
    assert fuse_opponent_candidates([], []) == []
    assert fuse_opponent_candidates([], [secondary]) == [secondary]
    tracker = OpponentTracker()
    assert tracker.update([], 0.10) == []
    assert tracker.update([secondary], 0.10) == []
    assert tracker.update([secondary], 0.10) == []
    assert len(tracker.update([secondary], 0.10)) == 1


def test_wall_edge_fragment_is_not_a_vehicle_candidate():
    detector = OpponentDetector()
    centerline = (Point2(1.0, 0.0), Point2(3.0, 0.0), Point2(5.0, 0.0))
    assert detector.detect(scan_line(3.0, 250, center_y=0.82), walls(), centerline) == []


def test_vehicle_face_split_by_short_beam_hole_is_one_candidate():
    left = scan_line(3.0, 300)[:3]
    right = [
        ScanPoint(item.beam_index + 8, item.range, item.point)
        for item in scan_line(3.0, 303)[3:]
    ]
    detected = OpponentDetector().detect(
        left + right, walls(), (Point2(3.0, 0.0),)
    )
    assert len(detected) == 1
    assert detected[0].position == "FRONT"


def test_two_ray_distant_vehicle_candidate_is_temporally_trackable():
    points = scan_line(7.0, 300)[2:4]
    detected = OpponentDetector().detect(points, walls(), (Point2(7.0, 0.0),))
    assert len(detected) == 1
    assert detected[0].position == "FRONT"


def test_curved_wall_fragment_far_from_centerline_is_not_a_vehicle_candidate():
    centerline = tuple(Point2(x, 0.08 * x) for x in (1.0, 3.0, 5.0, 7.0))
    fragment = scan_line(7.0, 250, center_y=-1.75)
    assert OpponentDetector().detect(fragment, walls(), centerline) == []


def test_rear_flank_wall_fragment_is_not_a_vehicle_candidate():
    assert OpponentDetector().detect(scan_line(-3.0, 680, center_y=0.60), walls()) == []


def test_rear_detection_can_be_disabled_for_occluded_sensor_mount():
    detector = OpponentDetector(allow_rear_candidates=False)
    assert detector.detect(scan_line(-3.0, 680), walls()) == []


def advance_to_pass(strategy: RaceStrategy, front, measured_corridor):
    decision = None
    for _ in range(15):
        decision = strategy.update([front], measured_corridor, 2.0, 0.10)
        if decision.race_state.startswith("PASS_") or decision.race_state == "FOLLOW" and _ > 2:
            break
    return decision


def test_confirmed_vehicle_in_selected_pass_lane_blocks_pass_until_it_leaves():
    strategy = RaceStrategy()
    front = candidate(4.0, -0.35, track_id=1)
    side = candidate(4.1, 0.35, track_id=2)
    decisions = []
    for _ in range(8):
        decisions.append(strategy.update([front, side], corridor(), 2.0, 0.10))
    assert not any(item.race_state.startswith("PASS_") for item in decisions)
    assert decisions[-1].selected_pass_side == "NONE"

    decision = None
    for _ in range(16):
        decision = strategy.update([front], corridor(), 2.0, 0.10)
        if decision.race_state.startswith("PASS_"):
            break
    assert decision.race_state == "PASS_LEFT"


def test_unconfirmed_side_candidate_vetoes_new_pass_without_becoming_target():
    strategy = RaceStrategy()
    front = candidate(4.0, -0.30, track_id=1)
    pending_side = OpponentCandidate(
        4.1, 0.30, 4.11, 0.25, 0.07, "FRONT", (310, 311)
    )
    for _ in range(18):
        decision = strategy.update(
            [front], corridor(), 2.0, 0.10, pass_candidates=[pending_side]
        )
        assert not decision.race_state.startswith("PASS_")
        assert strategy.pass_track_id is None
    for _ in range(18):
        decision = strategy.update([front], corridor(), 2.0, 0.10)
        if decision.race_state.startswith("PASS_"):
            break
    assert strategy.pass_track_id == front.track_id


def test_approaching_front_vehicle_enters_follow_and_caps_speed():
    strategy = RaceStrategy()
    decision = strategy.update([candidate(4.5, closing=1.0)], corridor(), 3.0, 0.10)
    assert decision.race_state == "FOLLOW"
    assert decision.speed_cap < 3.0


def test_wider_left_space_selects_pass_left():
    decision = advance_to_pass(RaceStrategy(), candidate(4.0, -0.30), corridor())
    assert decision.race_state == "PASS_LEFT"
    assert decision.selected_pass_side == "LEFT"
    assert decision.left_pass_clearance > decision.right_pass_clearance
    assert decision.speed_cap == pytest.approx(1.50)
    assert decision.desired_lateral_offset > 0.0


def test_wider_right_space_selects_pass_right():
    decision = advance_to_pass(RaceStrategy(), candidate(4.0, 0.30), corridor())
    assert decision.race_state == "PASS_RIGHT"
    assert decision.selected_pass_side == "RIGHT"
    assert decision.right_pass_clearance > decision.left_pass_clearance
    assert decision.speed_cap == pytest.approx(1.50)
    assert decision.desired_lateral_offset < 0.0


def test_both_sides_too_narrow_stays_follow():
    strategy = RaceStrategy()
    decision = advance_to_pass(strategy, candidate(4.0), corridor(0.45, -0.45))
    assert decision.race_state == "FOLLOW"
    assert decision.selected_pass_side == "NONE"


def test_completed_pass_enters_return_and_smoothly_recenters():
    strategy = RaceStrategy(lateral_rate_mps=1.0)
    front = candidate(4.0, -0.30)
    assert advance_to_pass(strategy, front, corridor()).race_state == "PASS_LEFT"
    for _ in range(5):
        strategy.update([front], corridor(), 2.0, 0.10)
    before = strategy.offset
    decision = strategy.update([candidate(-0.7, -0.30, track_id=1)], corridor(), 2.0, 0.10)
    assert decision.race_state == "RETURN"
    assert 0.0 <= decision.desired_lateral_offset < before


def test_temporary_track_loss_during_pass_holds_offset():
    strategy = RaceStrategy(lateral_rate_mps=1.0)
    front = candidate(4.0, -0.30)
    advance_to_pass(strategy, front, corridor())
    before = strategy.offset
    decision = strategy.update([], corridor(), 2.0, 0.10)
    assert decision.race_state == "PASS_LEFT"
    assert decision.desired_lateral_offset == pytest.approx(before)


def test_alongside_pass_latches_selected_side_through_transient_gap_change():
    strategy = RaceStrategy(lateral_rate_mps=1.0)
    front = candidate(4.0, -0.30)
    assert advance_to_pass(strategy, front, corridor()).race_state == "PASS_LEFT"
    alongside = candidate(0.50, -0.30)
    decision = strategy.update([alongside], corridor(0.55, -0.55), 2.0, 0.10)
    assert decision.race_state == "PASS_LEFT"
    assert decision.selected_pass_side == "LEFT"


def test_one_frame_dynamic_clearance_warning_holds_target_without_aborting():
    strategy = RaceStrategy(lateral_rate_mps=1.0)
    front = candidate(4.0, -0.30)
    assert advance_to_pass(strategy, front, corridor()).race_state == "PASS_LEFT"
    before = strategy.offset
    inward = candidate(3.7, -0.30, lateral_velocity=1.0)
    decision = strategy.update([inward], corridor(), 2.0, 0.10)
    assert decision.race_state == "PASS_LEFT"
    assert decision.pass_safety_state == "WARNING"
    assert decision.desired_lateral_offset == pytest.approx(before)
    assert decision.predicted_pass_clearance < 0.30
    assert decision.opponent_lateral_velocity == pytest.approx(1.0)


def test_sustained_unsafe_gap_aborts_while_opponent_is_still_ahead():
    strategy = RaceStrategy(lateral_rate_mps=1.0)
    front = candidate(4.0, -0.30)
    narrow = corridor(0.90, -1.10)
    assert advance_to_pass(strategy, front, narrow).race_state == "PASS_LEFT"
    inward = candidate(3.5, -0.30, lateral_velocity=1.0)
    decision = None
    for _ in range(8):
        decision = strategy.update([inward], narrow, 2.0, 0.10)
        if decision.race_state == "FOLLOW":
            break
    assert decision.race_state == "FOLLOW"
    assert decision.pass_safety_state == "ABORT"
    assert decision.remaining_pass_gap < 0.0


def test_aborted_track_does_not_retry_opposite_side_in_same_encounter():
    strategy = RaceStrategy(lateral_rate_mps=1.0)
    front = candidate(4.0, -0.30)
    narrow = corridor(0.90, -1.10)
    advance_to_pass(strategy, front, narrow)
    inward = candidate(3.5, -0.30, lateral_velocity=1.0)
    for _ in range(8):
        decision = strategy.update([inward], narrow, 2.0, 0.10)
        if decision.race_state == "FOLLOW":
            break
    assert decision.race_state == "FOLLOW"
    recovered = candidate(3.2, -0.30)
    assert strategy.update([recovered], corridor(1.50, -1.50), 2.0, 0.10).race_state == "FOLLOW"
    for _ in range(5):
        strategy.update([recovered], corridor(1.50, -1.50), 2.0, 0.10)
    decision = strategy.update([recovered], corridor(1.50, -1.50), 2.0, 0.10)
    assert decision.race_state == "FOLLOW"
    assert decision.pass_safety_state == "CLEAR"
    for _ in range(11):
        decision = strategy.update([], corridor(1.50, -1.50), 2.0, 0.10)
    assert decision.race_state == "CRUISE"


def test_sustained_alongside_risk_holds_side_and_reduces_speed():
    strategy = RaceStrategy(lateral_rate_mps=1.0, medium_speed_mps=1.50)
    front = candidate(4.0, -0.30)
    assert advance_to_pass(strategy, front, corridor()).race_state == "PASS_LEFT"
    before = strategy.offset
    alongside = candidate(0.50, -0.40, lateral_velocity=1.0)
    for _ in range(4):
        decision = strategy.update([alongside], corridor(), 2.0, 0.10)
    assert decision.race_state == "PASS_LEFT"
    assert decision.selected_pass_side == "LEFT"
    assert decision.pass_safety_state == "HOLD"
    assert decision.desired_lateral_offset == pytest.approx(before)
    assert decision.speed_cap == pytest.approx(1.05)


def test_lost_front_track_cannot_be_predicted_into_completed_pass():
    strategy = RaceStrategy(lateral_rate_mps=1.0)
    front = candidate(4.0, -0.30)
    advance_to_pass(strategy, front, corridor())
    predicted_rear = TrackedOpponent(
        1, -0.70, -0.30, 0.76, 0.24, -2.74,
        -3.0, 0.0, -3.0, "REAR", 12,
        missed=5, last_observed_x=1.20,
    )
    decision = strategy.update([predicted_rear], corridor(), 2.0, 0.10)
    assert decision.race_state == "PASS_LEFT"


def test_side_observed_track_can_bridge_rear_blind_sector_to_return():
    strategy = RaceStrategy(lateral_rate_mps=1.0)
    front = candidate(4.0, -0.30)
    advance_to_pass(strategy, front, corridor())
    predicted_rear = TrackedOpponent(
        1, -0.70, -0.30, 0.76, 0.24, -2.74,
        -1.0, 0.0, -1.0, "REAR", 12,
        missed=4, last_observed_x=0.20,
    )
    decision = strategy.update([predicted_rear], corridor(), 2.0, 0.10)
    assert decision.race_state == "RETURN"


def test_fast_rear_vehicle_suppresses_new_lateral_move():
    strategy = RaceStrategy()
    front = candidate(4.0, -0.30)
    rear = candidate(-3.0, 0.10, track_id=2, closing=1.2)
    strategy.update([front, rear], corridor(), 2.0, 0.10)
    decision = strategy.update([front, rear], corridor(), 2.0, 0.10)
    assert decision.race_state == "FOLLOW"
    assert decision.desired_lateral_offset == pytest.approx(0.0)
    assert decision.rear_closing_rate == pytest.approx(1.2)


def test_stale_disappeared_side_track_cannot_become_new_pass_target():
    strategy = RaceStrategy()
    observed = candidate(4.0, -0.25, track_id=1)
    stale = TrackedOpponent(
        2, 3.5, 0.10, 3.50, 0.25, 0.03,
        -1.0, 0.0, 1.0, "FRONT", 12,
        missed=5, last_observed_x=4.0,
    )
    decision = None
    for _ in range(15):
        decision = strategy.update([observed, stale], corridor(), 2.0, 0.10)
        if decision.race_state.startswith("PASS_"):
            break
    assert strategy.pass_track_id == 1


def test_fast_rear_vehicle_holds_pass_lane_instead_of_starting_return():
    strategy = RaceStrategy(lateral_rate_mps=1.0)
    front = candidate(4.0, -0.30, track_id=1)
    assert advance_to_pass(strategy, front, corridor()).race_state == "PASS_LEFT"
    passed = candidate(-0.70, -0.30, track_id=1)
    rear = candidate(-3.0, 0.10, track_id=2, closing=1.2)
    before = strategy.offset
    decision = strategy.update([passed, rear], corridor(), 2.0, 0.10)
    assert decision.race_state == "PASS_LEFT"
    assert decision.desired_lateral_offset == pytest.approx(before)
    assert decision.rear_closing_rate == pytest.approx(1.2)


def test_pass_filter_removes_only_selected_vehicle_points():
    selected = candidate(3.0, -0.2)
    vehicle_points = (Point2(3.0, -0.3), Point2(3.0, -0.1))
    wall_points = (Point2(3.0, 1.1), Point2(3.0, -1.1))
    assert filter_selected_pass_obstacle(vehicle_points + wall_points, selected) == wall_points


def test_no_opponent_is_exact_solo_path_noop():
    points = (Point2(0.5, 0.1), Point2(1.0, 0.2), Point2(1.5, 0.3))
    decision = RaceStrategy().update([], corridor(), 3.0, 0.10)
    assert decision.race_state == "CRUISE"
    assert math.isinf(decision.speed_cap)
    assert offset_path(points, decision.desired_lateral_offset) == points


def test_wall_box_opponent_is_masked_before_final_wall_refit():
    raw = sorted([*wall_points(), *scan_line(3.0, 300, -0.20)], key=lambda item: item.beam_index)
    segments = continuous_segments(raw, 0.30, 2, 1)
    tracker = wall_tracker()
    provisional = tracker.observe(segments)
    detector = OpponentDetector()
    candidates = detector.detect(raw, provisional, (Point2(1.0, 0.0), Point2(3.0, 0.0), Point2(5.0, 0.0)))
    temporal = OpponentTracker()
    assert temporal.update(candidates, 0.10) == []
    assert temporal.update(candidates, 0.10) == []
    confirmed = temporal.update(candidates, 0.10)
    assert len(confirmed) == 1

    final_points, removed = mask_confirmed_opponents(raw, confirmed)
    final = tracker.update(continuous_segments(final_points, 0.30, 2, 1))
    assert removed
    assert not set(range(300, 305)).isdisjoint(removed)
    assert final.left is not None and final.right is not None
    assert final.left.y(3.0) == pytest.approx(1.10, abs=0.02)
    assert final.right.y(3.0) == pytest.approx(-1.10, abs=0.02)


def test_sparse_f1_cross_section_confirms_from_three_frames():
    detector = OpponentDetector()
    sparse = [
        ScanPoint(300, Point2(4.00, -0.12).norm, Point2(4.00, -0.12)),
        ScanPoint(301, Point2(3.99, -0.10).norm, Point2(3.99, -0.10)),
        ScanPoint(310, Point2(4.02, 0.10).norm, Point2(4.02, 0.10)),
        ScanPoint(311, Point2(4.01, 0.12).norm, Point2(4.01, 0.12)),
    ]
    detected = detector.detect(sparse, walls(), (Point2(4.0, 0.0),))
    assert len(detected) == 1
    assert detected[0].width >= 0.20
    tracker = OpponentTracker()
    assert tracker.update(detected, 0.10) == []
    assert tracker.update(detected, 0.10) == []
    confirmed = tracker.update(detected, 0.10)
    assert len(confirmed) == 1
    assert confirmed[0].position == "FRONT"
    assert confirmed[0].width >= 0.25


def test_sparse_far_fragment_is_rejected_without_longitudinal_centerline_support():
    sparse = [
        ScanPoint(365, Point2(6.694, 0.322).norm, Point2(6.694, 0.322)),
        ScanPoint(366, Point2(6.695, 0.381).norm, Point2(6.695, 0.381)),
        ScanPoint(367, Point2(6.696, 0.440).norm, Point2(6.696, 0.440)),
    ]
    detected = OpponentDetector().detect(
        sparse,
        walls(1.818, -0.504),
        (Point2(0.5, 0.65), Point2(1.2, 0.60)),
    )
    assert detected == []


def test_sparse_far_f1_is_kept_with_longitudinal_centerline_support():
    sparse = [
        ScanPoint(365, Point2(6.694, 0.322).norm, Point2(6.694, 0.322)),
        ScanPoint(366, Point2(6.695, 0.381).norm, Point2(6.695, 0.381)),
        ScanPoint(367, Point2(6.696, 0.440).norm, Point2(6.696, 0.440)),
    ]
    detected = OpponentDetector().detect(
        sparse,
        walls(1.818, -0.504),
        (Point2(5.0, 0.20), Point2(6.5, 0.35), Point2(7.0, 0.42)),
    )
    assert len(detected) == 1
    assert detected[0].beam_indices == (365, 366, 367)


def test_split_vehicle_fragments_merge_but_walls_do_not():
    fragments = [
        ScanPoint(300, Point2(3.0, -0.16).norm, Point2(3.0, -0.16)),
        ScanPoint(301, Point2(3.0, -0.12).norm, Point2(3.0, -0.12)),
        ScanPoint(315, Point2(3.02, 0.12).norm, Point2(3.02, 0.12)),
        ScanPoint(316, Point2(3.02, 0.16).norm, Point2(3.02, 0.16)),
    ]
    detected = OpponentDetector().detect(
        [*fragments, *wall_points()], walls(), (Point2(3.0, 0.0),)
    )
    assert len(detected) == 1
    assert set(detected[0].beam_indices) == {300, 301, 315, 316}


def test_angled_partial_vehicle_section_is_shape_agnostic():
    points = []
    for index, (x, y) in enumerate(((4.15, -0.13), (4.08, -0.05), (4.00, 0.03), (3.93, 0.11))):
        point = Point2(x, y)
        points.append(ScanPoint(300 + index, point.norm, point))
    detected = OpponentDetector().detect(points, walls(), (Point2(4.0, 0.0),))
    assert len(detected) == 1
    assert detected[0].position == "FRONT"


def test_wall_near_opponent_is_detected_but_pass_is_rejected_when_gap_is_small():
    detected = OpponentDetector().detect(
        scan_line(3.0, 300, center_y=0.55),
        walls(),
        (Point2(3.0, 0.0),),
    )
    assert len(detected) == 1
    tracked = candidate(3.0, 0.55)
    decision = advance_to_pass(RaceStrategy(), tracked, corridor())
    assert decision.race_state == "PASS_RIGHT"
    assert decision.right_pass_clearance >= 0.732
    assert decision.left_pass_clearance < 0.732


def test_chicane_opening_wall_chains_are_not_opponents():
    points = []
    for side, first in ((2.10, 100), (-2.10, 500)):
        for index in range(45):
            point = Point2(0.20 + 0.16 * index, side + 0.015 * math.sin(index / 4))
            points.append(ScanPoint(first + index, point.norm, point))
    observation = walls(2.10, -2.10)
    centerline = tuple(Point2(x, 0.0) for x in (1.0, 3.0, 5.0, 7.0))
    assert OpponentDetector().detect(points, observation, centerline) == []


def test_no_opponent_provisional_final_wall_path_is_numerically_identical():
    raw = sorted(wall_points(), key=lambda item: item.beam_index)
    segments = continuous_segments(raw, 0.30, 2, 1)
    direct_tracker = wall_tracker()
    direct = direct_tracker.update(segments)
    pipeline_tracker = wall_tracker()
    provisional = pipeline_tracker.observe(segments)
    final_points, removed = mask_confirmed_opponents(raw, [])
    final = pipeline_tracker.update(continuous_segments(final_points, 0.30, 2, 1))
    assert removed == ()
    assert provisional.left == direct.left
    assert provisional.right == direct.right
    assert final == direct


def test_pass_target_preserves_extra_wall_recovery_margin():
    strategy = RaceStrategy(wall_clearance_m=0.25, opponent_clearance_m=0.25)
    front = candidate(4.0, -0.30)
    decision = advance_to_pass(strategy, front, corridor())
    for _ in range(10):
        decision = strategy.update([front], corridor(), 2.0, 0.10)
    target_y = corridor().corridor_center_y + decision.desired_lateral_offset
    opponent_left = front.y + 0.5 * front.width
    wall_clearance = 1.10 - (target_y + 0.116)
    opponent_clearance = (target_y - 0.116) - opponent_left
    assert decision.race_state == "PASS_LEFT"
    assert wall_clearance >= 0.25
    assert opponent_clearance >= 0.25
    assert wall_clearance - opponent_clearance == pytest.approx(0.10, abs=1e-6)


def test_offset_path_reaches_pass_target_smoothly_over_three_metres():
    path = tuple(Point2(float(index), 0.0) for index in range(1, 5))
    shifted = offset_path(path, 0.50, transition_length_m=3.0)
    assert 0.0 < shifted[0].y < shifted[1].y < shifted[2].y
    assert shifted[2].y == pytest.approx(0.50)
    assert shifted[3].y == pytest.approx(0.50)


def test_single_front_has_no_multicar_prepare_delay():
    strategy = RaceStrategy()
    front = candidate(5.3, -0.30)
    for _ in range(3):
        decision = strategy.update([front], corridor(), 1.5, 1.0 / 30.0)
    assert decision.race_state == "PASS_LEFT"
    assert strategy.pass_track_id == front.track_id


def test_single_pass_trajectory_matches_isolated_legacy_path():
    normal = RaceStrategy(opponent_clearance_m=0.32)
    reference = RaceStrategy(opponent_clearance_m=0.32,
                             rear_awareness_enabled=False,
                             safety_recovery_enabled=False,
                             side_occupancy_enabled=False)
    for index in range(65):
        x = 5.3 - index * 0.10
        lead = candidate(x, -0.30 if x > 0.7 else -0.40)
        actual = normal.update([lead], corridor(), 1.5, 0.1)
        expected = reference.update([lead], corridor(), 1.5, 0.1)
        assert actual == expected
        assert normal.last_recovery_correction == reference.last_recovery_correction


def test_single_hold_does_not_invent_second_vehicle_recovery():
    strategy = RaceStrategy()
    strategy.state = "PASS_LEFT"
    strategy.pass_track_id = 1
    strategy.offset = 0.63
    strategy.pass_safety_state = "HOLD"
    decision = strategy.update([candidate(0.2, -0.40)], corridor(), 1.5, 0.1)
    assert decision.pass_safety_state == "HOLD"
    assert decision.desired_lateral_offset == pytest.approx(0.63)
    assert strategy.last_recovery_correction == pytest.approx(0.0)


def test_new_hold_recovery_requires_a_separate_confirmed_threat():
    strategy = RaceStrategy()
    strategy.state = "PASS_LEFT"
    strategy.pass_track_id = 1
    strategy.offset = 0.63
    strategy.pass_safety_state = "HOLD"
    lead = candidate(0.2, -0.40)
    rear = candidate(-3.0, 0.10, track_id=2, closing=1.2)
    decision = strategy.update([lead, rear], corridor(), 1.5, 0.1)
    assert decision.race_state == "PASS_LEFT"
    assert 0.63 < decision.desired_lateral_offset <= 0.75


def test_return_does_not_double_filter_the_completed_pass_goal():
    strategy = RaceStrategy(filter_return_offset=False)
    strategy.state = "PASS_LEFT"
    strategy.pass_track_id = 1
    strategy.offset = 0.63
    decision = strategy.update([candidate(-0.7, -0.50)], corridor(), 1.5, 0.033)
    assert decision.race_state == "RETURN"
    assert decision.desired_lateral_offset == pytest.approx(0.0)


def test_small_verified_clearance_guard_moves_balanced_target_only_five_mm():
    lead = candidate(5.0, -0.30)
    baseline = RaceStrategy(opponent_clearance_m=0.32)._pass_space(lead, corridor())
    guarded = RaceStrategy(opponent_clearance_m=0.33)._pass_space(lead, corridor())
    assert guarded[2] - baseline[2] == pytest.approx(0.005)
    assert guarded[3] - baseline[3] == pytest.approx(-0.005)


@pytest.mark.parametrize("side", [1.0, -1.0])
def test_held_pass_path_is_parallel_not_another_outbound_lane_change(side):
    path = tuple(Point2(x, -side * 0.60) for x in (0.4, 0.8, 1.2, 2.0, 3.0))
    held = offset_path(path, side * 0.63, transition_length_m=3.0,
                       initial_lateral_offset_m=side * 0.63)
    assert [point.y for point in held] == pytest.approx([side * 0.03] * len(path))
    assert [point.x for point in held] == pytest.approx([point.x for point in path])


def test_zero_offset_path_remains_exact_solo_no_op_with_default_origin():
    path = (Point2(0.4, -0.6), Point2(0.8, -0.5), Point2(1.2, -0.4))
    assert offset_path(path, 0.0) == path


def test_pass_transition_starts_from_sensed_ego_lane_not_road_centre():
    path = tuple(Point2(x, 0.23) for x in (0.1, 0.4, 1.0, 3.0))
    shifted = offset_path(path, 0.63, 3.0, initial_lateral_offset_m=-0.23)
    assert shifted[0].y == pytest.approx(0.86 * (0.1 / 3.0) ** 2 * (3 - 2 * 0.1 / 3.0))
    assert shifted[-1].y == pytest.approx(0.86)


def test_predicted_side_track_cannot_become_a_new_front_target():
    from dataclasses import replace
    lead = candidate(5.0, -0.30, track_id=1)
    removed = replace(candidate(4.0, 0.40, track_id=2), missed=1)
    strategy = RaceStrategy()
    decision = strategy.update([removed, lead], corridor(), 1.5, 0.033)
    assert decision.closest_front_distance == pytest.approx(lead.distance)
    assert decision.primary_opponent_x == pytest.approx(lead.x)


def test_observed_gap_clearing_does_not_wait_out_a_blocked_retry_timer():
    strategy = RaceStrategy()
    strategy.state = "FOLLOW"
    strategy.pass_retry_time = 0.25
    lead = candidate(5.0, -0.30, track_id=1)
    decision = strategy.update([lead], corridor(), 1.5, 0.033)
    assert decision.race_state == "PREPARE_PASS"
    decision = strategy.update([lead], corridor(), 1.5, 0.033)
    assert decision.race_state == "PASS_LEFT"
    assert strategy.pass_track_id == 1


def test_distinct_side_blocker_waits_without_spending_pass_distance_or_offset():
    strategy = RaceStrategy()
    lead = candidate(5.3, -0.30, track_id=1)
    side = candidate(5.3, 0.35, track_id=2)
    for _ in range(8):
        decision = strategy.update([lead, side], corridor(), 1.5, 0.033)
        assert decision.race_state in {"FOLLOW", "PREPARE_PASS"}
        assert decision.speed_cap == pytest.approx(0.50)
        assert decision.desired_lateral_offset == pytest.approx(0.0)
    decision = strategy.update([lead], corridor(), 1.5, 0.033)
    assert decision.speed_cap > 0.50
    for _ in range(2):
        decision = strategy.update([lead], corridor(), 1.5, 0.033)
    assert decision.race_state == "PASS_LEFT"
    assert strategy.pass_track_id == 1


def test_side_blocker_does_not_apply_waiting_speed_to_active_pass():
    strategy = RaceStrategy()
    strategy.state = "PASS_LEFT"
    strategy.pass_track_id = 1
    strategy.offset = 0.63
    decision = strategy.update([candidate(0.20, -0.40),
                                candidate(-0.50, 0.40, track_id=2)],
                               corridor(), 1.5, 0.033)
    assert decision.race_state == "PASS_LEFT"
    assert decision.speed_cap > 0.50


def test_side_blocked_encounter_keeps_wait_cap_until_lateral_clearance_exists():
    strategy = RaceStrategy()
    lead = candidate(5.3, -0.30, track_id=1)
    side = candidate(5.3, 0.35, track_id=2)
    strategy.update([lead, side], corridor(), 1.5, 0.033)
    for _ in range(3):
        decision = strategy.update([lead], corridor(), 1.5, 0.033)
    assert decision.race_state == "PASS_LEFT"
    assert decision.speed_cap == pytest.approx(0.50)
    decision = strategy.update([candidate(3.0, -0.70)], corridor(), 1.5, 0.033)
    assert decision.speed_cap == pytest.approx(1.50)
    assert strategy.side_wait_track_id is None
    strategy.reset()
    assert strategy.side_wait_track_id is None


def test_side_wait_cap_cannot_leak_into_a_different_front_id():
    strategy = RaceStrategy()
    strategy.state = "PASS_LEFT"
    strategy.pass_track_id = 3
    strategy.side_wait_track_id = 1
    decision = strategy.update([candidate(3.0, -0.30, track_id=3)], corridor(), 1.5, 0.033)
    assert decision.speed_cap == pytest.approx(1.50)
