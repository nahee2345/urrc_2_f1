from team_01_race.camera_health import CameraHealth,image_age_with_geometry


def test_delayed_timeout_heartbeat_cannot_override_newer_decoded_camera_frame():
    age=image_age_with_geometry(.30,1.,.95,'CAMERA_PATH')
    assert abs(age-.05)<1e-9
    # Time advances from the original image, not from receipt of the heartbeat.
    assert image_age_with_geometry(.40,1.30,.95,'CAMERA_PATH')>.25


def test_lidar_future_and_missing_geometry_do_not_hide_camera_timeout():
    assert image_age_with_geometry(.30,1.,.95,'LIDAR_PATH')==.30
    assert image_age_with_geometry(.30,1.,1.1,'CAMERA_PATH')==.30
    assert image_age_with_geometry(.30,1.,float('-inf'),'CAMERA_PATH')==.30


def test_three_distinct_frames_and_repeated_health_publication():
    health = CameraHealth()
    for i in range(3):
        health.accept(i*.03, i)
        assert health.ready(i*.03) == (i == 2)
    assert health.ready(.10)
    assert health.ready(.15)


def test_timeout_requires_three_new_frames():
    health = CameraHealth()
    for i in range(3):
        health.accept(i*.03, i)
    assert not health.ready(.4)
    for i in range(3):
        health.accept(.5+i*.03, i+3)
        assert health.ready(.5+i*.03) == (i == 2)


def test_gap_without_watchdog_still_resets_count():
    health = CameraHealth()
    health.accept(0., 0)
    health.accept(.03, 1)
    health.accept(.5, 2)
    assert not health.ready(.5)


def test_replay_cannot_override_or_refresh_valid_health_and_decode_error_clears():
    health = CameraHealth()
    for i in range(3):
        health.accept(i*.03, i)
    health.accept(.09, 2)
    assert health.ready(.09)
    assert health.last_valid==.06
    assert not health.ready(.4)
    for i in range(3):
        health.accept(.12+i*.03, i+3)
    assert health.ready(.18)
    health.invalidate()
    assert not health.ready(.18)
