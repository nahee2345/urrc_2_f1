import numpy as np
from team10_race.start_vision import FiveRedDetector

def scene(n=0):
    rgb=np.full((180,360,3),180,dtype=np.uint8)
    rgb[60:110,60:300]=12
    for i in range(5):
        rgb[72:93,78+i*42:98+i*42]=[245,2,2] if i<n else [4,1,1]
    return rgb

def test_initial_dark_cannot_start_and_requires_five_red_then_real_dark():
    d=FiveRedDetector()
    for _ in range(4):assert not d.update(scene())[0]
    for n in range(1,5):assert not d.update(scene(n))[0]
    for _ in range(3):assert not d.update(scene(5))[0]
    for _ in range(2):assert not d.update(scene())[0]
    assert d.update(scene())[0]

def test_occlusion_partial_and_missing_baseline_never_mean_off():
    d=FiveRedDetector()
    for _ in range(5):assert not d.update(scene(5))[0]
    assert d.reference is None
    d.reset();d.update(scene())
    for _ in range(3):d.update(scene(5))
    for _ in range(4):
        blocked=scene();blocked[55:115,55:305]=0
        assert not d.update(blocked)[0]
        assert not d.update(scene(2))[0]
        assert not d.update(np.zeros_like(blocked))[0]
    assert not d.released

def test_stream_interruption_clears_red_confirmation_even_with_saved_baseline():
    d=FiveRedDetector();d.update(scene())
    for _ in range(3):d.update(scene(5))
    # Exact policy used by the ROS watchdog: preserve comparison images only.
    history=d.history;d.reset();d.history=history
    for _ in range(5):assert not d.update(scene())[0]
    for _ in range(3):assert not d.update(scene(5))[0]
    for _ in range(2):assert not d.update(scene())[0]
    assert d.update(scene())[0]


def test_ready_and_camera_lease_are_both_required(monkeypatch):
    from unittest.mock import Mock
    from std_msgs.msg import Bool
    from team10_race.start_interlock import StartInterlock
    import team10_race.start_interlock as module
    clock=[10.0];monkeypatch.setattr(module.time,'monotonic',lambda:clock[0])
    callback=Mock();gate=StartInterlock(Mock(),'/ready',callback,None)
    gate.on_ready(Bool(data=True));assert not gate.active
    gate.on_camera(Bool(data=True));assert gate.active
    clock[0]+=.26;gate.check();assert not gate.active
    gate.on_camera(Bool(data=True));assert gate.active
    gate.on_ready(Bool(data=False));assert not gate.active
    gate.on_camera(Bool(data=True));assert not gate.active

def test_camera_watchdog_does_not_alternate_two_faults_in_one_tick():
    from unittest.mock import Mock
    from types import SimpleNamespace
    from team10_race.start_camera_node import StartCamera
    import time
    fake=SimpleNamespace(ready=True,allowed=False,last_frame=time.monotonic()-1,
        last_stamp=1.,frames=20,last_rejection=None,reason='CAMERA_TIMEOUT',detector=FiveRedDetector(),
        pub=Mock(),status=Mock(),invalidate=Mock(),last_log='CAMERA_TIMEOUT',log_time=time.monotonic())
    fake.get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=3_000_000_000))
    StartCamera.tick(fake)
    fake.invalidate.assert_called_once_with('CAMERA_TIMEOUT')
    assert fake.pub.publish.call_args.args[0].data is False


def test_camera_audit_full_red_off_callback_serializes_numpy_boxes(tmp_path):
    import json
    from types import SimpleNamespace
    from unittest.mock import Mock
    from sensor_msgs.msg import Image
    from team10_race.start_camera_node import StartCamera
    clock=[1.]
    fake=SimpleNamespace(ready=True,detector=FiveRedDetector(),last_frame=None,last_stamp=None,
        frames=0,allowed=False,reason='WAIT_FIVE_RED',last_rejection=None,
        audit_dir=tmp_path,audit_reference=None,audit_frame_count=0,invalidate=Mock())
    fake.get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=int(clock[0]*1e9)))
    for rgb in [scene(),scene(5),scene(5),scene(5),scene(),scene(),scene()]:
        msg=Image();msg.height,msg.width=rgb.shape[:2];msg.step=msg.width*3;msg.encoding='rgb8'
        msg.data=rgb.tobytes();msg.header.stamp.sec=int(clock[0]);msg.header.stamp.nanosec=round((clock[0]%1)*1e9)
        StartCamera.on_image(fake,msg);clock[0]+=.05
    rows=[json.loads(l) for l in (tmp_path/'frames.jsonl').read_text().splitlines()]
    assert len(rows)==7 and rows[-1]['allowed'] and fake.allowed
    assert rows[1]['boxes'] and (tmp_path/rows[1]['reference']).exists()
    fake.invalidate.assert_not_called()


def test_recorded_settling_dark_then_recent_dark_and_real_red_off():
    import cv2
    from pathlib import Path
    folder=Path(__file__).parent/'data/start_real_20261122'
    def load(name):return cv2.cvtColor(cv2.imread(str(folder/name)),cv2.COLOR_BGR2RGB)
    d=FiveRedDetector();old=load('old_dark.png');recent=load('recent_dark.png')
    assert not d.update(old)[0]
    assert not d.update(recent)[0]
    for i in range(3):assert not d.update(load(f'red{i}.png'))[0]
    assert np.array_equal(d.reference,recent)
    assert not d.update(load('off0.png'))[0]
    assert not d.update(load('off1.png'))[0]
    assert d.update(load('off2.png'))[0]
