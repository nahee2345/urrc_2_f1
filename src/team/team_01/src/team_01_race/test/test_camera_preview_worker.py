"""An occupied far-preview worker must not stall primary Camera frames."""
import os
os.environ['ROS_DOMAIN_ID']='105'
import json
import threading
import time
from types import SimpleNamespace
import numpy as np
import rclpy
from sensor_msgs.msg import CameraInfo
from team_01_race.camera_node import CameraNode
from team_01_race.image_codec import from_bgr


def test_primary_frames_continue_while_one_far_frame_is_busy():
    rclpy.init(args=['--ros-args','-p','camera.preview_worker_process:=false','-p','camera.rgb_far_debug_enabled:=true']);node=CameraNode();release=threading.Event();started=threading.Event();bundles=[]
    try:
        node.max_fps=1000.
        info=CameraInfo(width=640,height=480);info.k=[185.,0.,320.,0.,185.,240.,0.,0.,1.]
        node.on_info(info)
        node.bundle_pub=SimpleNamespace(publish=lambda m:bundles.append(json.loads(m.data)))
        def busy(*_args):
            started.set();release.wait(timeout=3.)
            return dict(path=None,confidence=0.)
        node.preview_vision.process=busy
        def frame():
            msg=from_bgr(np.zeros((480,640,3),np.uint8));msg.header.stamp=node.get_clock().now().to_msg()
            node.on_image(msg)
        frame()
        assert started.wait(timeout=1.)
        original=node.preview_future
        assert original is not None and not original.done()
        time.sleep(.12)  # Far preview is due again, but its worker is still busy.
        frame()
        assert len(bundles)==2 and node.preview_future is original and not original.done()
        assert bundles[1]['sec']+bundles[1]['nanosec']*1e-9>bundles[0]['sec']+bundles[0]['nanosec']*1e-9
    finally:
        release.set();node.destroy_node();rclpy.shutdown()


def test_spawned_far_worker_returns_original_geometry_without_blocking_near():
    from pathlib import Path
    import cv2
    rclpy.init(args=['--ros-args','-p','camera.rgb_far_debug_enabled:=true']);node=CameraNode();packets=[];bundles=[]
    try:
        node.preview_pub=SimpleNamespace(publish=lambda m:packets.append(json.loads(m.data)))
        node.bundle_pub=SimpleNamespace(publish=lambda m:bundles.append(json.loads(m.data)))
        info=CameraInfo(width=640,height=480);info.k=[185.,0.,320.,0.,185.,240.,0.,0.,1.];node.on_info(info)
        image=cv2.imread(str(Path(__file__).parent/'fixtures/monza_lights_off.png'))
        for _ in range(2):
            msg=from_bgr(image);msg.header.stamp=node.get_clock().now().to_msg();node.on_image(msg)
            time.sleep(.06)
        assert len(bundles)==2
        node.preview_future.result(timeout=8.)
        assert packets and packets[-1]['frame_id']=='team_01/base_link'
        assert packets[-1]['stamp']<=bundles[-1]['sec']+bundles[-1]['nanosec']*1e-9
        assert packets[-1]['compute_wall_sec']>=0.
    finally:node.destroy_node();rclpy.shutdown()
