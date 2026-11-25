"""Pure far-image work in one spawned process; no ROS objects or publishers.

The parent keeps the existing one-in-flight thread/future and original stamp.
Independent process state preserves the existing far CameraVision history while
avoiding Python GIL contention with near geometry and traffic-light callbacks.
"""
import time
import cv2
from .vision import CameraVision
from .far_geometry import analyze_preview

_vision=None
_config=None


def compute_preview(image,k,distortion,now,config):
    global _vision,_config
    if _vision is None or config!=_config:
        cv2.setNumThreads(1)
        _vision=CameraVision(config);_config=config
    started=time.monotonic()
    preview=_vision.process(image,k,distortion,now)
    pixels=None
    if len(preview.get('left_samples',[]))>=5 and len(preview.get('right_samples',[]))>=5:
        pixels=tuple(_vision.project(samples,k) for samples in (preview['left_samples'],preview['right_samples']))
    geometry=analyze_preview(preview['path'],preview.get('left_samples'),preview.get('right_samples'),preview['confidence'],source_pixels=pixels)
    return preview,geometry,time.monotonic()-started
