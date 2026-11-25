"""Stride-aware ROS Image conversion without a NumPy-ABI-specific extension."""
import copy
import cv2
import numpy as np
from sensor_msgs.msg import Image


def to_bgr(msg):
    channels = {'rgb8':3, 'bgr8':3, 'rgba8':4, 'bgra8':4, 'mono8':1, '8UC1':1}
    if msg.encoding not in channels:
        raise ValueError(f'unsupported camera encoding: {msg.encoding}')
    count = channels[msg.encoding]
    if msg.height <= 0 or msg.width <= 0 or msg.step < msg.width*count or len(msg.data) < msg.height*msg.step:
        raise ValueError('invalid image dimensions/stride/data length')
    array = np.frombuffer(msg.data, dtype=np.uint8, count=msg.height*msg.step)
    array = array.reshape(msg.height, msg.step)[:, :msg.width*count].reshape(msg.height,msg.width,count)
    if msg.encoding == 'rgb8':return cv2.cvtColor(array, cv2.COLOR_RGB2BGR)
    if msg.encoding == 'rgba8':return cv2.cvtColor(array, cv2.COLOR_RGBA2BGR)
    if msg.encoding == 'bgra8':return cv2.cvtColor(array, cv2.COLOR_BGRA2BGR)
    if count == 1:return cv2.cvtColor(array, cv2.COLOR_GRAY2BGR)
    return np.ascontiguousarray(array)


def from_bgr(image, header=None):
    image = np.asarray(image)
    if image.dtype != np.uint8 or image.ndim != 3 or image.shape[2] != 3:
        raise ValueError('expected HxWx3 uint8 BGR image')
    msg = Image()
    if header is not None:msg.header = copy.deepcopy(header)
    msg.height, msg.width = image.shape[:2]
    msg.encoding = 'bgr8'
    msg.is_bigendian = 0
    msg.step = msg.width*3
    msg.data = np.ascontiguousarray(image).tobytes()
    return msg
