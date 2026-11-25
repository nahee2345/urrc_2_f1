import math
from geometry_msgs.msg import TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from .ros_support import ConfigNode


def quaternion(roll, pitch, yaw):
    cr, sr = math.cos(roll/2), math.sin(roll/2)
    cp, sp = math.cos(pitch/2), math.sin(pitch/2)
    cy, sy = math.cos(yaw/2), math.sin(yaw/2)
    return (sr*cp*cy-cr*sp*sy, cr*sp*cy+sr*cp*sy,
            cr*cp*sy-sr*sp*cy, cr*cp*cy+sr*sp*sy)


class SensorTFNode(ConfigNode):
    def __init__(self):
        super().__init__('team_01_sensor_tf')
        geometry = self.geometry()
        base_link = geometry['chassis_pose']
        transforms = []
        mast=geometry.get('mast_pose')
        if mast is not None:transforms.append(self.transform('team_01/base_link','team_01/camera_mast_link',[mast[i]-base_link[i] for i in range(3)]+mast[3:]))
        for child, pose in [('team_01/laser_frame', geometry['lidar_pose']), ('team_01/camera_link', geometry['camera_pose']), ('team_01/imu_link', geometry['imu_pose'])]:
            if pose is None:
                raise ValueError(f'{child} sensor missing from model')
            parent='team_01/camera_mast_link' if child=='team_01/camera_link' and mast is not None else 'team_01/base_link'
            origin=mast if parent.endswith('camera_mast_link') else base_link
            relative = [pose[i]-origin[i] for i in range(3)]+pose[3:]
            transforms.append(self.transform(parent, child, relative))
        transforms.append(self.transform('team_01/camera_link', 'team_01/camera_optical_frame', [0,0,0,-math.pi/2,0,-math.pi/2]))
        self.broadcaster = StaticTransformBroadcaster(self)
        self.broadcaster.sendTransform(transforms)

    def transform(self, parent, child, pose):
        msg = TransformStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id, msg.child_frame_id = parent, child
        msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z = map(float, pose[:3])
        q = quaternion(*pose[3:])
        msg.transform.rotation.x, msg.transform.rotation.y, msg.transform.rotation.z, msg.transform.rotation.w = q
        return msg
