"""Read the submitted SDF; do not duplicate vehicle geometry in tuning files."""
import math
import xml.etree.ElementTree as ET


def pose(link):
    return [float(v) for v in link.findtext('pose', '0 0 0 0 0 0').split()]


def vehicle_geometry(filename, mount=None):
    from .camera_mount import configured_tree
    tree=configured_tree(filename,mount) if mount is not None else ET.parse(filename)
    model = tree.getroot().find('model')
    links = {link.get('name'): link for link in model.findall('link')}
    joints = {joint.get('name'): joint for joint in model.findall('joint')}
    steering_limit = float(joints['front_left_steering_joint'].findtext('axis/limit/upper'))
    front = pose(links['front_left_wheel'])
    rear = pose(links['rear_left_wheel'])
    radius = float(links['rear_left_wheel'].findtext('collision/geometry/cylinder/radius'))
    half_width = 0.0
    rear_overhang = 0.0
    for name, link in links.items():
        lp = pose(link)
        box = link.find('collision/geometry/box/size')
        cylinder = link.find('collision/geometry/cylinder')
        if box is not None:
            half_width = max(half_width, abs(lp[1]) + float(box.text.split()[1]) / 2)
            rear_overhang = max(rear_overhang, -lp[0]+float(box.text.split()[0])/2)
        elif cylinder is not None:
            r = float(cylinder.findtext('radius'))
            length = float(cylinder.findtext('length'))
            if 'wheel' in name:
                angle = steering_limit if name.startswith('front') else 0.0
                extent = length / 2 * math.cos(angle) + r * math.sin(angle)
            else:
                extent = r
            half_width = max(half_width, abs(lp[1]) + extent)
            rear_overhang = max(rear_overhang, -lp[0]+r)
    base_link = pose(links['base_link'])
    lidar = pose(links['team_01_lidar_link'])
    camera = pose(links['camera_link']) if 'camera_link' in links else None
    return dict(wheelbase_m=front[0] - rear[0], steering_track_m=2 * abs(front[1]),
                wheel_radius_m=radius, vehicle_width_m=2 * half_width,
                max_steering_angle_rad=steering_limit, rear_overhang_m=rear_overhang, lidar_pose=lidar,
                camera_pose=camera, chassis_pose=base_link,
                mast_pose=pose(links['camera_mast_link']) if 'camera_mast_link' in links else None,
                imu_pose=pose(links['imu_link']) if 'imu_link' in links else None,
                camera_height_m=camera[2] + radius if camera else None)
