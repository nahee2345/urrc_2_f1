#!/usr/bin/env python3
"""HYBRID RACER: physical Ackermann vehicle, sensors and camera start gate.
Manual driving, camera start interlock and sensor-only low-speed autonomy.
"""
import argparse
from collections import deque
from dataclasses import dataclass, field
import math
from pathlib import Path
import subprocess
import time
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parent
BASE = '/team07/hybrid'
DRIVE_TOPICS = ('left_drive', 'right_drive', 'front_left_drive', 'front_right_drive')
REAR_WHEELS = ('rear_left_spin', 'rear_right_spin')
DRIVEN_WHEELS = REAR_WHEELS + ('front_left_spin', 'front_right_spin')
WHEELBASE, TRACK, RADIUS = .30, .12, .045
LIMIT = math.radians(35)
CENTER_LIMIT = math.atan(WHEELBASE / (WHEELBASE / math.tan(LIMIT) + TRACK / 2))


def resolve_path_mode(path_mode, mode):
    """Race and practice default to the sensor-only center path."""
    if path_mode=='auto':return 'center'
    if path_mode not in ('center','left_wall'):raise ValueError('invalid path mode')
    return path_mode


def cruise_limit(candidate=False):
    import json
    import os
    path=Path(os.environ.get('TEAM07_LOG_ROOT',str(ROOT)))/'.runtime/highspeed_limit.json'
    try:
        data=json.loads(path.read_text())
        value=float(data['candidate_cruise' if candidate else 'allowed_cruise'])
        return value if math.isfinite(value) and 3.<=value<=5. else 3.
    except (OSError,ValueError,KeyError,TypeError):return 3.


def restrict_highspeed(reason):
    """Persist fail-closed ceiling for subsequent CLI and running high-speed nodes."""
    import json
    import os
    path=Path(os.environ.get('TEAM07_LOG_ROOT',str(ROOT)))/'.runtime/highspeed_limit.json'
    path.parent.mkdir(parents=True,exist_ok=True)
    temporary=path.with_suffix('.tmp')
    temporary.write_text(json.dumps(dict(allowed_cruise=3.,candidate_cruise=3.,failure=reason),indent=2)+'\n')
    temporary.replace(path)


# Planning assumptions, deliberately weaker than 3.018m/s / 1.201m evidence.
# These are NOT measured 4m/s braking or sensor-error guarantees.
RACE_BRAKING = .7
RACE_LATENCY = .35  # scan age + processing/control/actuation allowance
RACE_RESERVE = .65
RACE_STRAIGHT_GOAL_MPS = 15./3.6
RACE_EXPERIMENTAL_CEILING_MPS = 4.
RACE_STRAIGHT_TRIAL_MPS = 2.25
LEGACY_WALL_TRACKING_M = 8.
STRAIGHT_WALL_TRACKING_M = 12.


def race_stopping_distance(speed, approaching=0.):
    v=max(0.,float(speed)); u=max(0.,float(approaching))
    return RACE_RESERVE+(v+u)*RACE_LATENCY+v*v/(2*RACE_BRAKING)+u*v/RACE_BRAKING


def race_distance_speed(available, approaching=0., braking=RACE_BRAKING):
    u=max(0.,float(approaching))
    b=braking*RACE_LATENCY+u
    budget=max(0.,float(available)-RACE_RESERVE-u*RACE_LATENCY)
    return max(0.,math.sqrt(b*b+2*braking*budget)-b)


def wall_stopping_distance(speed):
    v=abs(float(speed))
    return RACE_RESERVE+v*RACE_LATENCY+v*v/(2*.35)


def wall_distance_speed(available):
    b=.35*RACE_LATENCY
    return max(0.,math.sqrt(b*b+2*.35*max(0.,available-RACE_RESERVE))-b)


def lateral_wall_offset(speed, base_offset, curvature_error, speed_buffer):
    # Same geometric/error budget; no braking distance is spent sideways.
    return base_offset+.5*wall_stopping_distance(speed)**2*curvature_error+speed_buffer*abs(speed)


def lateral_wall_speed(clearance, base_offset, curvature_error, speed_buffer, ceiling=.5):
    # The caller also checks both steering trajectories before using relief.
    if lateral_wall_offset(0.,base_offset,curvature_error,speed_buffer)>=clearance:return 0.
    low,high=0.,ceiling
    for _ in range(28):
        mid=(low+high)/2
        if lateral_wall_offset(mid,base_offset,curvature_error,speed_buffer)<clearance:low=mid
        else:high=mid
    return low


def race_preview_speed(path, speed, ceiling, braking=RACE_BRAKING):
    import numpy as np
    xy=np.asarray([(0.,0.)]+list(path))
    delta=np.diff(xy,axis=0)
    heading=np.unwrap(np.arctan2(delta[:,1],delta[:,0]))
    ds=np.linalg.norm(delta,axis=1)
    limit=ceiling
    for i,k in enumerate(abs(np.diff(heading))/np.maximum(.5*(ds[:-1]+ds[1:]),.01)):
        bend_speed=math.sqrt(.65/max(float(k),.001))
        distance=max(0.,float(xy[i+1,0])-RACE_RESERVE-abs(speed)*RACE_LATENCY)
        limit=min(limit,math.sqrt(bend_speed*bend_speed+2*braking*distance))
    return limit


def observed_corner_preview(path, speed, ceiling, position_error=0.):
    """Signed heading change on supported path chords, with a braking envelope."""
    import numpy as np
    if len(path)<3:return None
    xy=np.asarray(path)
    segments=np.diff(xy,axis=0)
    lengths=np.linalg.norm(segments,axis=1)
    stations=np.r_[max(0.,xy[0,0]),max(0.,xy[0,0])+np.cumsum(lengths)]
    profile=[]
    limit=ceiling
    for i in range(1,len(xy)-1):
        # Use chords with a useful baseline and keep both ends inside the scan.
        lo=max(0,int(np.searchsorted(stations,stations[i]-.6,side='right'))-1)
        hi=min(len(xy)-1,int(np.searchsorted(stations,stations[i]+.6)))
        span=stations[hi]-stations[lo]
        if span<1.2:continue
        incoming,outgoing=xy[i]-xy[lo],xy[hi]-xy[i]
        turn=math.atan2(float(incoming[0]*outgoing[1]-incoming[1]*outgoing[0]),
            float(incoming@outgoing))
        curvature=2.*turn/span
        uncertainty=4.*position_error/span**2
        if abs(curvature)<max(.035,uncertainty):continue
        entry=float(stations[lo])
        local_cap=math.sqrt(.35/(abs(curvature)+uncertainty))
        distance=max(0.,entry-RACE_RESERVE-abs(speed)*RACE_LATENCY)
        limit=min(limit,math.sqrt(local_cap**2+2*.35*distance))
        profile.append((entry,float(curvature)))
    if not profile:return None
    first_entry,first_curvature=profile[0]
    return dict(direction='left' if first_curvature>0 else 'right',
        entry_m=first_entry,peak_curvature_per_m=max(abs(k) for _,k in profile),
        curvature_profile=profile,speed_cap_mps=limit)


def lamp_observation(msg):
    """Five positive lens detections, including dark red OFF lenses. No map/pose input."""
    import numpy as np
    import cv2
    if (msg.width<=0 or msg.height<=0 or msg.encoding not in ('rgb8','bgr8')
            or msg.step < msg.width*3 or len(msg.data) != msg.step*msg.height):
        return None
    rgb = np.frombuffer(msg.data,dtype=np.uint8).reshape(msg.height,msg.step)[:,:msg.width*3].reshape(msg.height,msg.width,3)
    if msg.encoding == 'bgr8':rgb = rgb[:,:,::-1]
    r,g,b = (rgb[:,:,i].astype(np.float32) for i in range(3))
    mask = ((r>=4)&(r>1.6*g+1)&(r>1.6*b+1)).astype(np.uint8)
    mask[int(msg.height*.65):] = 0
    _,_,stats,centers = cv2.connectedComponentsWithStats(mask)
    candidates = [(s,c) for s,c in zip(stats[1:],centers[1:])
        if 12<=s[4]<=msg.width*msg.height*.005 and 4<=s[2]<=msg.width*.08
        and 4<=s[3]<=msg.height*.1 and .5<s[2]/s[3]<1.8 and s[4]/(s[2]*s[3])>.5]
    if len(candidates)>80:return None
    groups = {}
    for _,anchor in candidates:
        row = sorted([(s,c) for s,c in candidates if abs(c[1]-anchor[1])<3.5],key=lambda item:item[1][0])
        for i in range(len(row)-4):
            group = row[i:i+5]
            xy = np.array([c for _,c in group])
            widths = np.array([s[2] for s,_ in group])
            heights = np.array([s[3] for s,_ in group])
            gaps = np.diff(xy[:,0])
            if (gaps.min()<widths.mean()*1.1 or gaps.max()>widths.mean()*2.5
                    or gaps.std()/gaps.mean()>.15 or widths.std()/widths.mean()>.25
                    or heights.max()/heights.min()>1.6):continue
            groups[tuple(round(v) for v in xy[:,0])] = group
    if len(groups)!=1:return None  # Ambiguous scene or partial occlusion never authorizes GO.
    group = next(iter(groups.values()))
    states = []
    for s,c in group:
        x,y = map(round,c)
        patch = rgb[max(0,y-2):y+3,max(0,x-2):x+3]
        red,green,blue = np.median(patch.reshape(-1,3),axis=0)
        if red>=90 and red>green*2 and red>blue*2:states.append(1)
        elif 4<=red<=40 and red>green*1.6 and red>blue*1.6:states.append(0)
        else:return None
    return tuple(states), tuple((float(c[0]),float(c[1])) for _,c in group)


@dataclass
class StartGate:
    state: str = 'WAITING'
    times: dict = field(default_factory=dict)
    seen_counts: list = field(default_factory=list)
    reason: str = ''
    previous: tuple = ()
    stable: int = 0
    candidate_stamp: float = 0.
    last_stamp: float = -math.inf
    last_wall: float = -math.inf
    centers: tuple = ()
    direction: int = 0
    count: int = 0
    gap_details: dict = field(default_factory=dict)

    @property
    def authorized(self):
        return self.state in ('GO','RACING')

    def watchdog(self, now, wall=None):
        if self.authorized:return
        wall = time.monotonic() if wall is None else wall
        if 'first_red' in self.times and (now-self.last_stamp>.30 or wall-self.last_wall>.60):
            if not self.reason:self.gap_details=dict(sim_gap=now-self.last_stamp,wall_gap=wall-self.last_wall,now=now,last_stamp=self.last_stamp)
            self.reason = 'camera_gap'
        if 'first_red' in self.times and now-self.times['first_red']>20:
            self.reason = 'start_deadline_exceeded'

    def observe(self, observation, stamp, wall=None):
        wall = time.monotonic() if wall is None else wall
        if self.authorized:return  # Permission latches only after positive ALL_RED -> OFF evidence.
        if stamp<=self.last_stamp:return
        self.watchdog(stamp,wall)
        self.last_stamp,self.last_wall = stamp,wall
        if self.reason:return
        if observation is None:
            self.previous,self.stable = (),0
            if self.state!='WAITING':self.reason = 'lens_occluded_or_uncertain'
            return
        states,centers = observation
        if self.centers and max(math.dist(a,b) for a,b in zip(self.centers,centers))>8:
            if self.state!='WAITING':
                self.reason = 'signal_moved_or_occluded';return
            self.previous,self.stable = (),0  # Reacquire after initial suspension settling.
        self.centers = centers
        if states==self.previous:self.stable+=1
        else:self.previous,self.stable,self.candidate_stamp = states,1,stamp
        if self.stable<2:return
        n = sum(states)
        if self.state=='WAITING':
            if n!=1:return
            if states[0]:self.direction = 1
            elif states[4]:self.direction = -1
            else:return
            self.times['first_red'] = self.candidate_stamp
            self.state = 'COUNTDOWN'
        if self.state=='COUNTDOWN':
            expected = tuple(int(i<n) for i in range(5)) if self.direction==1 else tuple(int(i>=5-n) for i in range(5))
            if states!=expected or n not in (self.count,self.count+1) or n==0:
                self.reason = 'nonsequential_lights';return
            if n>self.count:self.seen_counts.append(n)
            self.count = n
            if n==5:
                self.state = 'ALL_RED'
                self.times['all_red_observed'] = self.candidate_stamp
                self.times['all_red'] = stamp
        elif self.state=='ALL_RED':
            if n==5:return
            if n!=0:self.reason = 'partial_extinction';return
            if self.stable>=3 and stamp-self.candidate_stamp>=.09:
                self.times['off_observed'] = self.candidate_stamp
                self.times['go'] = stamp
                self.state = 'GO'

    def wheel_motion(self, stamp, speed):
        if self.state=='GO' and abs(speed)>.05:
            self.times['wheel_motion_estimate'] = stamp
            self.state = 'RACING'

    def report(self):
        return dict(state=self.state,times=self.times,seen_counts=self.seen_counts,
                    reason=self.reason,authorized=self.authorized,lenses=self.previous,gap_details=self.gap_details)


@dataclass
class SensorBuffer:
    """Bounded, time-stamped observations; never used as ground-truth pose."""
    samples: dict = field(default_factory=dict)
    rejected: dict = field(default_factory=dict)

    def put(self, kind, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        queue = self.samples.setdefault(kind, deque(maxlen=16))
        if queue and stamp < queue[-1][0]:
            # Drop reordered data. Restart the observer after a simulation time reset.
            self.rejected[kind] = self.rejected.get(kind, 0)+1
            return False
        valid = bool(msg.header.frame_id)
        if kind == 'camera':
            valid &= msg.encoding in ('rgb8', 'bgr8') and msg.width > 0 and msg.height > 0
            valid &= msg.step >= msg.width*3 and len(msg.data) == msg.step*msg.height
        elif kind == 'camera_info':
            valid &= msg.width > 0 and msg.height > 0 and msg.k[0] > 0 and msg.k[4] > 0
        elif kind == 'scan':
            valid &= len(msg.ranges) > 1 and msg.range_max > msg.range_min >= 0
            valid &= any(not math.isnan(v) and v >= msg.range_min for v in msg.ranges)
        elif kind == 'points':
            valid &= {'x', 'y', 'z'} <= {f.name for f in msg.fields}
            valid &= msg.width > 0 and msg.height > 0 and msg.point_step > 0
            valid &= len(msg.data) == msg.row_step*msg.height
        elif kind == 'imu':
            q = msg.orientation
            values = (q.x,q.y,q.z,q.w,msg.angular_velocity.x,msg.angular_velocity.y,
                      msg.angular_velocity.z,msg.linear_acceleration.x,
                      msg.linear_acceleration.y,msg.linear_acceleration.z)
            valid &= all(math.isfinite(v) for v in values)
            valid &= abs(sum(v*v for v in values[:4])-1) < .05
        if not valid:
            self.rejected[kind] = self.rejected.get(kind, 0)+1
            return False
        queue.append((stamp, time.monotonic(), msg))
        return True

    def camera_scan_pair(self, tolerance=.06):
        """Approximate simulation-time pairing, not a hardware synchronization claim."""
        cameras, scans = self.samples.get('camera'), self.samples.get('scan')
        if not cameras or not scans:
            return None
        camera = cameras[-1]
        scan = min(scans, key=lambda s: abs(s[0]-camera[0]))
        if abs(scan[0]-camera[0]) <= tolerance and time.monotonic()-min(scan[1],camera[1]) < .5:
            return camera[2], scan[2]
        return None

    def attitude(self):
        """IMU-only diagnostic attitude. No driving decisions use this in stage 2."""
        history = self.samples.get('imu')
        if not history:
            return None
        q = history[-1][2].orientation
        return (math.atan2(2*(q.w*q.x+q.y*q.z),1-2*(q.x*q.x+q.y*q.y)),
                math.asin(max(-1.,min(1.,2*(q.w*q.y-q.z*q.x)))),
                math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)))


def sensor_bridge_args(clock=False):
    args = [BASE+'/camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
            BASE+'/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            BASE+'/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            BASE+'/lidar3d/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            BASE+'/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            BASE+'/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model']
    if clock:
        args.append('/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock')
    return args


def vehicle_metrics():
    """Primitive envelope and inertial CoM from the generated SDF, before steering."""
    root = ET.fromstring(model_xml())
    low, high = [math.inf]*3, [-math.inf]*3
    mass, moment = 0., [0.]*3
    for link in root.findall('.//link'):
        x,y,z,roll,pitch,yaw = map(float,link.findtext('pose').split())
        m = float(link.findtext('inertial/mass'))
        mass += m
        for i,v in enumerate((x,y,z)):
            moment[i] += m*v
        geom = link.find('collision/geometry')
        if geom.find('box') is not None:
            half = [float(v)/2 for v in geom.findtext('box/size').split()]
        elif geom.find('cylinder') is not None:
            r = float(geom.findtext('cylinder/radius'))
            half = [r,r,float(geom.findtext('cylinder/length'))/2]
        else:
            half = [float(geom.findtext('sphere/radius'))]*3
        cr,sr,cp,sp,cy,sy = math.cos(roll),math.sin(roll),math.cos(pitch),math.sin(pitch),math.cos(yaw),math.sin(yaw)
        rotation = ((cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr),
                    (sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr),(-sp,cp*sr,cp*cr))
        for i,center in enumerate((x,y,z)):
            radius = sum(abs(v)*h for v,h in zip(rotation[i],half))
            low[i] = min(low[i],center-radius)
            high[i] = max(high[i],center+radius)
    return dict(mass_kg=mass,com_m=[v/mass for v in moment],
                straight_size_m=[b-a for a,b in zip(low,high)],
                swept_width_bound_m=2*(TRACK/2+math.hypot(RADIUS,.013)),top_m=high[2])


def design_review():
    """Offline engineering review only; map data never enters the controller."""
    import csv
    import json
    package = ROOT.parent/'urrc_track_gazebo'
    report = dict(vehicle=vehicle_metrics(),tracks={})
    for name in ('monza','hungaroring'):
        with (package/'tracks/processed'/f'{name}_centerline.csv').open() as f:
            rows = list(csv.DictReader(f))
        points = [tuple(float(r[k]) for k in ('x_m','y_m','z_m')) for r in rows]
        slopes = []
        for a,b in zip(points,points[1:]+points[:1]):
            distance = math.hypot(b[0]-a[0],b[1]-a[1])
            if distance > 1e-6:
                slopes.append(math.degrees(math.atan2(b[2]-a[2],distance)))
        grid = json.loads((package/'tracks/processed'/f'{name}.json').read_text())['grid'][0]
        gate = ET.parse(package/'worlds'/f'{name}_race.sdf').find('.//model[@name="race_start_gantry"]')
        gx,gy,gz,_,_,gyaw = map(float,gate.findtext('pose').split())
        pixels = []
        for i in range(1,6):
            lens = gate.find(f'.//visual[@name="red_lens_{i}"]')
            lx,ly,lz,*_ = map(float,lens.findtext('pose').split())
            dx,dy = gx+math.cos(gyaw)*lx-math.sin(gyaw)*ly-grid['x'], gy+math.sin(gyaw)*lx+math.cos(gyaw)*ly-grid['y']
            forward = math.cos(grid['yaw'])*dx+math.sin(grid['yaw'])*dy-.175
            lateral = -math.sin(grid['yaw'])*dx+math.cos(grid['yaw'])*dy
            up = gz+lz-grid['road_z']-.005-.145
            cx,cz = math.cos(.4)*forward+math.sin(.4)*up,-math.sin(.4)*forward+math.cos(.4)*up
            focal = 320/math.tan(.7)
            u,v = 320-focal*lateral/cx,180-focal*cz/cx
            pixels.append([round(u,1),round(v,1)])
            assert cx>0 and 0<=u<640 and 0<=v<360, f'{name} lens {i} outside nominal camera FOV'
        report['tracks'][name] = dict(centerline_z_range_m=[min(p[2] for p in points),max(p[2] for p in points)],
            nominal_grade_deg=[min(slopes),max(slopes)],nominal_lens_pixels=pixels)
    (ROOT/'design_review.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))


def targets(speed, steering):
    """ROS steering +right; Gazebo joint angle +left. Limit INNER wheel to 35deg."""
    if not all(math.isfinite(v) for v in (speed, steering)):
        return 0., 0., 0., 0.
    speed = max(-max(3.,cruise_limit(True)), min(max(3.,cruise_limit(True)), speed))
    delta = -max(-CENTER_LIMIT, min(CENTER_LIMIT, steering))
    curvature = math.tan(delta) / WHEELBASE
    left = math.atan2(WHEELBASE * curvature, 1 - TRACK * curvature / 2)
    right = math.atan2(WHEELBASE * curvature, 1 + TRACK * curvature / 2)
    return left, right, speed * (1 - TRACK * curvature / 2) / RADIUS, speed * (1 + TRACK * curvature / 2) / RADIUS


def four_wheel_targets(values):
    left, right, rear_left, rear_right = values
    return (*values, rear_left/math.cos(left), rear_right/math.cos(right))


def model_xml():
    parts = []
    def link(name, pose, mass, inertia, geometry, color):
        parts.append(f'<link name="{name}"><pose>{pose}</pose><inertial><mass>{mass}</mass><inertia><ixx>{inertia[0]}</ixx><iyy>{inertia[1]}</iyy><izz>{inertia[2]}</izz></inertia></inertial>')
        # Exactly the same primitive and pose for rendering and contact geometry.
        parts.append(f'<collision name="collision"><geometry>{geometry}</geometry><surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction></surface></collision>')
        parts.append(f'<visual name="visual"><geometry>{geometry}</geometry><material><ambient>{color}</ambient><diffuse>{color}</diffuse></material></visual></link>')
    def joint(name, parent, child, axis, lower, upper, effort, damping):
        parts.append(f'<joint name="{name}" type="revolute"><parent>{parent}</parent><child>{child}</child><axis><xyz>{axis}</xyz><limit><lower>{lower}</lower><upper>{upper}</upper><effort>{effort}</effort><velocity>100</velocity></limit><dynamics><damping>{damping}</damping></dynamics></axis></joint>')
    link('base_link_07', '0 0 .11 0 0 0', 1.2, (.0016,.0178,.01908), '<box><size>.42 .12 .04</size></box>', '.1 .35 .8 1')
    for axle, x in [('front', .15), ('rear', -.15)]:
        for side, y in [('left', TRACK/2), ('right', -TRACK/2)]:
            name = f'{axle}_{side}'
            parent = 'base_link_07'
            if axle == 'front':
                parent = name + '_knuckle'
                link(parent, f'{x} {y} .045 0 0 0', .03, (2e-6,2e-6,2e-6), '<sphere><radius>.008</radius></sphere>', '.2 .2 .2 1')
                joint(name+'_steer', 'base_link_07', parent, '0 0 1', -LIMIT, LIMIT, 1., .03)
                parts.append(f'<plugin filename="gz-sim-joint-position-controller-system" name="gz::sim::systems::JointPositionController"><joint_name>{name}_steer</joint_name><topic>{BASE}/{side}_steer</topic><p_gain>4</p_gain><d_gain>.01</d_gain><cmd_min>-1</cmd_min><cmd_max>1</cmd_max></plugin>')
            # Cylinder local Z becomes world +Y. Wheel joint axis is world +Y.
            link(name+'_wheel', f'{x} {y} .045 {-math.pi/2} 0 0', .08, (.0000450067,.0000450067,.000081), '<cylinder><radius>.045</radius><length>.026</length></cylinder>', '.04 .04 .04 1')
            # Axis expressed in model coordinates avoids the rotated cylinder frame.
            joint(name+'_spin', parent, name+'_wheel', '0 1 0', -1e16, 1e16, .3, .0001)
            parts[-1] = parts[-1].replace('<xyz>', '<xyz expressed_in="__model__">')
            drive_topic = side+'_drive' if axle=='rear' else name+'_drive'
            parts.append(f'<plugin filename="gz-sim-joint-controller-system" name="gz::sim::systems::JointController"><joint_name>{name}_spin</joint_name><topic>{BASE}/{drive_topic}</topic><use_force_commands>true</use_force_commands><p_gain>.03</p_gain><i_gain>.05</i_gain><i_min>-.1</i_min><i_max>.1</i_max><cmd_min>-.3</cmd_min><cmd_max>.3</cmd_max></plugin>')
    for name, pose, mass, inertia, geometry in [
        ('team07_camera', '.175 0 .145 0 -.4 0', .025, (2.47e-6,2.03e-6,2.83e-6),
         '<box><size>.024 .028 .020</size></box>'),
        ('team07_lidar', '0 0 .164 0 0 0', .045, (3.12e-6,3.12e-6,3.24e-6),
         '<cylinder><radius>.012</radius><length>.020</length></cylinder>'),
        ('team07_imu', '-.04 0 .136 0 0 0', .005, (1.82e-7,1.82e-7,3.33e-7),
         '<box><size>.020 .020 .006</size></box>')]:
        link(name, pose, mass, inertia, geometry, '.18 .18 .18 1')
        if name == 'team07_camera':
            sensors = f'<sensor name="team07_rgb" type="camera"><topic>{BASE}/camera/image</topic><gz_frame_id>team07_camera_optical</gz_frame_id><always_on>true</always_on><update_rate>20</update_rate><camera><camera_info_topic>{BASE}/camera/camera_info</camera_info_topic><horizontal_fov>1.4</horizontal_fov><image><width>640</width><height>360</height><format>R8G8B8</format></image><clip><near>.025</near><far>80</far></clip></camera></sensor>'
        elif name == 'team07_lidar':
            sensors = ''
            for suffix, count, rows, rate, vertical in [('scan',720,1,20,0),('lidar3d',360,16,10,.261799)]:
                near = .06 if rows == 1 else .25
                sensors += f'<sensor name="team07_{suffix}" type="gpu_lidar"><topic>{BASE}/{suffix}</topic><gz_frame_id>team07_lidar</gz_frame_id><always_on>true</always_on><update_rate>{rate}</update_rate><lidar><scan><horizontal><samples>{count}</samples><resolution>1</resolution><min_angle>{-math.pi}</min_angle><max_angle>{math.pi}</max_angle></horizontal><vertical><samples>{rows}</samples><resolution>1</resolution><min_angle>{-vertical}</min_angle><max_angle>{vertical}</max_angle></vertical></scan><range><min>{near}</min><max>40</max><resolution>.01</resolution></range></lidar></sensor>'
        else:
            sensors = f'<sensor name="team07_imu" type="imu"><topic>{BASE}/imu</topic><gz_frame_id>team07_imu</gz_frame_id><always_on>true</always_on><update_rate>100</update_rate><imu/></sensor>'
        parts[-1] = parts[-1].replace('</link>', sensors+'</link>')
        parts.append(f'<joint name="{name}_mount" type="fixed"><parent>base_link_07</parent><child>{name}</child></joint>')
    # Competition worlds already have Sensors(ogre2), but no Imu system.
    # The participant model carries the installed Imu system; operator worlds stay intact.
    parts.append('<plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>')
    parts.append(f'<plugin filename="gz-sim-joint-state-publisher-system" name="gz::sim::systems::JointStatePublisher"><topic>{BASE}/joint_states</topic><joint_name>rear_left_spin</joint_name><joint_name>rear_right_spin</joint_name><joint_name>front_left_spin</joint_name><joint_name>front_right_spin</joint_name><joint_name>front_left_steer</joint_name><joint_name>front_right_steer</joint_name></plugin>')
    return '<sdf version="1.9"><model name="team07_hybrid">'+''.join(parts)+'</model></sdf>'



@dataclass
class LocalDriver:
    """Sensor-only local corridor tracker. +Y/curvature left; ROS steering right.

    No map, Gazebo pose or predefined route enters this planner. Unknown rays
    are not free space. A sampled swept footprint rejects unsafe trajectories.
    """
    speed: float = 0.
    steering: float = 0.
    width: float = 0.
    filtered_desired: float = 0.
    speed_envelope: float = 0.
    safety_fault: str = ''
    safety_event: dict = field(default_factory=dict)
    candidate_audit: dict = field(default_factory=dict)
    path_mode: str = 'center'
    race_center: bool = False
    left_wall_distance: float = .30
    wall_margin: float = .04
    wall_memory_seconds: float = .5
    experimental_race_speed: bool = False
    left_speed_limit: float = .3  # commissioning ceiling until real clearance validation
    wall_steering_error: float = .04  # exceeds prior 0.034rad sampled command/encoder mismatch
    wall_target_buffer: float = .01
    wall_distance_error: float = .06  # planning allowance; not a certified error bound
    approach_slope_limit: float = .35
    approach_curvature_limit: float = .55
    approach_steering_limit: float = .22
    entry_state: dict = field(default_factory=dict)
    wall_target_adjustment: object = None
    obstacle_tracks: list = field(default_factory=list)
    obstacle_stamp: object = None
    obstacle_yaw: float = 0.
    obstacle_reseed: tuple | None = None
    obstacle_next_id: int = 1
    wall_observations: dict = field(default_factory=dict)
    wall_observation_stamp: object = None
    wall_observation_yaw: float = 0.
    wall_memory_audit: dict = field(default_factory=dict)
    straight_support_history: deque = field(default_factory=lambda: deque(maxlen=3))
    wall_recovery_active: bool = False
    wall_recovery_target: tuple | None = None
    wall_handoff_stamp: float | None = None
    width_age: float = math.inf
    width_seen_wall: float = -math.inf

    def __post_init__(self):
        if self.path_mode not in ('center','left_wall'):
            raise ValueError('path_mode must be center or left_wall')
        values=(self.left_wall_distance,self.wall_margin,self.wall_memory_seconds,self.left_speed_limit,
                self.approach_slope_limit,self.approach_curvature_limit,self.approach_steering_limit,self.wall_distance_error,self.wall_target_buffer,self.wall_steering_error)
        if not all(math.isfinite(v) and v>0 for v in values):
            raise ValueError('wall settings must be finite and positive')
        self.swept_half_width=vehicle_metrics()['swept_width_bound_m']/2
        if self.approach_steering_limit>=min(CENTER_LIMIT,.35):
            raise ValueError('approach steering must stay below physical/test limits')
        if self.left_speed_limit>(4. if self.experimental_race_speed else .3):
            raise ValueError('left_wall commissioning speed ceiling is 0.3m/s')


    def reset_left_entry(self):
        self.entry_state.clear()
        self.wall_target_adjustment=None
        self.straight_support_history.clear()

    @staticmethod
    def segmented_wall(group, near):
        """Validate two overlapping tangents on a single ray-connected chain."""
        import numpy as np
        xx,yy=near.T
        seam=.5*(xx.min()+xx.max())
        fits=[]
        residuals=[]
        origin=near[np.argmin(abs(xx-seam))]
        for subset in (near[xx<=seam+.20],near[xx>=seam-.20]):
            if len(subset)<12 or np.ptp(subset[:,0])<.75:return None
            chord=subset[np.argmax(subset[:,0])]-subset[np.argmin(subset[:,0])]
            tangent=chord/np.linalg.norm(chord)
            normal=np.array([-tangent[1],tangent[0]])
            u,v=(subset-origin)@tangent,(subset-origin)@normal
            steps=np.diff(u)
            if not (np.all(steps>0) or np.all(steps<0)):return None
            fit=np.polynomial.Polynomial.fit(u,v,2).convert()
            slope=fit.deriv()
            curvature=fit.deriv(2)(u)/np.hypot(1,slope(u))**3
            residual=float(np.max(abs(fit(u)-v)/np.hypot(1,slope(u))))
            if residual>.025 or abs(slope(u)).max()>.85 or abs(curvature).max()>.50:return None
            fits.append((fit,tangent,normal))
            residuals.append(residual)
        order=np.argsort(xx)
        overlap_x=np.array([seam-.15,seam,seam+.15])
        overlap=np.column_stack((overlap_x,np.interp(overlap_x,xx[order],yy[order])))-origin
        join=[]
        for fit,tangent,normal in fits:
            u=overlap@tangent
            slope=fit.deriv()(u)
            join.append((u[:,None]*tangent+fit(u)[:,None]*normal,
                math.atan2(tangent[1],tangent[0])+np.arctan(slope),
                fit.deriv(2)(u)/np.hypot(1,slope)**3))
        # The fits cover different lengths of the bend; compare their
        # curvature estimates while requiring tight position/tangent agreement.
        if (np.max(np.linalg.norm(join[0][0]-join[1][0],axis=1))>.02
                or np.max(abs(join[0][1]-join[1][1]))>.12
                or np.max(abs(join[0][2]-join[1][2]))>.32):
            return None
        # Retain a contiguous interval in ray order. Only add sampled points
        # that agree with the adjacent local fit, within its support scale.
        x=group[:,0]
        errors=[]
        for fit,tangent,normal in fits:
            u,v=(group-origin)@tangent,(group-origin)@normal
            errors.append(abs(fit(u)-v)/np.hypot(1,fit.deriv()(u)))
        rear_error,front_error=errors
        consistent=np.where(x<=seam,rear_error,front_error)<=.025
        consistent&=(x>=xx.min()-.20)&(x<=xx.max()+.45)
        support=np.flatnonzero((x>=xx.min())&(x<=xx.max()))
        first,last=int(support[0]),int(support[-1])
        if not np.all(consistent[first:last+1]):return None
        while first>0 and consistent[first-1]:first-=1
        while last+1<len(group) and consistent[last+1]:last+=1
        return group[first:last+1],float(np.interp(0.,xx[order],yy[order])),max(residuals)

    @staticmethod
    def continuous_boundaries(points, indices, angle_increment, audit=None):
        """Geometric hypotheses only: continuous returns anchored beside body.

        No semantic vehicle/wall recognition is possible from a single scan.
        Short wall support or multiple boundary candidates cause rejection.
        Unexplained objects retain all returns for path collision checking. All raw points remain in the downstream collision checker.
        """
        import numpy as np
        keep=(points[:,0]>-.45)&(points[:,0]<4.5)&(abs(points[:,1])<8.)
        selected=points[keep];ids=indices[keep]
        candidates={1:[],-1:[]};residuals=[]
        if len(selected)<3:return np.empty((0,2)),np.empty((0,2)),0.,None
        gap=np.linalg.norm(np.diff(selected,axis=0),axis=1)
        threshold=.06+2*np.linalg.norm(selected[:-1],axis=1)*angle_increment
        cuts=np.flatnonzero((np.diff(ids)!=1)|(gap>threshold))+1
        groups=np.split(selected,cuts)
        for group in groups:
            near=group[(group[:,0]>=-.3)&(group[:,0]<=1.5)]
            if audit is not None:
                detail=dict(points=len(group), near_points=len(near), span_m=float(np.ptp(group[:,0])),
                    near_y_m=float(np.median(near[:,1])) if len(near) else None)
                audit.append(detail)
            if (len(near)<8 or np.ptp(group[:,0])<1.2 or
                    np.count_nonzero(abs(near[:,0])<.25)<3):
                if audit is not None:detail['result']='insufficient_support'
                continue
            fit=np.polynomial.polynomial.polyfit(near[:,0],near[:,1],1)
            residual=float(np.max(abs(np.polynomial.polynomial.polyval(near[:,0],fit)-near[:,1])))/math.hypot(1,fit[1])
            if audit is not None:detail['line_residual_m']=residual
            curved=False
            if residual>.025:
                # Test a smooth local bend on the same adjacent rays and
                # with the same near-wall tolerance. Require longitudinal
                # support on both sides of the anchor and no fold in ray order.
                xx,yy=near.T
                direction=1. if xx[-1]>xx[0] else -1.
                if xx.min()>-.15 or xx.max()<1.35 or np.any(direction*np.diff(xx)<=0):
                    if audit is not None:detail['result']='curve_support'
                    continue
                curve=np.polynomial.polynomial.polyfit(xx,yy,2)
                slope=curve[1]+2*curve[2]*xx
                error=abs(np.polynomial.polynomial.polyval(xx,curve)-yy)/np.hypot(1,slope)
                curve_residual=float(np.max(error))
                if curve_residual>.025 or curve_residual>.6*residual or abs(curve[2])>.15:
                    segmented=LocalDriver.segmented_wall(group,near)
                    if segmented is None:
                        if audit is not None:detail['result']='curve_residual'
                        continue
                    prefix,signed,residual=segmented
                    if abs(signed)<.08:continue
                    candidates[1 if signed>0 else -1].append(prefix)
                    residuals.append(residual)
                    if audit is not None:
                        detail.update(result='accepted',model='segmented',prefix_points=len(prefix),
                            segmented_residual_m=residual,curve_residual_m=curve_residual)
                    continue
                fit,residual,curved=curve,curve_residual,True
                if audit is not None:detail['curve_residual_m']=residual
            signed=float(fit[0]/math.hypot(1,fit[1]))
            if abs(signed)<.08:
                if audit is not None:detail['result']='inside_body_reference'
                continue
            # A return chain may bend; retain only its contiguous near-line
            # support, never extend through an occluder or an unknown gap.
            slope=fit[1]+2*fit[2]*group[:,0] if curved else fit[1]
            consistent=abs(np.polynomial.polynomial.polyval(group[:,0],fit)-group[:,1])/np.hypot(1,slope)<(.025 if curved else .06)
            end=float(np.min(group[(~consistent)&(group[:,0]>.25),0],initial=math.inf))
            prefix=group[consistent&(group[:,0]<end)]
            if audit is not None:
                detail.update(prefix_points=len(prefix), result='accepted' if len(prefix)>=8 else 'short_prefix')
            if len(prefix)<8:continue
            candidates[1 if signed>0 else -1].append(prefix)
            residuals.append(residual)
        if any(len(v)>1 for v in candidates.values()):
            return None,None,0.,'ambiguous_wall_boundary'
        left=candidates[1][0] if candidates[1] else np.empty((0,2))
        right=candidates[-1][0] if candidates[-1] else np.empty((0,2))
        # Nearby returns outside either supported boundary cannot be silently
        # labelled as a wall: stop on the ambiguous object instead.
        if len(left) and len(right):
            for group in groups:
                near=group[(group[:,0]>.1)&(group[:,0]<1.5)]
                if len(near)<3:continue
                explained=np.zeros(len(near),dtype=bool)
                for boundary in (left,right):
                    fit=np.polynomial.polynomial.polyfit(boundary[:,0],boundary[:,1],1)
                    explained|=abs(np.polynomial.polynomial.polyval(near[:,0],fit)-near[:,1])<.10
                    remaining=np.flatnonzero(~explained)
                    explained[remaining]=np.any(
                        np.all(near[remaining,None,:]==boundary[None,:,:],axis=2),axis=1)
                if np.count_nonzero(~explained)>=3:
                    return left,right,max(residuals,default=0.),'ambiguous_obstacle_in_corridor'
        return left,right,max(residuals,default=0.),None

    @staticmethod
    def extend_observed_boundary(seed, points, indices, audit=None):
        """Extend a classified near wall through adjacent live rays only.

        .75m is a conservative sampling ceiling, not sensor range or guaranteed
        obstacle resolution. Never cross a missing ray or invent wall points.
        """
        import numpy as np
        if audit is not None:audit['end_reason']='anchor_unavailable'
        if len(seed)<8 or not len(points):return seed
        end=int(np.argmax(seed[:,0]))
        j=int(np.argmin(np.linalg.norm(points-seed[end],axis=1)))
        if np.linalg.norm(points[j]-seed[end])>1e-8:return seed
        choices=[k for k in (j-1,j+1) if 0<=k<len(points) and points[k,0]>points[j,0]]
        if len(choices)!=1:return seed
        direction=choices[0]-j
        result=list(seed)
        support=seed[np.argsort(seed[:,0])]
        reason='end_of_scan'
        while 0<=j+direction<len(points):
            k=j+direction;point=points[k];previous=points[j]
            if abs(int(indices[k])-int(indices[j]))!=1:
                reason='missing_ray';break
            if point[0]<=previous[0] or point[0]>16.5:
                reason='forward_extent';break
            if np.linalg.norm(point-previous)>.75:
                reason='sample_gap';break
            local=support[support[:,0]>=support[-1,0]-1.5]
            if len(local)<3:local=support[-3:]
            if np.ptp(local[:,0])<.05:
                reason='insufficient_tangent_support';break
            fit=np.polynomial.polynomial.polyfit(local[:,0],local[:,1],1)
            error=abs(point[1]-np.polynomial.polynomial.polyval(point[0],fit))/math.hypot(1,fit[1])
            if error>.04+.002*point[0]:
                reason='wall_residual';break
            tangent=math.atan2(point[1]-previous[1],point[0]-previous[0])
            if abs(tangent-math.atan(fit[1]))>.15:
                reason='wall_tangent';break
            result.append(point);support=np.vstack((support,point));j=k
        if audit is not None:audit.update(end_m=float(support[-1,0]),end_reason=reason)
        return np.asarray(result)

    def occluded_boundaries(self, left, right, scan, stamp, speed, yaw):
        """Reuse only previously measured, currently object-occluded wall points.

        Never label hidden space free: all live ranges still gate trajectories.
        Each cached point keeps its original age/travel budget across refreshes.
        """
        import numpy as np
        elapsed=None if self.wall_observation_stamp is None else stamp-self.wall_observation_stamp
        if elapsed==0:
            return tuple(self.wall_observations.get(k,{}).get('output',v) for k,v in (('left',left),('right',right)))
        self.wall_memory_audit=dict(reused_points=0,max_age_s=0.,max_travel_m=0.,confidence=1.)
        if elapsed is not None and not 0<elapsed<=.3:
            self.wall_observations.clear()
        turn=math.atan2(math.sin(yaw-self.wall_observation_yaw),math.cos(yaw-self.wall_observation_yaw))
        rotation=np.array([[math.cos(turn),math.sin(turn)],[-math.sin(turn),math.cos(turn)]])
        translation=np.array([speed*(elapsed or 0.)*math.cos(turn/2),speed*(elapsed or 0.)*math.sin(turn/2)])
        # Require current bilateral anchors; complete boundary loss is not occlusion.
        anchors=len(left)>=8 and len(right)>=8
        objects=[]
        if elapsed is not None and 0<elapsed<=.3:
            for track in self.obstacle_tracks:
                if track['confident']:
                    objects.append((rotation@(np.asarray(track['position'])-translation)+
                        elapsed*(rotation@np.asarray(track['velocity'])),track['radius']))
        outputs=[]
        ranges=np.asarray(scan.ranges,dtype=float)
        for key,live in (('left',left),('right',right)):
            old=self.wall_observations.get(key)
            reused=np.empty((0,2));ages=np.empty(0);travel=np.empty(0)
            if old and anchors and objects and elapsed is not None and 0<elapsed<=.3:
                cached=(old['points']-translation)@rotation.T
                age=old['age']+elapsed;distance=old['travel']+abs(speed)*elapsed
                # A changed visible anchor invalidates this side's memory.
                fit=np.polynomial.polynomial.polyfit(live[:,0],live[:,1],1)
                consistent=abs(cached[:,1]-np.polynomial.polynomial.polyval(cached[:,0],fit))/math.hypot(1,fit[1])<.08
                angle=np.arctan2(cached[:,1],cached[:,0])
                idx=np.rint((angle-scan.angle_min)/scan.angle_increment).astype(int)
                valid=(idx>=0)&(idx<len(ranges));idx=np.clip(idx,0,len(ranges)-1)
                observed=ranges[idx]
                hit=np.column_stack((observed*np.cos(angle),observed*np.sin(angle)))
                occluder=np.zeros(len(cached),dtype=bool)
                for center,radius in objects:
                    occluder|=np.linalg.norm(hit-center,axis=1)<radius+.06
                keep=(valid&consistent&np.isfinite(observed)&(observed>.08)&
                    (observed<np.linalg.norm(cached,axis=1)-.15)&occluder&
                    (age<=.5)&(distance<=.08)&(cached[:,0]>.25)&(cached[:,0]<2.))
                reused=cached[keep];ages=age[keep];travel=distance[keep]
                if len(ages):
                    self.wall_memory_audit['max_age_s']=max(self.wall_memory_audit['max_age_s'],float(ages.max()))
                    self.wall_memory_audit['max_travel_m']=max(self.wall_memory_audit['max_travel_m'],float(travel.max()))
                    self.wall_memory_audit['reused_points']+=len(ages)
            output=np.concatenate((live,reused))
            self.wall_observations[key]=dict(points=output,age=np.r_[np.zeros(len(live)),ages],
                travel=np.r_[np.zeros(len(live)),travel],output=output)
            outputs.append(output)
        self.wall_observation_stamp=stamp;self.wall_observation_yaw=yaw
        self.wall_memory_audit['confidence']=max(0.,1.-max(self.wall_memory_audit['max_age_s']/.5,
            self.wall_memory_audit['max_travel_m']/.08))
        return tuple(outputs)

    def track_obstacles(self, points, indices, left, right, stamp, speed, yaw,
                        live_extension=None, far_end=LEGACY_WALL_TRACKING_M, can_reseed=False):
        """Local scan clusters; no truth pose, map, or semantic vehicle labels."""
        import numpy as np
        if not math.isfinite(stamp):return [],'obstacle_time_invalid'
        elapsed=None if self.obstacle_stamp is None else stamp-self.obstacle_stamp
        if elapsed is not None and elapsed==0:return self.obstacle_tracks,None
        if (self.race_center and not self.obstacle_tracks
                and ((len(left)<8) != (len(right)<8))
                and (elapsed is None or 0.<elapsed<=.3)):
            # A fresh single-wall scan has no tracks to associate. Keep the
            # scan clock current while raw returns still gate every drive arc.
            self.obstacle_stamp,self.obstacle_yaw=stamp,yaw
            self.obstacle_reseed=None
            return [],None
        reseeding=elapsed is not None and not 0<elapsed<=.3
        if reseeding:
            # An old association clock cannot advance while the boundary is
            # missing. Establish a new tracking epoch only from current scans.
            signature=[]
            if elapsed>0 and can_reseed and abs(speed)<.02:
                for chain in (left,right):
                    order=chain[np.argsort(chain[:,0])]
                    if len(order)<8 or order[0,0]>.6 or order[-1,0]<1.:break
                    signature.extend(np.interp((.6,1.),order[:,0],order[:,1]))
            if len(signature)!=4:
                self.obstacle_reseed=None
                return self.obstacle_tracks,'obstacle_scan_gap'
            previous_seed=self.obstacle_reseed
            if previous_seed and stamp==previous_seed[0]:
                return self.obstacle_tracks,'obstacle_scan_gap'
            consistent=previous_seed and 0<stamp-previous_seed[0]<=.15 and (
                abs(math.atan2(math.sin(yaw-previous_seed[1]),math.cos(yaw-previous_seed[1])))<=.015
                and np.max(abs(np.asarray(signature)-previous_seed[2]))<=.10)
            count=previous_seed[3]+1 if consistent else 1
            self.obstacle_reseed=(stamp,yaw,np.asarray(signature),count)
            if count<3:return self.obstacle_tracks,'obstacle_scan_gap'
            elapsed=None
        keep=(points[:,0]>-.4)&(points[:,0]<4.)
        if len(left)<8 or len(right)<8:
            return self.obstacle_tracks,('obstacle_boundary_unavailable' if self.obstacle_tracks else None)
        keep&=(points[:,0]>=max(left[:,0].min(),right[:,0].min()))
        keep&=(points[:,0]<=min(left[:,0].max(),right[:,0].max()))
        # Only subtract established wall support, never an extrapolated wall.
        for wall in (left,right):
            if len(wall)<8:continue
            fit=np.polynomial.polynomial.polyfit(wall[:,0],wall[:,1],1)
            wall_return=(points[:,0]>=wall[:,0].min())&(points[:,0]<=wall[:,0].max())
            wall_return&=abs(points[:,1]-np.polynomial.polynomial.polyval(points[:,0],fit))/math.hypot(1,fit[1])<.10
            remaining=np.flatnonzero(keep&~wall_return)
            wall_return[remaining]=np.any(
                np.all(points[remaining,None,:]==wall[None,:,:],axis=2),axis=1)
            keep&=~wall_return
        if live_extension is not None:
            live_left,live_right=live_extension
            far_end=min(far_end,float(live_left[:,0].max()),float(live_right[:,0].max()))
            far_indices=np.flatnonzero((points[:,0]>=4.)&(points[:,0]<=far_end))
            # Exclude only the live contiguous wall samples themselves.
            # An extrapolated line could mask a real obstacle beside the wall.
            for wall in live_extension:
                on_wall=np.any(np.all(points[far_indices,None,:]==wall[None,:,:],axis=2),axis=1)
                far_indices=far_indices[~on_wall]
            keep[far_indices]=True
        pp=points[keep];ii=indices[keep]
        cuts=np.flatnonzero((np.diff(ii)!=1)|(np.linalg.norm(np.diff(pp,axis=0),axis=1)>.18))+1
        detections=[]
        for group in np.split(pp,cuts):
            # Past 8m a narrow object may hit only one beam. Keep it as
            # uncertain obstacle evidence as well as a raw swept-check return.
            if len(group)<2 and (not len(group) or group[0,0]<=LEGACY_WALL_TRACKING_M):continue
            center=group.mean(axis=0)
            detections.append((center,float(np.max(np.linalg.norm(group-center,axis=1)))+.08))
        previous=[] if reseeding else self.obstacle_tracks
        rotation=np.eye(2)
        if elapsed is not None:
            turn=math.atan2(math.sin(yaw-self.obstacle_yaw),math.cos(yaw-self.obstacle_yaw))
            rotation=np.array([[math.cos(turn),math.sin(turn)],[-math.sin(turn),math.cos(turn)]])
        predicted=[]
        for track in previous:
            old=np.asarray(track['position'])
            translation=np.array([speed*elapsed*math.cos(turn/2),speed*elapsed*math.sin(turn/2)])
            predicted.append(rotation@(old-translation))
        matches=[];used=set();fault=None
        for center,radius in detections:
            distances=[float(np.linalg.norm(center-(pos+rotation@np.asarray(previous[j]['velocity'])*elapsed)))
                for j,pos in enumerate(predicted)]
            possible=[j for j,d in enumerate(distances) if d<.25+.8*elapsed]
            if len(possible)>1 or any(j in used for j in possible):
                fault='obstacle_association_failed';break
            if possible:
                j=possible[0];used.add(j);old=previous[j]
                velocity=(center-predicted[j])/elapsed
                hits=old['hits']+1
                track_id=old['id']
                if np.linalg.norm(velocity)>2. or np.linalg.norm(velocity-rotation@np.asarray(old['velocity']))>.6:
                    hits=1
                velocity=.5*velocity+.5*(rotation@np.asarray(old['velocity']))
            else:
                velocity=np.zeros(2);hits=1
                track_id=self.obstacle_next_id;self.obstacle_next_id+=1
            matches.append(dict(id=track_id,position=center.tolist(),radius=radius,velocity=velocity.tolist(),
                relative_velocity=(velocity-np.array([speed,0.])).tolist(),hits=hits,confident=hits>=3))
        if len(used)!=len(previous):fault='obstacle_track_lost'
        if fault:return previous,fault
        self.obstacle_tracks=matches;self.obstacle_stamp=stamp;self.obstacle_yaw=yaw
        self.obstacle_reseed=None
        return matches,None

    @staticmethod
    def supported_wall_prefix(walls):
        """Keep contiguous measured support, never bridge a distant inferred wall.

        The existing short-lived width fallback is restricted to the near field
        when no bilateral prefix exists. It remains explicitly uncertain.
        """
        supported=[]
        inferred=bool(walls and walls[0].get('one_side',False))
        for wall in walls:
            if wall['distance']>(1.5 if inferred else 4.) and not (not inferred and wall.get('long_observed',False)):break
            if bool(wall.get('one_side',False))!=inferred:break
            supported.append(wall)
        return supported

    def left_wall_entry(self, reference, walls, safe_offset, measured_speed=0., dt=0., steering_actual=None, target_distance=None, adaptive_gap=False):
        """Smooth entry from body origin/heading; pursuit controller unchanged.

        Blend lateral displacement slowly and wall heading sooner, so replanning
        does not keep demanding the entire lateral error at the short lookahead.
        Check a dense path against measured corridor and physical steering bounds.
        """
        import numpy as np
        walls=self.supported_wall_prefix(walls)
        if len(walls)<2 or walls[-1]['distance']<max(1.,wall_stopping_distance(measured_speed)):
            return None,dict(entry_rejection='insufficient_observed_support')
        px,py=np.asarray(reference[:len(walls)]).T
        fit_points=px<=4.  # preserve near-field fit; far walls constrain/check, never refit near steering
        fit=np.polynomial.polynomial.polyfit(px[fit_points],py[fit_points],
            min(2,int(np.count_nonzero(fit_points))-1))
        residual=float(np.max(abs(np.polynomial.polynomial.polyval(px[fit_points],fit)-py[fit_points])))
        if residual>.10:return None,dict(entry_rejection='wall_fit',entry_fit_residual_m=residual)
        far_fit=None
        if px[-1]>LEGACY_WALL_TRACKING_M and walls[-1].get('straight_extension'):
            far=px>=5.
            far_fit=np.polynomial.polynomial.polyfit(px[far],py[far],1)
        goal=max(self.left_wall_distance,safe_offset) if target_distance is None else target_distance
        a,b=float(fit[0]),float(fit[1])
        curvature_limit=min(self.approach_curvature_limit,math.tan(self.approach_steering_limit)/WHEELBASE)
        length=max(2.,2*abs(a)/self.approach_slope_limit,
            math.sqrt(6*abs(a)/curvature_limit))
        near=px<=1.5
        wall_fit=np.polynomial.polynomial.polyfit(px[near],
            np.asarray([w['left'] for w in walls])[near],1)
        wall_distance=float(wall_fit[0]/math.hypot(1.,wall_fit[1]))
        right_fit=np.polynomial.polynomial.polyfit(px[near],
            np.asarray([w['right'] for w in walls])[near],1)
        right_distance=float(-right_fit[0]/math.hypot(1.,right_fit[1]))
        trusted=(not self.wall_memory_audit.get('reused_points') and
            all(w.get('left_count',0)>=2 and not w.get('one_side',True) for w in walls[:2]))
        heading_error=math.atan(float(wall_fit[1]))
        step=max(0.,min(self.left_speed_limit,measured_speed))*dt if trusted else 0.
        state=self.entry_state
        # Re-anchor on discontinuous geometry or a long observation gap. New
        # walls/obstacles are always checked below; no old path is executed.
        if (not state or abs(wall_distance-state['last_distance'])>.20+step or
                time.monotonic()-state['seen']>self.wall_memory_seconds):
            state.clear()
            state.update(initial_error=max(.01,wall_distance-goal),
                initial_goal=goal,initial_length=length,travel=0.,phase=0.)
        state['last_distance']=wall_distance
        state['seen']=time.monotonic()
        state['travel']+=step
        progress_goal=state['initial_goal'] if adaptive_gap else goal
        observed=max(0.,min(1.,1-(wall_distance-progress_goal)/state['initial_error']))
        # Invert smoothstep using bisection: observed lateral closure expressed
        # as distance along the original approach. Wheel travel alone cannot
        # keep advancing a phase when LiDAR reports no lateral progress.
        low,high=0.,1.
        for _ in range(16):
            mid=(low+high)/2
            if mid**3*(10-15*mid+6*mid*mid)<observed:low=mid
            else:high=mid
        observed_phase=(low+high)/2*state['initial_length']
        proposed=min(.5*(state['travel']+observed_phase),observed_phase+.35*state['initial_length'])
        actual=self.steering if steering_actual is None else steering_actual
        actual_curvature=-math.tan(actual)/WHEELBASE
        following=(trusted and abs(a)<=(.07 if state.get('following') else .03)
            and abs(heading_error)<=(.14 if state.get('following') else .08))
        if adaptive_gap and trusted:
            # Admit a small correction on a near-parallel, already cleared
            # path. Use the fitted reference tangent as well as the wall.
            latched=state.get('following',False)
            following=(abs(a)<=(.10 if latched else .065)
                and abs(wall_distance-goal)<=(.12 if latched else .08)
                and max(abs(heading_error),abs(math.atan(b)))<=(.14 if latched else .08))
        heading_blend=max(0.,min(1.,(.14-abs(heading_error))/.06))
        budget=max(0.,1-abs(heading_error)/math.atan(self.approach_slope_limit))
        budget*=max(0.,1-abs(actual_curvature)/curvature_limit)
        if budget<.1:budget=0.  # align before spending the last curvature margin
        previous=state['phase']
        proposed=max(0.,min(proposed,previous+2*step*budget))
        # Commit progress only after a feasible curve is found. Hold/back off
        # the approach if heading alignment consumes the curvature budget.
        phases=list(dict.fromkeys((proposed,previous,max(0.,previous-2*step))))
        # Include x=0 for geometry checks, but retain the existing first PP point.
        xx=np.linspace(0,px[-1],max(61,math.ceil(px[-1]/.04)+1))
        def boundary(key):
            values=np.asarray([w[key] for w in walls])
            result=np.interp(xx,px,values)
            before=xx<px[0]
            result[before]=values[0]+(values[1]-values[0])/(px[1]-px[0])*(xx[before]-px[0])
            return result
        left,right=boundary('left'),boundary('right')
        # Conservative allowance for normal offset, using each boundary slope.
        far_error=.002*np.maximum(0.,xx-4.)  # planning allowance, not certified sensor error
        left_normal=np.hypot(1,np.gradient(left,xx))
        right_normal=np.hypot(1,np.gradient(right,xx))
        lower=right+(safe_offset+far_error)*right_normal
        upper=left-(safe_offset+far_error)*left_normal
        def smooth(u):
            u=np.clip(u,0,1)
            return u*u*u*(10+u*(-15+6*u))
        rejected_candidates=[]
        candidates=[(phase,scale) for phase in phases for scale in (1.,1.3,1.7,2.2)]
        offset_error=wall_distance-goal
        correction_distance=0.
        if adaptive_gap and trusted:
            # Start unwinding before the lateral motion during the existing
            # response-time allowance carries the car across the target.
            closure=abs(measured_speed)*max(0.,-math.sin(heading_error))*RACE_LATENCY
            correction_distance=min(.07,closure)*smooth((.50-offset_error)/.30)*smooth((offset_error+.15)/.15)
        corrections=(correction_distance,0.) if correction_distance>0. else (0.,)
        for phase,scale,nudge in ((phase,scale,nudge) for nudge in corrections for phase,scale in candidates):
            entry_length=max(1.5,length-phase)*scale
            heading_length=entry_length
            # Quintic Hermite correction: start at current position, heading
            # and measured curvature; join wall position/tangent/curvature.
            # Unlike independent blends, these endpoint conditions cannot
            # introduce a new heading discontinuity when phase increases.
            c0=-a
            c1=-b*entry_length
            c2=(.5*actual_curvature-(float(fit[2]) if len(fit)>2 else 0.))*entry_length**2
            correction=(c0,c1,c2,-10*c0-6*c1-3*c2,
                15*c0+8*c1+3*c2,-6*c0-3*c1-c2)
            def curve(x):
                u=np.clip(x/entry_length,0,1)
                approach=(np.polynomial.polynomial.polyval(x,fit)
                    +np.polynomial.polynomial.polyval(u,correction))
                tracking=smooth((.12-abs(a))/.09)*smooth(heading_blend)
                unwind=nudge*smooth(x/1.25)*smooth((4.-x)/1.75)
                planned=(1-tracking)*approach+tracking*np.polynomial.polynomial.polyval(x,fit)-unwind
                if far_fit is not None:
                    weight=smooth((x-4.)/4.)
                    planned+=weight*(np.polynomial.polynomial.polyval(x,far_fit)-planned)
                return planned
            yy=curve(xx)
            slope=np.gradient(yy,xx)
            curvature=np.gradient(slope,xx)/(1+slope*slope)**1.5
            samples=xx>=px[0]
            pursuit_y=np.interp(xx[samples],px,curve(px))
            pursuit=abs(2*pursuit_y/(xx[samples]**2+pursuit_y**2))
            reasons=[]
            violations=np.zeros(len(xx),dtype=bool)
            if np.max(abs(slope))>self.approach_slope_limit:
                reasons.append('slope');violations|=abs(slope)>self.approach_slope_limit
            if np.max(abs(curvature))>curvature_limit:
                reasons.append('curvature');violations|=abs(curvature)>curvature_limit
            if np.max(pursuit)>math.tan(self.approach_steering_limit)/WHEELBASE:
                reasons.append('pursuit_steering')
                violations[samples]|=pursuit>math.tan(self.approach_steering_limit)/WHEELBASE
            if reasons:
                rejected_candidates.append(dict(phase=phase,scale=scale,reasons=reasons,
                    first_unsafe_x=float(xx[np.flatnonzero(violations)[0]])))
                continue
            near_curvature=float(np.interp(.3,xx,curvature))
            entry_steering=-math.atan(WHEELBASE*near_curvature)
            if abs(entry_steering-actual)>.65*(.2-.08):
                rejected_candidates.append(dict(phase=phase,scale=scale,reasons=['steering_reach'],first_unsafe_x=.3))
                continue
            outside=(yy<lower)|(yy>upper)
            if np.any(outside):
                rejected_candidates.append(dict(phase=phase,scale=scale,reasons=['wall_corridor'],
                    first_unsafe_x=float(xx[np.flatnonzero(outside)[0]])))
                continue
            state['phase']=phase
            state['following']=following
            return list(zip(px.tolist(),curve(px).tolist())),dict(
                entry_candidates_rejected=rejected_candidates,entry_heading_error_rad=heading_error,entry_actual_curvature=actual_curvature,
                entry_progress_budget=budget,entry_progress_held=bool(phase<=previous),
                entry_near_steering=entry_steering,
                entry_observed_end_m=float(px[-1]),entry_fit_end_m=float(px[-1]),
                entry_far_extrapolation_m=0.,entry_near_projection_m=float(px[0]),
                entry_support_inferred=bool(walls[0].get('one_side',False)),
                entry_length_m=entry_length,entry_heading_length_m=heading_length,
                entry_phase_m=phase,entry_travel_m=state['travel'],entry_observed_phase_m=observed_phase,
                entry_wall_distance_m=wall_distance,entry_right_wall_distance_m=right_distance,entry_trusted=trusted,
                entry_unwind_adjustment_m=float(nudge),
                path_left_wall_margin_m=float(np.min((upper-yy)/left_normal)),
                path_right_wall_margin_m=float(np.min((yy-lower)/right_normal)),
                entry_mode='wall_follow' if following else 'approach',
                entry_max_slope=float(np.max(abs(slope))),
                entry_max_curvature=float(np.max(abs(curvature))),entry_fit_residual_m=residual)
        return None,dict(entry_rejection='geometry_or_corridor',entry_fit_residual_m=residual,entry_candidates_rejected=rejected_candidates)

    def feasible_steering(self, scan, points, ranges, clear, speed, dt, slew,
                          desired, actual, cruise, braking):
        """Rate-reachable PP commands; encoder angle + measured 80ms delay model.
        The delay is a conservative prediction assumption, not a new controller.
        """
        import numpy as np
        latency=RACE_LATENCY if self.experimental_race_speed else .08
        required=max(1.,speed*speed/(2*braking)+speed*latency+.65)
        probe=min(cruise,max(speed,self.speed)+.5)
        horizon=max(required,probe*probe/(2*braking)+probe*latency+.65)
        distances=np.linspace(0,horizon,max(24,math.ceil(horizon/.10)+1))
        ds=float(distances[1]-distances[0])
        lateral_limit=math.atan(.65*WHEELBASE/max(speed*speed,.01))
        lower=max(-CENTER_LIMIT,self.steering-slew*dt,-lateral_limit)
        upper=min(CENTER_LIMIT,self.steering+slew*dt,lateral_limit)
        audit=[];best=None
        candidates=sorted(set([*np.linspace(lower,upper,21),
            max(lower,min(upper,desired)),max(lower,min(upper,self.steering))])) if lower<=upper else [self.steering]
        for command in candidates:
            reasons=[];first_unsafe=None
            if abs(command)>CENTER_LIMIT+1e-9 or abs(actual)>CENTER_LIMIT+1e-9:reasons.append('steering_limit')
            if abs(command-self.steering)>slew*dt+1e-9:reasons.append('steering_rate_limit')
            if speed*speed*abs(math.tan(command)/WHEELBASE)>.65+1e-9:reasons.append('lateral_acceleration')
            if not reasons:
                steps=np.arange(1,len(distances))
                step_time=ds/max(speed,.2)
                early=distances[1:]/max(speed,.2)<=latency
                early_count=int(np.count_nonzero(early))
                delayed_actual=actual+np.clip(self.steering-actual,-.65*step_time*early_count,.65*step_time*early_count)
                angles=np.where(early,
                    actual+np.clip(self.steering-actual,-.65*step_time*steps,.65*step_time*steps),
                    delayed_actual+np.clip(command-delayed_actual,-.65*step_time*np.maximum(0,steps-early_count),.65*step_time*np.maximum(0,steps-early_count)))
                k=-np.tan(angles)/WHEELBASE
                if np.any(speed*speed*abs(k)>.65+1e-9):reasons.append('lateral_acceleration')
            if not reasons:
                headings=np.r_[0.,np.cumsum(k*ds)]
                middle=headings[:-1]+k*ds/2
                xx=np.r_[0.,-WHEELBASE/2+np.cumsum(ds*np.cos(middle))+WHEELBASE/2*np.cos(headings[1:])]
                yy=np.r_[0.,np.cumsum(ds*np.sin(middle))+WHEELBASE/2*np.sin(headings[1:])]
                unsafe=np.zeros(len(distances),dtype=bool);uncertain=np.zeros(len(distances),dtype=bool)
                for offset in (-.10,.10):
                    x=xx+offset*np.cos(headings);y=yy+offset*np.sin(headings)
                    radius=np.hypot(x,y)
                    index=np.clip(np.rint((np.arctan2(y,x)-scan.angle_min)/scan.angle_increment).astype(int),0,len(ranges)-1)
                    uncertain|=(radius>.30)&(~clear[index])
                    unsafe|=(radius>.30)&(ranges[index]<radius+.23)
                    # Exact broad phase: points outside this expanded rectangle
                    # cannot intersect any of the original 0.23m circles.
                    nearby=points[(points[:,0]>=x.min()-.23)&(points[:,0]<=x.max()+.23)&
                        (points[:,1]>=y.min()-.23)&(points[:,1]<=y.max()+.23)]
                    if len(nearby):unsafe|=np.min((nearby[:,0,None]-x)**2+(nearby[:,1,None]-y)**2,axis=0)<.23**2
                invalid=unsafe|uncertain
                if np.any(invalid):
                    first=int(np.flatnonzero(invalid)[0]);first_unsafe=float(distances[first])
                    if first_unsafe<=required+1e-9:
                        if unsafe[first]:reasons.append('predicted_collision')
                        if uncertain[first]:reasons.append('sensor_uncertainty')
                        reasons.append('stopping_distance_insufficient')
            audit.append(dict(command=float(command),reasons=reasons,first_unsafe_distance=first_unsafe))
            if not reasons:
                free_distance=max(0.,(first_unsafe-ds) if first_unsafe is not None else horizon)
                admission=max(0.,math.sqrt((braking*latency)**2+2*braking*max(0.,free_distance-.65))-braking*latency)
                score=abs(command-desired)+.15*abs(command-self.steering)
                if best is None or score<best[0]:best=(score,float(command),admission)
        self.candidate_audit=dict(required_distance=required,preview_distance=horizon,
            actual_steering=actual,latency_s=latency,reachable_lower=lower,reachable_upper=upper,candidates=audit)
        return best

    completion_width: float = 0.

    def completion_center_path(self, points):
        """Distance slices, percentiles and width memory from the completed laps."""
        import numpy as np
        x=points[:,0]
        path = []
        side_estimates=[]
        widths=[]
        rejected_slices=0
        for distance in (.6,1.,1.5,2.,3.,4.,6.):
            local = points[abs(x-distance)<max(.22,.10*distance)]
            left = local[local[:,1]>.24,1]
            right = local[local[:,1]<-.24,1]
            if len(left)>=2 and len(right)>=2:
                l,h = float(np.percentile(right,85)),float(np.percentile(left,15))
                width = h-l
                if not .65<width<14:continue
                if distance<=1.5:widths.append(width)
                center = (h+l)/2
                unilateral=False
            elif self.completion_width and (len(left)>=2 or len(right)>=2):
                unilateral=True
                center = (float(np.percentile(left,15))-self.completion_width/2 if len(left)>=2
                          else float(np.percentile(right,85))+self.completion_width/2)
            else:continue
            # Reject disconnected far branches (e.g. opposite chicane wall).
            if len(path)>=2:
                px,py=path[-1];qx,qy=path[-2]
                slope=max(-1.5,min(1.5,(py-qy)/(px-qx)))
                predicted=py+slope*(distance-px)
                if abs(center-predicted)>.45+.45*(distance-px):
                    rejected_slices+=1;continue
            path.append((distance,center));side_estimates.append(unilateral)
        if widths:
            observed=float(np.median(widths))
            self.completion_width=observed if self.completion_width==0 else .9*self.completion_width+.1*observed
        return path, side_estimates

    def plan(self, scan, attitude, measured_speed, dt, cruise=.5, yaw_rate=None, steering_actual=None):
        import numpy as np
        requested_cruise=cruise
        wheel_input=measured_speed
        track_safety=self.race_center or self.path_mode=='left_wall'
        def stop(reason):
            if self.path_mode=='left_wall':
                self.reset_left_entry()
                self.wall_recovery_active=False
                self.wall_recovery_target=None
                self.wall_handoff_stamp=None
            return 0.,self.steering,dict(reason=reason,path=[],safety_trigger=self.safety_event or None,
                speed_limits_mps=dict(safety_stop=0.),active_speed_limits=[reason],speed_command_mps=0.)
        if (scan.header.frame_id != 'team07_lidar' or attitude is None
                or not math.isfinite(dt) or not 0<dt<=.15
                or not math.isfinite(cruise) or not math.isfinite(measured_speed)):
            return stop('missing_frame_or_imu')
        if (not all(math.isfinite(v) for v in attitude)
                or not math.isfinite(scan.angle_min)
                or not math.isfinite(scan.angle_increment)
                or scan.angle_increment<=0
                or not 6.<scan.angle_increment*(len(scan.ranges)-1)<6.4):
            return stop('invalid_sensor_geometry')
        if self.path_mode=='left_wall':
            self.width_age+=dt
            cruise=min(cruise,self.left_speed_limit)
        staged_wall_speed=(self.path_mode=='left_wall' and self.experimental_race_speed
            and .5<cruise<=RACE_STRAIGHT_TRIAL_MPS)
        legacy_high_cruise=cruise>2. and not staged_wall_speed
        authorized_ceiling=4. if self.experimental_race_speed else cruise_limit(True)
        highspeed=self.experimental_race_speed or cruise>3.
        if highspeed:
            if self.safety_fault:return stop(self.safety_fault)
            if cruise>authorized_ceiling:return stop('highspeed_ceiling_withdrawn')
            if yaw_rate is None or not math.isfinite(yaw_rate):return stop('missing_yaw_rate')
            if steering_actual is None or not math.isfinite(steering_actual):return stop('missing_steering_encoder')
            # Wheel locking must not immediately collapse pursuit distance or
            # justify stationary steering. This is a conservative envelope,
            # not a claim of measured body velocity.
            self.speed_envelope=max(abs(measured_speed),self.speed_envelope-.7*dt)
            lateral_indicator=abs(yaw_rate)*max(self.speed_envelope,.2)
            yaw_trigger=lateral_indicator>1.5
            reverse_trigger=measured_speed<-.1
            if yaw_trigger or reverse_trigger:
                self.safety_event=dict(conditions=([ 'yaw_envelope_product' ] if yaw_trigger else [])+
                    ([ 'negative_wheel_speed' ] if reverse_trigger else []),
                    yaw_rate=yaw_rate,wheel_speed=measured_speed,speed_envelope=self.speed_envelope,
                    yaw_envelope_product=lateral_indicator,yaw_limit=1.5,reverse_limit=-.1)
                self.safety_fault='highspeed_yaw_or_reverse_stop'
                self.speed=0.
                restrict_highspeed(self.safety_fault)
                return stop(self.safety_fault)
            measured_speed=self.speed_envelope
        slew=.65 if not highspeed else min(.65,.65/max(1.,abs(measured_speed)))
        roll,pitch,_ = attitude
        if abs(roll)>.25 or abs(pitch)>.18:
            return stop('attitude_limit')
        r = np.asarray(scan.ranges, dtype=float)
        a = scan.angle_min + np.arange(len(r))*scan.angle_increment
        valid = np.isfinite(r)&(r>=max(.08,scan.range_min))&(r<=scan.range_max)
        clear = valid | np.isposinf(r)
        front = abs(a)<math.pi/2
        if len(r)<180 or np.count_nonzero(clear&front)<.85*np.count_nonzero(front):
            return stop('incomplete_scan')
        # Planar projection only for mild pitch/roll. Larger slopes stop above.
        x = r[valid]*np.cos(a[valid])*math.cos(pitch)
        y = r[valid]*np.sin(a[valid])*math.cos(roll)
        points = np.column_stack((x,y))
        boundary_residual=0.
        untracked_object=False
        tracks=[]
        boundary_inferred=False
        obstacle_horizon=4.
        straight_observed_end=0.
        boundary_extent_audit={}
        if track_safety:
            # Range-max returns do not establish a physical wall.
            observed=r[valid]<scan.range_max*.98
            left_chain,right_chain,boundary_residual,fault=self.continuous_boundaries(
                points[observed],np.flatnonzero(valid)[observed],scan.angle_increment)
            if fault=='ambiguous_obstacle_in_corridor':untracked_object=True
            elif fault:return stop(fault)
            header_stamp=getattr(scan.header,'stamp',None)
            stamp=(header_stamp.sec+header_stamp.nanosec*1e-9 if header_stamp is not None
                else (self.obstacle_stamp or 0.)+dt)
            live_extension=None
            if staged_wall_speed and min(len(left_chain),len(right_chain))>=8:
                boundary_extent_audit=dict(left={},right={})
                live_indices=np.flatnonzero(valid)[observed]
                live_left=self.extend_observed_boundary(left_chain,points[observed],live_indices,boundary_extent_audit['left'])
                live_right=self.extend_observed_boundary(right_chain,points[observed],live_indices,boundary_extent_audit['right'])
                bilateral_end=min(float(live_left[:,0].max()),float(live_right[:,0].max()))
                live_end=min(LEGACY_WALL_TRACKING_M,bilateral_end)
                if live_end>4.:
                    live_extension=(live_left,live_right)
                    obstacle_horizon=live_end
            live_wall_counts=(len(left_chain),len(right_chain))
            left_chain,right_chain=self.occluded_boundaries(left_chain,right_chain,scan,stamp,measured_speed,attitude[2])
            # A current single wall plus recently measured width may support
            # a bounded center candidate; never refresh width from this inference.
            if ((len(left_chain)<8) != (len(right_chain)<8)) and self.width>0 and self.width_age<=self.wall_memory_seconds and time.monotonic()-self.width_seen_wall<=self.wall_memory_seconds:
                boundary_inferred=True
                if len(left_chain)<8:left_chain=right_chain+np.array([0.,self.width])
                else:right_chain=left_chain-np.array([0.,self.width])
            if (live_extension is not None and bilateral_end>LEGACY_WALL_TRACKING_M
                    and not (untracked_object or boundary_inferred or self.wall_memory_audit.get('reused_points'))
                    and abs(pitch)<=.04 and abs(roll)<=.08):
                terminal=math.floor(min(STRAIGHT_WALL_TRACKING_M,bilateral_end)*10.)/10.
                ordered=tuple(chain[np.argsort(chain[:,0])] for chain in live_extension)
                # A bend at the end must not invalidate an earlier straight
                # prefix. Retry earlier stations using identical evidence gates.
                for end in (terminal, *np.arange(math.floor(terminal*2)/2, 8., -.5)):
                    headings=[]
                    for chain in ordered:
                        xx,yy=chain[:np.searchsorted(chain[:,0],end)+1].T
                        line=np.polynomial.polynomial.polyfit(xx,yy,1)
                        residual=abs(yy-np.polynomial.polynomial.polyval(xx,line))/math.hypot(1.,line[1])
                        if np.any(residual>.04+.002*np.maximum(xx,0.)):break
                        headings.append(math.atan(line[1]))
                    if len(headings)==2 and abs(headings[0]-headings[1])<=.05:
                        straight_observed_end=float(end)
                        obstacle_horizon=max(obstacle_horizon,straight_observed_end)
                        break
            last_track_stamp=self.obstacle_stamp
            can_reseed=(min(live_wall_counts)>=8 and not (boundary_inferred or untracked_object
                or self.wall_memory_audit.get('reused_points')) and abs(wheel_input)<.02
                and yaw_rate is not None and math.isfinite(yaw_rate) and abs(yaw_rate)<.03)
            tracks,track_fault=self.track_obstacles(points[observed],np.flatnonzero(valid)[observed],
                left_chain,right_chain,stamp,measured_speed,attitude[2],live_extension,obstacle_horizon,can_reseed)
            tracking_audit=dict(live_wall_counts=live_wall_counts,finite_returns=int(observed.sum()),
                last_accepted_scan_stamp=last_track_stamp,
                reseeded=last_track_stamp is not None and stamp-last_track_stamp>.3 and not track_fault)
            if track_fault:
                speed,steer,info=stop(track_fault)
                return speed,steer,dict(info,obstacle_tracking=tracking_audit)
            # Tracking/cache retain their original near-boundary inputs. Far
            # geometry is rebuilt from this scan after all ambiguity checks.
            if live_extension is not None and not self.wall_memory_audit.get('reused_points'):
                left_chain,right_chain=live_extension
            else:
                left_chain=self.extend_observed_boundary(left_chain,points[observed],np.flatnonzero(valid)[observed])
                right_chain=self.extend_observed_boundary(right_chain,points[observed],np.flatnonzero(valid)[observed])

        path = []
        one_side = False
        side_estimates=[]
        widths=[]
        rejected_slices=0
        wall_slices=[]
        distances=(.6,1.,1.5,2.,3.,4.,5.,6.,7.,8.,9.,10.,11.,12.,13.,14.,15.,16.)
        completion_reference, completion_sides = self.completion_center_path(points)
        completion_by_distance = dict(zip((x for x, _ in completion_reference),
                                          zip((y for _, y in completion_reference), completion_sides)))
        use_live_brackets=False
        if staged_wall_speed:
            end=max(LEGACY_WALL_TRACKING_M,straight_observed_end)
            if (live_extension is not None and not tracks
                    and not (boundary_inferred or untracked_object or self.wall_memory_audit.get('reused_points'))):
                # Recover the measured fraction of the final interval instead
                # of discarding it when one wall falls just short of a station.
                end=max(4.,min(end,math.floor(obstacle_horizon*10.)/10.))
                use_live_brackets=True
                live_brackets=tuple(chain[np.argsort(chain[:,0])] for chain in live_extension)
            distances=tuple(distance for distance in distances if distance<end)+(end,)
        if self.race_center:
            distances=()
        elif self.path_mode=='center':
            distances=(.6,1.,1.5,2.,3.,4.,6.)
            if cruise>2.:distances+=(8.,10.,12.)
            if highspeed:distances+=(14.,16.)
        for distance in distances:
            local = points[abs(x-distance)<max(.22,.10*distance)]
            # A yawed vehicle can see BOTH walls on the same side of body y=0.
            # Partition about the corridor predicted from preceding slices;
            # never remove these returns from the swept collision checker.
            split=0.
            if legacy_high_cruise and len(path)>=2:
                px,py=path[-1];qx,qy=path[-2]
                split=py+max(-1.5,min(1.5,(py-qy)/(px-qx)))*(distance-px)
            left = local[local[:,1]>split+.24,1]
            right = local[local[:,1]<split-.24,1]
            left_level=right_level=None
            if self.path_mode=='left_wall':
                lc=left_chain[abs(left_chain[:,0]-distance)<max(.22,.10*distance)]
                rc=right_chain[abs(right_chain[:,0]-distance)<max(.22,.10*distance)]
                left,right=lc[:,1],rc[:,1]
                def level(chain):
                    if len(chain)<2 or np.ptp(chain[:,0])<.01:return None
                    if distance>4. and not (chain[:,0].min()<=distance<=chain[:,0].max()):return None
                    if distance>LEGACY_WALL_TRACKING_M and staged_wall_speed:
                        order=np.argsort(chain[:,0])
                        return float(np.interp(distance,chain[order,0],chain[order,1]))
                    fit=np.polynomial.polynomial.polyfit(chain[:,0],chain[:,1],1)
                    return float(np.polynomial.polynomial.polyval(distance,fit))
                left_level,right_level=level(lc),level(rc)
                if use_live_brackets and distance>4.:
                    levels=[]
                    for chain in live_brackets:
                        j=int(np.searchsorted(chain[:,0],distance))
                        if 0<j<len(chain) and np.linalg.norm(chain[j]-chain[j-1])<=.75:
                            bracket=chain[j-1:j+1]
                            levels.append((float(np.interp(distance,bracket[:,0],bracket[:,1])),bracket[:,1]))
                        else:levels.append((None,np.array([])))
                    (left_level,left),(right_level,right)=levels
                if left_level is None:left=np.array([])
                if right_level is None:right=np.array([])
            wall_info=dict(distance=distance,split=split,left_count=len(left),right_count=len(right))
            wall_slices.append(wall_info)
            if len(left)>=2 and len(right)>=2:
                l,h = ((right_level,left_level) if self.path_mode=='left_wall' else
                    (float(np.percentile(right,85)),float(np.percentile(left,15))))
                width = h-l
                wall_info.update(left=float(h),right=float(l),observed_width=width)
                if not .65<width<14:
                    wall_info['rejected']='width';continue
                if distance<=1.5 and self.path_mode=='center':widths.append(width)
                center = (h+l)/2
                unilateral=boundary_inferred if self.path_mode=='left_wall' else False
            elif (self.width and (self.path_mode=='center' or
                    (self.width_age<=self.wall_memory_seconds and
                     time.monotonic()-self.width_seen_wall<=self.wall_memory_seconds))
                    and (len(left)>=2 or len(right)>=2)):
                unilateral=True
                center = ((left_level if self.path_mode=='left_wall' else float(np.percentile(left,15)))-self.width/2 if len(left)>=2
                          else (right_level if self.path_mode=='left_wall' else float(np.percentile(right,85)))+self.width/2)
            else:
                wall_info['rejected']='no_walls';continue
            if self.path_mode=='left_wall':
                # Only width is remembered; wall positions always come from this scan.
                wall_info.update(left=left_level if len(left)>=2 else center+self.width/2,
                    right=right_level if len(right)>=2 else center-self.width/2)
            wall_info.update(center=center,one_side=unilateral)
            # Reject disconnected far branches (e.g. opposite chicane wall).
            if len(path)>=2:
                px,py=path[-1];qx,qy=path[-2]
                slope=max(-1.5,min(1.5,(py-qy)/(px-qx)))
                predicted=py+slope*(distance-px)
                if abs(center-predicted)>.45+.45*(distance-px):
                    rejected_slices+=1;wall_info['rejected']='disconnected';continue
            if self.path_mode=='left_wall' and not unilateral and distance<=1.5:widths.append(width)
            if self.path_mode=='left_wall' and distance>4.:
                if unilateral or not wall_slices[-2].get('accepted'):break
                wall_info['long_observed']=True
                wall_info['straight_extension']=staged_wall_speed and distance>LEGACY_WALL_TRACKING_M
            wall_info['accepted']=True
            path.append((distance,center));side_estimates.append(unilateral)
        if widths:
            observed=float(np.median(widths))
            self.width=observed if self.width==0 else .9*self.width+.1*observed
            self.width_age=0.
            self.width_seen_wall=time.monotonic()
        if self.race_center:
            path,side_estimates=completion_reference,completion_sides
            self.width=self.completion_width
        if not path:return stop('no_corridor')
        wall_speed_limit=cruise
        wall_caps=dict(wall_clearance=cruise,wall_support=cruise,wall_confidence=cruise,obstacle_prediction=cruise)
        wall_center_clearance=None
        legacy_wall_clearance_limit=cruise
        wall_motion_audit={}
        effective_wall_distance=None
        entry_diagnostic=dict(selected_reference='completion_center') if self.race_center else {}
        primary_entry_blocked=False
        proactive_recovery=False
        keep_wall_reserve=False
        if self.path_mode=='left_wall':
            # Same finite-return boundary extraction above, same PP/collision checker below.
            observed_walls=[w for w in wall_slices if w.get('accepted')]
            accepted=self.supported_wall_prefix(observed_walls)
            center_fallback=boundary_inferred or len(accepted)<2
            if center_fallback and self.width>0 and self.width_age<=self.wall_memory_seconds and time.monotonic()-self.width_seen_wall<=self.wall_memory_seconds:
                # Same midpoint reference as center mode, with stricter local
                # support, uncertainty, entry feasibility and collision checks.
                accepted=[dict(w,one_side=True) for w in observed_walls if w['distance']<=1.5]
            elif center_fallback:return stop('center_fallback_untrusted_width')
            if len(accepted)<2 or path[0][0]>1.:
                return stop('left_wall_low_confidence')
            px=np.asarray([w['distance'] for w in accepted])
            ly=np.asarray([w['left'] for w in accepted])
            ry=np.asarray([w['right'] for w in accepted])
            slopes=np.gradient(ly,px)
            right_slopes=np.gradient(ry,px)
            # Existing swept checker uses 0.23m circles. Honour that envelope too.
            stopping_extent=(race_stopping_distance(abs(measured_speed)) if self.experimental_race_speed else
                .21+abs(measured_speed)*.08+measured_speed**2/(2*.35))
            actual=self.steering if steering_actual is None else steering_actual
            angle_error=max(self.wall_steering_error,abs(actual-self.steering))
            curvature_error=max(abs(math.tan(actual+angle_error)-math.tan(actual)),
                abs(math.tan(actual-angle_error)-math.tan(actual)))/WHEELBASE
            steering_reserve=.5*stopping_extent**2*curvature_error
            uncertainty=self.wall_distance_error+boundary_residual+.02*(1.-self.wall_memory_audit.get('confidence',1.))+(.03 if any(w.get('one_side') for w in accepted) else 0.)
            safe_offset=max(.23,self.swept_half_width)+self.wall_margin+uncertainty+steering_reserve
            if self.experimental_race_speed:safe_offset+=.05*abs(measured_speed)
            entry_diagnostic.update(wall_uncertainty_m=uncertainty,steering_reserve_m=steering_reserve,
                wall_memory=dict(self.wall_memory_audit))
            # Keep the hard reserve calculation identical. Separately measure
            # the verified left wall so an unrelated return cannot trigger a shift.
            nearby=points[abs(x)<.32]
            if len(nearby):
                closest=nearby[int(np.argmin(np.linalg.norm(nearby,axis=1)))]
                wall_center_clearance=float(np.linalg.norm(closest))
                clearance_reserve=wall_center_clearance-safe_offset
                entry_diagnostic.update(wall_nearest_return_xy_m=closest.tolist(),
                    wall_required_offset_m=safe_offset,wall_remaining_reserve_m=clearance_reserve)
            verified_corridor=(not (center_fallback or tracks or untracked_object or rejected_slices
                or self.wall_memory_audit.get('reused_points'))
                and not any(w['one_side'] for w in accepted))
            if (verified_corridor and wall_center_clearance is not None and cruise<=.3 and steering_actual is not None
                    and yaw_rate is not None and math.isfinite(yaw_rate) and ly[0]-ry[0]>1.2):
                near_left=left_chain[abs(left_chain[:,0])<.32]
                if len(near_left):
                    left_reserve=float(np.min(np.linalg.norm(near_left,axis=1)))-safe_offset
                    ahead_reserve=float(ly[0]/math.hypot(1.,slopes[0]))-safe_offset
                    # Read closing from the measured wall ahead; don't wait
                    # for the nearest return to cross the stop threshold.
                    proactive_recovery=clearance_reserve>0 and (left_reserve<.06 or
                        (left_reserve<.32 and ahead_reserve<min(.12,left_reserve-.015)))
                    keep_wall_reserve=self.wall_recovery_active and min(left_reserve,ahead_reserve)<.20
                    entry_diagnostic.update(left_wall_reserve_m=left_reserve,
                        left_wall_ahead_reserve_m=ahead_reserve,proactive_wall_recovery=proactive_recovery)
            if self.wall_memory_audit.get('reused_points'):wall_caps['wall_confidence']=min(wall_caps['wall_confidence'],.1)
            wall_speed_limit=min(wall_speed_limit,wall_caps['wall_confidence'])
            effective_wall_distance=(self.width/2 if self.path_mode=='left_wall' else
                max(self.left_wall_distance,safe_offset+self.wall_target_buffer))
            if 'left_wall_reserve_m' in entry_diagnostic and center_fallback:
                effective_wall_distance=max(effective_wall_distance,safe_offset+.12)
            if staged_wall_speed and not center_fallback:
                supported_speed=min(cruise,wall_distance_speed(min(obstacle_horizon,accepted[-1]['distance'])))
                speed_offset=lateral_wall_offset(supported_speed,
                    max(.23,self.swept_half_width)+self.wall_margin+uncertainty,
                    math.tan(self.wall_steering_error)/WHEELBASE,.05)
                effective_wall_distance=max(effective_wall_distance,
                    min(self.width/2-.15,speed_offset+self.wall_target_buffer+.025))
            offsets=np.full(len(px),effective_wall_distance)
            adaptive_gap=(staged_wall_speed and not center_fallback and len(px)>=4
                and not self.wall_memory_audit.get('reused_points')
                and not any(w['one_side'] for w in accepted))
            if adaptive_gap:
                center_slope=.5*(slopes+right_slopes)
                normal_width=(ly-ry)/np.hypot(1.,center_slope)
                bend=np.gradient(center_slope,px)/(1.+center_slope**2)**1.5
                weights=np.exp(-((px[:,None]-px)/1.25)**2)
                bend=(weights@bend)/weights.sum(axis=1)
                reserve=max(safe_offset+self.wall_target_buffer,
                    speed_offset+self.wall_target_buffer+.025)
                # Keep a small left preference on straights. As the desired
                # reserve fills the corridor, share the space on both sides.
                bias=.15-.11*np.clip((supported_speed-1.)/.5,0.,1.)
                balanced=.5*normal_width-bias
                # Move away from the inside wall on tighter bends.
                turn_shift=np.clip(2.*bend,-.10,.10)
                offsets=np.maximum(effective_wall_distance,
                    np.minimum(reserve,balanced)+turn_shift)
                offsets=np.minimum(offsets,.5*normal_width+np.maximum(0.,turn_shift))
                # Rate-limit the near target across fresh control updates and
                # taper the offset along the observed prefix before fitting.
                if self.wall_target_adjustment is not None:
                    limited=float(np.clip(offsets[0],self.wall_target_adjustment-.12*dt,
                        self.wall_target_adjustment+.12*dt))
                    offsets+=limited-offsets[0]
                self.wall_target_adjustment=float(offsets[0])
                for i in range(1,len(offsets)):
                    step=.10*(px[i]-px[i-1])
                    offsets[i]=np.clip(offsets[i],offsets[i-1]-step,offsets[i-1]+step)
                effective_wall_distance=float(offsets[0])
            else:
                self.wall_target_adjustment=None
            entry_diagnostic['adaptive_wall_gap']=adaptive_gap
            entry_diagnostic['selected_reference']='completion_center'
            entry_diagnostic['completion_reference']=completion_reference
            if center_fallback:wall_caps['wall_confidence']=min(wall_caps['wall_confidence'],.1)
            wall_speed_limit=min(wall_speed_limit,wall_caps['wall_confidence'])
            shifted=[]
            for w,slope,right_slope,offset in zip(accepted,slopes,right_slopes,offsets):
                normal=math.hypot(1.,float(slope))
                lower=w['right']+safe_offset*math.hypot(1.,float(right_slope))
                upper=w['left']-safe_offset*normal
                if lower>upper:break  # never bridge an unconfirmed narrow section
                candidate=completion_by_distance.get(w['distance'])
                if candidate is None:break
                target_y,estimated=candidate
                if estimated:wall_caps['wall_confidence']=min(wall_caps['wall_confidence'],.1)
                shifted.append((w['distance'],max(lower,min(upper,target_y))))
            path=shifted
            side_estimates=side_estimates[:len(path)]
            if len(path)<2 or path[-1][0]<1.:
                return stop('left_wall_no_safe_corridor')
            wall_speed_limit=min(wall_speed_limit,wall_caps['wall_confidence'])
            reference=path
            import copy
            initial_entry=copy.deepcopy(self.entry_state)
            entry_attempts=[]
            while True:
                self.entry_state=copy.deepcopy(initial_entry)
                path,entry_info=self.left_wall_entry(reference,accepted[:len(reference)],safe_offset,
                    measured_speed,dt,steering_actual,effective_wall_distance,adaptive_gap)
                entry_attempts.append(dict(end_m=float(reference[-1][0]),approved=path is not None,
                    rejection=entry_info.get('entry_rejection'),candidates=entry_info.get('entry_candidates_rejected',[])))
                if path is not None or len(reference)<=2:break
                # Same collision/geometry checks on a shorter measured prefix.
                # Never approve a prefix shorter than current stopping need.
                if reference[-2][0]<max(1.,wall_stopping_distance(measured_speed)):break
                reference=reference[:-1]
            entry_diagnostic['entry_attempts']=entry_attempts
            entry_diagnostic['prefix_shortened']=len(entry_attempts)>1
            entry_diagnostic['wall_required_stop_m']=wall_stopping_distance(measured_speed)
            entry_diagnostic['long_observed_end_m']=float(accepted[-1]['distance'])
            entry_diagnostic['straight_observed_end_m']=straight_observed_end
            entry_diagnostic['long_path_end_m']=float(path[-1][0]) if path else 0.
            entry_diagnostic['long_sampling_gap_limit_m']=.75
            entry_diagnostic['long_extra_error_allowance_m']=.002*max(0.,entry_diagnostic['long_path_end_m']-4.)
            entry_diagnostic.update(entry_info)
            if path is None:
                failures=[c for attempt in entry_attempts for c in attempt['candidates']]
                # These limits apply to the rejected primary fit. Recovery
                # checks its own PP steering, live corridor and swept arcs.
                primary_geometry={'steering_reach','slope','curvature','pursuit_steering','wall_corridor'}
                primary_entry_blocked=((self.wall_recovery_active or proactive_recovery) and bool(failures)
                    and all(set(c['reasons'])<=primary_geometry for c in failures))
                if not primary_entry_blocked:
                    self.reset_left_entry()
                    return 0.,self.steering,dict(reason='left_wall_no_safe_entry',path=[],speed_limits_mps=dict(safety_stop=0.),active_speed_limits=['entry_safety_stop'],**entry_diagnostic)
                # During a verified wall recovery the primary fit can reject
                # the recovery steering. The raw reference is only a preview
                # input; it cannot authorize motion through the failed fit.
                path=reference
                entry_diagnostic.update(entry_heading_error_rad=math.atan(float(slopes[0])),
                    entry_wall_distance_m=float(ly[0]/math.hypot(1.,float(slopes[0]))))
            if entry_diagnostic['entry_fit_residual_m']>.04:
                wall_caps['entry_fit']=.15
                wall_caps['wall_confidence']=min(wall_caps['wall_confidence'],.15)
                wall_speed_limit=min(wall_speed_limit,wall_caps['wall_confidence'])
            wall_caps['wall_support']=wall_distance_speed(path[-1][0])
            wall_speed_limit=min(wall_speed_limit,wall_caps['wall_support'])
            if wall_center_clearance is not None:
                if clearance_reserve<=0:
                    speed,steer,info=stop('left_wall_clearance_stop')
                    return speed,steer,dict(info,wall_center_clearance_m=wall_center_clearance,**entry_diagnostic)
                legacy_wall_clearance_limit=math.sqrt(2*.35*clearance_reserve)
                base_offset=max(.23,self.swept_half_width)+self.wall_margin+uncertainty
                lateral_cap=lateral_wall_speed(wall_center_clearance,base_offset,curvature_error,
                    .05 if self.experimental_race_speed else 0.,cruise if staged_wall_speed else .5)
                wall_caps['wall_clearance']=max(legacy_wall_clearance_limit,lateral_cap)
                wall_motion_audit.update(legacy_wall_cap=legacy_wall_clearance_limit,
                    lateral_model_cap=lateral_cap,relief_ceiling_mps=cruise if staged_wall_speed else .5,
                    base_offset_m=base_offset,curvature_error_per_m=curvature_error)
                wall_speed_limit=min(wall_speed_limit,wall_caps['wall_clearance'])
            if any(side_estimates) or rejected_slices or len(path)<4:
                wall_caps['wall_confidence']=min(wall_caps['wall_confidence'],.15)
                wall_speed_limit=min(wall_speed_limit,wall_caps['wall_confidence'])
        preview = max(abs(2*py/(px*px+py*py)) for px,py in path)
        corner=None
        if self.race_center and not any(side_estimates):
            corner=observed_corner_preview(path,measured_speed,cruise,boundary_residual+.01)
        if self.path_mode=='left_wall' and not center_fallback and not self.wall_memory_audit.get('reused_points'):
            # Prefer the bilateral corridor: the approach curve can reflect a
            # lateral correction on a straight, rather than a road turn.
            road=[(w['distance'],.5*(w['left']+w['right']))
                for w in accepted[:len(path)] if not w['one_side']]
            corner=observed_corner_preview(road,measured_speed,cruise,boundary_residual+.01)
        lookahead=max(.75,min(1.6,(.9+.55*abs(measured_speed))/(1+.45*preview)))
        # Opt-in higher-speed practice: extend pursuit only above the legacy range.
        if cruise>1. and abs(measured_speed)>1.:
            lookahead=max(lookahead,min(3.,(.9+.8*abs(measured_speed))/(1+.45*preview)))
        if highspeed:
            lookahead=max(lookahead,min(4.5,1.2+1.0*abs(measured_speed)))
        if corner and corner['entry_m']<=lookahead:
            lookahead=min(lookahead,max(.75,math.sqrt(.15/corner['peak_curvature_per_m'])))
        tx=min(max(lookahead,path[0][0]),path[-1][0])
        ty=float(np.interp(tx,[p[0] for p in path],[p[1] for p in path]))
        raw_desired=max(-CENTER_LIMIT,min(CENTER_LIMIT,-math.atan(WHEELBASE*2*ty/(tx*tx+ty*ty))))
        self.filtered_desired+=min(1.,dt/.12)*(raw_desired-self.filtered_desired)
        desired=self.filtered_desired
        near_sides=[side for point,side in zip(path,side_estimates) if point[0]<=max(1.5,lookahead)]
        one_side=not near_sides or any(near_sides)
        best = None
        braking = .7 if legacy_high_cruise else .35
        horizon = max(1., measured_speed*measured_speed/(2*braking)+.65)
        # 0.42m body, swept width <= .214m, plus 0.10m lateral margin.
        footprint_radius = .23
        if self.path_mode=='left_wall':
            footprint_radius+=uncertainty+steering_reserve+.025  # sampling/error allowance
        elif self.race_center:
            footprint_radius+=self.wall_distance_error+.025
        admission_limit=cruise
        if highspeed and self.path_mode!='left_wall':
            selected=self.feasible_steering(scan,points,r,clear,abs(measured_speed),dt,slew,
                desired,steering_actual,cruise,braking)
            best=None if selected is None else selected[:2]
            if selected is not None:admission_limit=selected[2]
            future_best=best
        else:
            def trajectory_safe(steer, check_horizon=None, check_radius=None, curvature_uncertainty=0., audit=None):
                evaluation_horizon=horizon if check_horizon is None else check_horizon
                base_radius=footprint_radius if check_radius is None else check_radius
                curvature = -math.tan(steer)/WHEELBASE
                distances = np.linspace(0,evaluation_horizon,max(24,math.ceil(evaluation_horizon/.12)) if legacy_high_cruise or check_horizon is not None else 24)
                # Integrate the same curvature-error bound over distance.
                # The extra linear term bounds rotation of the body circles.
                radius=np.maximum(footprint_radius,base_radius+curvature_uncertainty*distances*(.5*distances+.10))
                if abs(curvature)<1e-6:
                    xx,yy = distances,np.zeros_like(distances)
                else:
                    xx,yy = np.sin(distances*curvature)/curvature,(1-np.cos(distances*curvature))/curvature
                # Check both front and rear extent using circles along body centre.
                safe = True
                for offset in (-.10,.10):
                    xx2,yy2 = xx+offset*np.cos(distances*curvature),yy+offset*np.sin(distances*curvature)
                    ranges = np.hypot(xx2,yy2)
                    angles = np.arctan2(yy2,xx2)
                    idx = np.rint((angles-scan.angle_min)/scan.angle_increment).astype(int)
                    idx = np.clip(idx,0,len(r)-1)
                    if np.any((ranges>.30)&(~clear[idx])):
                        if audit is not None:audit['reason']='unknown_ray'
                        safe = False;break
                    hits=np.any((points[:,0,None]-xx2)**2+(points[:,1,None]-yy2)**2<radius**2,axis=1)
                    if np.any(hits):
                        if audit is not None:
                            blocking=points[hits]
                            sides={}
                            for side,chain in (('left',left_chain),('right',right_chain)):
                                sides[side]=np.any(np.all(blocking[:,None,:]==chain[None,:,:],axis=2),axis=1)
                            audit.update(reason='wall_return' if np.all(sides['left']|sides['right']) else 'unclassified_return',
                                wall_sides=[side for side,mask in sides.items() if np.any(mask)],
                                first_return_xy_m=blocking[0].tolist())
                        safe = False;break
                    if np.any((ranges>.30)&(r[idx]<ranges+radius)):
                        if audit is not None:audit['reason']='beam_range'
                        safe = False;break
                if safe and track_safety:
                    # Spatial union over a short prediction window is conservative:
                    # no assumption that ego and object arrive at the same time.
                    for track in tracks:
                        for seconds in np.linspace(0,1.,11):
                            predicted=np.asarray(track['position'])+seconds*np.asarray(track['velocity'])
                            growth=.04+.10*seconds+(.08 if not track['confident'] else 0.)
                            for offset in (-.10,.10):
                                xx2=xx+offset*np.cos(distances*curvature)
                                yy2=yy+offset*np.sin(distances*curvature)
                                if np.any((predicted[0]-xx2)**2+(predicted[1]-yy2)**2<(radius+track['radius']+growth)**2):
                                    if audit is not None:audit['reason']='tracked_obstacle'
                                    return False
                return safe
            candidates = list(np.linspace(-CENTER_LIMIT,CENTER_LIMIT,41))
            candidates += [self.steering, self.steering+max(-slew*dt,min(slew*dt,desired-self.steering))]
            # Prefer rate-reachable paths; a stopped vehicle may align its steering
            # towards a future safe path without moving into a blocked trajectory.
            future_best=None
            if self.race_center:
                nominal=candidates[-1]
                if steering_actual is None or not math.isfinite(steering_actual):
                    return stop('missing_steering_encoder')
                if not trajectory_safe(steering_actual):
                    return stop('actual_trajectory_blocked')
                candidates=[nominal]
            if self.path_mode=='left_wall':
                nominal=self.steering+max(-slew*dt,min(slew*dt,desired-self.steering))
                actual=self.steering if steering_actual is None else steering_actual
                if not math.isfinite(actual):return stop('missing_steering_encoder')
                actual_audit={};nominal_audit={}
                actual_safe=trajectory_safe(actual,audit=actual_audit)
                nominal_safe=trajectory_safe(nominal,audit=nominal_audit)
                current_safe=nominal_safe and actual_safe
                # Positive steering bends right. Evaluate only after the
                # preferred reachable pursuit trajectory is blocked.
                right_safe=[] if current_safe else [float(v) for v in candidates
                    if v>max(0.,nominal)+1e-6 and trajectory_safe(v)]
                self.candidate_audit=dict(current_path_safe=current_safe,
                    actual_sweep=actual_audit,nominal_sweep=nominal_audit,
                    right_candidates_checked=0 if current_safe else int(sum(v>max(0.,nominal)+1e-6 for v in candidates)),
                    right_safe_candidates=right_safe,object_motion='estimated' if tracks and all(t['confident'] for t in tracks) else 'uncertain',
                    obstacle_tracks=tracks,
                    bypass_authorized=False)
                wall_only_block=not current_safe and all(
                    verdict or audit.get('reason')=='wall_return'
                    for verdict,audit in ((actual_safe,actual_audit),(nominal_safe,nominal_audit)))
                handoff_ready=False
                if self.wall_recovery_active:
                    previous=self.wall_recovery_target
                    continuous=(previous is not None and steering_actual is not None and
                        max(abs(raw_desired-previous[2]),abs(raw_desired-actual))<=self.wall_steering_error)
                    handoff_safe=(not (primary_entry_blocked or keep_wall_reserve or proactive_recovery)
                        and verified_corridor and not center_fallback and not rejected_slices
                        and current_safe and continuous
                        and trajectory_safe(nominal,check_radius=footprint_radius+.025)
                        and trajectory_safe(raw_desired,check_radius=footprint_radius+.025))
                    if not handoff_safe:self.wall_handoff_stamp=None
                    elif self.wall_handoff_stamp is None or stamp<self.wall_handoff_stamp:
                        self.wall_handoff_stamp=stamp
                    handoff_ready=handoff_safe and stamp-self.wall_handoff_stamp>=.20
                    self.candidate_audit.update(primary_handoff_safe=handoff_safe,
                        primary_handoff_ready=handoff_ready)
                hold_recovery=self.wall_recovery_active and not handoff_ready
                request_recovery=proactive_recovery or keep_wall_reserve or hold_recovery
                recover=(wall_only_block or (current_safe and request_recovery)) and (
                    steering_actual is not None and yaw_rate is not None and math.isfinite(yaw_rate)
                    and not (tracks or untracked_object or center_fallback or boundary_inferred
                        or self.wall_memory_audit.get('reused_points') or rejected_slices)
                    and not any(w['one_side'] for w in accepted)
                    and abs(measured_speed)<=.3 and accepted[-1]['distance']>=horizon)
                recovered=False
                if recover:
                    # Keep the shorter lookahead for this recovery if forward
                    # support drops. A longer scan alone should not change it.
                    previous=self.wall_recovery_target
                    target_x=min(previous[0] if previous else max(tx,2.),float(px[-1]))
                    bounds=[float(np.interp(target_x,px,side)) for side in (ry,ly)]
                    fractions=(.5,.65,.35,.8,.2)
                    if previous:fractions=tuple(dict.fromkeys((previous[1],*fractions)))
                    targets=[]
                    for fraction in fractions:
                        target_y=bounds[0]+fraction*(bounds[1]-bounds[0])
                        if previous and fraction==previous[1]:
                            raw=-math.atan(WHEELBASE*2*target_y/(target_x**2+target_y**2))
                            angle=previous[2]+max(-slew*dt,min(slew*dt,raw-previous[2]))
                            curvature=-math.tan(angle)/WHEELBASE
                            reach=curvature*target_x
                            if abs(reach)<=1.:
                                limited_y=target_x*reach/(1.+math.sqrt(1.-reach**2))
                                targets.append((fraction,limited_y))
                        targets.append((fraction,target_y))
                    for fraction,target_y in targets:
                        if min(target_y-bounds[0],bounds[1]-target_y)<safe_offset:continue
                        curvature=2*target_y/(target_x**2+target_y**2)
                        angle=-math.atan(WHEELBASE*curvature)
                        if abs(angle)>CENTER_LIMIT or abs(curvature)*horizon>=math.pi/2:continue
                        arc_s=np.linspace(0.,horizon,32)
                        if abs(curvature)<1e-6:
                            arc_x,arc_y=arc_s,np.zeros_like(arc_s)
                        else:
                            arc_x=np.sin(curvature*arc_s)/curvature
                            arc_y=(1.-np.cos(curvature*arc_s))/curvature
                        supported=arc_x>=px[0]
                        if not np.any(supported):continue
                        xx,yy=arc_x[supported],arc_y[supported]
                        if (np.any(yy<np.interp(xx,px,ry)+safe_offset)
                                or np.any(yy>np.interp(xx,px,ly)-safe_offset)
                                or not trajectory_safe(angle)):continue
                        reachable=self.steering+max(-slew*dt,min(slew*dt,angle-self.steering))
                        reachable_safe=actual_safe and trajectory_safe(reachable)
                        ready=reachable_safe and abs(actual-angle)<=self.wall_steering_error
                        creep=False
                        if not ready and reachable_safe:
                            # Cover intermediate curvatures and encoder error
                            # over the entire existing stopping sweep.
                            actual_curvature=-math.tan(actual)/WHEELBASE
                            uncertainty_k=max(abs(-math.tan(a+e)/WHEELBASE-actual_curvature)
                                for a in (actual,reachable) for e in (-self.wall_steering_error,self.wall_steering_error))
                            creep=trajectory_safe(actual,horizon,footprint_radius,uncertainty_k)
                        self.wall_recovery_active=True
                        self.wall_recovery_target=(target_x,fraction,angle)
                        self.candidate_audit.update(recovery_reference='center_fallback' if fraction==.5 else 'wall_clearance_candidate',
                            recovery_target=(target_x,target_y),recovery_steering=angle,recovery_reachable=ready,
                            primary_entry_blocked=primary_entry_blocked,proactive_wall_recovery=proactive_recovery,
                            recovery_alignment_creep=creep)
                        if not (ready or creep):
                            self.speed=0.
                            if abs(wheel_input)<.02 and abs(yaw_rate)<.03:
                                self.steering=reachable
                            return 0.,self.steering,dict(reason='aligning_wall_recovery',path=path,
                                candidate_audit=self.candidate_audit,**entry_diagnostic)
                        path=list(zip(arc_x[1:].tolist(),arc_y[1:].tolist()))
                        tx,ty=target_x,target_y
                        raw_desired=desired=angle
                        self.filtered_desired=angle
                        nominal=reachable
                        current_safe=True
                        recovered=True
                        center_fallback=True
                        wall_caps['wall_recovery']=min(.06 if creep else .1,wall_distance_speed(float(arc_x[-1])))
                        wall_speed_limit=min(wall_speed_limit,wall_caps['wall_recovery'])
                        entry_diagnostic.update(selected_reference=self.candidate_audit['recovery_reference'],
                            entry_mode='wall_recovery',entry_progress_held=False,
                            path_left_wall_margin_m=float(np.min(np.interp(xx,px,ly)-yy-safe_offset)),
                            path_right_wall_margin_m=float(np.min(yy-np.interp(xx,px,ry)-safe_offset)))
                        break
                if not current_safe or (request_recovery and not recovered):
                    self.speed=0.
                    self.wall_recovery_target=None
                    return 0.,self.steering,dict(reason='blocked_path_motion_unknown' if right_safe else 'no_safe_trajectory',
                        path=[] if primary_entry_blocked else path,candidate_audit=self.candidate_audit)
                self.wall_recovery_active=recovered
                if not recovered:
                    self.wall_recovery_target=None
                    self.wall_handoff_stamp=None
                # Keep the current path; do not steer towards an off-path object.
                candidates=[nominal]
                if untracked_object or any(not t['confident'] for t in tracks):
                    wall_caps['wall_confidence']=min(wall_caps['wall_confidence'],.1)
                    wall_speed_limit=min(wall_speed_limit,wall_caps['wall_confidence'])
                for track in tracks:
                    ox,oy=track['position']
                    # Follow only objects overlapping the live reference corridor.
                    lateral=float(np.interp(max(0.,ox),[0.]+[p[0] for p in path],[0.]+[p[1] for p in path]))
                    if ox>0 and abs(oy-lateral)<footprint_radius+track['radius']+.15:
                        gap=ox-track['radius']-.35
                        closing=max(0.,-track['relative_velocity'][0])
                        reserve=.20+.3*closing+closing**2/(2*.35)
                        wall_caps['obstacle_prediction']=min(wall_caps['obstacle_prediction'],math.sqrt(max(0.,2*.35*(gap-reserve))))
                        wall_speed_limit=min(wall_speed_limit,wall_caps['obstacle_prediction'])
            for steer in sorted(set(candidates),key=lambda value:abs(value-self.steering)):
                if best is not None and abs(steer-self.steering)>slew*dt+1e-6:continue
                safe = trajectory_safe(steer)
                if safe:
                    score = abs(steer-desired)+.15*abs(steer-self.steering)
                    if future_best is None or score<future_best[0]:future_best=(score,float(steer))
                    if abs(steer-self.steering)<=slew*dt+1e-6:
                        if best is None or score<best[0]:best = (score,float(steer))
        if best is None:
            self.speed = 0.
            # A zero command does not mean the car has physically stopped.
            # Brake before attempting the stationary steering alignment.
            if legacy_high_cruise and abs(measured_speed)>.2:
                return 0.,self.steering,dict(reason='braking_before_alignment',path=path,lookahead=lookahead,candidate_audit=self.candidate_audit)
            if future_best is not None:
                self.steering+=max(-slew*dt,min(slew*dt,future_best[1]-self.steering))
                return 0.,self.steering,dict(reason='aligning_safe_steering',path=path,lookahead=lookahead)
            return stop('no_safe_trajectory')
        self.steering = best[1]
        acceleration = .5 if legacy_high_cruise else (.3 if cruise>1. else .15)
        if self.path_mode=='left_wall':
            heading=entry_diagnostic['entry_heading_error_rad']
            path_segment=min(len(path)-1,max(1,int(np.searchsorted([p[0] for p in path],tx))))
            x0,y0=path[path_segment-1]
            x1,y1=path[path_segment]
            distance_error=entry_diagnostic['entry_wall_distance_m']-effective_wall_distance
            pursuit_heading=math.atan2(y1-y0,x1-x0)
            entry_diagnostic.update(
                entry_wall_offset_error_m=distance_error,
                pursuit_path_heading_rad=pursuit_heading,
                pursuit_target=(tx,ty))
            if entry_diagnostic['entry_progress_held']:
                # A held phase may still yield a safe, reachable PP approach.
                # Retain the cap unless live geometry and both steering arcs
                # allow stopping before consuming the remaining wall offset.
                approach_safe=False
                following=entry_diagnostic['entry_mode']=='wall_follow'
                closing=heading<0. and (ty>0. or staged_wall_speed)
                recovering=(staged_wall_speed and abs(distance_error)<=.12
                    and abs(heading)<=.14 and (ty*distance_error>=0.
                        or (adaptive_gap and abs(pursuit_heading-heading)<=.10
                            and abs(pursuit_heading)<=.10)))
                if ((cruise<=.5 or staged_wall_speed) and steering_actual is not None
                        and (following or closing or recovering)
                        and entry_diagnostic['entry_trusted'] and not center_fallback
                        and not self.wall_memory_audit.get('reused_points')
                        and entry_diagnostic['entry_fit_residual_m']<=.04
                        and max(abs(actual-raw_desired),abs(self.steering-raw_desired))<=self.wall_steering_error):
                    approach_cap=min(wall_speed_limit,1. if staged_wall_speed else .5)
                    probe_target=wall_speed_limit if following else approach_cap
                    if staged_wall_speed:
                        # Check the next rate-limited step, so a failed check
                        # at the final cruise target cannot trap the ramp.
                        probe_target=min(probe_target,self.speed+acceleration*dt)
                    probe=max(abs(measured_speed),probe_target)
                    stop_extent=wall_stopping_distance(probe)
                    closing_curvature=max(0.,-math.tan(actual)/WHEELBASE,-math.tan(self.steering)/WHEELBASE)
                    closure=max(0.,-math.sin(heading))*stop_extent+.5*closing_curvature*stop_extent**2
                    reserve=.5*stop_extent**2*curvature_error
                    radius=max(footprint_radius,.23+uncertainty+reserve+.025)
                    remaining=(entry_diagnostic['entry_wall_distance_m']-safe_offset
                        if following or recovering else distance_error)
                    remaining-=closure+max(0.,reserve-steering_reserve)
                    if self.experimental_race_speed:
                        remaining-=.05*max(0.,probe-abs(measured_speed))
                    approach_safe=(path[-1][0]>=stop_extent and remaining>self.wall_target_buffer
                        and trajectory_safe(actual,stop_extent,radius)
                        and trajectory_safe(self.steering,stop_extent,radius)
                        and (not staged_wall_speed or trajectory_safe(raw_desired,stop_extent,radius)))
                    entry_diagnostic['entry_stopping_offset_remaining_m']=remaining
                    if approach_safe and not following:
                        wall_caps['entry_approach_speed']=approach_cap
                        wall_speed_limit=min(wall_speed_limit,approach_cap)
                entry_diagnostic['entry_alignment_relief']=bool(approach_safe)
                if not approach_safe:
                    wall_caps['entry_alignment']=.15
                    wall_caps['wall_confidence']=min(wall_caps['wall_confidence'],.15)
                    wall_speed_limit=min(wall_speed_limit,wall_caps['wall_confidence'])
        curvature = max(preview,abs(math.tan(self.steering)/WHEELBASE),.01)
        fit_residual=None
        road_curvature_cap=math.sqrt(.65/curvature)
        target = min(max(.1,min(authorized_ceiling,cruise)),road_curvature_cap)
        if legacy_high_cruise and len(path)>=4:
            legacy_target=target
            # Separate heading/cross-track error from road bending. Preserve the
            # original pursuit steering and its actual lateral-acceleration cap.
            px,py=np.asarray(path).T
            fit=np.polyfit(px,py,2)
            residual=float(np.max(abs(np.polyval(fit,px)-py)))
            fit_residual=residual
            road_k=abs(2*fit[0])/(1+np.polyval(np.polyder(fit),px)**2)**1.5
            corner_limits=np.sqrt(.65/np.maximum(road_k,.001)+2*braking*np.maximum(0,px-lookahead-(1.0 if highspeed else 0.)))
            road_curvature_cap=float(corner_limits.min())
            steering_k=abs(math.tan(self.steering)/WHEELBASE)
            target=min(cruise,authorized_ceiling,float(corner_limits.min()),math.sqrt(.65/max(steering_k,.001)))
            # Far corner branches can invalidate a single quadratic; retain
            # the proven conservative pursuit limit rather than force 0.5m/s.
            if highspeed:
                uncertainty=min(1.,max(0.,(residual-.1)/.1))
                target=min(target,cruise-(cruise-1.5)*uncertainty)
            if residual>.2:target=min(target,legacy_target,1.5 if highspeed else cruise)
            target=min(target,math.sqrt(max(0.,2*braking*(path[-1][0]-.65))))
        if highspeed:target=min(target,admission_limit)
        if one_side or len(path)<4:target = min(target,.3)
        if abs(pitch)>.04:target = min(target,.8*cruise,.55)
        if abs(roll)>.08:target=min(target,.3)
        ahead = points[(x>0)&(abs(y)<.25),0]
        clearance = float(ahead.min())-.35 if len(ahead) else scan.range_max
        if self.path_mode=='left_wall':
            clearance=horizon+.15  # entire stopping horizon passed the swept check
        target = min(target,math.sqrt(max(0.,2*braking*(clearance-.15))))
        hard_limit=min(cruise,math.sqrt(.65/max(abs(math.tan(self.steering)/WHEELBASE),.001)),
            math.sqrt(max(0.,2*braking*(clearance-.15))))
        if highspeed:hard_limit=min(hard_limit,admission_limit)
        if legacy_high_cruise:hard_limit=min(hard_limit,math.sqrt(max(0.,2*braking*(path[-1][0]-.65))))
        if one_side or len(path)<4:hard_limit=min(hard_limit,.3)
        if abs(pitch)>.04:hard_limit=min(hard_limit,.8*cruise,.55)
        if abs(roll)>.08:hard_limit=min(hard_limit,.3)
        if fit_residual is not None and fit_residual>.2 and highspeed:hard_limit=min(hard_limit,1.5)
        if highspeed and not one_side and len(path)>=4 and clearance>measured_speed**2/(2*braking)+1.:
            # Routine preview braking follows the same conservative deceleration
            # used in planning; imminent obstacles/sensor faults still stop now.
            target=min(hard_limit,max(target,self.speed-braking*dt))
        race_audit={}
        if self.experimental_race_speed:
            # Live observed support only; never infer clear distance from a
            # successful stopping-horizon check. All earlier collision checks stay.
            available=float(path[-1][0])
            if self.path_mode!='left_wall':available=min(available,float(clearance))
            prediction_braking=.35 if staged_wall_speed else RACE_BRAKING
            distance_cap=race_distance_speed(available,braking=prediction_braking)
            obstacle_cap=cruise
            for track in tracks:
                ox,oy=track['position']
                vx,vy=track['velocity']
                times=(0.,)
                if staged_wall_speed:
                    reaction_speed=max(abs(measured_speed),min(cruise,self.speed+acceleration*dt))
                    times=np.linspace(0.,RACE_LATENCY+reaction_speed/.35,13)
                for seconds in times:
                    px,py=ox+seconds*vx,oy+seconds*vy
                    if px<=0.:continue
                    lateral=float(np.interp(px,[0.]+[p[0] for p in path],[0.]+[p[1] for p in path]))
                    if abs(py-lateral)>=footprint_radius+track['radius']+.15+.10*seconds:continue
                    gap=max(0.,min(ox,px)-track['radius']-.35)
                    approaching=max(0.,-track['velocity'][0])
                    obstacle_cap=min(obstacle_cap,race_distance_speed(gap,approaching,prediction_braking))
            yaw_cap=.65/max(abs(yaw_rate),.001)
            confidence_cap=.3 if one_side or rejected_slices else cruise
            # Segment curvature preview also covers corners a quadratic fit
            # could smooth away. Brake before the observed bend, with latency.
            corner_cap=race_preview_speed(path,max(abs(measured_speed),self.speed),cruise,prediction_braking)
            target=min(target,distance_cap,obstacle_cap,yaw_cap,confidence_cap,corner_cap)
            race_audit=dict(race_available_distance_m=available,
                race_required_stop_m=(wall_stopping_distance(measured_speed) if staged_wall_speed
                    else race_stopping_distance(abs(measured_speed))),
                race_distance_cap=distance_cap,race_obstacle_cap=obstacle_cap,
                race_yaw_cap=yaw_cap,race_corner_cap=corner_cap,
                race_latency_s=RACE_LATENCY,race_braking_assumption=prediction_braking,
                experimental_race_speed=True)
        limits=dict(user_target=requested_cruise,left_wall=self.left_speed_limit if self.path_mode=='left_wall' else authorized_ceiling,
            legacy_plan=target,path_curvature=road_curvature_cap,
            steering_lateral=math.sqrt(.65/max(abs(math.tan(self.steering)/WHEELBASE),.001)),
            trajectory_admission=admission_limit,
            sensor_confidence=.3 if one_side or len(path)<4 else cruise,
            attitude=min(.8*cruise,.55) if abs(pitch)>.04 else (.3 if abs(roll)>.08 else cruise))
        if self.path_mode=='left_wall':
            # Stay within both the observed corridor and the tracking horizon.
            limits.update(wall_caps)
            limits.update(wall_combined=wall_speed_limit,
                observed_distance=wall_distance_speed(path[-1][0]),
                obstacle_tracking_horizon=wall_distance_speed(min(obstacle_horizon,path[-1][0])))
            entry_diagnostic['obstacle_tracking_horizon_m']=min(obstacle_horizon,path[-1][0])
        if self.race_center:
            limits['observed_distance']=wall_distance_speed(min(obstacle_horizon,path[-1][0]))
            if one_side or min(live_wall_counts)<8 or untracked_object or any(not t['confident'] for t in tracks):
                limits['sensor_confidence']=min(cruise,.1)
        if self.experimental_race_speed:
            limits.update(race_distance=distance_cap,obstacle_prediction=min(limits.get('obstacle_prediction',cruise),obstacle_cap),
                yaw_rate=yaw_cap,preview_corner=corner_cap,sensor_confidence=confidence_cap)
        if corner:
            limits['observed_corner']=corner['speed_cap_mps']
        # Preserve legacy PP preview behavior at <=2m/s. The diagnostic preview
        # is already part of its curvature term there; higher speeds use fit.
        straight_wall_follow = bool(staged_wall_speed
                and entry_diagnostic['entry_mode']=='wall_follow'
                and entry_diagnostic['entry_trusted'] and not center_fallback
                and wall_center_clearance is not None and len(path)>=4
                and not (rejected_slices or one_side or untracked_object)
                and abs(entry_diagnostic['entry_heading_error_rad'])<=.06
                and entry_diagnostic['entry_max_curvature']<=.03
                and max(abs(actual-raw_desired),abs(self.steering-raw_desired))<=self.wall_steering_error)
        if staged_wall_speed and cruise>2.:
            live_support=min(path[-1][0],obstacle_horizon)
            qualified = (straight_wall_follow and adaptive_gap and not tracks
                and live_support>LEGACY_WALL_TRACKING_M)
            history=self.straight_support_history
            if not qualified or (history and not 0.<=stamp-history[-1][0]<=.15):
                history.clear()
            if qualified and (not history or stamp>history[-1][0]):
                history.append((stamp,live_support))
            stable_support=min([live_support]+[length for _,length in history]) if len(history)==3 else 0.
            limits['straight_trial']=min(cruise,max(2.,wall_distance_speed(stable_support)))
            entry_diagnostic['stable_straight_support_m']=stable_support
        entry_diagnostic['boundary_extent_audit']=boundary_extent_audit
        target=min(target,*limits.values())
        next_straight_speed = self.speed + .32*dt
        # Pose history supports a small increase on settled Monza straights.
        # Use the same observed corridor and retain the prospective sweep below.
        straight_acceleration = bool(straight_wall_follow and adaptive_gap and not tracks
            and cruise>1. and self.speed>=.5 and abs(wheel_input-self.speed)<=.08
            and abs(pitch)<=.04 and abs(roll)<=.08
            and wall_stopping_distance(max(abs(measured_speed),next_straight_speed))
                <= min(path[-1][0],obstacle_horizon))
        if straight_acceleration:
            acceleration = .32
        limits['acceleration_ramp']=self.speed+acceleration*dt
        command=min(target,limits['acceleration_ramp'])
        if straight_wall_follow:
            # Replace the scalar nearest-return comparison only when its
            # more specific swept-volume check can support a higher command.
            other_limits=[v for k,v in limits.items() if k not in ('wall_clearance','wall_combined')]
            candidate=min(wall_caps['wall_confidence'],*other_limits)
            if candidate>max(command,wall_caps['wall_clearance']):
                probe=max(abs(measured_speed),candidate)
                extent=wall_stopping_distance(probe)
                base_radius=.23+self.wall_margin+uncertainty+.025+.05*probe
                angles=(actual,self.steering,raw_desired)
                steering_error=max(abs(math.tan(angle+delta)-math.tan(angle))/WHEELBASE
                    for angle in angles for delta in (-angle_error,angle_error))
                if (extent<=min(path[-1][0],obstacle_horizon)
                        and all(trajectory_safe(angle,extent,base_radius,steering_error)
                            for angle in angles)):
                    wall_motion_audit.update(straight_swept_speed_mps=candidate,
                        straight_swept_extent_m=extent)
                    limits['wall_clearance']=candidate
                    wall_speed_limit=min(candidate,*(v for k,v in wall_caps.items() if k!='wall_clearance'))
                    limits['wall_combined']=wall_speed_limit
                    target=max(target,candidate)
                    command=candidate
        staged_sweep=staged_wall_speed and max(abs(measured_speed),command)>.5
        if (self.path_mode=='left_wall' and 'straight_swept_speed_mps' not in wall_motion_audit
                and (staged_sweep or command>legacy_wall_clearance_limit+1e-8)):
            probe=max(abs(measured_speed),command)
            prospective_reserve=.5*wall_stopping_distance(probe)**2*curvature_error
            prospective_radius=max(footprint_radius,.23+uncertainty+prospective_reserve+.025)
            prospective_horizon=wall_stopping_distance(probe)
            safe=(trajectory_safe(self.steering,prospective_horizon,prospective_radius)
                and trajectory_safe(actual,prospective_horizon,prospective_radius))
            wall_motion_audit.update(prospective_speed_mps=probe,
                prospective_radius_m=prospective_radius,prospective_horizon_m=prospective_horizon,
                prospective_swept_safe=bool(safe))
            if (safe and straight_wall_follow and adaptive_gap and not tracks
                    and .5<=self.speed<command and command>=wall_caps['wall_clearance']-.08):
                # Look just beyond the new command before spending the final
                # sweep margin. Holding can only lower an already safe command.
                buffered=probe+.04
                extent=wall_stopping_distance(buffered)
                radius=max(footprint_radius,.23+uncertainty+.5*extent**2*curvature_error+.025)
                if (extent>min(path[-1][0],obstacle_horizon)
                        or not (trajectory_safe(actual,extent,radius)
                            and trajectory_safe(self.steering,extent,radius))):
                    limits['wall_sweep_headroom']=self.speed
                    command=self.speed
            if not safe:
                fallback_cap=min(.5,legacy_wall_clearance_limit) if staged_sweep else legacy_wall_clearance_limit
                if staged_sweep and command>max(.5,abs(measured_speed)) and path[-1][0]>=prospective_horizon:
                    # Keep the measured-speed floor in every prospective
                    # check. Only credit a lower speed whose two arcs pass.
                    low=max(.5,abs(measured_speed))
                    high=command
                    for step in range(4):
                        candidate=low if step==0 else (low+high)/2
                        extent=wall_stopping_distance(candidate)
                        radius=max(footprint_radius,.23+uncertainty+.5*extent**2*curvature_error+.025)
                        if trajectory_safe(actual,extent,radius) and trajectory_safe(self.steering,extent,radius):
                            fallback_cap=low=candidate
                            wall_motion_audit['prospective_admitted_speed_mps']=candidate
                        else:
                            if step==0:break
                            high=candidate
                limits['wall_prospective_stop']=fallback_cap
                command=min(command,fallback_cap)
        active=[k for k,v in limits.items() if abs(v-command)<1e-8]
        self.speed = command  # immediate safe deceleration
        return self.speed,self.steering,dict(reason='tracking',path=path,one_side=one_side,
            candidate_audit=self.candidate_audit,admission_speed_limit=admission_limit,**race_audit,
            speed_limits_mps=limits,active_speed_limits=active,speed_command_mps=self.speed,
            drive_rolling_ceiling_mps=100*RADIUS,wheel_speed_input_mps=wheel_input,
            confirmed_distance_m=float(path[-1][0]),wall_motion_audit=wall_motion_audit,
            **entry_diagnostic,path_mode=self.path_mode,wall_center_clearance_m=wall_center_clearance,
            effective_wall_distance_m=effective_wall_distance,wall_speed_limit=wall_speed_limit,
            width=self.width,wall_slices=wall_slices,clearance=clearance,preview_curvature=preview,lookahead=lookahead,
            planned_speed_limit=target,acceleration_limit=acceleration,braking_assumption=braking,
            straight_acceleration=straight_acceleration,observed_corner=corner,
            obstacle_tracking=tracking_audit if track_safety else None,
            raw_desired=raw_desired,rejected_slices=rejected_slices,
            speed_envelope=self.speed_envelope,steering_slew_limit=slew,yaw_rate=yaw_rate,
            path_fit_residual_m=fit_residual,routine_braking_eligible=bool(highspeed and not one_side and len(path)>=4 and clearance>measured_speed**2/(2*braking)+1.),
            estimated_lateral_accel=measured_speed**2*abs(math.tan(self.steering)/WHEELBASE))


@dataclass
class ProgressEstimate:
    """Wheel/IMU dead reckoning is diagnostic; cannot certify a lap without localization."""
    x: float = 0.
    y: float = 0.
    distance: float = 0.
    yaw0: object = None
    previous: object = None
    departed: bool = False
    forward: float = 0.
    reverse: float = 0.
    heading: float = 0.
    total_turn: float = 0.

    def update(self, stamp, speed, yaw, pitch):
        if self.yaw0 is None:self.yaw0 = yaw
        heading = math.atan2(math.sin(yaw-self.yaw0),math.cos(yaw-self.yaw0))
        if self.previous is not None:
            dt = stamp-self.previous
            if not 0<dt<.25:
                self.previous = stamp
                return
            ds = speed*math.cos(pitch)*dt
            self.x += ds*math.cos(heading);self.y += ds*math.sin(heading)
            self.distance += abs(ds)
            self.forward += max(ds,0);self.reverse += max(-ds,0)
            self.total_turn += math.atan2(math.sin(heading-self.heading),math.cos(heading-self.heading))
        self.heading,self.previous = heading,stamp
        self.departed |= math.hypot(self.x,self.y)>10

    def report(self):
        candidate = (self.departed and self.distance>500 and self.reverse<.01*max(self.forward,1)
                     and abs(self.y)<2 and abs(self.x)<2 and abs(self.heading)<.3
                     and abs(abs(self.total_turn)-2*math.pi)<.6)
        return dict(x=self.x,y=self.y,distance=self.distance,departed=self.departed,
                    reverse=self.reverse,heading=self.heading,loop_candidate=candidate,
                    lap_confirmed=False,localization='wheel_imu_unbounded_drift')


def control():
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import ExternalShutdownException
    from std_msgs.msg import Float32, Bool, String
    from sensor_msgs.msg import Image, CameraInfo, LaserScan, PointCloud2, Imu, JointState
    from rclpy.qos import qos_profile_sensor_data
    from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
    from geometry_msgs.msg import TransformStamped
    from gz.transport13 import Node as GzNode
    from gz.msgs10.double_pb2 import Double

    class Racer(Node):
        def __init__(self):
            super().__init__('team07_hybrid_racer')
            self.speed = self.steering = 0.
            self.last_speed = self.last_steering = -math.inf
            self.gz = GzNode()
            self.pubs = [self.gz.advertise(BASE+'/'+s, Double) for s in ('left_steer','right_steer',*DRIVE_TOPICS)]
            self.declare_parameter('speed_topic', BASE+'/speed')
            self.declare_parameter('steering_topic', BASE+'/steering')
            self.declare_parameter('start_gate',False)
            self.declare_parameter('start_speed',.3)
            self.declare_parameter('start_duration',1.)
            self.declare_parameter('require_start_authorization',False)
            self.declare_parameter('race_start_topic','/race/start/team_07')
            self.declare_parameter('autonomy_mode','off')
            self.declare_parameter('practice_authorized',False)
            self.declare_parameter('cruise_speed',.5)
            self.declare_parameter('path_mode','auto')
            self.declare_parameter('left_wall_distance',.30)
            self.declare_parameter('wall_distance_error',.06)
            self.declare_parameter('wall_margin',.04)
            self.declare_parameter('wall_memory_seconds',.5)
            self.declare_parameter('left_speed_limit',.3)
            self.declare_parameter('record_control_trace',False)
            self.command_trace=None
            if self.get_parameter('record_control_trace').value:
                from gz.msgs10.double_v_pb2 import Double_V
                self.command_trace=self.gz.advertise(BASE+'/command_generated',Double_V)
            self.auto_previous_speed=0.
            self.wheel_stamp=None
            self.steering_actual=None
            self.steering_wall=-math.inf
            self.steering_stamp=None
            self.declare_parameter('race_start_mode','camera')
            self.declare_parameter('experimental_race_speed',False)
            self.declare_parameter('highspeed_validation',False)
            ceiling=cruise_limit(self.get_parameter('highspeed_validation').value)
            if self.get_parameter('experimental_race_speed').value:ceiling=4.
            if not .1<=self.get_parameter('cruise_speed').value<=ceiling:
                raise ValueError(f'cruise_speed exceeds authorized ceiling {ceiling}')
            self.mode = self.get_parameter('autonomy_mode').value
            if self.mode not in ('off','practice','race'):
                raise ValueError('autonomy_mode must be off, practice or race')
            if self.mode!='off' and not self.get_parameter('use_sim_time').value:
                raise ValueError('Autonomy requires use_sim_time=true')
            if self.mode=='race' and not self.get_parameter('start_gate').value:
                raise ValueError('Race autonomy requires camera start_gate=true')
            if (self.mode!='race' or self.get_parameter('race_start_mode').value!='camera'
                    or not self.get_parameter('require_start_authorization').value
                    or self.get_parameter('experimental_race_speed').value
                    or self.get_parameter('cruise_speed').value>.3
                    or self.get_parameter('left_speed_limit').value>.3):
                raise ValueError('Submission requires camera race, operator approval and a 0.3 m/s ceiling')
            driver_options={k:self.get_parameter(k).value for k in
                ('path_mode','left_wall_distance','wall_margin','wall_memory_seconds','left_speed_limit','wall_distance_error','experimental_race_speed')}
            driver_options['path_mode']=resolve_path_mode(driver_options['path_mode'],self.mode)
            if driver_options['path_mode']!='center':
                raise ValueError('Submission Race requires the completion center path')
            driver_options['race_center']=True
            self.driver,self.progress=LocalDriver(**driver_options),ProgressEstimate()
            self.wheel_speed,self.wheel_wall = 0.,-math.inf
            self.wheel_peak_speed=None
            self.last_auto_stamp = None
            self.auto_pub = self.create_publisher(String,BASE+'/autonomy/status',10)
            self.auto_speed_pub = self.create_publisher(Float32,self.get_parameter('speed_topic').value,10)
            self.auto_steer_pub = self.create_publisher(Float32,self.get_parameter('steering_topic').value,10)
            self.auto_log = None
            self.corner_scan_log = None
            self.auto_last_log = -math.inf
            if self.mode!='off':
                import os
                logdir = Path(os.environ.get('TEAM07_LOG_ROOT',str(ROOT)))/'.autonomy'
                logdir.mkdir(parents=True,exist_ok=True)
                trace_path=os.environ.get('TEAM07_RACE_CONTROL_TRACE')
                self.auto_log = Path(trace_path or logdir/f'drive_{time.time_ns()}.jsonl').open('w',buffering=1)
                if trace_path and self.driver.path_mode == 'left_wall':
                    self.corner_scan_log = Path(trace_path).with_name('corner_scans.jsonl').open('w', buffering=1)
                    self.corner_scans = deque(maxlen=60)
                    self.corner_scan_stamp = None
                    self.corner_scan_until = -math.inf
                    self.corner_scan_fault = False
            self.gate = StartGate()
            self.start_enabled = self.get_parameter('start_gate').value
            self.operator_allowed = not self.get_parameter('require_start_authorization').value
            self.status_pub = self.create_publisher(String,BASE+'/start/status',10)
            self.last_status = -math.inf
            self.last_gate_state = None
            self.create_subscription(Bool,self.get_parameter('race_start_topic').value,self.on_authorization,10)
            self.create_subscription(JointState,BASE+'/joint_states',self.on_wheels,qos_profile_sensor_data)
            self.create_subscription(Float32, self.get_parameter('speed_topic').value, self.on_speed, 10)
            self.create_subscription(Float32, self.get_parameter('steering_topic').value, self.on_steering, 10)
            self.sensors = SensorBuffer()
            for kind, msg_type, suffix in [('camera',Image,'camera/image'),
                    ('camera_info',CameraInfo,'camera/camera_info'),('scan',LaserScan,'scan'),
                    ('points',PointCloud2,'lidar3d/points'),('imu',Imu,'imu')]:
                self.create_subscription(msg_type, BASE+'/'+suffix,
                    lambda msg, key=kind: self.on_sensor(key,msg), qos_profile_sensor_data)
            self.tf = StaticTransformBroadcaster(self)
            transforms = []
            for child, parent, xyz, quaternion in [
                ('team07_camera','base_link_07',(.175,0,.035),(0,math.sin(-.2),0,math.cos(-.2))),
                ('team07_camera_optical','team07_camera',(0,0,0),(-.5,.5,-.5,.5)),
                ('team07_lidar','base_link_07',(0,0,.054),(0,0,0,1)),
                ('team07_imu','base_link_07',(-.04,0,.026),(0,0,0,1))]:
                t = TransformStamped()
                t.header.stamp = self.get_clock().now().to_msg()
                t.header.frame_id, t.child_frame_id = parent, child
                t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = map(float,xyz)
                t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w = map(float,quaternion)
                transforms.append(t)
            self.tf.sendTransform(transforms)
            self.create_timer(2., self.sensor_status)
            self.create_timer(.02, self.tick)
            if self.mode!='off':
                from rclpy.clock import Clock, ClockType
                self.clock_watch_stamp = None
                self.clock_watch_wall = time.monotonic()
                self.safety_clock = Clock(clock_type=ClockType.STEADY_TIME)
                self.create_timer(.1,self.clock_watchdog,clock=self.safety_clock)
            self.get_logger().info(f'Camera gate={self.start_enabled}. Autonomy={self.mode}. Steering +right. Manual input timeout 0.5s.')

        def on_sensor(self,kind,msg):
            if not self.sensors.put(kind,msg):
                if self.start_enabled and kind=='camera' and self.gate.state in ('COUNTDOWN','ALL_RED'):
                    self.gate.reason = 'invalid_camera_frame'
                return
            if self.start_enabled and kind=='camera':
                stamp = msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
                now = self.get_clock().now().nanoseconds*1e-9
                if now-stamp>.30:
                    self.gate.watchdog(now);return
                self.gate.observe(lamp_observation(msg),stamp)

        def on_authorization(self,msg):
            self.operator_allowed = msg.data

        def on_wheels(self,msg):
            if set(DRIVEN_WHEELS)<=set(msg.name) and len(msg.velocity)==len(msg.name):
                velocities = [msg.velocity[msg.name.index(n)] for n in DRIVEN_WHEELS]
                if all(math.isfinite(v) and abs(v)<100 for v in velocities):
                    # Rear wheels measure the longitudinal component directly.
                    self.wheel_speed = RADIUS*sum(velocities[:2])/2
                    self.wheel_peak_speed = RADIUS*max(map(abs,velocities))
                    self.wheel_wall = time.monotonic()
                    self.wheel_stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
                    attitude = self.sensors.attitude()
                    if attitude is not None:
                        self.progress.update(self.wheel_stamp,self.wheel_speed,attitude[2],attitude[1])
                    if self.start_enabled:
                        self.gate.wheel_motion(self.wheel_stamp,self.wheel_peak_speed)
            if {'front_left_steer','front_right_steer'}<=set(msg.name) and len(msg.position)==len(msg.name):
                left=msg.position[msg.name.index('front_left_steer')];right=msg.position[msg.name.index('front_right_steer')]
                if all(math.isfinite(v) and abs(v)<=LIMIT+.02 for v in (left,right)):
                    tl,tr=math.tan(left),math.tan(right)
                    self.steering_actual=-math.atan(WHEELBASE*.5*(tl/(WHEELBASE+TRACK/2*tl)+tr/(WHEELBASE-TRACK/2*tr)))
                    self.steering_wall=time.monotonic()
                    self.steering_stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9

        def on_speed(self, msg):
            self.speed = msg.data
            self.last_speed = time.monotonic()

        def on_steering(self, msg):
            self.steering = msg.data
            self.last_steering = time.monotonic()

        def send(self, values):
            values=four_wheel_targets(values)
            if getattr(self,'command_trace',None):
                from gz.msgs10.double_v_pb2 import Double_V
                self.command_trace.publish(Double_V(data=[self.get_clock().now().nanoseconds*1e-9,*values]))
            for pub, value in zip(self.pubs, values):
                pub.publish(Double(data=value))

        def sensor_status(self):
            ages = {k: round(time.monotonic()-v[-1][1],2) for k,v in self.sensors.samples.items() if v}
            angles = self.sensors.attitude()
            attitude = None if angles is None else tuple(round(math.degrees(v),1) for v in angles)
            self.get_logger().info(f'Sensors age_s={ages} synchronized={self.sensors.camera_scan_pair() is not None} imu_rpy_deg={attitude}; diagnostics only')

        def clock_watchdog(self):
            stamp = self.get_clock().now().nanoseconds
            if stamp!=self.clock_watch_stamp:
                self.clock_watch_stamp,self.clock_watch_wall = stamp,time.monotonic()
            if stamp==0 or time.monotonic()-self.clock_watch_wall>.4:
                self.send((0.,0.,0.,0.))
                self.driver.speed = 0.
                self.last_speed = self.last_steering = -math.inf

        def autonomous_input(self):
            import json
            now = self.get_clock().now().nanoseconds*1e-9
            dt = .02 if self.last_auto_stamp is None else now-self.last_auto_stamp
            self.last_auto_stamp = now
            allowed = ((self.mode=='practice' and self.get_parameter('practice_authorized').value)
                       or (self.mode=='race' and self.gate.authorized and self.operator_allowed and not self.gate.reason))
            speed,steer = 0.,self.driver.steering
            diagnostic = dict(reason='awaiting_start')
            samples = self.sensors.samples
            fresh = all(k in samples and time.monotonic()-samples[k][-1][1]<.4
                        and -.02<=now-samples[k][-1][0]<.25 for k in ('scan','imu'))
            fresh &= time.monotonic()-self.wheel_wall<.4
            if self.driver.experimental_race_speed:
                # Bound the .35s planning allowance: .15s input age + .10s
                # planning budget + .08s actuator + .02s control period.
                fresh = bool(fresh and all(time.monotonic()-samples[k][-1][1]<.15
                    and -.02<=now-samples[k][-1][0]<.15 for k in ('scan','imu')))
                fresh &= time.monotonic()-self.wheel_wall<.15
            fresh &= (self.steering_actual is not None and self.steering_stamp is not None
                      and time.monotonic()-self.steering_wall<.15 and -.02<=now-self.steering_stamp<.15)
            if allowed and fresh and 0<dt<.15:
                scan = samples['scan'][-1][2]
                planning_started=time.monotonic()
                speed,steer,diagnostic = self.driver.plan(scan,self.sensors.attitude(),self.wheel_speed,dt,
                    self.get_parameter('cruise_speed').value,samples['imu'][-1][2].angular_velocity.z,
                    self.steering_actual if time.monotonic()-self.steering_wall<.15 and self.steering_stamp is not None and -.02<=now-self.steering_stamp<.15 else None)
                diagnostic['planner_wall_s']=time.monotonic()-planning_started
                if self.driver.experimental_race_speed and diagnostic['planner_wall_s']>.10:
                    speed=0.
                    diagnostic['reason']='race_planning_deadline_stop'
            elif allowed:
                diagnostic = dict(reason='stale_sensor_or_clock')
            if speed==0 and diagnostic['reason']!='tracking':
                diagnostic.update(speed_limits_mps={**diagnostic.get('speed_limits_mps',{}),'control_stop':0.},active_speed_limits=[diagnostic['reason']])
            diagnostic['speed_command_mps']=speed
            if not diagnostic.get('active_speed_limits'):
                diagnostic.update(speed_limits_mps=dict(control_stop=0.),active_speed_limits=[diagnostic['reason']])
            if speed==0:
                self.driver.speed = 0.
                self.driver.reset_left_entry()
                if not allowed or not fresh or diagnostic['reason']=='race_planning_deadline_stop':
                    self.driver.wall_recovery_active=False
                    self.driver.wall_recovery_target=None
                    self.driver.wall_handoff_stamp=None
            # Preserve the existing interfaces and their timeout, refreshing both.
            speed_msg,steer_msg = Float32(data=float(speed)),Float32(data=float(steer))
            self.auto_speed_pub.publish(speed_msg);self.auto_steer_pub.publish(steer_msg)
            self.on_speed(speed_msg);self.on_steering(steer_msg)
            event=self.driver.safety_event
            if event and 'control_stamp' not in event:
                event.update(control_stamp=now,wheel_stamp=self.wheel_stamp,
                    imu_stamp=samples['imu'][-1][0] if samples.get('imu') else None)
            commanded_acceleration=(speed-self.auto_previous_speed)/dt if 0<dt<.15 else None
            self.auto_previous_speed=speed
            if self.mode=='race' or now-self.auto_last_log>=.1 or self.get_parameter('record_control_trace').value or (event and event['control_stamp']==now):
                report = dict(stamp=now,mode=self.mode,control_dt=dt,attitude=self.sensors.attitude(),speed_command=speed,steering=steer,
                    wheel_speed=self.wheel_speed,wheel_peak_speed=self.wheel_peak_speed,progress=self.progress.report(),**diagnostic)
                report.update(steering_actual=self.steering_actual,steering_stamp=self.steering_stamp,scan_stamp=samples['scan'][-1][0] if samples.get('scan') else None,
                    scan_age_sim=now-samples['scan'][-1][0] if samples.get('scan') else None,
                    scan_age_wall=time.monotonic()-samples['scan'][-1][1] if samples.get('scan') else None,
                    steering_command=steer,acceleration_command=commanded_acceleration,
                    safety_trigger=event or None,wheel_stamp=self.wheel_stamp,
                    imu_stamp=samples['imu'][-1][0] if samples.get('imu') else None,
                    yaw_rate=samples['imu'][-1][2].angular_velocity.z if samples.get('imu') else None)
                encoded = json.dumps(report,allow_nan=False)
                self.auto_pub.publish(String(data=encoded))
                if self.auto_log:self.auto_log.write(encoded+'\n')
                if self.corner_scan_log and allowed and fresh and 0<dt<.15:
                    self.record_corner_scan(scan, report)
                self.auto_last_log = now

        def record_corner_scan(self, scan, report):
            import json
            stamp = report['scan_stamp']
            if stamp == self.corner_scan_stamp:
                return
            self.corner_scan_stamp = stamp
            tracking = report.get('obstacle_tracking', {})
            counts = tracking.get('live_wall_counts')
            self.corner_scans.append((scan, report))
            fault = (counts is not None and min(counts)<8
                or report['reason'] in ('obstacle_track_lost', 'obstacle_boundary_unavailable',
                    'obstacle_association_failed', 'obstacle_scan_gap'))
            if fault and not self.corner_scan_fault:
                self.corner_scan_until = stamp + 3.
            self.corner_scan_fault = fault
            while self.corner_scans and stamp <= self.corner_scan_until:
                message, control = self.corner_scans.popleft()
                record = dict(control=control, frame_id=message.header.frame_id,
                    angle_min=message.angle_min, angle_max=message.angle_max,
                    angle_increment=message.angle_increment, range_min=message.range_min,
                    range_max=message.range_max,
                    ranges=[v if math.isfinite(v) else str(v) for v in message.ranges])
                self.corner_scan_log.write(json.dumps(record, allow_nan=False)+'\n')

        def tick(self):
            import json
            if self.mode!='off':self.autonomous_input()

            fresh = time.monotonic() - min(self.last_speed, self.last_steering) < .5
            speed,steer = (self.speed,self.steering) if fresh else (0.,0.)
            if self.start_enabled:
                now = self.get_clock().now().nanoseconds*1e-9
                self.gate.watchdog(now)
                if not self.gate.authorized or not self.operator_allowed or self.gate.reason:
                    speed,steer = 0.,0.
                elif self.mode=='off' and not fresh and now-self.gate.times['go'] < max(0.,min(2.,self.get_parameter('start_duration').value)):
                    speed,steer = max(0.,min(.5,self.get_parameter('start_speed').value)),0.
                if ('first_red' in self.gate.times and 'wheel_motion_estimate' not in self.gate.times
                        and now-self.gate.times['first_red']>20):
                    self.gate.reason = 'start_deadline_exceeded';speed,steer = 0.,0.
                if self.gate.state!=self.last_gate_state or now-self.last_status>=.1:
                    report = self.gate.report();report.update(command_speed=speed,operator_allowed=self.operator_allowed,stamp=now)
                    self.status_pub.publish(String(data=json.dumps(report)))
                    self.last_status = now
                    self.last_gate_state = self.gate.state
            self.send(targets(speed,steer))

    rclpy.init()
    node = Racer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt,ExternalShutdownException):
        pass
    except RuntimeError:
        if rclpy.ok():raise
    finally:
        node.send((0.,0.,0.,0.))
        if node.auto_log:node.auto_log.close()
        if node.corner_scan_log:node.corner_scan_log.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def analyze_brake_fixture(run):
    import csv
    import json
    import hashlib
    import numpy as np
    run=Path(run);poses=np.asarray(json.loads((run/'poses.json').read_text()))
    states=[json.loads(l) for l in (run/'fixture_state.jsonl').read_text().splitlines()]
    joints=[json.loads(l) for l in (run/'joints.jsonl').read_text().splitlines()]
    motors=[json.loads(l) for l in (run/'drive_pi.jsonl').read_text().splitlines()]
    meta=json.loads((run/'results.json').read_text());t0=meta['brake_request_stamp'];end=meta['fixture_end_time'];pre=t0-.01
    position=lambda t:[float(np.interp(t,poses[:,0],poses[:,i])) for i in (1,2,3)]
    state=min(states,key=lambda s:abs(s['stamp']-pre))
    before=poses[poses[:,0]<t0];last,prior=before[-1],before[-2]
    pre_j=max((j for j in joints if j['stamp']<=pre),key=lambda j:j['stamp'])
    pre_m=max((j for j in motors if j['stamp']<=pre),key=lambda j:j['stamp'])
    world=ET.parse(run/'world.sdf');world.find(".//plugin[@name='team07::DrivePI']/brake_integral_reset").text='false'
    replay_path=world.find(".//fixture_replay_file")
    if replay_path is not None:replay_path.text=hashlib.sha256(Path(replay_path.text).read_bytes()).hexdigest()
    start=position(t0);heading=float(np.interp(t0,poses[:,0],np.unwrap(poses[:,4])))
    sampled=np.array([start]+[list(p[1:4]) for p in poses if t0<p[0]<=end])
    delta=sampled[:,:2]-np.asarray(start[:2]);forward=delta@np.array([math.cos(heading),math.sin(heading)])
    lateral=delta@np.array([-math.sin(heading),math.cos(heading)])
    contacts=[]
    for line in (run/'contacts.jsonl').read_text().splitlines():
        row=json.loads(line);force=np.zeros(3);points=0;available=False;normal_force=0.;tangent_force=0.
        for contact in row['message'].get('contact',[]):
            names=[contact.get(k,{}).get('name','') for k in ('collision1','collision2')]
            wheel=next((i for i,n in enumerate(names) if 'rear_'+row['side']+'_wheel' in n),None)
            if wheel is None:continue
            for wi,wrench in enumerate(contact.get('wrench',[])):
                a=wrench.get('body_1_wrench',{}).get('force')
                b=wrench.get('body_2_wrench',{}).get('force')
                if a is None or b is None:continue
                f1=np.array([a.get(k,0.) for k in ('x','y','z')]);f2=np.array([b.get(k,0.) for k in ('x','y','z')])
                f=f1 if wheel==0 else f2;force+=f;available=True;points+=1
                normals=contact.get('normal',[]);nraw=normals[min(wi,len(normals)-1)] if normals else {}
                n=np.array([nraw.get(k,0.) for k in ('x','y','z')]);norm=np.linalg.norm(n)
                if norm>0:
                    n/=norm;normal_force+=abs(float(f@n));tangent_force+=float(np.linalg.norm(f-(f@n)*n))
                else:available=False
        contacts.append(dict(stamp=row['stamp'],side=row['side'],available=available,
            fx=float(force[0]),fy=float(force[1]),normal=normal_force,tangent=tangent_force,points=points))
    with (run/'contact_forces.csv').open('w') as f:
        writer=csv.DictWriter(f,fieldnames=list(contacts[0]) if contacts else ['stamp']);writer.writeheader();writer.writerows(contacts)
    legacy=json.loads((run/'braking_analysis.json').read_text())
    result=dict(initial_state=dict(time=pre,position=position(pre),rpy=state['rpy'],
        velocity_world=((last[1:4]-prior[1:4])/(last[0]-prior[0])).tolist(),
        joint_stamp=pre_j['stamp'],joints=pre_j['joints'],motor_stamp=pre_m['stamp'],motor=pre_m['values']),
        normalized_world_sha256=hashlib.sha256(ET.tostring(world.getroot())).hexdigest(),
        primary_distance_definition=f'max displacement along fixed heading at motor zero t={t0}s, until {end}s',
        forward_extent_m=float(max(forward)),max_lateral_excursion_m=float(max(abs(lateral))),
        total_path_after_zero_m=float(np.linalg.norm(np.diff(sampled[:,:2],axis=0),axis=1).sum()),
        final_speed=float(np.linalg.norm(poses[-1,1:3]-poses[-2,1:3])/(poses[-1,0]-poses[-2,0])),
        legacy_projection=legacy,contact={})
    aligned=list(csv.DictReader((run/'braking_aligned.csv').open()))
    for side in ('left','right'):
        pre_force=[c for c in contacts if c['side']==side and c['available'] and t0-1<=c['stamp']<t0]
        active=[]
        for c in contacts:
            if c['side']!=side or not c['available'] or not t0<=c['stamp']<=end:continue
            row=min(aligned,key=lambda r:abs(float(r['stamp'])-c['stamp']))
            if float(row['body_forward_speed'])>.5:
                active.append(dict(c,torque=float(row[side+'_torque_command']),wheel_speed=RADIUS*float(row[side+'_actual_omega']),body_speed=float(row['body_forward_speed'])))
        result['contact'][side]=dict(measured=bool(active),pre_normal_mean=float(np.mean([c['normal'] for c in pre_force])) if pre_force else None,
            braking_normal_mean=float(np.mean([c['normal'] for c in active])) if active else None,
            braking_normal_min=min((c['normal'] for c in active),default=None),
            braking_tangent_mean=float(np.mean([c['tangent'] for c in active])) if active else None,
            first_near_lock=next((c for c in active if abs(c['wheel_speed'])<.2*c['body_speed']),None),
            force_source='Gazebo contact wrench projected on recorded contact normal; analysis only')
    (run/'fixture_analysis.json').write_text(json.dumps(result,indent=2)+'\n')
    return result


def compare_brake_checkpoint(source, restored):
    """Fail closed: exposed state AND contact evidence must match before comparison."""
    import json
    import numpy as np
    source,restored=Path(source),Path(restored)
    limits=dict(time_s=1e-9,position_m=1e-6,orientation_rad=1e-4,linear_velocity_m_s=1e-3,
        angular_velocity_rad_s=1e-3,joint_position_rad=1e-4,joint_velocity_rad_s=1e-3,
        pi_state=1e-7,steering_pi_state=1e-7,steering_target_rad=1e-7,contact_force_n=.01)
    import hashlib
    result=dict(matched=False,method='full applied-input warmup; ECM/solver restore NOT claimed',limits=limits,errors={},missing=[],
        applied_input_sha256=hashlib.sha256((source/'applied_inputs.csv').read_bytes()).hexdigest(),source=str(source),restored=str(restored),
        contact_phase='last completed contact message strictly BEFORE brake apply tick')
    if not all((p/'checkpoint.json').exists() for p in (source,restored)):
        result['missing'].append('native checkpoint');return result
    a,b=[json.loads((p/'checkpoint.json').read_text()) for p in (source,restored)]
    result['source_stamp']=a['stamp'];result['restored_stamp']=b['stamp']
    e=result['errors'];e['time_s']=abs(a['stamp']-b['stamp'])
    if len(a['body'])==len(b['body'])==13:
        x,y=np.array(a['body']),np.array(b['body']);e['position_m']=float(np.linalg.norm(x[:3]-y[:3]))
        q=abs(float(x[3:7]@y[3:7]));e['orientation_rad']=2*math.acos(min(1.,max(0.,q)))
        e['linear_velocity_m_s']=float(np.linalg.norm(x[7:10]-y[7:10]));e['angular_velocity_rad_s']=float(np.linalg.norm(x[10:13]-y[10:13]))
    else:result['missing'].append('body pose/velocity')
    try:
        x,y=np.array([j[1:] for j in a['joints']],float),np.array([j[1:] for j in b['joints']],float)
        if [j[0] for j in a['joints']]!=[j[0] for j in b['joints']] or not np.isfinite(x).all() or not np.isfinite(y).all():raise ValueError('joint schema')
        e['joint_position_rad']=float(np.max(abs(x[:,0]-y[:,0])));e['joint_velocity_rad_s']=float(np.max(abs(x[:,1]-y[:,1])))
        e['pi_state']=float(np.max(abs(np.array(a['pi_before'])[:,:6]-np.array(b['pi_before'])[:,:6])))
        e['steering_pi_state']=float(np.max(abs(np.array(a['steering_pi_before'])-np.array(b['steering_pi_before']))))
        e['steering_target_rad']=float(np.max(abs(np.array(a['steering_target'])-np.array(b['steering_target']))))
    except (ValueError,TypeError):result['missing'].append('joint/PI state')
    def contacts(path, stamp,side):
        rows=[json.loads(l) for l in (path/'contacts.jsonl').open()]
        rows=[r for r in rows if r['side']==side and r['stamp']<stamp-1e-9]
        if not rows:return None
        row=max(rows,key=lambda r:r['stamp'])
        if stamp-row['stamp']>.006:return None
        points=[]
        for c in row['message'].get('contact',[]):
            names=[c.get(k,{}).get('name','') for k in ('collision1','collision2')]
            for i,w in enumerate(c.get('wrench',[])):
                if i>=len(c.get('position',[])) or i>=len(c.get('normal',[])):return None
                points.append(dict(collisions=names,position=c['position'][i],normal=c['normal'][i],wrench=w))
        return dict(stamp=row['stamp'],points=points)
    result['contact']={};maxforce=0.;geometry_ok=True
    for side in ('left','right'):
        ca,cb=contacts(source,a['stamp'],side),contacts(restored,b['stamp'],side)
        result['contact'][side]=dict(source=ca,restored=cb)
        if ca is None or cb is None or not ca['points'] or len(ca['points'])!=len(cb['points']):
            result['missing'].append(side+' contact topology/time');continue
        if abs(ca['stamp']-cb['stamp'])>.001:geometry_ok=False
        for pa,pb in zip(ca['points'],cb['points']):
            if pa['collisions']!=pb['collisions']:geometry_ok=False
            for kind in ('position','normal'):
                if np.linalg.norm([pa[kind].get(k,0)-pb[kind].get(k,0) for k in ('x','y','z')])>1e-5:geometry_ok=False
            for body in ('body_1_wrench','body_2_wrench'):
                if 'force' not in pa['wrench'].get(body,{}) or 'force' not in pb['wrench'].get(body,{}):
                    result['missing'].append(side+' contact wrench');continue
                fa,fb=pa['wrench'][body]['force'],pb['wrench'][body]['force']
                maxforce=max(maxforce,float(np.linalg.norm([fa.get(k,0)-fb.get(k,0) for k in ('x','y','z')])))
    e['contact_force_n']=maxforce;result['contact_geometry_matched']=geometry_ok
    result['matched']=not result['missing'] and geometry_ok and all(k in e and math.isfinite(e[k]) and e[k]<=v for k,v in limits.items())
    (restored/'checkpoint_validation.json').write_text(json.dumps(result,indent=2)+'\n')
    return result


def matched_brake_test(mode, monza_replay=False, low_speed_ab=False, ab_library=None, monza_ab_source=None):
    """Private actuator fixtures; production Pure Pursuit is untouched."""
    import os
    import json
    import signal
    from gz.transport13 import Node as GzNode, NodeOptions
    from gz.msgs10.pose_v_pb2 import Pose_V
    from gz.msgs10.model_pb2 import Model
    from gz.msgs10.double_v_pb2 import Double_V
    from gz.msgs10.contacts_pb2 import Contacts
    from google.protobuf.json_format import MessageToDict
    os.environ.setdefault('GZ_IP','127.0.0.1')
    monza_replay=monza_replay or monza_ab_source is not None
    prefix='monza_low_brake_ab' if monza_ab_source else 'low_speed_brake_ab' if low_speed_ab else ('monza_replay' if monza_replay else 'matched_brake')
    run=ROOT/'.autonomy'/f'{prefix}_{mode}_{time.time_ns()}'
    run.mkdir(parents=True)
    env=os.environ.copy();env['GZ_PARTITION']='team07_matched_'+str(os.getpid())
    brake_time=6. if low_speed_ab else 10.;end_time=11. if low_speed_ab else 16.;reference=None
    checkpoint_source=os.environ.get('TEAM07_CHECKPOINT_SOURCE')
    if checkpoint_source and not monza_replay:raise ValueError('checkpoint source requires Monza replay')
    if monza_ab_source:
        if checkpoint_source:raise ValueError('Do not mix checkpoint source with low-speed Monza A/B')
        import hashlib
        reference=Path(monza_ab_source).resolve()
        if ROOT.resolve() not in reference.parents:raise ValueError('Monza source must be under team_07')
        data=json.loads((reference/'results.json').read_text())
        if data.get('track')!='monza' or data.get('mode')!='race' or data.get('cruise_speed',1.)>.5:raise ValueError('Monza source must be low-speed race')
        evidence=data.get('braking',{})
        if not .4<=evidence.get('body_speed_before_zero_backward_50ms_mps',0.)<=.55 or evidence.get('peak_body_speed_mps',1.)>.6:
            raise ValueError('Monza source needs measured approximately 0.5m/s body-speed evidence')
        commands=json.loads((reference/'command_generated.json').read_text())
        source_zero=data['braking']['zero_target_generated_stamp']
        source_poses=json.loads((reference/'body_poses.json').read_text())
        offset=source_poses[0][0]
        brake_time=source_zero-offset;end_time=brake_time+12.
        world_path=ROOT/'.runtime/urrc_track_gazebo/worlds/monza_race.sdf'
        sdf=ET.parse(world_path).getroot();world=sdf.find('world')
        model=ET.parse(ROOT/'model.sdf').getroot().find('model')
        grid=data['spawn_grid'];pose=model.find('pose')
        if pose is None:pose=ET.SubElement(model,'pose')
        pose.text=f"{grid['x']} {grid['y']} {grid['road_z']+.005} 0 0 {grid['yaw']}"
        env['GZ_SIM_RESOURCE_PATH']=str(ROOT/'.runtime/urrc_track_gazebo/models')+os.pathsep+env.get('GZ_SIM_RESOURCE_PATH','')
        replay=[(0.,0.,0.,0.,0.)]
        for row in commands:
            if row[0]>=source_zero:break
            if len(row)!=5 or not all(math.isfinite(v) for v in row):raise ValueError('invalid replay target')
            if abs(RADIUS*(row[3]+row[4])/2)>.50001:raise ValueError('replay exceeds 0.5m/s')
            replay.append((max(0.,row[0]-offset),row[3],row[4],row[1],row[2]))
        replay.append((brake_time,0.,0.,0.,0.))
        for tick in range(101):
            speed=min(.3,.15*tick*.02)
            replay.append((brake_time+5.+tick*.02,speed/RADIUS,speed/RADIUS,0.,0.))
        replay.append((brake_time+8.,0.,0.,0.,0.))
        with (run/'replay.csv').open('w') as stream:
            for row in replay:stream.write(','.join(map(repr,row))+'\n')
        (run/'reference.json').write_text(json.dumps(dict(source=str(reference),offset=offset,brake_time=brake_time,
            method='Recorded race targets, same complete warmup; Monza ground, not public stock-PI checkpoint restoration',
            world_sha256=hashlib.sha256(world_path.read_bytes()).hexdigest(),commands_sha256=hashlib.sha256((reference/'command_generated.json').read_bytes()).hexdigest()),indent=2))
    elif checkpoint_source:
        reference=Path(checkpoint_source).resolve()
        if ROOT.resolve() not in reference.parents:raise ValueError('checkpoint must be under team_07')
        import hashlib
        provenance=json.loads((reference/'provenance.json').read_text())
        for relative,digest in provenance.get('track_asset_sha256',{}).items():
            if hashlib.sha256((ROOT.parent/relative).read_bytes()).hexdigest()!=digest:raise ValueError('track assets differ from checkpoint capture')
        if mode=='candidate':
            accepted=json.loads((reference/'checkpoint_validation.json').read_text())
            import hashlib
            if not accepted.get('matched') or accepted.get('applied_input_sha256')!=hashlib.sha256((reference/'applied_inputs.csv').read_bytes()).hexdigest():raise ValueError('baseline checkpoint restoration not validated; candidate comparison blocked')
        brake_time=json.loads((reference/'checkpoint.json').read_text())['stamp'];end_time=brake_time+3.
        sdf=ET.parse(reference/'world.sdf').getroot();world=sdf.find('world');model=world.find("model[@name='team07_hybrid']");world.remove(model)
        env['GZ_SIM_RESOURCE_PATH']=str(ROOT.parent/'urrc_track_gazebo/models')+os.pathsep+env.get('GZ_SIM_RESOURCE_PATH','')
        (run/'replay.csv').write_text((reference/'applied_inputs.csv').read_text())
        (run/'reference.json').write_text(json.dumps(dict(source=str(reference),method='1kHz applied-input warmup reconstruction; no hidden solver state injection')))
    elif monza_replay:
        reference=ROOT/'.autonomy/monza_1794373567807542258'
        reference_result=json.loads((reference/'braking_analysis.json').read_text())
        brake_time=reference_result['zero_effort_target_stamp'];end_time=brake_time+6.
        sdf=ET.parse(reference/'world.sdf').getroot();world=sdf.find('world')
        model=world.find("model[@name='team07_hybrid']");world.remove(model)
        env['GZ_SIM_RESOURCE_PATH']=str(ROOT.parent/'urrc_track_gazebo/models')+os.pathsep+env.get('GZ_SIM_RESOURCE_PATH','')
        import bisect
        status=json.loads((reference/'status.json').read_text());stamps=[v['stamp'] for v in status]
        motor=[json.loads(l) for l in (reference/'drive_pi.jsonl').read_text().splitlines()]
        with (run/'replay.csv').open('w') as f:
            f.write('0,0,0,0,0\n')
            for row in motor:
                if row['stamp']>=brake_time:break
                i=bisect.bisect_right(stamps,row['stamp'])-1
                angle=status[i]['steering'] if i>=0 else 0.
                wl,wr=row['values'][0],row['values'][8]
                if abs(wl+wr)>1e-6:angle=-math.atan(WHEELBASE*2*(wr-wl)/(TRACK*(wl+wr)))
                left,right,_,_=targets(0.,angle)
                f.write(','.join(map(repr,(row['stamp'],row['values'][0],row['values'][8],left,right)))+'\n')
            # Hold final native Ackermann target during stop.
            f.write(','.join(map(repr,(brake_time,0.,0.,left,right)))+'\n')
        (run/'reference.json').write_text((ROOT/'.autonomy/monza_brake_reference.json').read_text())
    else:
        model=ET.fromstring(model_xml()).find('model')
        ET.SubElement(model,'pose').text='0 0 .005 0 0 0'
    for plugin in list(model.findall('plugin')):
        if plugin.get('filename') in ('gz-sim-joint-controller-system','gz-sim-pose-publisher-system') or plugin.get('name')=='team07::DrivePI' or (monza_replay and plugin.get('filename')=='gz-sim-joint-position-controller-system'):model.remove(plugin)
    for link in model.findall('link'):
        if not checkpoint_source:
            for sensor in list(link.findall('sensor')):link.remove(sensor)
    library=Path(ab_library).resolve() if low_speed_ab and ab_library else ROOT/'.install/team07_hybrid_racer/lib/libteam07_drive_pi.so'
    if ROOT.resolve() not in library.resolve().parents:raise ValueError('A/B library must stay under team_07')
    plugin=ET.SubElement(model,'plugin',filename=str(library),name='team07::DrivePI')
    ET.SubElement(plugin,'brake_integral_reset').text='true' if mode=='candidate' and not low_speed_ab else 'false'
    if monza_ab_source:
        ET.SubElement(plugin,'four_wheel_diagnostic').text='true'
        ET.SubElement(plugin,'capture_first_brake_only').text='true'
    if low_speed_ab:
        ET.SubElement(plugin,'zero_energy_guard').text='true' if mode=='candidate' else 'false'
        ET.SubElement(plugin,'capture_directory').text=str(run)
    ET.SubElement(plugin,'fixture_brake_time').text=str(brake_time)
    if monza_replay:ET.SubElement(plugin,'fixture_replay_file').text=str(run/'replay.csv')
    if checkpoint_source:
        ET.SubElement(plugin,'native_steering').text='true';ET.SubElement(plugin,'capture_directory').text=str(run)
    if os.environ.get('TEAM07_FOUR_WHEEL')=='1':
        if not checkpoint_source or mode!='baseline':raise ValueError('four-wheel diagnostic requires baseline checkpoint replay')
        ET.SubElement(plugin,'four_wheel_diagnostic').text='true'
    ET.SubElement(plugin,'fixture_speed').text='.5' if low_speed_ab else '3'
    pub=model.find("plugin[@filename='gz-sim-joint-state-publisher-system']")
    for joint in ('front_left_spin','front_right_spin'):
        if joint not in [j.text for j in pub.findall('joint_name')]:ET.SubElement(pub,'joint_name').text=joint
    pose_pub=ET.SubElement(model,'plugin',filename='gz-sim-pose-publisher-system',name='gz::sim::systems::PosePublisher')
    for key,value in dict(publish_model_pose='true',publish_link_pose='false',use_pose_vector_msg='true',topic=BASE+'/test_pose',update_frequency='200').items():ET.SubElement(pose_pub,key).text=value
    for side in ('left','right'):
        link=model.find(f"link[@name='rear_{side}_wheel']")
        existing=link.find("sensor[@name='contact_"+side+"']")
        if existing is not None:link.remove(existing)
        sensor=ET.SubElement(link,'sensor',name='contact_'+side,type='contact')
        ET.SubElement(sensor,'always_on').text='true';ET.SubElement(sensor,'update_rate').text='200'
        topic=BASE+'/contact/'+side;ET.SubElement(sensor,'topic').text=topic
        contact=ET.SubElement(sensor,'contact');ET.SubElement(contact,'collision').text='collision';ET.SubElement(contact,'topic').text=topic
    if not monza_replay:
        sdf=ET.Element('sdf',version='1.9');world=ET.SubElement(sdf,'world',name='matched_brake')
        physics=ET.SubElement(world,'physics',name='physics',type='ignored')
        ET.SubElement(physics,'max_step_size').text='.001';ET.SubElement(physics,'real_time_factor').text='1'
        for filename,name in [('gz-sim-physics-system','Physics'),('gz-sim-contact-system','Contact'),('gz-sim-scene-broadcaster-system','SceneBroadcaster')]:ET.SubElement(world,'plugin',filename=filename,name='gz::sim::systems::'+name)
        ground=ET.SubElement(world,'model',name='ground');ET.SubElement(ground,'static').text='true'
        ground.append(ET.fromstring('<link name="ground"><collision name="floor"><geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry><surface><friction><ode><mu>1</mu><mu2>1</mu2></ode></friction></surface></collision></link>'))
    elif world.find("plugin[@filename='gz-sim-contact-system']") is None:
        ET.SubElement(world,'plugin',filename='gz-sim-contact-system',name='gz::sim::systems::Contact')
    world.append(model);ET.ElementTree(sdf).write(run/'world.sdf')
    (run/'drive_pi_source.cc').write_text((ROOT/'drive_pi.cc').read_text())
    options=NodeOptions();options.partition=env['GZ_PARTITION'];transport=GzNode(options)
    poses=[];motors=[];joints=[];states=[];contact_times={}
    streams={name:(run/(name+'.jsonl')).open('w',buffering=1) for name in ('poses','drive_pi','joints','contacts','fixture_state')}
    def receive_pose(msg):
        for p in msg.pose:
            if p.name!='team07_hybrid':continue
            stamp=p.header.stamp.sec+p.header.stamp.nsec*1e-9
            if stamp==0:stamp=msg.header.stamp.sec+msg.header.stamp.nsec*1e-9
            q=p.orientation;yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
            roll=math.atan2(2*(q.w*q.x+q.y*q.z),1-2*(q.x*q.x+q.y*q.y))
            pitch=math.asin(max(-1.,min(1.,2*(q.w*q.y-q.z*q.x))))
            row=[stamp,p.position.x,p.position.y,p.position.z,yaw]
            if poses and stamp<=poses[-1][0]:continue
            poses.append(row);streams['poses'].write(json.dumps(row)+'\n')
            state=dict(stamp=stamp,position=row[1:4],rpy=[roll,pitch,yaw]);states.append(state);streams['fixture_state'].write(json.dumps(state)+'\n')
    def receive_motor(msg):
        if len(msg.data)<17:return
        row=dict(stamp=msg.data[0],values=list(msg.data[1:]));motors.append(row);streams['drive_pi'].write(json.dumps(row)+'\n')
    def receive_joint(msg):
        stamp=msg.header.stamp.sec+msg.header.stamp.nsec*1e-9
        if joints and stamp-joints[-1]['stamp']<.005:return
        row=dict(stamp=stamp,
            joints={j.name:dict(position=j.axis1.position,velocity=j.axis1.velocity) for j in msg.joint})
        joints.append(row);streams['joints'].write(json.dumps(row)+'\n')
    def receive_contact(msg,side):
        stamp=msg.header.stamp.sec+msg.header.stamp.nsec*1e-9
        if not checkpoint_source and not low_speed_ab and stamp-contact_times.get(side,-1)<.005:return
        contact_times[side]=stamp
        streams['contacts'].write(json.dumps(dict(stamp=stamp,side=side,message=MessageToDict(msg,preserving_proto_field_name=True)))+'\n')
    transport.subscribe(Pose_V,BASE+'/test_pose',receive_pose)
    transport.subscribe(Model,BASE+'/joint_states',receive_joint)
    transport.subscribe(Double_V,BASE+'/drive_pi',receive_motor)
    for side in ('left','right'):transport.subscribe(Contacts,BASE+'/contact/'+side,lambda msg,side=side:receive_contact(msg,side))
    error=None;proc=None;log=(run/'gazebo.log').open('w')
    try:
        proc=subprocess.Popen(['gz','sim','-s','-r','--headless-rendering','--seed','42','-v','3',str(run/'world.sdf')],env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        deadline=time.monotonic()+max(45.,end_time*3)
        while time.monotonic()<deadline:
            if proc.poll() is not None:raise RuntimeError('fixture Gazebo exited')
            if poses and poses[-1][0]>=end_time:break
            if states and (abs(states[-1]['rpy'][0])>.2 or abs(states[-1]['rpy'][1])>.18 or abs(math.atan2(math.sin(states[-1]['rpy'][2]-float(model.findtext('pose').split()[-1])),math.cos(states[-1]['rpy'][2]-float(model.findtext('pose').split()[-1]))))>.6):raise RuntimeError('attitude safety stop')
            time.sleep(.02)
        else:raise RuntimeError('fixture clock/telemetry timeout')
    except Exception as exc:error=str(exc)
    finally:
        if proc and proc.poll() is None:
            os.killpg(proc.pid,signal.SIGINT)
            try:proc.wait(timeout=5)
            except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL);proc.wait()
        for topic in (BASE+'/test_pose',BASE+'/joint_states',BASE+'/drive_pi',BASE+'/contact/left',BASE+'/contact/right'):transport.unsubscribe(topic)
        log.close()
        for stream in streams.values():stream.close()
    status=[]
    for motor in motors:
        v=motor['values'];state=min(states,key=lambda s:abs(s['stamp']-motor['stamp']))
        command=actual=0.
        if monza_replay and len(v)>=24:
            def central(left,right):
                tl,tr=math.tan(left),math.tan(right)
                return -math.atan(WHEELBASE*.5*(tl/(WHEELBASE+TRACK/2*tl)+tr/(WHEELBASE-TRACK/2*tr)))
            command=central(v[20],v[21]);actual=central(v[22],v[23])
        status.append(dict(stamp=motor['stamp'],speed_command=RADIUS*(v[0]+v[8])/2,steering=command,
            steering_actual=actual,steering_stamp=motor['stamp'],attitude=state['rpy'],safety_trigger=None,source='fixture native velocity target, not ROS autonomy'))
    (run/'status.json').write_text(json.dumps(status));(run/'poses.json').write_text(json.dumps(poses))
    (run/'results.json').write_text(json.dumps(dict(test='monza_low_speed_brake_restart_ab' if monza_ab_source else 'low_speed_brake_ab' if low_speed_ab else ('monza_trace_replay' if monza_replay else 'matched_uniform_fixture'),mode=mode,brake_request_stamp=brake_time,fixture_end_time=end_time,error=error,
        schedule='Recorded race warmup, zero 5s, restart at 0.15m/s2 to 0.3m/s for 3s, zero 4s' if monza_ab_source else '0..1 rest,1..2 ramp to 0.5,2..6 hold,6..11 zero; identical PI warmup' if low_speed_ab else 'recorded Monza motor/steering targets; stock PI until brake in BOTH' if monza_replay else '0..1 rest,1..7 ramp,7..10 hold,10..16 stop; stock PI until brake in BOTH',
        surface='original Monza world and initial spawn' if monza_replay else 'uniform plane, mu=1, zero slope',steering_target=None if monza_replay else 0.,seed=42,integration_step=.001),indent=2))
    if not error:
        if checkpoint_source:
            validation=compare_brake_checkpoint(reference,run)
            if mode=='baseline':(reference/'checkpoint_validation.json').write_text(json.dumps(validation,indent=2)+'\n')
            if not validation['matched']:error='checkpoint mismatch: performance comparison blocked'
        if not error and not monza_ab_source:
            analyze_braking(run)
            analyze_brake_fixture(run)
    if error:
        result=json.loads((run/'results.json').read_text());result['error']=error;(run/'results.json').write_text(json.dumps(result,indent=2))
    print('Artifacts:',run,flush=True)
    if error:raise RuntimeError(error)


def physics_test():
    """Isolated flat-world test; never opens or modifies competition worlds."""
    import os
    import threading
    from gz.transport13 import Node as GzNode, NodeOptions
    from gz.msgs10.double_pb2 import Double
    from gz.msgs10.pose_v_pb2 import Pose_V
    partition = 'team07_physics_test_' + str(os.getpid())
    options = NodeOptions()
    options.partition = partition
    transport = GzNode(options)
    latest = []
    guard = threading.Lock()

    def receive(msg):
        for pose in msg.pose:
            if pose.name == 'team07_hybrid':
                q = pose.orientation
                yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
                with guard:
                    latest[:] = [pose.position.x, pose.position.y, pose.position.z, yaw]

    transport.subscribe(Pose_V, BASE+'/test_pose', receive)
    pubs = [transport.advertise(BASE+'/'+s, Double) for s in ('left_steer','right_steer','left_drive','right_drive')]
    model = model_xml().replace('<model name="team07_hybrid">', '<model name="team07_hybrid"><pose>0 0 .005 0 0 0</pose>')
    model = model.replace('</model>', '<plugin filename="gz-sim-pose-publisher-system" name="gz::sim::systems::PosePublisher"><publish_model_pose>true</publish_model_pose><publish_link_pose>false</publish_link_pose><use_pose_vector_msg>true</use_pose_vector_msg><topic>'+BASE+'/test_pose</topic><update_frequency>50</update_frequency></plugin></model>')
    world = ROOT/'physics_test.sdf'
    world.write_text('<sdf version="1.9"><world name="team07_test"><physics name="physics" type="ignored"><max_step_size>.001</max_step_size><real_time_factor>1</real_time_factor></physics><plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/><plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/><model name="ground"><static>true</static><link name="ground"><collision name="floor"><geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry></collision></link></model>'+model.removeprefix('<sdf version="1.9">').removesuffix('</sdf>')+'</world></sdf>')
    env = os.environ.copy()
    env['GZ_PARTITION'] = partition  # Only this private test server; control() inherits operator environment.
    def snapshot():
        with guard:
            if not latest:
                raise RuntimeError('No physics pose received')
            return latest.copy()
    def command(speed, steering, duration):
        end = time.monotonic()+duration
        while time.monotonic() < end:
            for pub, value in zip(pubs, targets(speed, steering)):
                pub.publish(Double(data=value))
            if proc.poll() is not None:
                raise RuntimeError('Test server exited early')
            time.sleep(.02)
    with (ROOT/'physics_test.log').open('w') as log:
        proc = subprocess.Popen(['gz','sim','-s','-r','-v','3',str(world)],env=env,stdout=log,stderr=subprocess.STDOUT)
        try:
            deadline = time.monotonic()+15
            while not latest and time.monotonic()<deadline:
                time.sleep(.1)
            command(0,0,1)
            for label, speed, steer in [('forward',.5,0),('reverse',-.5,0),('left',.5,-.3),('right',.5,.3)]:
                command(0,0,.5)
                before=snapshot()
                command(speed,steer,2)
                after=snapshot()
                dx,dy=after[0]-before[0],after[1]-before[1]
                forward=dx*math.cos(before[3])+dy*math.sin(before[3])
                dyaw=math.atan2(math.sin(after[3]-before[3]),math.cos(after[3]-before[3]))
                print(f'{label}: longitudinal={forward:.3f}m yaw_change={dyaw:.3f}rad z={after[2]:.4f}m',flush=True)
                assert forward*speed > .05, label+' translation failed'
                if steer:
                    assert dyaw*steer < -.01, label+' steering failed'
                assert abs(after[2]) < .03, 'Unexpected lift/fall'
            command(0,0,1)
            print('PASS: four physical driving directions on isolated flat ground',flush=True)
        finally:
            proc.terminate()
            try:
                proc.wait(timeout=8)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.wait()


def control_test():
    """Exercise actual ROS subscriptions, Gazebo publication and stale-command stop."""
    import os
    import rclpy
    from std_msgs.msg import Float32
    from gz.transport13 import Node as GzNode, NodeOptions
    from gz.msgs10.double_pb2 import Double
    rclpy.init()
    tester = rclpy.create_node('team07_control_test')
    options = NodeOptions()
    options.partition = 'team07_control_test_' + str(os.getpid())
    gz = GzNode(options)
    test_topic = BASE+'/test_'+str(os.getpid())
    observed = [None]*4
    for i, suffix in enumerate(('left_steer','right_steer','left_drive','right_drive')):
        def cb(msg, index=i):
            observed[index] = msg.data
        gz.subscribe(Double, BASE+'/'+suffix, cb)
    speed_pub = tester.create_publisher(Float32, test_topic+'/speed', 10)
    steer_pub = tester.create_publisher(Float32, test_topic+'/steering', 10)
    with (ROOT/'control_test.log').open('w') as log:
        env = os.environ.copy()
        env['GZ_PARTITION'] = options.partition
        child = subprocess.Popen(['python3','-B',str(Path(__file__).resolve()),'--ros-args','-p','speed_topic:='+test_topic+'/speed','-p','steering_topic:='+test_topic+'/steering'], env=env, stdout=log, stderr=subprocess.STDOUT)
        try:
            deadline = time.monotonic()+10
            while time.monotonic()<deadline and (not speed_pub.get_subscription_count() or not steer_pub.get_subscription_count()):
                rclpy.spin_once(tester, timeout_sec=.1)
            assert speed_pub.get_subscription_count() and steer_pub.get_subscription_count(), 'ROS subscriptions unavailable'
            for speed, steer in [(1.,0.),(-1.,0.),(1.,-.3),(1.,.3),(100.,100.),(float('nan'),0.)]:
                end = time.monotonic()+.7
                while time.monotonic()<end:
                    speed_pub.publish(Float32(data=speed))
                    steer_pub.publish(Float32(data=steer))
                    rclpy.spin_once(tester, timeout_sec=.02)
                expected = targets(speed,steer)
                assert all(v is not None and abs(v-e)<1e-5 for v,e in zip(observed,expected)), (observed,expected)
            end = time.monotonic()+.8
            while time.monotonic()<end:
                rclpy.spin_once(tester, timeout_sec=.02)
            assert observed == [0.,0.,0.,0.], 'Command timeout failed'
            print('PASS: ROS speed/steering -> Gazebo commands; reverse, turns, limits, NaN stop, timeout stop')
        finally:
            child.send_signal(__import__('signal').SIGINT)
            try:
                child.wait(timeout=5)
            except subprocess.TimeoutExpired:
                child.kill()
                child.wait()
            tester.destroy_node()
            rclpy.shutdown()


def sensor_test():
    """Real Gazebo rendering -> ROS bridge -> the same sensor validation buffer.

    Test-only world geometry is never supplied to the driving controller.
    """
    import os
    import signal
    import json
    import rclpy
    from rclpy.parameter import Parameter
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image, CameraInfo, LaserScan, PointCloud2, Imu
    from tf2_msgs.msg import TFMessage
    from rclpy.qos import QoSProfile, DurabilityPolicy
    domain = 120 + os.getpid()%60
    env = os.environ.copy()
    env['ROS_DOMAIN_ID'] = str(domain)
    env['GZ_PARTITION'] = 'team07_sensor_test_'+str(os.getpid())
    env['ROS_LOG_DIR'] = str(ROOT/'.logs')
    rclpy.init(domain_id=domain)
    node = rclpy.create_node('team07_sensor_test', parameter_overrides=[Parameter('use_sim_time',value=True)])
    buffer = SensorBuffer()
    counts, stamps = {}, {}
    frames = {}
    tf_subscription = node.create_subscription(TFMessage,'/tf_static',
        lambda msg: frames.update({t.child_frame_id:t for t in msg.transforms}),
        QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    def receive(kind,msg):
        if buffer.put(kind,msg):
            counts[kind] = counts.get(kind,0)+1
            stamps.setdefault(kind,[]).append(buffer.samples[kind][-1][0])
    subscriptions = []
    for kind, cls, suffix in [('camera',Image,'camera/image'),('camera_info',CameraInfo,'camera/camera_info'),
            ('scan',LaserScan,'scan'),('points',PointCloud2,'lidar3d/points'),('imu',Imu,'imu')]:
        subscriptions.append(node.create_subscription(cls,BASE+'/'+suffix,
            lambda msg,key=kind: receive(key,msg),qos_profile_sensor_data))
    world = ROOT/'sensor_test.sdf'
    vehicle = model_xml().removeprefix('<sdf version="1.9">').removesuffix('</sdf>')
    vehicle = vehicle.replace('<model name="team07_hybrid">','<model name="team07_hybrid"><pose>0 0 .005 0 0 0</pose>')
    world.write_text('<sdf version="1.9"><world name="team07_sensors"><physics name="physics" type="ignored"><max_step_size>.001</max_step_size><real_time_factor>1</real_time_factor></physics>'
        '<plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>'
        '<plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors"><render_engine>ogre2</render_engine></plugin>'
        '<light name="sun" type="directional"><pose>0 0 10 0 0 0</pose><diffuse>1 1 1 1</diffuse><direction>-.2 .1 -1</direction></light>'
        '<model name="floor"><static>true</static><link name="floor"><collision name="floor"><geometry><box><size>20 20 .1</size></box></geometry><pose>0 0 -.05 0 0 0</pose></collision><visual name="floor"><geometry><box><size>20 20 .1</size></box></geometry><pose>0 0 -.05 0 0 0</pose></visual></link></model>'
        '<model name="target"><static>true</static><pose>2 0 .7 0 0 0</pose><link name="target"><collision name="target"><geometry><box><size>.5 .6 1.4</size></box></geometry></collision><visual name="target"><geometry><box><size>.5 .6 1.4</size></box></geometry><material><diffuse>1 0 0 1</diffuse><ambient>1 0 0 1</ambient></material></visual></link></model>'
        +vehicle+'</world></sdf>')
    processes = []
    try:
        with (ROOT/'sensor_test.log').open('w') as simlog, (ROOT/'sensor_bridge_test.log').open('w') as bridgelog:
            processes.append(subprocess.Popen(['gz','sim','-s','-r','--headless-rendering','-v','3',str(world)],env=env,stdout=simlog,stderr=subprocess.STDOUT,start_new_session=True))
            processes.append(subprocess.Popen(['ros2','run','ros_gz_bridge','parameter_bridge',*sensor_bridge_args(clock=True)],env=env,stdout=bridgelog,stderr=subprocess.STDOUT,start_new_session=True))
            processes.append(subprocess.Popen(['python3','-B',str(Path(__file__).resolve()),'--ros-args','-p','use_sim_time:=true'],env=env,stdout=bridgelog,stderr=subprocess.STDOUT,start_new_session=True))
            deadline = time.monotonic()+45
            paired = False
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=.05)
                paired |= buffer.camera_scan_pair() is not None
                if all(counts.get(k,0)>=12 for k in ('camera','camera_info','scan','points','imu')) and paired and node.get_clock().now().nanoseconds > 0 and len(frames)>=4:
                    break
                if any(proc.poll() is not None for proc in processes):
                    raise RuntimeError('Sensor server/bridge exited; inspect sensor_test.log and sensor_bridge_test.log')
            assert all(counts.get(k,0)>=12 for k in ('camera','camera_info','scan','points','imu')), f'Missing/invalid sensors: {counts}, rejected={buffer.rejected}'
            assert paired, 'No synchronized camera/scan pair'
            assert node.get_clock().now().nanoseconds > 0, 'No simulation clock'
            assert {'team07_camera','team07_camera_optical','team07_lidar','team07_imu'} <= frames.keys(), 'Sensor TF missing'
            assert abs(frames['team07_lidar'].transform.translation.z-.054)<1e-9
            camera = buffer.samples['camera'][-1][2]
            scan = buffer.samples['scan'][-1][2]
            cloud = buffer.samples['points'][-1][2]
            info = buffer.samples['camera_info'][-1][2]
            assert len(set(camera.data[::97])) > 2, 'Uniform/empty rendered image'
            assert len(scan.ranges) == 720, 'Invalid planar scan shape'
            front = [v for i,v in enumerate(scan.ranges) if abs(scan.angle_min+i*scan.angle_increment)<.04 and math.isfinite(v)]
            assert front and abs(min(front)-1.75)<.1, f'Known target range mismatch: front={front[:8]} angle_min={scan.angle_min} inc={scan.angle_increment} imu={buffer.attitude()}'
            assert cloud.width*cloud.height == 360*16, 'Invalid 3D scan shape'
            assert info.width == 640 and info.height == 360 and info.k[0]>0, 'Invalid camera calibration'
            expected_frames = dict(camera='team07_camera_optical',scan='team07_lidar',points='team07_lidar',imu='team07_imu')
            for kind,frame in expected_frames.items():
                assert buffer.samples[kind][-1][2].header.frame_id == frame, (kind,buffer.samples[kind][-1][2].header.frame_id)
            rates = {k:round((len(v)-1)/(v[-1]-v[0]),1) for k,v in stamps.items() if len(v)>1 and v[-1]>v[0]}
            report = dict(counts=counts,rates_sim_hz=rates,frames=expected_frames,front_range_m=min(front),
                imu_rpy_rad=buffer.attitude(),camera_scan_paired=paired,tf_frames=list(frames),rejected=buffer.rejected)
            (ROOT/'sensor_test_results.json').write_text(json.dumps(report,indent=2)+'\n')
            print('PASS sensor ROS reception:',json.dumps(report),flush=True)
    finally:
        for proc in reversed(processes):
            if proc.poll() is None:
                os.killpg(proc.pid,signal.SIGINT)
                try:
                    proc.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    os.killpg(proc.pid,signal.SIGKILL)
                    proc.wait()
        node.destroy_node()
        rclpy.shutdown()


def start_test(track, capture_only=False, camera_fault=None):
    """Private operator session; original resources stay read-only. No truth in controller."""
    import os
    import sys
    import json
    import shutil
    import signal
    import numpy as np
    import cv2
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.parameter import Parameter
    from sensor_msgs.msg import Image
    from std_msgs.msg import String, Float32
    from gz.transport13 import Node as GzNode, NodeOptions
    from gz.msgs10.pose_v_pb2 import Pose_V
    package = ROOT.parent/'urrc_track_gazebo'
    run = ROOT/'.validation'/(track+('_'+camera_fault if camera_fault else ''))
    run.mkdir(parents=True,exist_ok=True)
    runtime = ROOT/'.validation'/track/'urrc_track_gazebo'
    if not runtime.exists():
        shutil.copytree(package,runtime,ignore=shutil.ignore_patterns('__pycache__'))
    # The same first-run preparation as the official launcher, confined to its runtime copy.
    if not (runtime/'worlds'/f'{track}_signal_active.sdf').exists():
        subprocess.run([sys.executable,'-B','-c',
            'import sys; from pathlib import Path; p=Path(sys.argv[1]); sys.path.insert(0,str(p)); from urrc_tracks.v10_assets import enhance_track; enhance_track(p,sys.argv[2])',
            str(runtime),track],check=True)
    grid = json.loads((runtime/'tracks/processed'/f'{track}.json').read_text())['grid'][0]
    env = os.environ.copy()
    domain = 130+os.getpid()%60
    env.update(ROS_DOMAIN_ID=str(domain),GZ_PARTITION='team07_start_'+str(os.getpid()),
        GZ_SIM_RESOURCE_PATH=str(runtime/'models')+os.pathsep+env.get('GZ_SIM_RESOURCE_PATH',''),
        ROS_LOG_DIR=str(ROOT/'.logs'))
    rclpy.init(domain_id=domain)
    node = rclpy.create_node('team07_start_test',parameter_overrides=[Parameter('use_sim_time',value=True)])
    frames, saved, statuses, poses = [], set(), [], []
    fault_active = False
    relay = node.create_publisher(Image,BASE+'/test_camera',10) if camera_fault else None
    speed_pub = node.create_publisher(Float32,BASE+'/speed',10)
    steer_pub = node.create_publisher(Float32,BASE+'/steering',10)
    if camera_fault:
        def demand_motion():
            speed_pub.publish(Float32(data=.5));steer_pub.publish(Float32(data=0.))
        demand_timer = node.create_timer(.05,demand_motion)
    options = NodeOptions();options.partition = env['GZ_PARTITION']
    transport = GzNode(options)
    def pose(msg):
        for p in msg.pose:
            if p.name=='team07_hybrid':
                if not poses:(run/'pose_sample.txt').write_text(str(msg))
                stamp = p.header.stamp.sec+p.header.stamp.nsec*1e-9
                if stamp==0:stamp = msg.header.stamp.sec+msg.header.stamp.nsec*1e-9
                if stamp==0:stamp = node.get_clock().now().nanoseconds*1e-9
                poses.append((stamp,p.position.x,p.position.y))
    transport.subscribe(Pose_V,BASE+'/test_pose',pose)
    status_subscription = node.create_subscription(String,BASE+'/start/status',lambda msg: statuses.append(json.loads(msg.data)),10)
    def image(msg):
        nonlocal fault_active
        rgb = np.frombuffer(msg.data,dtype=np.uint8).reshape(msg.height,msg.step)[:,:msg.width*3].reshape(msg.height,msg.width,3)
        if msg.encoding == 'bgr8':
            rgb = rgb[:,:,::-1]
        stamp = msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        mask = ((rgb[:,:,0]>90)&(rgb[:,:,0]>rgb[:,:,1].astype(float)*1.8)&(rgb[:,:,0]>rgb[:,:,2].astype(float)*1.8)).astype(np.uint8)
        n, labels, stats, centers = cv2.connectedComponentsWithStats(mask)
        blobs = [list(map(int,s)) for s in stats[1:] if 3<=s[4]<=400 and s[1]<220]
        count = len(blobs)
        observation = lamp_observation(msg)
        if camera_fault:
            fault_active |= bool(statuses and statuses[-1]['state']=='ALL_RED')
            if not fault_active:relay.publish(msg)
            elif camera_fault=='occlude':
                import copy
                damaged = copy.deepcopy(msg)
                damaged.data = bytes(len(msg.data))
                relay.publish(damaged)
        frames.append(dict(stamp=stamp,red_blobs=blobs,lenses=None if observation is None else observation[0]))
        if count not in saved:
            cv2.imwrite(str(run/f'red_{count}.png'),cv2.cvtColor(rgb,cv2.COLOR_RGB2BGR))
            saved.add(count)
    subscription = node.create_subscription(Image,BASE+'/camera/image',image,qos_profile_sensor_data)
    logs, processes = [], []
    def launch(cmd,name):
        log = (run/name).open('w');logs.append(log)
        proc = subprocess.Popen(cmd,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        processes.append(proc)
        return proc
    try:
        launch(['gz','sim','-s','-r','--headless-rendering','-v','3',str(runtime/'worlds'/f'{track}_race.sdf')],'gazebo.log')
        launch(['ros2','run','ros_gz_bridge','parameter_bridge',*sensor_bridge_args(clock=True)],'bridge.log')
        vehicle = run/'vehicle.sdf'
        vehicle.write_text(model_xml().replace('</model>',f'<plugin filename="gz-sim-pose-publisher-system" name="gz::sim::systems::PosePublisher"><publish_model_pose>true</publish_model_pose><publish_link_pose>false</publish_link_pose><use_pose_vector_msg>true</use_pose_vector_msg><topic>{BASE}/test_pose</topic><update_frequency>100</update_frequency></plugin></model>'))
        # Ego pose is test instrumentation only, never bridged or provided to Racer.
        subprocess.run(['ros2','run','ros_gz_sim','create','-world',track,'-file',str(vehicle),'-name','team07_hybrid',
            '-x',str(grid['x']),'-y',str(grid['y']),'-z',str(grid['road_z']+.005),'-Y',str(grid['yaw'])],env=env,check=True,timeout=60,stdout=subprocess.DEVNULL)
        if not capture_only:
            cmd = [sys.executable,'-B',str(Path(__file__).resolve()),'--ros-args','-p','use_sim_time:=true','-p','start_gate:=true']
            if camera_fault:cmd.extend(['-r',BASE+'/camera/image:='+BASE+'/test_camera'])
            launch(cmd,'controller.log')
        deadline = time.monotonic()+60
        while (len(frames)<10 or (not capture_only and not statuses)) and time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.05)
        assert len(frames)>=10,'No vehicle camera images'
        lights = launch([sys.executable,'-B',str(runtime/'scripts/traffic_lights_v10.py'),'--track',track,'--seed','7'],'lights.log')
        deadline = time.monotonic()+80
        ended = None
        while time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.05)
            if lights.poll() is not None:
                if ended is None: ended = time.monotonic()
                if time.monotonic()-ended>3:break
        assert lights.poll() == 0,'Operator light sequence failed'
        (run/'capture.json').write_text(json.dumps(frames,indent=2)+'\n')
        if not capture_only:
            (run/'status.json').write_text(json.dumps(statuses,indent=2)+'\n')
            (run/'poses.json').write_text(json.dumps(poses)+'\n')
            assert statuses,'No controller state received'
            result = statuses[-1]
            if camera_fault:
                assert fault_active and result['reason'],result
                assert not any(s['authorized'] or s['command_speed']!=0 for s in statuses),result
                red_poses = [p for p in poses if p[0]>=result['times']['first_red']]
                drift = max(math.hypot(p[1]-red_poses[0][1],p[2]-red_poses[0][2]) for p in red_poses)
                assert drift<.002,f'Fault-test rolling {drift:.4f}m'
                report = dict(track=track,fault=camera_fault,blocked=True,pre_go_drift_m=drift,**result)
                (run/'start_results.json').write_text(json.dumps(report,indent=2)+'\n')
                print('PASS camera fault interlock:',json.dumps(report),flush=True)
                return
            assert result['state']=='RACING' and not result['reason'],result
            times = result['times']
            assert result['seen_counts']==[1,2,3,4,5],result
            assert all(s['command_speed']==0 for s in statuses if s['state'] not in ('GO','RACING'))
            red_poses = [p for p in poses if times['first_red']<=p[0]<times['go']]
            assert len(red_poses)>2,'No stationary measurements'
            drift = max(math.hypot(p[1]-red_poses[0][1],p[2]-red_poses[0][2]) for p in red_poses)
            assert drift<.002, f'Pre-start rolling {drift:.4f}m'
            baseline = red_poses[-1]
            moving = next((p[0] for p in poses if p[0]>=times['go'] and math.hypot(p[1]-baseline[1],p[2]-baseline[2])>.01),None)
            assert moving is not None,'No physical launch movement'
            assert moving-times['first_red']<=20,'20s start limit exceeded'
            report = dict(track=track,**result,actual_movement=moving,
                elapsed_to_movement=moving-times['first_red'],pre_go_drift_m=drift,
                movement_measurement='Independent ego pose displacement > 1cm; test-only')
            (run/'start_results.json').write_text(json.dumps(report,indent=2)+'\n')
            print('PASS camera start:',json.dumps(report),flush=True)
        print(f'{track}: {len(frames)} images, blob counts {sorted(saved)}, artifacts {run}',flush=True)
    finally:
        for proc in reversed(processes):
            if proc.poll() is None:
                os.killpg(proc.pid,signal.SIGINT)
                try:proc.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    os.killpg(proc.pid,signal.SIGKILL);proc.wait()
        for log in logs:log.close()
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


def vision_test():
    """Adversarial gating tests using actual rendered lens snapshots from both tracks."""
    import cv2
    import numpy as np
    import json
    from sensor_msgs.msg import Image
    def message(rgb):
        msg = Image();msg.height,msg.width = rgb.shape[:2]
        msg.step = msg.width*3;msg.encoding = 'rgb8';msg.data = rgb.tobytes()
        return msg
    checked = []
    for track in ('monza','hungaroring'):
        images = [cv2.cvtColor(cv2.imread(str(ROOT/'.validation'/track/f'red_{n}.png')),cv2.COLOR_BGR2RGB) for n in range(6)]
        observations = []
        for n,rgb in enumerate(images):
            o = lamp_observation(message(rgb))
            assert o is not None and sum(o[0])==n,(track,n,o)
            observations.append(o)
            dim = lamp_observation(message((rgb.astype(float)*.6).astype(np.uint8)))
            assert dim is not None and sum(dim[0])==n,(track,'dim',n)
        assert lamp_observation(message(np.zeros_like(images[0]))) is None
        partial = images[5].copy()
        x,y = map(round,observations[5][1][2]);partial[y-9:y+10,x-9:x+10] = 0
        assert lamp_observation(message(partial)) is None
        def feed(sequence):
            gate = StartGate();stamp = 0.
            for n in sequence:
                for _ in range(4):
                    stamp += .05
                    observation = None if n is None else (observations[n][0],observations[5][1])
                    gate.observe(observation,stamp,stamp)
            return gate,stamp
        gate,stamp = feed([0,1,2,3,4,5,0])
        assert gate.state=='GO' and gate.seen_counts==[1,2,3,4,5]
        gate.watchdog(stamp+10,stamp+10);gate.observe(None,stamp+11,stamp+11)
        assert gate.authorized,'GO must latch'
        for sequence in ([0,0,0],[5,0],[1,2,3,4,0],[1,3,4,5,0],[1,2,3,4,5,1,0],[1,2,3,4,5,None,0]):
            blocked,_ = feed(sequence)
            assert not blocked.authorized,(track,sequence,blocked.report())
        blocked,stamp = feed([0,1,2,3,4,5])
        blocked.watchdog(stamp+.7,stamp+.7)
        for _ in range(4):
            stamp+=.05;blocked.observe(observations[0],stamp+.7,stamp+.7)
        assert not blocked.authorized and blocked.reason
        late,stamp = feed([0,1]);late.watchdog(stamp+21,stamp+21)
        assert not late.authorized and late.reason=='start_deadline_exceeded'
        checked.append(track)
    report = dict(passed=True,tracks=checked,cases=['individual 0..5 lenses','60% brightness',
        'blank image','one-lens occlusion','all-dark initial scene','missing sequence step',
        'partial extinction','camera loss','20s timeout','GO latch'])
    (ROOT/'.validation/vision_results.json').write_text(json.dumps(report,indent=2)+'\n')
    print('PASS vision safety:',json.dumps(report))




def finish_crossed(previous, current, origin, yaw, half_width):
    """Independent evaluator only: interpolate actual forward finish-plane crossing."""
    nx,ny=math.cos(yaw),math.sin(yaw)
    before=(previous[1]-origin[0])*nx+(previous[2]-origin[1])*ny
    after=(current[1]-origin[0])*nx+(current[2]-origin[1])*ny
    if not before<0<=after or after<=before:return False
    fraction=-before/(after-before)
    x=previous[1]+fraction*(current[1]-previous[1])-origin[0]
    y=previous[2]+fraction*(current[2]-previous[2])-origin[1]
    heading=abs(math.atan2(math.sin(current[4]-yaw),math.cos(current[4]-yaw)))
    return abs(-ny*x+nx*y)<half_width and heading<.4


def analyze_braking(run):
    """Motor PI values come from the physics-step actuator; pose is analysis only."""
    import csv
    import json
    run=Path(run)
    states=json.loads((run/'status.json').read_text())
    poses=json.loads((run/'poses.json').read_text())
    motors=[json.loads(l) for l in (run/'drive_pi.jsonl').read_text().splitlines()] if (run/'drive_pi.jsonl').exists() else []
    request=json.loads((run/'results.json').read_text())['brake_request_stamp']
    if request is None:
        result=dict(valid=False,reason='No brake request recorded; test terminated before stop acknowledgement')
        (run/'braking_analysis.json').write_text(json.dumps(result,indent=2)+'\n')
        return result
    zero=next((m['stamp'] for m in motors if m['stamp']>=request and abs(m['values'][0])+abs(m['values'][8])<1e-9),None)
    if zero is None:zero=next((s['stamp'] for s in states if s['stamp']>=request and s['speed_command']==0),None)
    body=[]
    for a,b in zip(poses,poses[1:]):
        dt=b[0]-a[0]
        if dt<=0:continue
        dx,dy=b[1]-a[1],b[2]-a[2];heading=(a[4]+b[4])/2
        body.append(dict(stamp=(a[0]+b[0])/2,start_stamp=a[0],end_stamp=b[0],speed=math.hypot(dx,dy)/dt,
            forward=(dx*math.cos(heading)+dy*math.sin(heading))/dt,distance=math.hypot(dx,dy),
            yaw_rate=math.atan2(math.sin(b[4]-a[4]),math.cos(b[4]-a[4]))/dt,dt=dt))
    aligned=[]
    for motor in motors:
        t=motor['stamp'];v=motor['values']
        if len(v)<16:continue
        b=min(body,key=lambda b:abs(b['stamp']-t))
        s=min(states,key=lambda s:abs(s['stamp']-t))
        row=dict(stamp=t,body_stamp=b['stamp'],body_speed=b['speed'],body_forward_speed=b['forward'],
            yaw_rate_truth=b['yaw_rate'],speed_command=s['speed_command'],steering_command=s['steering'],
            steering_actual=s.get('steering_actual'),steering_actual_stamp=s.get('steering_stamp'),pitch=s['attitude'][1] if s.get('attitude') else None,
            safety_trigger=json.dumps(s.get('safety_trigger')))
        for i,side in enumerate(('left','right')):
            for j,label in enumerate(('target_omega','actual_omega','torque_command','integral_torque','proportional_torque','zero_mode','integral_frozen','reset_enabled')):
                row[side+'_'+label]=v[8*i+j]
        if len(v)>=20:row.update(left_raw_pi_torque=v[16],right_raw_pi_torque=v[17],balanced_brake=v[18],raw_total_torque=v[19])
        aligned.append(row)
    if aligned:
        with (run/'braking_aligned.csv').open('w') as f:
            writer=csv.DictWriter(f,fieldnames=list(aligned[0]));writer.writeheader();writer.writerows(aligned)
    after=[dict(b,active_dt=b['end_stamp']-max(b['start_stamp'],zero)) for b in body if zero is not None and b['end_stamp']>zero]
    first_rest=next((b['stamp'] for b in after if b['speed']<.02 or b['forward']<=0),None)
    first_reverse=next((b['stamp'] for b in after if b['forward']<-.01),None)
    start=min(body,key=lambda b:abs(b['stamp']-zero)) if zero is not None and body else None
    motor_after=[r for r in aligned if zero is not None and r['stamp']>=zero]
    reverse_driving=[r for r in motor_after if any(r[side+'_actual_omega']<-.2 and r[side+'_torque_command']<-.005 for side in ('left','right'))]
    result=dict(zero_effort_target_stamp=zero,stop_request_stamp=request,
        speed_at_zero=start['speed'] if start else None,
        speed_before_zero=next((b['speed'] for b in reversed(body) if zero is not None and b['end_stamp']<=zero),None),
        first_body_rest_stamp=first_rest,
        first_reverse_stamp=first_reverse,forward_braking_distance_m=sum(max(0.,b['forward']*b['active_dt']) for b in after if first_rest is None or b['stamp']<=first_rest),
        path_distance_to_first_rest_m=sum(b['speed']*b['active_dt'] for b in after if first_rest is None or b['stamp']<=first_rest),
        peak_braking_yaw_rate=max((abs(b['yaw_rate']) for b in after),default=0),
        reverse_distance_m=sum(max(0.,-b['forward']*b['active_dt']) for b in after),
        peak_reverse_speed=max((-b['forward'] for b in after),default=0),
        peak_stop_integral_torque=max((abs(r[side+'_integral_torque']) for r in motor_after for side in ('left','right')),default=None),
        reverse_motoring_samples=len(reverse_driving),first_reverse_motoring=reverse_driving[0] if reverse_driving else None,
        pi_source='exact gz::math::PID.Errors, 1kHz controller, sampled at 200Hz' if motors else 'not recorded',
        torque_source='exact JointForceCmd actuator input; not a ground-contact reaction torque measurement' if motors else 'not recorded',
        zero_state=min(aligned,key=lambda r:abs(r['stamp']-zero)) if aligned and zero is not None else None)
    result['zero_velocity_target_stamp']=zero
    braking=[r for r in motor_after if r['body_forward_speed']>.5]
    if braking:
        import numpy as np
        wheel=np.asarray([[r[side+'_actual_omega']*RADIUS for side in ('left','right')] for r in braking])
        velocity=np.asarray([r['body_forward_speed'] for r in braking])
        torque=np.asarray([[r[side+'_torque_command'] for side in ('left','right')] for r in braking])
        locked=abs(wheel)<.2*velocity[:,None]
        result.update(near_lock_definition='abs(wheel_speed)<0.2*forward_body_speed, body>0.5m/s; analysis only',
            near_lock_fraction=locked.mean(axis=0).tolist(),
            mean_wheel_body_gap=(velocity[:,None]-wheel).mean(axis=0).tolist(),
            mean_left_right_wheel_gap=float(np.mean(abs(wheel[:,0]-wheel[:,1]))),
            mean_left_right_torque_gap=float(np.mean(abs(torque[:,0]-torque[:,1]))),
            mean_total_braking_torque=float(np.mean(-torque.sum(axis=1))),
            pi_saturation_fraction=[sum(abs(r[side+'_proportional_torque']+r[side+'_integral_torque'])>=.3-1e-9 for r in braking)/len(braking) for side in ('left','right')])
        balanced=[r for r in braking if r.get('balanced_brake',0) and r['left_actual_omega']>.5 and r['right_actual_omega']>.5
                  and r.get('left_raw_pi_torque',1)<=0 and r.get('right_raw_pi_torque',1)<=0]
        result['balanced_total_torque_error_max']=max((abs(r['left_torque_command']+r['right_torque_command']-r['raw_total_torque']) for r in balanced),default=None)
        result['balanced_preserving_samples']=len(balanced)
    if zero is not None and (run/'joints.jsonl').exists():
        joints=[json.loads(l) for l in (run/'joints.jsonl').read_text().splitlines()]
        angles=[]
        for j in joints:
            if zero-.05<=j['stamp']<=zero:
                values=j['joints'];left=math.tan(values['front_left_steer']['position']);right=math.tan(values['front_right_steer']['position'])
                angle=-math.atan(WHEELBASE*.5*(left/(WHEELBASE+TRACK/2*left)+right/(WHEELBASE-TRACK/2*right)))
                angles.append((j['stamp'],angle))
        if angles:
            result['pre_zero_joint_steering']=angles[-1]
            result['pre_zero_joint_steering_mean_50ms']=sum(a[1] for a in angles)/len(angles)
    (run/'braking_analysis.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    return result


def analyze_telemetry(run, track='monza'):
    """Offline verifier only. Gazebo truth and official route never enter control."""
    import csv
    import json
    import numpy as np
    run=Path(run)
    statuses=json.loads((run/'status.json').read_text())
    poses=np.asarray(json.loads((run/'poses.json').read_text()),dtype=float)
    joint_path=run/'joints.jsonl'
    joints=[json.loads(line) for line in joint_path.read_text().splitlines()] if joint_path.exists() else []
    route=np.genfromtxt(ROOT.parent/'urrc_track_gazebo/tracks/processed'/f'{track}_centerline.csv',delimiter=',',names=True)
    xy=np.column_stack((route['x_m'],route['y_m']))
    vectors=np.roll(xy,-1,axis=0)-xy
    norm2=np.sum(vectors*vectors,axis=1)
    model=ET.parse(run/'world.sdf').find(".//model[@name='team07_hybrid']")
    radius=float(model.find("link[@name='rear_left_wheel']/collision/geometry/cylinder/radius").text)
    assert abs(radius-RADIUS)<1e-12, 'Telemetry conversion must match actual SDF radius'
    body=[]
    for a,b in zip(poses,poses[1:]):
        dt=b[0]-a[0]
        if dt<=0:continue
        delta=b[1:3]-a[1:3]
        rel=b[1:3]-xy
        fraction=np.clip(np.sum(rel*vectors,axis=1)/np.maximum(norm2,1e-12),0,1)
        errors=rel-fraction[:,None]*vectors
        index=int(np.argmin(np.sum(errors*errors,axis=1)))
        signed_error=float(np.cross(vectors[index],errors[index])/math.sqrt(norm2[index]))
        body.append(dict(stamp=float((a[0]+b[0])/2),body_speed=float(np.linalg.norm(delta)/dt),
            body_forward_speed=float(np.dot(delta,[math.cos(a[4]),math.sin(a[4])])/dt),
            yaw_rate_truth=math.atan2(math.sin(b[4]-a[4]),math.cos(b[4]-a[4]))/dt,
            lateral_error=signed_error,pose_interval_s=float(dt)))
    for i,b in enumerate(body):
        previous=body[i-1] if i else None
        b['measured_acceleration']=(b['body_speed']-previous['body_speed'])/(b['stamp']-previous['stamp']) if previous else None
    # Pair by simulation timestamp; report sample age, never silently forward fill
    # absent actual steering/old safety data. Pose differences are interval means.
    rows=[]
    for i,s in enumerate(statuses):
        stamp=s['stamp']
        b=min(body,key=lambda b:abs(b['stamp']-stamp)) if body else None
        if b and abs(b['stamp']-stamp)>.08:b=None
        joint=min(joints,key=lambda j:abs(j['stamp']-stamp)) if joints else None
        if joint and abs(joint['stamp']-stamp)>.08:joint=None
        actual=None;left=None;right=None;wheel_from_joint=None
        if joint:
            values=joint['joints']
            left=values['front_left_steer']['position'];right=values['front_right_steer']['position']
            tl,tr=math.tan(left),math.tan(right)
            k=.5*(tl/(WHEELBASE+TRACK/2*tl)+tr/(WHEELBASE-TRACK/2*tr))
            actual=-math.atan(WHEELBASE*k)
            if all(n in values for n in ('rear_left_spin','rear_right_spin')):
                wheel_from_joint=radius*(values['rear_left_spin']['velocity']+values['rear_right_spin']['velocity'])/2
        previous=statuses[i-1] if i else None
        ac=s.get('acceleration_command')
        if ac is None and previous and stamp>previous['stamp']:
            ac=(s['speed_command']-previous['speed_command'])/(stamp-previous['stamp'])
        row=dict(stamp=stamp,target_speed=s['speed_command'],steering_command=s['steering'],
            steering_actual=actual,steering_actual_left=left,steering_actual_right=right,
            steering_actual_stamp=joint['stamp'] if joint else None,
            wheel_speed=s['wheel_speed'],wheel_speed_from_joint=wheel_from_joint,
            wheel_stamp=s.get('wheel_stamp'),yaw_rate=s.get('yaw_rate'),imu_stamp=s.get('imu_stamp'),
            lookahead=s.get('lookahead'),acceleration_command=ac,
            acceleration_command_source='controller_cycle' if s.get('acceleration_command') is not None else 'sampled_status_difference',
            safety_trigger=json.dumps(s.get('safety_trigger'),ensure_ascii=False),reason=s['reason'],
            body_stamp=b['stamp'] if b else None,body_speed=b['body_speed'] if b else None,
            body_forward_speed=b['body_forward_speed'] if b else None,
            measured_acceleration=b['measured_acceleration'] if b else None,
            yaw_rate_truth=b['yaw_rate_truth'] if b else None,lateral_error=b['lateral_error'] if b else None,
            path_fit_residual_m=s.get('path_fit_residual_m'),routine_braking_eligible=s.get('routine_braking_eligible'))
        row['wheel_body_difference']=s['wheel_speed']-b['body_speed'] if b else None
        rows.append(row)
    if rows:
        with (run/'telemetry_aligned.csv').open('w') as f:
            writer=csv.DictWriter(f,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)
    events=[s['safety_trigger'] for s in statuses if s.get('safety_trigger')]
    trigger=events[0] if events else None
    stop=next((s for s in statuses if s['reason']=='highspeed_yaw_or_reverse_stop'),None)
    # Aggregate delay fit is descriptive; it is not a measured actuator deadtime.
    lag=None
    usable=[r for r in rows if r['steering_actual'] is not None]
    if len(usable)>=40:
        times=np.asarray([r['stamp'] for r in usable]);command=np.asarray([r['steering_command'] for r in usable])
        actual=np.asarray([r['steering_actual'] for r in usable])
        sample_times=np.arange(times[0]+.5,times[-1],.01)
        command_now=np.interp(sample_times,times,command)
        moving=abs(np.gradient(command_now,.01))>.03 if len(sample_times)>2 else np.zeros(len(sample_times),dtype=bool)
        if np.count_nonzero(moving)>=30 and np.std(command)>.01:
            actual_now=np.interp(sample_times,times,actual)
            lags=np.arange(0,.501,.01)
            mse=[float(np.mean((np.interp(sample_times-delay,times,command)[moving]-actual_now[moving])**2)) for delay in lags]
            best=int(np.argmin(mse))
            lag=dict(delay_fit_s=float(lags[best]),rmse_rad=math.sqrt(mse[best]),zero_delay_rmse_rad=math.sqrt(mse[0]),
                excited_samples=int(np.count_nonzero(moving)),at_search_boundary=best in (0,len(lags)-1),
                method='0..0.5s angle MSE fit, 0.01s interpolation, command slew>0.03rad/s; descriptive only')
    largest_brake=min((r for r in rows if r['acceleration_command'] is not None),key=lambda r:r['acceleration_command'],default=None)
    braking_windows=[]
    for r in rows:
        if r['acceleration_command'] is not None and r['acceleration_command']<-3:
            t=r['stamp']
            braking_windows.append(dict(command_stamp=t,acceleration_command=r['acceleration_command'],
                before=min(rows,key=lambda q:abs(q['stamp']-(t-.2))),at=r,
                after=min(rows,key=lambda q:abs(q['stamp']-(t+.3)))))
    summary=dict(steering_delay=lag,actual_steering_available=bool(joints),sdf_wheel_radius_m=radius,
        exact_safety_trigger=trigger,first_fault_status_stamp=stop['stamp'] if stop else None,
        missing_old_trigger_inputs=bool(stop and not trigger),largest_sampled_brake=largest_brake,
        peak_abs_wheel_body_difference=max((abs(r['wheel_body_difference']) for r in rows if r['wheel_body_difference'] is not None),default=None),
        peak_abs_truth_yaw_rate=max((abs(b['yaw_rate_truth']) for b in body),default=None),
        braking_windows=braking_windows,
        sources=dict(command='ROS controller status',wheel='rear joint rotation sensor, SDF radius',yaw_rate='IMU, with imu_stamp',
            steering_actual='Gazebo joint state, test instance only',body='Gazebo pose differences, analysis only',
            lateral_error='Gazebo pose vs official centreline segments, analysis only',
            safety='first control-cycle predicate snapshot; unavailable in old logs'))
    (run/'telemetry_analysis.json').write_text(json.dumps(summary,indent=2,allow_nan=False)+'\n')
    return summary


def autonomy_test(duration=60., start_station=None, cruise=.5, wall_limit=None, fast=False, track='monza', brake_at_speed=None, brake_angle=None, path_mode='center', left_wall_distance=.30):
    """Explicit isolated practice test. Truth used only by this external verifier.

    Original world/model resources are read-only; only a participant-owned world
    instance receives the test vehicle and measurement plugin. No teleportation.
    """
    import os
    import sys
    import json
    import signal
    import numpy as np
    import rclpy
    from rclpy.parameter import Parameter
    from std_msgs.msg import String
    from sensor_msgs.msg import LaserScan
    from rclpy.qos import qos_profile_sensor_data
    from gz.transport13 import Node as GzNode, NodeOptions
    from gz.msgs10.pose_v_pb2 import Pose_V
    from gz.msgs10.model_pb2 import Model
    from gz.msgs10.double_v_pb2 import Double_V
    if track not in ('monza','hungaroring'):raise ValueError('Unsupported test track')
    package = ROOT.parent/'urrc_track_gazebo'
    run = ROOT/'.autonomy'/(track+'_'+str(time.time_ns()))
    run.mkdir(parents=True)
    env = os.environ.copy()
    domain = 140+os.getpid()%50
    env.update(ROS_DOMAIN_ID=str(domain),GZ_PARTITION='team07_autonomy_'+str(os.getpid()),
        ROS_LOG_DIR=str(run/'ros_logs'),TEAM07_LOG_ROOT=str(ROOT),
        GZ_SIM_RESOURCE_PATH=str(package/'models')+os.pathsep+env.get('GZ_SIM_RESOURCE_PATH',''))
    track_metadata=json.loads((package/'tracks/processed'/f'{track}.json').read_text())
    finish=track_metadata['start_finish']
    finish_origin=(finish['x'],finish['y'])
    finish_yaw=finish['yaw']
    grid = track_metadata['grid'][0]
    if start_station is not None:
        route_for_spawn=np.genfromtxt(package/'tracks/processed'/f'{track}_centerline.csv',delimiter=',',names=True)
        if not 0<=start_station<=float(route_for_spawn['source_parameter_m'][-1]):
            raise ValueError('test start station outside route')
        row=route_for_spawn[int(np.argmin(abs(route_for_spawn['source_parameter_m']-start_station)))]
        grid=dict(x=float(row['x_m']),y=float(row['y_m']),road_z=float(row['z_m']),yaw=float(row['yaw_rad']))
    world = ET.parse(package/'worlds'/f'{track}.sdf')
    if fast:world.getroot().find('world/physics/real_time_factor').text = '0'
    capture_state=os.environ.get('TEAM07_CAPTURE_STATE')=='1'
    if capture_state and os.environ.get('TEAM07_PI_BASELINE')!='1':raise ValueError('checkpoint capture requires isolated baseline PI')
    model = ET.fromstring(model_xml()).find('model')
    # Explicit isolated experiments only. Public vehicle retains stock PI
    # until stopping distance and yaw acceptance criteria are both met.
    if os.environ.get('TEAM07_PI_BASELINE')=='1' or os.environ.get('TEAM07_BRAKE_CANDIDATE')=='1':
        for plugin in list(model.findall('plugin')):
            if plugin.get('filename')=='gz-sim-joint-controller-system':model.remove(plugin)
        library=(ROOT/'.install/team07_hybrid_racer/lib/libteam07_drive_pi.so' if (ROOT/'CMakeLists.txt').exists()
            else ROOT.parent/'libteam07_drive_pi.so')
        plugin=ET.SubElement(model,'plugin',filename=str(library),name='team07::DrivePI')
        ET.SubElement(plugin,'brake_integral_reset').text='false' if os.environ.get('TEAM07_PI_BASELINE')=='1' else 'true' 
    if capture_state:
        for item in list(model.findall('plugin')):
            if item.get('filename')=='gz-sim-joint-position-controller-system':model.remove(item)
        pi=model.find("plugin[@name='team07::DrivePI']")
        ET.SubElement(pi,'native_steering').text='true';ET.SubElement(pi,'capture_directory').text=str(run)
        for side in ('left','right'):
            sensor=ET.SubElement(model.find(f"link[@name='rear_{side}_wheel']"),'sensor',name='contact_'+side,type='contact')
            for key,val in dict(always_on='true',update_rate='200',topic=BASE+'/contact/'+side).items():ET.SubElement(sensor,key).text=val
            c=ET.SubElement(sensor,'contact');ET.SubElement(c,'collision').text='collision';ET.SubElement(c,'topic').text=BASE+'/contact/'+side
        if world.getroot().find("world/plugin[@filename='gz-sim-contact-system']") is None:ET.SubElement(world.getroot().find('world'),'plugin',filename='gz-sim-contact-system',name='gz::sim::systems::Contact')
    joint_plugin=model.find("plugin[@filename='gz-sim-joint-state-publisher-system']")
    for joint_name in (('front_left_steer','front_right_steer','front_left_spin','front_right_spin') if capture_state else ('front_left_steer','front_right_steer')):
        if joint_name not in [j.text for j in joint_plugin.findall('joint_name')]:
            ET.SubElement(joint_plugin,'joint_name').text=joint_name
    ET.SubElement(model,'pose').text = f"{grid['x']} {grid['y']} {grid['road_z']+.005} 0 0 {grid['yaw']}"
    plugin = ET.SubElement(model,'plugin',filename='gz-sim-pose-publisher-system',name='gz::sim::systems::PosePublisher')
    for k,v in dict(publish_model_pose='true',publish_link_pose='false',use_pose_vector_msg='true',
                    topic=BASE+'/test_pose',update_frequency='200' if capture_state else '20').items():ET.SubElement(plugin,k).text=v
    # Practice selects scan + IMU; unused camera/3D rendering is disabled only
    # in this test instance. Competition vehicle generator remains unchanged.
    for sensor in model.findall('.//sensor'):
        if sensor.get('type')=='camera' or sensor.get('name')=='team07_lidar3d':
            sensor.find('always_on').text = 'false'
            sensor.find('update_rate').text = '.1'
    world.getroot().find('world').append(model)
    world.write(run/'world.sdf')
    import ast,hashlib
    source=Path(__file__).read_text()
    tree=ast.parse(source)
    components=[n for n in tree.body if isinstance(n,(ast.ClassDef,ast.FunctionDef))
                and n.name in ('LocalDriver','ProgressEstimate','control','targets','StartGate')]
    fingerprint=hashlib.sha256(''.join(ast.dump(n) for n in components).encode()).hexdigest()
    (run/'controller_source.txt').write_text(source)
    if (ROOT/'drive_pi.cc').exists():(run/'drive_pi_source.cc').write_text((ROOT/'drive_pi.cc').read_text())
    (run/'provenance.json').write_text(json.dumps(dict(track=track,initial_pose=grid,
        control_ast_sha256=fingerprint,reset_count=0,
        finish_validation='actual directed plane crossing, interpolated in lateral gate'),indent=2))
    if capture_state:
        provenance=json.loads((run/'provenance.json').read_text())
        provenance.update(seed=42,capture='native 1kHz inputs and pre-zero checkpoint; no hidden solver restore',
            track_asset_sha256={str(p.relative_to(ROOT.parent)):hashlib.sha256(p.read_bytes()).hexdigest()
                for p in (package/'models'/('urrc_'+track)).rglob('*') if p.is_file()})
        (run/'provenance.json').write_text(json.dumps(provenance,indent=2))
    rclpy.init(domain_id=domain)
    node = rclpy.create_node('team07_autonomy_verifier',parameter_overrides=[Parameter('use_sim_time',value=True)])
    options = NodeOptions();options.partition = env['GZ_PARTITION']
    transport = GzNode(options)
    poses,statuses,joint_samples = [],[],[]
    route_live=np.genfromtxt(package/'tracks/processed'/f'{track}_centerline.csv',delimiter=',',names=True)
    xy_live=np.column_stack((route_live['x_m'],route_live['y_m']))
    segment_live=np.linalg.norm(np.roll(xy_live,-1,axis=0)-xy_live,axis=1)
    cumulative_live=np.r_[0,np.cumsum(segment_live)]
    length_live=float(cumulative_live[-1])
    live=dict(distance_m=0.,forward_m=0.,reverse_m=0.,crossings=0,offtrack=False,
              last_station_m=None,last_position=None,lap_confirmed=False)
    pose_stream=(run/'poses.jsonl').open('w',buffering=1)
    status_stream=(run/'status.jsonl').open('w',buffering=1)
    def status_receive(msg):
        value=json.loads(msg.data);statuses.append(value)
        status_stream.write(json.dumps(value)+'\n')
    def receive(msg):
        for pose in msg.pose:
            if pose.name=='team07_hybrid':
                stamp = pose.header.stamp.sec+pose.header.stamp.nsec*1e-9
                if stamp==0:stamp = msg.header.stamp.sec+msg.header.stamp.nsec*1e-9
                if stamp==0:stamp = node.get_clock().now().nanoseconds*1e-9
                q=pose.orientation
                yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
                record=(stamp,pose.position.x,pose.position.y,pose.position.z,yaw)
                if poses and stamp<=poses[-1][0]:continue
                index=int(np.argmin(np.sum((xy_live-np.array(record[1:3]))**2,axis=1)))
                station=float(cumulative_live[index])
                offset=float(np.linalg.norm(xy_live[index]-np.array(record[1:3])))
                if poses:
                    ds=math.hypot(record[1]-poses[-1][1],record[2]-poses[-1][2])
                    live['distance_m']+=ds
                    previous=live['last_station_m']
                    delta=(station-previous+length_live/2)%length_live-length_live/2
                    live['forward_m']+=max(delta,0);live['reverse_m']+=max(-delta,0)
                    aligned=abs(math.atan2(math.sin(yaw-float(route_live['yaw_rad'][index])),math.cos(yaw-float(route_live['yaw_rad'][index]))))<.4
                    if finish_crossed(poses[-1],record,finish_origin,finish_yaw,float(route_live['road_width_m'][0])/2-.11):live['crossings']+=1
                live.update(last_station_m=station,last_position=list(record),lateral_offset_m=offset,
                    progress_percent=100*live['forward_m']/length_live)
                live['offtrack'] |= offset>float(route_live['road_width_m'][index])/2-.11
                live['lap_confirmed']=bool(start_station is None and not live['offtrack']
                    and live['distance_m']>.98*length_live and live['forward_m']>.98*length_live
                    and live['reverse_m']<.02*length_live and live['crossings']>=2)
                poses.append(record)
                # Verifier-only quaternion enables full 3D collision-gap measurement.
                pose_stream.write(json.dumps([*record,q.x,q.y,q.z,q.w] if path_mode=='left_wall' else record)+'\n')
    motor_stream=(run/'drive_pi.jsonl').open('w',buffering=1)
    def receive_motor(msg):
        motor_stream.write(json.dumps(dict(stamp=msg.data[0],
            values=list(msg.data[1:])))+'\n')
    transport.subscribe(Double_V,BASE+'/drive_pi',receive_motor)
    joint_stream=(run/'joints.jsonl').open('w',buffering=1)
    def receive_joints(msg):
        stamp=msg.header.stamp.sec+msg.header.stamp.nsec*1e-9
        if joint_samples and stamp-joint_samples[-1]['stamp']<.01:return
        values={j.name:dict(position=j.axis1.position,velocity=j.axis1.velocity) for j in msg.joint}
        if all(n in values for n in ('front_left_steer','front_right_steer')):
            record=dict(stamp=stamp,joints=values)
            joint_samples.append(record);joint_stream.write(json.dumps(record)+'\n')
    transport.subscribe(Model,BASE+'/joint_states',receive_joints)
    transport.subscribe(Pose_V,BASE+'/test_pose',receive)
    scan_stream=(run/'scans.jsonl').open('w',buffering=1)
    def receive_scan(msg):
        record=dict(stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9,
            received_sim=node.get_clock().now().nanoseconds*1e-9,received_wall=time.monotonic(),
            frame_id=msg.header.frame_id,angle_min=msg.angle_min,angle_increment=msg.angle_increment,
            range_min=msg.range_min,range_max=msg.range_max,
            ranges=[v if math.isfinite(v) else str(v) for v in msg.ranges])
        scan_stream.write(json.dumps(record)+'\n')
    scan_subscription=node.create_subscription(LaserScan,BASE+'/scan',receive_scan,qos_profile_sensor_data)
    subscription = node.create_subscription(String,BASE+'/autonomy/status',status_receive,10)
    contact_stream=(run/'contacts.jsonl').open('w',buffering=1) if capture_state else None
    generation_stream=(run/'command_generated.jsonl').open('w',buffering=1) if capture_state else None
    if capture_state:
        from gz.msgs10.contacts_pb2 import Contacts
        from google.protobuf.json_format import MessageToDict
        def contact_receive(msg,side):
            stamp=msg.header.stamp.sec+msg.header.stamp.nsec*1e-9
            contact_stream.write(json.dumps(dict(stamp=stamp,side=side,message=MessageToDict(msg,preserving_proto_field_name=True)))+'\n')
        for side in ('left','right'):transport.subscribe(Contacts,BASE+'/contact/'+side,lambda msg,side=side:contact_receive(msg,side))
        transport.subscribe(Double_V,BASE+'/command_generated',lambda msg:generation_stream.write(json.dumps(list(msg.data))+'\n'))
    processes,logs = [],[]
    def launch(cmd,name):
        log=(run/name).open('w');logs.append(log)
        proc=subprocess.Popen(cmd,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        processes.append(proc)
    error = None
    brake_stamp = None
    termination='starting'
    wall_started=time.monotonic()
    try:
        launch(['gz','sim','-s','-r','--headless-rendering',*(['--seed','42'] if capture_state else []),'-v','3',str(run/'world.sdf')],'gazebo.log')
        launch(['ros2','run','ros_gz_bridge','parameter_bridge',*sensor_bridge_args(clock=True)],'bridge.log')
        launch([sys.executable,'-B',str(Path(__file__).resolve()),'--ros-args','-p','use_sim_time:=true',
                '-p','autonomy_mode:=practice','-p','practice_authorized:=true','-p',f'cruise_speed:={cruise}','-p',f'path_mode:={path_mode}','-p',f'left_wall_distance:={left_wall_distance}','-p','highspeed_validation:=true','-p','record_control_trace:=true'],'controller.log')
        deadline=time.monotonic()+(wall_limit if wall_limit is not None else max(120.,duration*6))
        start=None
        progress_wall=time.monotonic()
        stagnant_since=None
        motion_anchor=None
        speed_ready_since=None
        while time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.05)
            if any(p.poll() is not None for p in processes):raise RuntimeError('Test process exited')
            if statuses and start is None:start=statuses[-1]['stamp']
            if start is not None and statuses[-1]['stamp']-start>=duration:
                termination='simulation_time_limit';break
            if brake_at_speed is not None and len(poses)>=2 and statuses:
                a,b=poses[-2:];pose_dt=b[0]-a[0]
                actual_speed=math.hypot(b[1]-a[1],b[2]-a[2])/pose_dt if pose_dt>0 else 0.
                actual_angle=statuses[-1].get('steering_actual')
                ready=(actual_speed>=brake_at_speed-.03 and actual_angle is not None and abs(actual_angle)<.02)
                if ready:
                    if speed_ready_since is None:speed_ready_since=b[0]
                    angle_matched=(brake_angle is None or (abs(actual_angle-brake_angle)<=.003 and abs(statuses[-1]['steering']-brake_angle)<=.005 and abs(statuses[-1].get('yaw_rate',0))<=.1))
                    if b[0]-speed_ready_since>=.25 and angle_matched:
                        termination='controlled_brake_speed';break
                else:speed_ready_since=None
            if path_mode=='left_wall' and statuses:
                latest=statuses[-1]
                wheel_angles=([abs(joint_samples[-1]['joints'][n]['position'])
                    for n in ('front_left_steer','front_right_steer')] if joint_samples else [])
                excessive=max([abs(latest['steering']),abs(latest.get('steering_actual') or 0.),*wheel_angles])>.35
                if excessive or (live['distance_m']>.1 and latest['reason']!='tracking'):
                    termination='left_wall_safety_abort:'+('excessive_steering' if excessive else latest['reason']);break
            if live['lap_confirmed']:
                termination='lap_confirmed';break
            if live['offtrack']:
                termination='offtrack';break
            if live['reverse_m']>(.1 if cruise>3. else .5):
                termination='reverse_instability';break
            if statuses and statuses[-1].get('attitude') and abs(statuses[-1]['attitude'][0])>.2:
                termination='roll_instability';break
            if statuses and statuses[-1].get('reason')=='highspeed_yaw_or_reverse_stop':
                termination='highspeed_yaw_or_reverse_stop';break
            if cruise>3. and live['distance_m']>.2 and statuses and statuses[-1]['reason'] not in ('tracking','awaiting_start'):
                termination='safety_stop:'+statuses[-1]['reason'];break
            if cruise>3. and poses:
                index=int(np.argmin(np.sum((xy_live-np.asarray(poses[-1][1:3]))**2,axis=1)))
                heading_error=math.atan2(math.sin(poses[-1][4]-float(route_live['yaw_rad'][index])),math.cos(poses[-1][4]-float(route_live['yaw_rad'][index])))
                if abs(heading_error)>.6:
                    termination='heading_instability';break
            if statuses:
                current=statuses[-1]['stamp']
                if motion_anchor is None:
                    motion_anchor=live['distance_m'];stagnant_since=current
                if live['distance_m']-motion_anchor>.15:
                    motion_anchor=live['distance_m'];stagnant_since=current
                if current-stagnant_since>12:
                    termination='stalled:'+statuses[-1]['reason'];break
            if time.monotonic()-progress_wall>=10:
                progress_wall=time.monotonic()
                snapshot=dict(**live,wall_elapsed_s=progress_wall-wall_started,
                    status=statuses[-1] if statuses else None,termination='running')
                (run/'live.json').write_text(json.dumps(snapshot,indent=2))
                print('PROGRESS',json.dumps({k:snapshot[k] for k in ('distance_m','last_station_m','progress_percent','wall_elapsed_s')}),flush=True)
        else:termination='wall_time_limit'
        if not statuses:raise RuntimeError('No autonomy telemetry / clock')
        if start is None:raise RuntimeError('Simulation time did not start')
        brake_stamp=statuses[-1]['stamp']
        # Request a stop through this test's isolated controller authorization.
        from rcl_interfaces.srv import SetParameters
        stop_client=node.create_client(SetParameters,'/team07_hybrid_racer/set_parameters')
        if not stop_client.wait_for_service(timeout_sec=5):raise RuntimeError('Stop service unavailable')
        request=SetParameters.Request()
        request.parameters=[Parameter('practice_authorized',value=False).to_parameter_msg()]
        if capture_state:(run/'brake_generated.json').write_text(json.dumps(dict(sim_time=node.get_clock().now().nanoseconds*1e-9,source='verifier parameter stop request; actual generated motor targets in command_generated.jsonl')))
        future=stop_client.call_async(request)
        deadline=time.monotonic()+25
        stopped_at=None
        while time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.02)
            if future.done() and (future.exception() or not all(r.successful for r in future.result().results)):
                raise RuntimeError('Stop authorization update rejected')
            latest=statuses[-1]
            body_still=False
            if len(poses)>=2 and joint_samples:
                a,b=poses[-2:];pose_dt=b[0]-a[0]
                rear=joint_samples[-1]['joints']
                body_still=(0<pose_dt<.2 and math.hypot(b[1]-a[1],b[2]-a[2])/pose_dt<.02
                    and abs(math.atan2(math.sin(b[4]-a[4]),math.cos(b[4]-a[4])))/pose_dt<.02
                    and all(abs(rear[n]['velocity']*RADIUS)<.02 for n in ('rear_left_spin','rear_right_spin')))
            if latest['stamp']>=brake_stamp and latest['speed_command']==0 and abs(latest['wheel_speed'])<.02 and body_still:
                if stopped_at is None:stopped_at=latest['stamp']
                if latest['stamp']-stopped_at>=1.:break
            else:stopped_at=None
        else:raise RuntimeError('Stop acknowledgement / stationary state not observed')
    except KeyboardInterrupt:
        error='Independent test interrupted';termination='interrupted'
    except Exception as exc:
        error=str(exc);termination='error'
    finally:
        # Only processes created by this test are stopped.
        for proc in reversed(processes):
            if proc.poll() is None:
                os.killpg(proc.pid,signal.SIGINT)
                try:proc.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    os.killpg(proc.pid,signal.SIGKILL);proc.wait()
        for log in logs:log.close()
        (run/'live_final.json').write_text(json.dumps(dict(**live,termination=termination,error=error),indent=2))
        pose_stream.close();status_stream.close();joint_stream.close();scan_stream.close();motor_stream.close()
        if contact_stream:
            for side in ('left','right'):transport.unsubscribe(BASE+'/contact/'+side)
            transport.unsubscribe(BASE+'/command_generated');contact_stream.close();generation_stream.close()
        node.destroy_node();rclpy.shutdown()
    (run/'poses.json').write_text(json.dumps(poses))
    (run/'status.json').write_text(json.dumps(statuses))
    # Independent truth verification: ordered nearest centreline progress,
    # direction, full route distance and forward finish-line wrap are required.
    route=np.genfromtxt(package/'tracks/processed'/f'{track}_centerline.csv',delimiter=',',names=True)
    xy=np.column_stack((route['x_m'],route['y_m']))
    segments=np.linalg.norm(np.roll(xy,-1,axis=0)-xy,axis=1)
    cumulative=np.r_[0,np.cumsum(segments)];length=float(cumulative[-1])
    distance=0.;speeds=[];truth_lateral=[];max_offset=0.;advance=0.;reverse=0.;last_s=None;crossings=0;offtrack=False
    initial=None;departed=False
    for first,second in zip(poses,poses[1:]):
        dt=second[0]-first[0]
        if dt<=0:continue
        ds=math.hypot(second[1]-first[1],second[2]-first[2])
        distance+=ds;speeds.append(ds/dt)
        yaw_rate=abs(math.atan2(math.sin(second[4]-first[4]),math.cos(second[4]-first[4])))/dt
        truth_lateral.append(ds/dt*yaw_rate)
        index=int(np.argmin(np.sum((xy-np.array(second[1:3]))**2,axis=1)))
        station=float(cumulative[index])
        nearest=float(np.linalg.norm(xy[index]-np.array(second[1:3])))
        max_offset=max(max_offset,nearest)
        offtrack |= nearest > float(route['road_width_m'][index])/2-.11
        if initial is None:initial=station
        if last_s is not None:
            delta=(station-last_s+length/2)%length-length/2
            advance+=max(delta,0);reverse+=max(-delta,0)
            aligned=abs(math.atan2(math.sin(second[4]-float(route['yaw_rad'][index])),math.cos(second[4]-float(route['yaw_rad'][index]))))<.4
            if finish_crossed(first,second,finish_origin,finish_yaw,float(route['road_width_m'][0])/2-.11):crossings+=1
        last_s=station
        departed |= distance>10
    confirmed=bool(start_station is None and error is None and not offtrack and departed and distance>.98*length and advance>.98*length and reverse<.02*length and crossings>=2)
    brake_poses=[p for p in poses if brake_stamp is not None and p[0]>=brake_stamp]
    brake_distance=sum(math.hypot(b[1]-a[1],b[2]-a[2]) for a,b in zip(brake_poses,brake_poses[1:]))
    first_zero=next((s['stamp'] for s in statuses if brake_stamp is not None
                     and s['stamp']>=brake_stamp and s['speed_command']==0),None)
    stopped=next((s['stamp'] for s in statuses if first_zero is not None
                  and s['stamp']>=first_zero and abs(s['wheel_speed'])<.02),None)
    stopped_poses=[p for p in poses if first_zero is not None and p[0]>=first_zero
                   and (stopped is None or p[0]<=stopped)]
    observed_braking=sum(math.hypot(b[1]-a[1],b[2]-a[2]) for a,b in zip(stopped_poses,stopped_poses[1:]))
    from collections import Counter
    driving=[v for v in statuses if brake_stamp is None or v['stamp']<=brake_stamp]
    # Evaluate continuous route segments, avoiding centreline sample-spacing bias.
    # Truth is used only here, never supplied to either steering controller.
    offsets=[];yaw_rates=[]
    route_vec=np.roll(xy,-1,axis=0)-xy
    lengths2=np.sum(route_vec**2,axis=1)
    for pose in poses:
        if brake_stamp is not None and pose[0]>brake_stamp:continue
        relative=np.asarray(pose[1:3])-xy
        t=np.clip(np.sum(relative*route_vec,axis=1)/np.maximum(lengths2,1e-12),0,1)
        offsets.append(float(np.min(np.linalg.norm(relative-t[:,None]*route_vec,axis=1))))
    for a,b in zip(poses,poses[1:]):
        if brake_stamp is not None and b[0]>brake_stamp:continue
        if b[0]>a[0]:yaw_rates.append(abs(math.atan2(math.sin(b[4]-a[4]),math.cos(b[4]-a[4])))/(b[0]-a[0]))
    steering_variation=sum(abs(b['steering']-a['steering']) for a,b in zip(driving,driving[1:]))
    report=dict(track=track,mean_path_error_m=float(np.mean(offsets)) if offsets else None,
        max_path_error_m=max(offsets,default=0),steering_total_variation_rad=steering_variation,
        mean_abs_yaw_rate_rad_s=float(np.mean(yaw_rates)) if yaw_rates else None,
        max_abs_yaw_rate_rad_s=max(yaw_rates,default=0),
        collision_verified=False,initial_grid=grid,control_ast_sha256=fingerprint,max_center_offset_m=max_offset,peak_truth_lateral_accel_mps2=max(truth_lateral,default=0),reason_counts=dict(Counter(v['reason'] for v in driving)),
        peak_roll_rad=max((abs(v['attitude'][0]) for v in driving if v.get('attitude')),default=0),
        peak_pitch_rad=max((abs(v['attitude'][1]) for v in driving if v.get('attitude')),default=0),
        peak_estimated_lateral_accel_mps2=max((v.get('estimated_lateral_accel',0) for v in driving),default=0),
        termination=termination,wall_elapsed_s=time.monotonic()-wall_started,cruise_speed=cruise,fast=fast,
        last_position=poses[-1] if poses else None,last_station_m=last_s,progress_percent=100*advance/length,
        stop_trigger_status=next((v for v in reversed(statuses) if brake_stamp is not None and v['stamp']<=brake_stamp),None),
        final_stop_reason=statuses[-1]['reason'] if statuses else None,
        first_zero_command_stamp=first_zero,stop_below_002_stamp=stopped,
        observed_distance_after_zero_command_m=observed_braking,
        observed_stop_time_s=None if stopped is None else stopped-first_zero,
        brake_request_stamp=brake_stamp,post_stop_request_distance_m=brake_distance,
        final_wheel_speed_mps=statuses[-1]['wheel_speed'] if statuses else None,
        braking_note='Authorization-request latency and sampled zero-command stopping distance are reported separately',
        test='isolated_practice_not_race_start',start_station_m=start_station,started_grid_1=start_station is None,error=error,truth_distance_m=distance,
        duration_sim_s=poses[-1][0]-poses[0][0] if len(poses)>1 else 0,
        mean_speed_mps=distance/(poses[-1][0]-poses[0][0]) if len(poses)>1 and poses[-1][0]>poses[0][0] else 0,
        max_speed_mps=max(speeds,default=0),lap_confirmed=confirmed,route_length_m=length,
        forward_progress_m=advance,reverse_progress_m=reverse,finish_crossings=crossings,
        offtrack=bool(offtrack),status_count=len(statuses),last_status=statuses[-1] if statuses else None)
    physical_reverse=0.
    for a,b in zip(poses,poses[1:]):
        heading=(a[4]+b[4])/2
        forward=(b[1]-a[1])*math.cos(heading)+(b[2]-a[2])*math.sin(heading)
        physical_reverse+=max(0.,-forward)
    report['physical_reverse_m']=physical_reverse
    if cruise>3. and (termination not in ('simulation_time_limit','controlled_brake_speed') or offtrack or reverse>.1 or physical_reverse>.01 or error
            or any(v['reason']!='tracking' for v in driving if v.get('progress',{}).get('distance',0)>.2)):
        restrict_highspeed(f'{run.name}: {termination}; physical_reverse={physical_reverse:.4f}m')
    (run/'results.json').write_text(json.dumps(report,indent=2))
    analyze_telemetry(run,track)
    analyze_braking(run)
    print(json.dumps(report,indent=2),flush=True)
    print('Artifacts:',run,flush=True)


def autonomy_checks():
    """Deterministic geometry/safety regression without simulator side effects."""
    import numpy as np
    from types import SimpleNamespace
    angles=np.linspace(-math.pi,math.pi,720)
    def scan(width=3.,obstacle=None):
        ranges=width/2/np.maximum(abs(np.sin(angles)),1e-8)
        ranges=np.minimum(ranges,40.)
        if obstacle is not None:ranges[abs(angles)<.3]=obstacle
        return SimpleNamespace(header=SimpleNamespace(frame_id='team07_lidar'),ranges=ranges,
            angle_min=-math.pi,angle_increment=2*math.pi/719,range_min=.06,range_max=40.)
    driver=LocalDriver()
    for _ in range(100):v,steer,report=driver.plan(scan(),(0,0,0),.3,.02)
    assert .2<v<=.5 and abs(steer)<.01, (v,steer,report)
    assert driver.plan(scan(obstacle=.4),(0,0,0),.5,.02)[0]==0
    # Safe turning arcs exist beyond the slew window: align at zero speed.
    blocked=scan(width=2.4)
    blocked.ranges=np.minimum(np.asarray(blocked.ranges),np.where(np.cos(angles)>0,
        .9/np.maximum(np.cos(angles),1e-8),40.))
    aligner=LocalDriver()
    previous=0.
    for _ in range(3):
        v,st,report=aligner.plan(blocked,(0,0,0),0.,.02)
        assert v==0 and report['reason']=='aligning_safe_steering'
        assert 0<abs(st-previous)<=.65*.02+1e-9
        previous=st
    bad=scan();bad.ranges[:]=float('nan')
    assert driver.plan(bad,(0,0,0),.5,.02)[0]==0
    assert driver.plan(scan(),(.3,0,0),.5,.02)[0]==0
    assert driver.plan(scan(),None,.5,.02)[0]==0
    # Wide and one-sided roads must preserve a plausible centre or fail closed.
    wide=LocalDriver()
    for _ in range(100):v,st,info=wide.plan(scan(width=8.),(0,0,0),.3,.02)
    assert v>0 and abs(st)<.01 and 7.5<wide.width<8.5
    partial=scan()
    partial.ranges[np.sin(angles)>0]=float('inf')
    single=LocalDriver(width=3.)
    for _ in range(120):v,st,info=single.plan(partial,(0,0,0),.3,.02)
    assert 0<v<=.3 and info['one_side'] and abs(st)<.02
    assert LocalDriver().plan(partial,(0,0,0),.3,.02)[0]==0
    # A left-bending corridor requires negative ROS steering and bounded slew.
    bend=scan();r=np.asarray(bend.ranges);x=r*np.cos(angles);y=r*np.sin(angles)
    # Shift corridor centre left at increasing forward distance using analytic walls.
    shifted=[]
    for a in angles:
        intersections=[]
        for side in (-1,1):
            # y=.3*x + side*1.5
            denominator=math.sin(a)-.3*math.cos(a)
            if abs(denominator)>1e-6:
                t=side*1.5/denominator
                if t>.06:intersections.append(t)
        shifted.append(min(intersections,default=40.))
    bend.ranges=np.array(shifted)
    d=LocalDriver();previous=0.
    for _ in range(40):
        v,st,info=d.plan(bend,(0,0,0),.3,.02)
        assert abs(st-previous)<=.65*.02+1e-9 and abs(st)<=CENTER_LIMIT
        previous=st
    assert st<-.01,(st,info)
    # Known geometry: both distant walls lie on one side of body y=0.
    rotated=scan(width=2.4)
    rotated.ranges=np.minimum(40.,1.2/np.maximum(abs(np.sin(angles)+.16*np.cos(angles)),1e-8))
    _,_,profile=LocalDriver().plan(rotated,(0,0,0),.3,.02,3.,0.)
    assert profile['path'][-1][0]==12.
    assert max(abs(y+.16*x) for x,y in profile['path'])<.1
    # A genuine radius-25m corridor must retain its measured bend. Its far
    # section is occluded by the inner wall; do not invent unseen free road.
    arc=scan(width=2.4);rays=[]
    for angle in angles:
        intersections=[]
        for radius in (25.-1.2,25.+1.2):
            discriminant=(25.*math.sin(angle))**2-(25.**2-radius**2)
            if discriminant>=0:
                intersections.extend(t for t in (25.*math.sin(angle)-math.sqrt(discriminant),
                    25.*math.sin(angle)+math.sqrt(discriminant)) if t>.06)
        rays.append(min(40.,min(intersections,default=40.)))
    arc.ranges=np.asarray(rays)
    _,_,profile=LocalDriver().plan(arc,(0,0,0),.3,.02,3.,0.)
    assert .02<profile['preview_curvature']<.08
    assert max(abs(y-(25.-math.sqrt(25.**2-x*x))) for x,y in profile['path'])<.04
    # High-speed candidate constraints and unknown/obstacle fail-closed cases.
    driver=LocalDriver()
    open_scan=scan(width=3.)
    rr=np.asarray(open_scan.ranges);valid=np.isfinite(rr)&(rr>=.08)&(rr<=40.)
    points=np.column_stack((rr[valid]*np.cos(angles[valid]),rr[valid]*np.sin(angles[valid])))
    selected=driver.feasible_steering(open_scan,points,rr,valid|np.isposinf(rr),2.,.02,.325,0.,0.,4.,.7)
    assert selected is not None
    for candidate in driver.candidate_audit['candidates']:
        assert abs(candidate['command'])<=.325*.02+1e-12
        assert 4.*abs(math.tan(candidate['command'])/WHEELBASE)<=.65+1e-12
    assert driver.feasible_steering(open_scan,np.empty((0,2)),np.full(720,float('nan')),
        np.zeros(720,dtype=bool),2.,.02,.325,0.,0.,4.,.7) is None
    assert all('sensor_uncertainty' in c['reasons'] for c in driver.candidate_audit['candidates'])
    obstacle=scan(obstacle=.5)
    rr=np.asarray(obstacle.ranges);points=np.column_stack((rr*np.cos(angles),rr*np.sin(angles)))
    assert driver.feasible_steering(obstacle,points,rr,np.ones(720,dtype=bool),2.,.02,.325,0.,0.,4.,.7) is None
    assert all('predicted_collision' in c['reasons'] for c in driver.candidate_audit['candidates'])
    progress=ProgressEstimate()
    for i in range(1000):progress.update(i*.02,0.,i*.02,0.)
    assert not progress.report()['loop_candidate'] and not progress.report()['lap_confirmed']
    # A nearest-waypoint wrap just before the line must not certify a crossing.
    assert not finish_crossed((0,-.3,0,0,0),(1,-.1,0,0,0),(0,0),0,1.)
    assert finish_crossed((0,-.1,0,0,0),(1,.1,0,0,0),(0,0),0,1.)
    assert not finish_crossed((0,.1,0,0,math.pi),(1,-.1,0,0,math.pi),(0,0),0,1.)
    assert not finish_crossed((0,-.1,2,0,0),(1,.1,2,0,0),(0,0),0,1.)
    # Execute the actual steady-clock watchdog method without a ROS graph.
    import ast
    tree=ast.parse(Path(__file__).read_text())
    method=next(n for n in ast.walk(tree) if isinstance(n,ast.FunctionDef) and n.name=='clock_watchdog')
    module=ast.Module(body=[method],type_ignores=[]);ast.fix_missing_locations(module)
    scope=dict(time=time,math=math)
    exec(compile(module,'watchdog-check','exec'),scope)
    calls=[]
    probe=SimpleNamespace(get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=100)),
        clock_watch_stamp=100,clock_watch_wall=time.monotonic()-1,
        send=lambda values:calls.append(values),driver=LocalDriver(),last_speed=0.,last_steering=0.)
    scope['clock_watchdog'](probe)
    assert calls==[(0.,0.,0.,0.)] and probe.last_speed==-math.inf
    print('PASS autonomy: straight, bend/sign/slew, obstacle, NaN, roll, missing IMU, stationary rotation, clock-loss stop')


def race_brake_assessment(poses, controls, commands, request_stamp, include_samples=False):
    """Verifier only. Native pose stamps; generated target time != PI receipt.

    Operational rest: <.02m/s for 1s, <=5mm position span, wheel <.02,
    IMU yaw <.03rad/s and zero commands; observe >=4s after zero target.
    """
    import numpy as np
    result=dict(body_stop_confirmed=False,reason='missing_stop_evidence',
        stop_request_stamp=request_stamp,minimum_observation_s=4.,hold_s=1.,
        body_speed_threshold_mps=.02,position_span_threshold_m=.005)
    if request_stamp is None:return result
    poses=list(poses);controls=list(controls);commands=list(commands)
    zero=[v for v in commands if len(v) in (5,7) and v[0]>=request_stamp and max(map(abs,v[3:]))<1e-9]
    if not zero:return result
    stamp=float(min(v[0] for v in zero));result['zero_target_generated_stamp']=stamp
    result['request_to_zero_target_s']=stamp-request_stamp
    if len(poses)<8 or len(controls)<2:return result
    ordered={float(v[0]):v for v in poses}
    pp=np.asarray([ordered[k] for k in sorted(ordered)],float)
    tt=pp[:,0];xy=pp[:,1:3]
    if tt[-1]-stamp<.10:return result
    grid=np.arange(tt[0]+.025,tt[-1]-.025,.02)
    left=np.column_stack([np.interp(grid-.025,tt,xy[:,j]) for j in range(2)])
    right=np.column_stack([np.interp(grid+.025,tt,xy[:,j]) for j in range(2)])
    vel=(right-left)/.05;speed=np.linalg.norm(vel,axis=1)
    quat=pp[:,4:8];qx,qy,qz,qw=quat.T
    yaw=np.unwrap(np.arctan2(2*(qw*qz+qx*qy),1-2*(qy*qy+qz*qz)))
    grid_yaw=np.interp(grid,tt,yaw)
    forward=vel[:,0]*np.cos(grid_yaw)+vel[:,1]*np.sin(grid_yaw)
    roll=np.arctan2(2*(qw*qx+qy*qz),1-2*(qx*qx+qy*qy))
    pitch=np.arcsin(np.clip(2*(qw*qy-qz*qx),-1,1))
    active=grid>=stamp
    first=np.flatnonzero(active&(speed<.02))
    first_stamp=float(grid[first[0]]) if len(first) else None
    def path_length(a,b):
        timeline=np.r_[a,tt[(tt>a)&(tt<b)],b]
        points=np.column_stack([np.interp(timeline,tt,xy[:,j]) for j in range(2)])
        return float(np.linalg.norm(np.diff(points,axis=0),axis=1).sum())
    last=float(grid[-1]);stable=grid>=last-1.
    recent=[v for v in controls if last-1.<=v['stamp']<=last+.05]
    span=float(np.linalg.norm(np.ptp(np.column_stack([np.interp(grid[stable],tt,xy[:,j]) for j in range(2)]),axis=0)))
    pose_recent=tt>=stamp
    result.update(reason='body_or_sensor_not_settled',pose_observation_s=float(tt[-1]-stamp),
        body_speed_at_zero_mps=float(np.interp(stamp,grid,speed)),
        body_speed_before_zero_backward_50ms_mps=float(np.linalg.norm([np.interp(stamp,tt,xy[:,j])-np.interp(stamp-.05,tt,xy[:,j]) for j in range(2)])/.05),
        zero_speed_note='Central estimate includes 25ms after zero; backward estimate uses pre-brake movement',
        peak_body_speed_mps=float(speed.max()),last_body_speed_mps=float(speed[-1]),
        first_rest_stamp=first_stamp,distance_to_first_rest_m=None if first_stamp is None else path_length(stamp,first_stamp),
        additional_path_after_first_rest_m=None if first_stamp is None else path_length(first_stamp,tt[-1]),
        reverse_distance_after_zero_m=float(np.maximum(0.,-forward[active]).sum()*.02),
        reverse_distance_over_002_m=float(np.where(forward[active]<-.002,-forward[active],0.).sum()*.02),
        last_hold_peak_body_speed_mps=float(speed[stable].max()),last_hold_position_span_m=span,
        yaw_change_zero_to_first_rest_rad=None if first_stamp is None else float(np.interp(first_stamp,tt,yaw)-np.interp(stamp,tt,yaw)),
        peak_roll_rad=float(np.max(abs(roll[pose_recent]))),peak_pitch_rad=float(np.max(abs(pitch[pose_recent]))),
        peak_yaw_rate_rad_s=max((abs(v.get('yaw_rate') or 0.) for v in controls if v['stamp']>=stamp),default=None),
        peak_steering_actual_rad=max((abs(v.get('steering_actual') or 0.) for v in controls),default=None))
    fresh_control=bool(recent and last-recent[-1]['stamp']<.15 and recent[0]['stamp']<=last-.85)
    command_hold=[v for v in commands if last-1.<=v[0]<=last+.05]
    quiet_commands=bool(command_hold and all(len(v) in (5,7) and max(map(abs,v[3:]))<1e-9 for v in command_hold)
        and last-command_hold[-1][0]<.15)
    body_quiet=bool(last-stamp>=4. and speed[stable].max()<.02 and span<=.005)
    sensors_quiet=bool(fresh_control and all(v.get('wheel_peak_speed',v.get('wheel_speed')) is not None
            and abs(v.get('wheel_peak_speed',v.get('wheel_speed')))<.02 for v in recent)
        and all(v.get('yaw_rate') is not None and abs(v['yaw_rate'])<.03 for v in recent)
        and all(v.get('speed_command')==0 for v in recent))
    result.update(body_stop_confirmed=bool(body_quiet and sensors_quiet and quiet_commands),
        last_hold_sensor_quiet=sensors_quiet,last_hold_zero_targets=quiet_commands)
    if result['body_stop_confirmed']:result['reason']='body_rest_held_1s_after_4s_observation'
    # A locking proxy is only meaningful above low-speed numerical noise.
    gaps=[];near_lock=0
    for v,s,f in zip(grid[active],speed[active],forward[active]):
        c=min(controls,key=lambda row:abs(row['stamp']-v))
        if abs(c['stamp']-v)>.06:continue
        wheel=c['wheel_speed'];gaps.append(abs(wheel-f))
        if s>.1 and abs(wheel)<.2*s:near_lock+=1
    result.update(peak_abs_wheel_body_gap_mps=max(gaps,default=None),near_lock_proxy_samples=near_lock,
        near_lock_proxy_definition='body>.1m/s and abs(mean rear wheel speed)<.2*body; not contact-slip measurement')
    if include_samples:
        result['body_samples']=[dict(stamp=float(v),speed=float(s),forward_speed=float(f)) for v,s,f in zip(grid,speed,forward)]
    return result


def run_race(track, duration=None, headless=False, cruise=.5, mode='practice', camera_fault=None, path_mode='auto', left_wall_distance=.30, experimental_race_speed=False, brake_diagnostics=False):
    """Practice/race orchestration. Signal permission comes only from RGB frames."""
    import os
    import sys
    import json
    import shutil
    import hashlib
    import signal
    import rclpy
    from rclpy.parameter import Parameter
    from rclpy.qos import qos_profile_sensor_data
    from google.protobuf import text_format
    from rcl_interfaces.srv import SetParameters
    from rosgraph_msgs.msg import Clock as RosClock
    from sensor_msgs.msg import Image, CameraInfo, LaserScan, PointCloud2, Imu, JointState
    from std_msgs.msg import String, Bool
    from gz.transport13 import Node as GzNode, NodeOptions
    from gz.msgs10.empty_pb2 import Empty
    from gz.msgs10.stringmsg_v_pb2 import StringMsg_V
    from gz.msgs10.scene_pb2 import Scene
    from gz.msgs10.pose_v_pb2 import Pose_V
    from gz.msgs10.world_stats_pb2 import WorldStatistics
    from gz.msgs10.double_v_pb2 import Double_V
    from gz.msgs10.double_pb2 import Double
    from gz.msgs10.image_pb2 import Image as GzImage
    from race_performance import course_geometry, performance

    if mode!='race' or experimental_race_speed or cruise>.3 or path_mode not in ('auto','center'):
        raise ValueError('Submission standalone mode requires camera race, the center path and a 0.3 m/s ceiling')
    project = next((p for p in ROOT.parents if (p/'run_track.sh').is_file()), None)
    if project is None:
        raise RuntimeError('Place the extracted team_07 folder inside the URRC workspace')
    track_package=project/'src'/'urrc_track_gazebo'
    path_mode=resolve_path_mode(path_mode,mode)
    requested_cruise=cruise
    if experimental_race_speed:cruise=min(cruise,RACE_EXPERIMENTAL_CEILING_MPS)
    course=None
    if mode=='race':
        course_file=track_package/'tracks'/'processed'/f'{track}_centerline.csv'
        course=course_geometry(course_file,max(.23,vehicle_metrics()['swept_width_bound_m']/2),.04+.06+.025)
    logdir=ROOT/'.logs'/f'race_{track}_{time.time_ns()}'
    logdir.mkdir(parents=True)
    children=[];handles=[];node=None;transport=None
    clocks=deque(maxlen=2048);pose=[];status={};sensors={};stats={}
    status_wall=[time.monotonic()];clock_wall=[time.monotonic()]
    result=dict(track=track,mode=mode,path_mode=path_mode,camera_fault=camera_fault,cruise_speed=cruise,experimental_race_speed=experimental_race_speed,moved=False,gui_verified=False,
                requested_cruise_mps=requested_cruise,straight_design_goal_mps=RACE_STRAIGHT_GOAL_MPS,
                reused_world=False,reused_vehicle=False,reused_clock=False,success=False)
    camera_timing=[];gz_camera_timing=[];clock_timing=[]
    lidar_evidence=[];lidar_last={}
    gate_status={};gate_history=[];pose_history=[];drive_history=[];camera_counts=set()
    fault_active=[False];all_red_seen=[False];off_seen=[False]
    camera_stream=(logdir/'camera_observations.jsonl').open('w',buffering=1)
    wall_start=time.monotonic();authorized=False
    body_poses=[];control_history=[];generated_commands=[];wheel_history=[];stop_request_stamp=None;stop_evidence=None
    permission_pub=None;relay=None
    def say(message):print('[실행] '+message,flush=True)
    def processes():
        found=[]
        for directory in Path('/proc').glob('[0-9]*'):
            try:
                comm=(directory/'comm').read_text().strip()
                cmd=(directory/'cmdline').read_bytes().replace(b'\0',b' ').decode(errors='replace')
                if comm not in ('ruby','gz','gz-sim','gz-sim-server'):continue
                if 'gz sim' not in cmd:continue
                values=dict(entry.split(b'=',1) for entry in (directory/'environ').read_bytes().split(b'\0') if b'=' in entry)
                found.append((int(directory.name),cmd,values.get(b'GZ_PARTITION',b'').decode()))
            except (OSError,ValueError):continue
        return found
    def start(command,name,env=None):
        stream=(logdir/(name+'.log')).open('w');handles.append(stream)
        proc=subprocess.Popen(command,stdout=stream,stderr=subprocess.STDOUT,start_new_session=True,env=env)
        children.append((name,proc));return proc
    def tick():
        rclpy.spin_once(node,timeout_sec=.05)
        for name,proc in children:
            if proc.poll() is not None and not (name.endswith('_finite') and proc.returncode==0):raise RuntimeError(f'{name} 종료({proc.returncode}): {logdir/name}.log 확인')
    def wait_for(check,description,timeout=90):
        limit=time.monotonic()+timeout
        while time.monotonic()<limit:
            if check():return
            tick()
        raise RuntimeError(description+' 준비 시간 초과')
    def request(service,response):
        # Keep synchronous service calls out of the Python subscription node.
        # CLI requests avoid coupling discovery to active Python topic callbacks.
        try:
            reply=subprocess.run(['gz','service','-s',service,'--reqtype','gz.msgs.Empty',
                '--reptype',response.DESCRIPTOR.full_name,'--timeout','1200','--req',''],
                capture_output=True,text=True,timeout=3)
            if reply.returncode:return None
            message=response();text_format.Parse(reply.stdout,message);return message
        except (subprocess.TimeoutExpired,text_format.ParseError):return None
    def scene():return request('/world/'+track+'/scene/info',Scene)
    def set_authorized(value,timeout=10):
        if mode=='race':
            limit=time.monotonic()+timeout
            while time.monotonic()<limit:
                permission_pub.publish(Bool(data=value))
                rclpy.spin_once(node,timeout_sec=.05)
                if gate_status.get('operator_allowed') is value:return
            raise RuntimeError('race 운영 승인 수신 확인 실패')
        client=node.create_client(SetParameters,'/team07_hybrid_racer/set_parameters')
        try:
            if not client.wait_for_service(timeout_sec=timeout):raise RuntimeError('제어 노드 파라미터 서비스 없음')
            req=SetParameters.Request();req.parameters=[Parameter('practice_authorized',value=value).to_parameter_msg()]
            future=client.call_async(req);limit=time.monotonic()+timeout
            while not future.done() and time.monotonic()<limit:rclpy.spin_once(node,timeout_sec=.05)
            if not future.done() or future.exception() or not all(v.successful for v in future.result().results):
                raise RuntimeError('연습 주행 승인 변경 실패')
        finally:node.destroy_client(client)
    def snapshot():
        return dict(**result,wall_elapsed_s=time.monotonic()-wall_start,
                    clock=clocks[-1] if clocks else None,pose=pose[:],status=status.copy(),gate_status=gate_status.copy(),
                    sensor_ages={k:time.monotonic()-v[0] for k,v in sensors.items()})
    class CameraFaultVerified(Exception):pass
    previous_signals={}
    def interrupt(signum,frame):raise KeyboardInterrupt
    try:
        if 'GZ_PARTITION' not in os.environ:
            partitions={v for _,_,v in processes() if v}
            if len(partitions)>1:raise RuntimeError('여러 Gazebo partition 감지: GZ_PARTITION을 명시하세요')
            os.environ['GZ_PARTITION']=next(iter(partitions),f'urrc_practice_{os.getuid()}')
        result['partition']=os.environ['GZ_PARTITION']
        result['ros_domain_id']=os.environ.get('ROS_DOMAIN_ID','0')
        say(f"{track}, GZ_PARTITION={result['partition']}, ROS_DOMAIN_ID={result['ros_domain_id']}")
        say('독립 연습 모드: 즉시 자율 출발합니다.' if mode=='practice' else 'race 모드: 카메라의 5등 점등→전체 소등 확인 전에는 출발하지 않습니다.')
        os.environ['XDG_STATE_HOME']=str(logdir/'state')
        os.environ['GZ_SIM_RESOURCE_PATH']=str(track_package/'models')+os.pathsep+os.environ.get('GZ_SIM_RESOURCE_PATH','')
        rclpy.init();node=rclpy.create_node('team07_race_supervisor')
        options=NodeOptions();options.partition=result['partition']
        transport=GzNode(options)
        for signum in (signal.SIGINT,signal.SIGTERM):previous_signals[signum]=signal.signal(signum,interrupt)
        def receive_clock(msg):
            stamp=msg.clock.sec+msg.clock.nanosec*1e-9
            if not clocks or stamp!=clocks[-1]:clock_wall[0]=time.monotonic()
            clocks.append(stamp)
            if mode=='race':clock_timing.append((time.monotonic(),stamp))
        node.create_subscription(RosClock,'/clock',receive_clock,10)
        def gz_camera_receive(msg):
            gz_camera_timing.append((time.monotonic(),msg.header.stamp.sec+msg.header.stamp.nsec*1e-9))
        if mode=='race':transport.subscribe(GzImage,BASE+'/camera/image',gz_camera_receive)
        def receive_pose(msg):
            for value in msg.pose:
                if value.name=='team07_hybrid':
                    pose[:]=[value.position.x,value.position.y,value.position.z,time.monotonic()]
                    native=[h.stamp.sec+h.stamp.nsec*1e-9 for h in (value.header,msg.header)
                        if h.HasField('stamp') and h.stamp.sec+h.stamp.nsec*1e-9>0]
                    if native:
                        stamp=native[0]
                        pose_history.append((stamp,value.position.x,value.position.y,value.position.z))
                        q=value.orientation
                        body_poses.append((stamp,value.position.x,value.position.y,value.position.z,q.x,q.y,q.z,q.w))
        transport.subscribe(Pose_V,f'/world/{track}/pose/info',receive_pose)
        def receive_stats(msg):stats.update(sim_time=msg.sim_time.sec+msg.sim_time.nsec*1e-9,paused=msg.paused)
        transport.subscribe(WorldStatistics,f'/world/{track}/stats',receive_stats)
        def receive_status(msg):
            value=json.loads(msg.data);status.update(value);status_wall[0]=time.monotonic()
            control_history.append({k:value.get(k) for k in ('stamp','speed_command','wheel_speed','wheel_peak_speed','steering',
                'steering_actual','yaw_rate','attitude','reason','wheel_stamp','imu_stamp')})
        node.create_subscription(String,BASE+'/autonomy/status',receive_status,10)
        def receive_gate(msg):
            value=json.loads(msg.data);gate_status.update(value);gate_history.append(value)
        node.create_subscription(String,BASE+'/start/status',receive_gate,10)
        transport.subscribe(Double_V,BASE+'/command_generated',lambda msg:generated_commands.append(list(msg.data)))
        permission_pub=node.create_publisher(Bool,'/race/start/team_07',10)
        if camera_fault:relay=node.create_publisher(Image,BASE+'/test_camera',10)
        for topic in DRIVE_TOPICS:
            def receive_drive(msg,key=topic):
                if clocks:drive_history.append((clocks[-1],key,msg.data))
            transport.subscribe(Double,BASE+'/'+topic,receive_drive)
        def record_camera(msg):
            import cv2
            import numpy as np
            received=time.monotonic()
            observation=lamp_observation(msg)
            stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
            camera_timing.append((received,stamp,clocks[-1] if clocks else None,time.monotonic()-received))
            count=None if observation is None else sum(observation[0])
            camera_stream.write(json.dumps(dict(stamp=stamp,lenses=None if observation is None else observation[0],
                centers=None if observation is None else observation[1]))+'\n')
            if count==5:all_red_seen[0]=True
            after_off=count==0 and all_red_seen[0]
            if after_off:off_seen[0]=True
            name='off_after_red' if after_off else f'red_{count}'
            if count is not None and name not in camera_counts:
                rgb=np.frombuffer(msg.data,dtype=np.uint8).reshape(msg.height,msg.step)[:,:msg.width*3].reshape(msg.height,msg.width,3)
                cv2.imwrite(str(logdir/(name+'.png')),cv2.cvtColor(rgb,cv2.COLOR_RGB2BGR) if msg.encoding=='rgb8' else rgb)
                camera_counts.add(name)
            if relay is not None:
                fault_active[0] |= gate_status.get('state')=='ALL_RED'
                if not fault_active[0]:relay.publish(msg)
                elif camera_fault=='occlude':
                    import copy
                    damaged=copy.deepcopy(msg)
                    data=np.frombuffer(msg.data,dtype=np.uint8).copy().reshape(msg.height,msg.step)
                    if observation is not None:
                        x,y=map(round,observation[1][2])
                        data[max(0,y-9):y+10,max(0,x-9)*3:(x+10)*3]=0
                    else:data[:]=0
                    damaged.data=data.tobytes();relay.publish(damaged)
                    if 'fault_occluded' not in camera_counts:
                        pixels=data[:,:msg.width*3].reshape(msg.height,msg.width,3)
                        cv2.imwrite(str(logdir/'fault_occluded.png'),cv2.cvtColor(pixels,cv2.COLOR_RGB2BGR) if msg.encoding=='rgb8' else pixels)
                        camera_counts.add('fault_occluded')
        stop_unused_subscriptions=[]
        for key,cls,suffix in [('camera',Image,'camera/image'),('camera_info',CameraInfo,'camera/camera_info'),
                ('scan',LaserScan,'scan'),('points',PointCloud2,'lidar3d/points'),('imu',Imu,'imu'),('wheels',JointState,'joint_states')]:
            def receive(msg,kind=key):
                sensors[kind]=(time.monotonic(),msg)
                if kind=='wheels':
                    wheel_history.append(dict(stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9,
                        names=list(msg.name),velocity=list(msg.velocity)))
                if kind in ('scan','points'):
                    stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
                    if stamp-lidar_last.get(kind,-math.inf)>=1.:
                        import numpy as np
                        if kind=='scan':
                            rr=np.asarray(msg.ranges);aa=msg.angle_min+np.arange(len(rr))*msg.angle_increment
                            valid=np.isfinite(rr)&(rr>=msg.range_min)&(rr<msg.range_max*.98)
                            lidar_evidence.append(dict(kind=kind,stamp=stamp,finite_hits=int(valid.sum()),
                                configured_max=msg.range_max,max_range=float(np.max(rr[valid],initial=0)),
                                max_forward_x=float(np.max((rr*np.cos(aa))[valid],initial=0))))
                        else:
                            fields={f.name:f for f in msg.fields}
                            if all(k in fields and fields[k].datatype==7 for k in ('x','y','z')):
                                dtype=np.dtype(dict(names=['x','y','z'],formats=[('>' if msg.is_bigendian else '<')+'f4']*3,
                                    offsets=[fields[k].offset for k in ('x','y','z')],itemsize=msg.point_step))
                                cloud=np.ndarray((msg.height,msg.width),dtype=dtype,buffer=msg.data,strides=(msg.row_step,msg.point_step))
                                xyz=np.stack([cloud[k].ravel() for k in ('x','y','z')],axis=1)
                                rr=np.linalg.norm(xyz,axis=1);valid=np.isfinite(rr)&(rr>=.25)&(rr<39.2)
                                lidar_evidence.append(dict(kind=kind,stamp=stamp,finite_hits=int(valid.sum()),
                                    max_range=float(np.max(rr[valid],initial=0)),max_forward_x=float(np.max(xyz[valid,0],initial=0))))
                        lidar_last[kind]=stamp
                if mode=='race' and kind=='camera':record_camera(msg)
            subscription=node.create_subscription(cls,BASE+'/'+suffix,receive,qos_profile_sensor_data)
            if key in ('camera','camera_info','scan','points'):
                stop_unused_subscriptions.append(subscription)
        # Bound discovery before deciding to start a world.
        until=time.monotonic()+2
        while time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.1)
        names=node.get_node_names()
        if any(name in names for name in ('team07_hybrid_racer','team07_sensor_bridge')):
            raise RuntimeError('기존 참가자 제어/센서 노드 감지: 중복 제어하지 않습니다. 기존 세션을 사용자가 정리한 뒤 재실행하세요.')
        worlds=request('/gazebo/worlds',StringMsg_V)
        available=list(worlds.data) if worlds is not None else []
        if available and available!=[track]:raise RuntimeError('현재 partition의 월드가 다릅니다: '+','.join(available))
        if available:
            result['reused_world']=True;say('기존 Gazebo 월드를 재사용합니다.')
        else:
            if not headless and not os.environ.get('DISPLAY'):raise RuntimeError('DISPLAY 없음: GUI 터미널에서 실행하세요')
            package=track_package
            runtime=ROOT/'.runtime'/'urrc_track_gazebo'
            digest=hashlib.sha256()
            for file in sorted(package.rglob('*')):
                if file.is_file() and '__pycache__' not in file.parts:
                    digest.update(str(file.relative_to(package)).encode());digest.update(file.read_bytes())
            signature=digest.hexdigest();manifest=ROOT/'.runtime/source.sha256'
            if not runtime.exists() or not manifest.exists() or manifest.read_text()!=signature:
                if runtime.exists():runtime.rename(runtime.with_name('previous_'+str(time.time_ns())))
                shutil.copytree(package,runtime,ignore=shutil.ignore_patterns('__pycache__'))
                manifest.write_text(signature)
            command=['bash',str(project/'run_track.sh'),track,'--view','grid','--package',str(runtime)]
            # Official finite-iteration mode leaves signal scheduling to this supervisor.
            # It loads the same race world without starting the operator's countdown early.
            if mode=='race':command+=['--iterations','2147483647']
            if headless:command.append('--server')
            start(command,'official_world');say('공식 맵 실행기를 호출했습니다 (운영 원본 대신 참가자 실행 사본 사용).')
        wait_for(lambda:scene() is not None,'Gazebo scene 서비스',120)
        if not headless:
            wait_for(lambda:any('gz sim gui' in cmd and part==result['partition'] for _,cmd,part in processes()),'Gazebo GUI',60)
            result['gui_verified']=True
        # A ROS publisher alone does not prove that its clock advances.
        if node.count_publishers('/clock'):
            result['reused_clock']=True;say('기존 /clock bridge를 재사용합니다.')
        else:
            start(['ros2','run','ros_gz_bridge','parameter_bridge','/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],'clock_bridge')
            say('/clock bridge 하나를 실행했습니다.')
        wait_for(lambda:len(clocks)>=2 and clocks[-1]>clocks[0]+.1,'진행하는 /clock (일시정지 여부 확인)',60)
        wait_for(lambda:'sim_time' in stats and abs(clocks[-1]-stats['sim_time'])<.3,'ROS/Gazebo 시계 일치',30)
        if stats.get('paused'):raise RuntimeError('월드가 일시정지 상태입니다. 운영자 상태를 변경하지 않습니다.')
        information=scene()
        existing=[model for model in information.model if model.name=='team07_hybrid']
        if len(existing)>1:raise RuntimeError('동일 이름 차량이 여러 개 있습니다')
        if existing and brake_diagnostics:raise RuntimeError('진단은 새 차량에만 적용합니다. 기존 차량/제어기는 변경하지 않습니다.')
        if existing:
            result['reused_vehicle']=True;say('기존 차량을 재사용합니다. 위치를 재설정하지 않습니다.')
        else:
            metadata=json.loads((track_package/'tracks/processed'/f'{track}.json').read_text())
            grid=metadata['grid'][0];result['spawn_grid']=grid
            vehicle_file=ROOT/'model.sdf'
            if brake_diagnostics:
                diagnostic=ET.parse(vehicle_file)
                plugin=ET.SubElement(diagnostic.getroot().find('model'),'plugin',
                    filename=str(ROOT/'.install/team07_hybrid_racer/lib/libteam07_brake_observer.so'),name='team07::BrakeObserver')
                ET.SubElement(plugin,'output_file').text=str(logdir/'stock_pi_observer.jsonl')
                vehicle_file=logdir/'diagnostic_vehicle.sdf';diagnostic.write(vehicle_file,encoding='unicode')
                result['brake_diagnostics']='stock PI unchanged; read-only effort/contact observer'
            subprocess.run(['ros2','run','ros_gz_sim','create','-world',track,'-file',str(vehicle_file),'-name','team07_hybrid',
                '-x',str(grid['x']),'-y',str(grid['y']),'-z',str(grid['road_z']+.005),'-Y',str(grid['yaw'])],
                check=True,timeout=45,stdout=subprocess.DEVNULL)
            say('1번 그리드에 차량을 최초 생성했습니다.')
        def vehicle_exists():
            value=scene()
            return value is not None and any(m.name=='team07_hybrid' for m in value.model)
        wait_for(vehicle_exists,'차량',30)
        launch=['ros2','launch','team07_hybrid_racer','race.launch.xml','race_start_mode:=camera',
                f'cruise_speed:={cruise}','record_control_trace:=true']
        if camera_fault:launch.append('camera_topic:='+BASE+'/test_camera')
        trace_path=logdir/'planner_trace.jsonl'
        start(launch,'autonomy_launch',dict(os.environ,TEAM07_RACE_CONTROL_TRACE=str(trace_path)) if course is not None else None)
        wait_for(lambda:all(k in sensors and time.monotonic()-sensors[k][0]<1. for k in ('camera','camera_info','scan','points','imu','wheels'))
                 and status.get('reason')=='awaiting_start' and pose,'센서/제어/실제 위치',120)
        if sensors['scan'][1].header.frame_id!='team07_lidar':raise RuntimeError('LiDAR 좌표계 불일치')
        if not set(DRIVEN_WHEELS)<=set(sensors['wheels'][1].name):raise RuntimeError('바퀴 관절 불일치')
        result['sensors_verified']=list(sensors)
        baseline=pose[:3];start_clock=clocks[-1]
        set_authorized(False)
        say('센서·시계·제어 준비 완료. 연습 주행을 승인했습니다.' if mode=='practice' else '카메라·제어 준비 완료. 출발 권한은 영상 게이트가 결정합니다.')
        if mode=='race':
            wait_for(lambda:gate_status.get('state')=='WAITING' and 'camera' in sensors,'카메라 게이트',30)
            if not result['reused_world']:
                ready=logdir/'vehicle_ready';ready.touch()
                start([sys.executable,'-B',str(runtime/'scripts/traffic_lights_v10.py'),'--track',track,
                    '--ready-file',str(ready)],'official_signal')
                # A completed light sequence is not a failed long-running service.
                children[-1]=(children[-1][0]+'_finite',children[-1][1])
            else:say('기존 월드는 운영자의 새 신호 순서를 기다립니다. 신호 상태를 재설정하지 않습니다.')
            if camera_fault:
                wait_for(lambda:bool(gate_status.get('reason')) and off_seen[0],'영상 장애 후 소등/출발 차단',90)
                raise CameraFaultVerified()
            def gate_open():
                if gate_status.get('reason'):raise RuntimeError('출발 게이트 차단: '+gate_status['reason'])
                return gate_status.get('authorized',False)
            wait_for(gate_open,'실제 5등 점등/전체 소등 카메라 확인',90)
            go=gate_status['times']['go']
            baseline_record=min(pose_history,key=lambda v:abs(v[0]-go))
            baseline=list(baseline_record[1:4])
            # The local verifier exercises the operator veto after camera GO.
            # The competition controller only receives Bool from the organizer.
            wait_for(lambda:clocks[-1]>=go+.4,'GO 후 운영 승인 보류 검증',10)
            result['operator_permission_sent_stamp']=clocks[-1]
            authorized=True
            set_authorized(True)
        threshold=.01 if mode=='race' else .03
        def actual_motion_confirmed():
            moving=bool(pose and math.hypot(pose[0]-baseline[0],pose[1]-baseline[1])>threshold
                        and abs(status.get('wheel_speed',0))>.05)
            if mode=='race' and not moving and clocks[-1]-gate_status['times']['first_red']>20:
                raise RuntimeError('첫 빨간불부터 20초 이내 실제 이동이 관측되지 않았습니다')
            return moving
        wait_for(actual_motion_confirmed,'실제 차량 출발',60)
        if status.get('path_mode')!='center' or status.get('selected_reference')!='completion_center':
            raise RuntimeError('실제 중앙 경로 추종 상태가 확인되지 않았습니다')
        result['moved']=True;move_clock=clocks[-1];result['movement_sim_time']=move_clock-start_clock
        if mode=='race':
            gate_times=dict(gate_status['times'])
            measured=next(v[0] for v in pose_history if v[0]>=gate_times['go'] and math.hypot(v[1]-baseline[0],v[2]-baseline[1])>.01)
            gate_times['actual_motion']=measured
            result['movement_detection_stamp']=move_clock
            result['signal_times']=gate_times
            result['first_red_to_motion_s']=measured-gate_times['first_red']
            result['off_observed_to_motion_s']=measured-gate_times['off_observed']
            result['go_to_motion_s']=measured-gate_times['go']
            if result['first_red_to_motion_s']>20:raise RuntimeError('실제 이동이 첫 빨간불부터 20초를 초과했습니다')
        say(f"실제 차량 출발 확인 ({result['movement_sim_time']:.2f}s). Ctrl+C로 이 실행기가 소유한 프로세스만 종료합니다.")
        heartbeat=time.monotonic();result['success']=True
        while duration is None or clocks[-1]-move_clock<duration:
            tick()
            if not stats.get('paused'):
                if time.monotonic()-clock_wall[0]>3:raise RuntimeError('실행 중 /clock 정지')
                if time.monotonic()-status_wall[0]>3:raise RuntimeError('제어 상태 수신 중단')
                if any(time.monotonic()-value[0]>3 for value in sensors.values()):raise RuntimeError('실행 중 센서 수신 중단')
            if status.get('reason') in ('highspeed_yaw_or_reverse_stop','highspeed_ceiling_withdrawn'):
                raise RuntimeError('고속 안전 중단: '+status['reason'])
            if time.monotonic()-heartbeat>=5:
                heartbeat=time.monotonic()
                say(f"sim={clocks[-1]:.1f}s speed={status.get('wheel_speed',0):.2f}m/s 상태={status.get('reason','unknown')}")
                (logdir/'live.json').write_text(json.dumps(snapshot(),indent=2))
        result['termination']='duration_completed'
    except CameraFaultVerified:
        result.update(fault_test_passed=True,success=True,termination='camera_fault_blocked')
    except KeyboardInterrupt:
        result['termination']='interrupted';say('종료 요청: 소유한 프로세스를 정리합니다.')
    except Exception as exc:
        result.update(success=False,error=str(exc),termination='error');say('오류: '+str(exc))
    finally:
        for signum in previous_signals:signal.signal(signum,signal.SIG_IGN)
        # Disable only our launch; never send stop commands to somebody else's controller.
        if authorized and node is not None and rclpy.ok():
            try:
                stop_request_stamp=clocks[-1] if clocks else None
                result['stop_request_stamp']=stop_request_stamp
                set_authorized(False,5)
                # Stop recording large perception messages in this supervisor
                # while it collects stop evidence. The controller's sensor
                # subscriptions and all verifier thresholds stay active.
                for subscription in stop_unused_subscriptions:
                    node.destroy_subscription(subscription)
                until=time.monotonic()+30
                last_assessment=-math.inf
                while time.monotonic()<until:
                    rclpy.spin_once(node,timeout_sec=.05)
                    if not clocks or stop_request_stamp is None:break
                    if clocks[-1]-last_assessment<.2:continue
                    last_assessment=clocks[-1]
                    stop_evidence=(list(body_poses),list(control_history),list(generated_commands))
                    assessment=race_brake_assessment(*stop_evidence,stop_request_stamp)
                    result['braking']=assessment
                    pose_fresh=bool(body_poses and -.05<=clocks[-1]-body_poses[-1][0]<.15)
                    if assessment['body_stop_confirmed'] and pose_fresh:break
                    if clocks[-1]-stop_request_stamp>=8.:break
                if stop_evidence is not None:
                    result['braking']=race_brake_assessment(*stop_evidence,stop_request_stamp)
                    result['stop_observation_end_stamp']=stop_evidence[0][-1][0] if stop_evidence[0] else None
                if not result['braking']['body_stop_confirmed']:
                    result.update(success=False,stop_error='Body rest was not verified; wheel rest/timeout is insufficient')
            except Exception as exc:result.update(success=False,stop_error=str(exc))
        if stop_request_stamp is not None and stop_evidence is None:
            stop_evidence=(list(body_poses),list(control_history),list(generated_commands))
        for name,proc in reversed(children):
            if proc.poll() is None:
                os.killpg(proc.pid,signal.SIGINT)
                try:proc.wait(timeout=20)
                except subprocess.TimeoutExpired:
                    os.killpg(proc.pid,signal.SIGTERM)
                    try:proc.wait(timeout=10)
                    except subprocess.TimeoutExpired:
                        os.killpg(proc.pid,signal.SIGKILL);proc.wait()
        (logdir/'body_poses.json').write_text(json.dumps(body_poses))
        (logdir/'wheel_history.json').write_text(json.dumps(wheel_history))
        (logdir/'control_history.json').write_text(json.dumps(control_history))
        (logdir/'command_generated.json').write_text(json.dumps(generated_commands))
        if stop_request_stamp is not None:
            assessment=race_brake_assessment(*stop_evidence,stop_request_stamp,True)
            assessment['evidence_frozen_before_teardown']=True
            (logdir/'braking.json').write_text(json.dumps(assessment,indent=2))
            result['braking']={k:v for k,v in assessment.items() if k!='body_samples'}
            if not assessment['body_stop_confirmed']:result['success']=False
        if mode=='race':
            trace_path=logdir/'planner_trace.jsonl'
            if trace_path.exists() and result.get('moved'):
                try:
                    with trace_path.open() as stream:
                        metrics=performance(body_poses,[json.loads(line) for line in stream if line.endswith('\n')],
                            course,result.get('signal_times',{}),stop_request_stamp)
                    if metrics is not None:
                        (logdir/'performance.json').write_text(json.dumps(metrics,indent=2))
                        result['performance']=metrics
                except (ValueError,OSError) as exc:
                    result['performance_error']=str(exc)
            (logdir/'camera_timing.json').write_text(json.dumps(dict(gz=gz_camera_timing,ros=camera_timing,clock=clock_timing,stats=stats,loadavg=Path('/proc/loadavg').read_text())))
            (logdir/'gate_history.json').write_text(json.dumps(gate_history,indent=2))
            (logdir/'lidar_evidence.json').write_text(json.dumps(lidar_evidence))
            (logdir/'poses.json').write_text(json.dumps(pose_history))
            (logdir/'drive_commands.json').write_text(json.dumps(drive_history))
            result['camera_frames_saved']=sorted(camera_counts)
            result['fault_active']=fault_active[0]
            go=gate_status.get('times',{}).get('go')
            if go is not None:
                result['pre_go_nonzero_status']=any(v.get('command_speed',0)!=0 for v in gate_history if not v.get('authorized',False))
                result['pre_go_nonzero_drive']=any(abs(v[2])>1e-9 for v in drive_history if v[0]<go-.05)
                permission=result.get('operator_permission_sent_stamp',go)
                result['pre_permission_nonzero_drive']=any(abs(v[2])>1e-9 for v in drive_history if v[0]<permission-.05)
            elif camera_fault:
                result['pre_go_nonzero_status']=any(v.get('command_speed',0)!=0 for v in gate_history)
                result['pre_go_nonzero_drive']=any(abs(v[2])>1e-9 for v in drive_history)
                if result['pre_go_nonzero_drive']:result.update(success=False,fault_test_passed=False)
            first_red=gate_status.get('times',{}).get('first_red')
            if first_red is not None:
                pre=[v for v in pose_history if first_red<=v[0] and (go is None or v[0]<go)]
                result['pre_go_displacement_m']=max((math.hypot(v[1]-pre[0][1],v[2]-pre[0][2]) for v in pre),default=0.)
            if result.get('pre_permission_nonzero_drive') or result.get('pre_go_nonzero_drive') or result.get('pre_go_nonzero_status') or result.get('pre_go_displacement_m',0)>.002:
                result.update(success=False,safety_error='pre_GO command or motion interlock failed')
                if camera_fault:result['fault_test_passed']=False
        camera_stream.close()
        result.update(snapshot());result['owned_processes']=[name for name,_ in children]
        (logdir/'results.json').write_text(json.dumps(result,indent=2))
        for stream in handles:stream.close()
        if transport is not None:
            transport.unsubscribe(f'/world/{track}/pose/info')
            transport.unsubscribe(f'/world/{track}/stats')
            transport.unsubscribe(BASE+'/command_generated')
            for topic in DRIVE_TOPICS:transport.unsubscribe(BASE+'/'+topic)
        if node is not None:node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
        for signum,handler in previous_signals.items():signal.signal(signum,handler)
        say('결과: '+str(logdir/'results.json'))
    return 0 if result.get('success') else 1


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--brake-diagnostics',action='store_true',help='Read-only stock PI effort/contact capture; race, cruise<=0.5, duration<=5')
    p.add_argument('--experimental-race-speed',action='store_true',help='Monza trial; plan goal up to 4.17m/s, existing command ceiling 4m/s; requires --duration <=30')
    p.add_argument('--cruise',type=float,default=.5,help='Monza validated ceiling from .runtime/highspeed_limit.json; Hungaroring ceiling 0.5m/s')
    p.add_argument('--run-mode',choices=('practice','race'),default='practice')
    p.add_argument('--run-race',choices=('monza','hungaroring'))
    p.add_argument('--duration',type=float,help='One-click smoke-test duration after movement, in simulation seconds')
    p.add_argument('--path-mode',choices=('auto','center','left_wall'),default='auto',help='auto: center; submission Race requires center')
    p.add_argument('--left-wall-distance',type=float,default=.30)
    p.add_argument('--headless',action='store_true')
    p.add_argument('--test-track',choices=('monza','hungaroring'),default='monza')
    p.add_argument('--test-cruise',type=float,default=.5)
    p.add_argument('--test-wall-limit',type=float)
    p.add_argument('--test-brake-angle',type=float,help='Verifier-only matched encoder/command angle at brake onset (rad)')
    p.add_argument('--test-brake-speed',type=float,help='Verifier only: stop after measured speed is reached; no truth input to planner')
    p.add_argument('--test-fast',action='store_true',help='Unthrottled private physics world; same integration step')
    p.add_argument('--test-start-station',type=float,help='Independent corner/slope test initial spawn only; never moves an existing car')
    p.add_argument('--autonomy-check',action='store_true')
    p.add_argument('--autonomy-test',type=float,metavar='SIM_SECONDS')
    p.add_argument('--vision-test',action='store_true')
    p.add_argument('--start-test',choices=('monza','hungaroring'))
    p.add_argument('--capture-only',action='store_true')
    p.add_argument('--camera-fault',choices=('drop','occlude'))
    p.add_argument('--review', action='store_true')
    p.add_argument('--sensor-test', action='store_true')
    p.add_argument('--control-test', action='store_true')
    p.add_argument('--monza-brake-replay',action='store_true',help='Private: replay recorded 3m/s Monza inputs in the original world')
    p.add_argument('--brake-ab-monza-source',type=Path,help='Private low-speed Monza ground A/B from a recorded race; no public policy change')
    p.add_argument('--brake-ab-low-speed',action='store_true',help='Private matched 0.5m/s fixture; no race policy change')
    p.add_argument('--ab-library',type=Path,help='Private A/B candidate library under team_07')
    p.add_argument('--brake-fixture',choices=('baseline','candidate'),help='Private deterministic uniform-ground brake comparison')
    p.add_argument('--physics-test', action='store_true')
    p.add_argument('--generate', action='store_true')
    p.add_argument('--check', action='store_true')
    p.add_argument('--spawn', metavar='WORLD', help='Explicit practice-only spawn into an already running world')
    p.add_argument('--pose', nargs=4, type=float, metavar=('X','Y','Z','YAW'))
    a, ros_args = p.parse_known_args()
    if ros_args and ros_args[0]!='--ros-args':p.error('unrecognized arguments: '+' '.join(ros_args))
    if not math.isfinite(a.left_wall_distance) or a.left_wall_distance<=0:p.error('positive finite wall distance required')
    if a.run_race and (a.brake_ab_low_speed or a.ab_library or a.brake_ab_monza_source):p.error('Private A/B options cannot activate the public race candidate')
    if a.brake_ab_monza_source and (not a.brake_ab_low_speed or not a.brake_fixture or a.monza_brake_replay):p.error('Monza low-speed A/B requires --brake-ab-low-speed and --brake-fixture')
    if (a.brake_ab_low_speed or a.ab_library) and (not a.brake_fixture or a.monza_brake_replay):p.error('low-speed A/B requires --brake-fixture and excludes Monza replay')
    if a.ab_library and not a.brake_ab_low_speed:p.error('--ab-library requires --brake-ab-low-speed')
    if a.monza_brake_replay and not a.brake_fixture:p.error('--monza-brake-replay requires --brake-fixture')
    if a.test_brake_angle is not None and (a.test_brake_speed is None or not math.isfinite(a.test_brake_angle) or abs(a.test_brake_angle)>CENTER_LIMIT):p.error('finite test brake angle requires --test-brake-speed')
    if a.test_brake_speed is not None and a.autonomy_test is None:p.error('--test-brake-speed requires --autonomy-test')
    if a.experimental_race_speed and (a.run_race!='monza' or a.duration is None or not math.isfinite(a.duration) or not 0<a.duration<=30):p.error('--experimental-race-speed requires --run-race monza and --duration 0..30')
    if a.brake_diagnostics and (a.run_race!='monza' or a.run_mode!='race' or a.duration is None or not 0<a.duration<=5 or not .1<=a.cruise<=.5):p.error('--brake-diagnostics requires Monza race, cruise 0.1..0.5 and duration 0..5')
    if a.run_race:
        if a.duration is not None and (not math.isfinite(a.duration) or a.duration<=0):p.error('positive duration required')
        maximum=(round(RACE_STRAIGHT_GOAL_MPS,2) if a.experimental_race_speed else cruise_limit()) if a.run_race=='monza' else .5
        if not math.isfinite(a.cruise) or not .1<=a.cruise<=maximum:p.error(f'one-click {a.run_race} cruise must be 0.1..{maximum}')
        if a.camera_fault and a.run_mode!='race':p.error('camera fault test requires race mode')
        raise SystemExit(run_race(a.run_race,a.duration,a.headless,a.cruise,a.run_mode,a.camera_fault,a.path_mode,a.left_wall_distance,a.experimental_race_speed,a.brake_diagnostics))
    elif a.autonomy_check:
        autonomy_checks()
    elif a.autonomy_test is not None:
        if not math.isfinite(a.autonomy_test) or a.autonomy_test<=0:p.error('positive finite duration required')
        if a.test_start_station is not None and not math.isfinite(a.test_start_station):p.error('finite station required')
        maximum=cruise_limit(True) if a.test_track=='monza' else .5
        if not math.isfinite(a.test_cruise) or not .1<=a.test_cruise<=maximum:p.error(f'test cruise must be 0.1..{maximum}m/s')
        if a.test_brake_speed is not None and (not math.isfinite(a.test_brake_speed) or not .1<=a.test_brake_speed<=a.test_cruise):p.error('test brake speed must be positive and <=test cruise')
        if a.test_wall_limit is not None and (not math.isfinite(a.test_wall_limit) or a.test_wall_limit<=0):p.error('positive wall limit required')
        autonomy_test(a.autonomy_test,a.test_start_station,a.test_cruise,a.test_wall_limit,a.test_fast,a.test_track,a.test_brake_speed,a.test_brake_angle,resolve_path_mode(a.path_mode,'practice'),a.left_wall_distance)
    elif a.vision_test:
        vision_test()
    elif a.start_test:
        start_test(a.start_test,a.capture_only,a.camera_fault)
    elif a.review:
        design_review()
    elif a.sensor_test:
        sensor_test()
    elif a.control_test:
        control_test()
    elif a.brake_fixture:
        matched_brake_test(a.brake_fixture,a.monza_brake_replay,a.brake_ab_low_speed,a.ab_library,a.brake_ab_monza_source)
    elif a.physics_test:
        physics_test()
    elif a.generate:
        (ROOT/'model.sdf').write_text(model_xml()+'\n')
    elif a.check:
        root = ET.parse(ROOT/'model.sdf').getroot()
        assert ET.tostring(root) == ET.tostring(ET.fromstring(model_xml())), 'Model differs from generator'
        assert len(root.findall('.//link')) == 10
        for link in root.findall('.//link'):
            assert ET.tostring(link.find('collision/geometry')) == ET.tostring(link.find('visual/geometry'))
        for n in range(-100,101):
            values = targets(1., n/100)
            assert max(abs(v) for v in values[:2]) <= LIMIT+1e-12
        # Swept wheel half-width <= .08 + hypot(.045,.013), length <= .42.
        assert 2*(TRACK/2 + math.hypot(RADIUS,.013)) < .25
        metrics = vehicle_metrics()
        assert metrics['straight_size_m'][0] <= .55 and metrics['swept_width_bound_m'] <= .25 and metrics['top_m'] <= .18
        print('Geometry/steering limits PASS:',metrics)
    elif a.spawn:
        if a.pose is None or not all(math.isfinite(v) for v in a.pose):
            p.error('--spawn requires explicit finite --pose X Y Z YAW; Z is model ground-projection origin')
        subprocess.run(['ros2','run','ros_gz_sim','create','-world',a.spawn,'-file',str(ROOT/'model.sdf'),'-name','team07_hybrid','-x',str(a.pose[0]),'-y',str(a.pose[1]),'-z',str(a.pose[2]),'-Y',str(a.pose[3])],check=True,timeout=45)
    else:
        control()


if __name__ == '__main__':
    main()
