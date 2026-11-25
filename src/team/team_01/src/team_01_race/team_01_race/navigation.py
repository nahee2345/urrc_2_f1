"""Camera path selection and LiDAR footprints; sensor-independent testable logic."""
from dataclasses import dataclass
import math
import numpy as np
from .lidar_filter import good_polyline
from .corridor_geometry import footprint_distance,corridor_membership,boundary_ok
from .obstacle_pass import ObstaclePassPlanner,inside_corridor
from .path_curvature import supported_curvature


@dataclass
class Obstacle:
    id: str
    x: float
    y: float
    width: float
    depth: float
    confidence: float
    closing_speed: float = 0.0

    @property
    def min_x(self):return self.x-self.depth/2
    @property
    def max_x(self):return self.x+self.depth/2
    @property
    def min_y(self):return self.y-self.width/2
    @property
    def max_y(self):return self.y+self.width/2
    @property
    def closest_distance(self):return math.hypot(max(0.,abs(self.x)-self.depth/2),max(0.,abs(self.y)-self.width/2))


class ObstacleTracker:
    def __init__(self, confirm_frames=3, match_distance=0.8, timeout=0.3, release_frames=3):
        self.confirm_frames, self.match_distance, self.timeout = confirm_frames, match_distance, timeout
        self.release_frames = release_frames
        self.tracks = {}
        self.next_id = 0

    def update(self, candidates, now):
        self.tracks = {k: v for k, v in self.tracks.items() if now-v['time'] <= self.timeout}
        matched = set()
        for x, y, width, depth in candidates:
            options = [(math.hypot(x-t['x'], y-t['y']), key) for key, t in self.tracks.items() if key not in matched]
            distance, key = min(options, default=(math.inf, None))
            if distance > self.match_distance:
                key = str(self.next_id)
                self.next_id += 1
                old = None
            else:
                old = self.tracks[key]
            dt = now-old['time'] if old else 0.0
            closing = (old['x']-x)/dt if old and dt > 1e-4 else 0.0
            if old:
                closing = 0.5*max(0.0, closing)+0.5*old['closing']
            self.tracks[key] = dict(x=x, y=y, width=width, depth=depth, time=now,
                                    count=old['count']+1 if old else 1, closing=max(0.0, closing), misses=0)
            matched.add(key)
        for key, track in list(self.tracks.items()):
            if key not in matched:
                track['misses'] += 1
                if track['count'] < self.confirm_frames or track['misses'] >= self.release_frames:
                    del self.tracks[key]
        return [Obstacle(k, t['x'], t['y'], t['width'], t['depth'],
                         min(1.0, t['count']/self.confirm_frames), t['closing'])
                for k, t in self.tracks.items()
                if t['count'] >= self.confirm_frames and now-t['time'] <= self.timeout]


def scan_candidates(ranges, angle_min, angle_increment, range_min, range_max, *,
                    lidar_x, lidar_y, lidar_yaw, horizon=5.0, cluster_gap=0.14,
                    min_width=0.12, max_width=0.80, min_depth=0.10, max_depth=1.20,
                    min_valid_beams=20, wall_attachment_gap=0.30):
    """Only compact footprints. No wall labels or centerline are constructed."""
    if (not ranges or not math.isfinite(angle_min) or not math.isfinite(angle_increment) or
            angle_increment <= 0 or not math.isfinite(range_min) or not math.isfinite(range_max) or
            range_min < 0 or range_max <= range_min):
        return False, []
    groups, valid, previous_index = [], 0, -2
    for i, distance in enumerate(ranges):
        angle = angle_min+i*angle_increment
        if not -math.pi/2 <= angle <= math.pi/2:
            continue
        if math.isinf(distance) and distance > 0:
            valid += 1
            continue
        if not math.isfinite(distance) or not range_min <= distance <= range_max:
            continue
        valid += 1
        if distance > horizon:
            continue
        point = np.array([lidar_x+distance*math.cos(angle+lidar_yaw),
                          lidar_y+distance*math.sin(angle+lidar_yaw)])
        if point[0] <= 0:
            continue
        if not groups or i != previous_index+1 or np.linalg.norm(point-groups[-1][-1]) > cluster_gap:
            groups.append([])
        groups[-1].append(point)
        previous_index = i
    candidates = []
    elongated = [np.asarray(group) for group in groups if len(group) >= 3 and
                 (np.ptp(np.asarray(group)[:,0]) > max_depth or
                  np.ptp(np.asarray(group)[:,1]) > max_width)]
    for group in groups:
        if len(group) < 3:
            continue
        points = np.asarray(group)
        low, high = points.min(axis=0), points.max(axis=0)
        depth, width = high-low
        if min_width <= width <= max_width and depth <= max_depth:
            # Range discontinuities can split a long wall into an apparently
            # compact end fragment. Reject fragments still attached spatially
            # to elongated returns; this never creates a wall path/centerline.
            attached = any(np.min(np.sum((points[:,None,:]-wall[None,:,:])**2,axis=2))
                           < wall_attachment_gap**2 for wall in elongated)
            if attached:
                continue
            # A frontal vehicle face can have almost zero measured depth.
            center = (low+high)/2
            candidates.append((float(center[0]), float(center[1]), float(width), float(max(min_depth, depth))))
    return valid >= min_valid_beams, candidates


def path_geometry(path):
    x, y = path[:, 0], path[:, 1]
    slope = np.gradient(y, x)
    curvature = np.gradient(slope, x)/(1+slope*slope)**1.5
    return np.arctan(slope), curvature


def valid_path(path, horizon=5.0):
    return (isinstance(path, np.ndarray) and path.ndim == 2 and len(path) >= 3 and path.shape[1] == 2 and
            np.isfinite(path).all() and np.all(path[:, 0] > 0) and
            np.all(np.diff(path[:, 0]) > 0) and path[-1, 0] <= horizon+1e-5 and
            np.max(np.abs(np.diff(path[:, 1]))) < 0.35)


def valid_boundary(path,horizon=5.0):
    # Measured walls are ordered polylines; a vertical or folding boundary
    # is valid even though the downstream PP trajectory must be X-monotone.
    return boundary_ok(path,horizon)


def temporal_y(xs,history):
    """Linear extension avoids artificial curvature at a moved sample boundary."""
    ys=np.interp(xs,history[:,0],history[:,1])
    for index,neighbor,mask in ((0,1,xs<history[0,0]),(-1,-2,xs>history[-1,0])):
        slope=(history[index,1]-history[neighbor,1])/(history[index,0]-history[neighbor,0])
        ys[mask]=history[index,1]+slope*(xs[mask]-history[index,0])
    return ys


def temporal_merge(path,history,alpha):
    result=path.copy();x=path[:,0]
    support=(x>=history[0,0]-1e-8)&(x<=history[-1,0]+1e-8)
    weight=np.full(len(path),alpha)
    if path[-1,0]>history[-1,0]+.05:
        u=np.clip((x-(history[-1,0]-.5))/.5,0,1)
        weight=alpha+(1-alpha)*u*u*(3-2*u)
    result[support,1]=(1-weight[support])*temporal_y(x[support],history)+weight[support]*path[support,1]
    return result


def pure_pursuit(path, speed, wheelbase, min_lookahead=1.0, max_lookahead=3.0, max_speed=10.0, lookahead_override=None):
    if not valid_path(path):
        raise ValueError('invalid selected trajectory')
    heading, curvature = path_geometry(path)
    curve_scale = min(1.0, float(np.max(np.abs(curvature)))/0.75)
    speed_scale = min(1.0, max(0.0, speed)/max_speed)
    # Preserve the existing 1..3 m range. Speed increases and curvature shortens it.
    lookahead = min_lookahead+(max_lookahead-min_lookahead)*float(np.clip(.25+.75*speed_scale-.6*curve_scale, 0, 1))
    lookahead = float(np.clip(lookahead, min_lookahead, max_lookahead))
    if lookahead_override is not None:
        lookahead = float(np.clip(lookahead_override, min_lookahead, max_lookahead))
    distances = np.linalg.norm(path, axis=1)
    candidates = np.flatnonzero(distances >= lookahead)
    target = path[candidates[0] if len(candidates) else -1]
    length_squared = float(target @ target)
    if target[0] <= 0 or length_squared < 1e-6:
        raise ValueError('target is not in front')
    internal_left_positive = math.atan2(2*wheelbase*float(target[1]), length_squared)
    return internal_left_positive, target.copy(), lookahead


class DecisionEngine:
    def __init__(self, *, vehicle_width, wheelbase, horizon=5.0, cruise=10.0,
                 temporal_alpha=0.30, history_sec=0.5, confirm_frames=3,
                 lateral_margin=0.15, longitudinal_margin=0.30,
                 max_lateral_accel=3.0, min_confidence=0.40,
                 emergency_ttc=0.6, caution_ttc=1.5, max_decel=2.5,
                 camera_timeout=0.25, lidar_timeout=0.25, max_sensor_skew=0.15,
                 require_official=True, yellow_behavior='stop', start_mode='red_lights_out', rear_overhang=0.0,
                 front_obstacle_stop_distance=0.5):
        self.width, self.wheelbase = vehicle_width, wheelbase
        self.rear_overhang = rear_overhang
        self.horizon, self.cruise = horizon, cruise
        self.supported_preview_distance = 0.0
        self.alpha, self.history_sec, self.confirm_frames = temporal_alpha, history_sec, confirm_frames
        self.margin, self.long_margin = lateral_margin, longitudinal_margin
        self.max_lateral_accel, self.min_confidence = max_lateral_accel, min_confidence
        self.emergency_ttc, self.caution_ttc, self.max_decel = emergency_ttc, caution_ttc, max_decel
        if not math.isfinite(front_obstacle_stop_distance) or front_obstacle_stop_distance<=0:
            raise ValueError('invalid front obstacle stop distance')
        self.front_obstacle_stop_distance=front_obstacle_stop_distance
        self.camera_timeout, self.lidar_timeout, self.max_sensor_skew = camera_timeout, lidar_timeout, max_sensor_skew
        if yellow_behavior not in ('stop','hold') or start_mode not in ('red_lights_out','green'):
            raise ValueError('invalid traffic policy')
        self.require_official, self.yellow_behavior, self.start_mode = require_official, yellow_behavior, start_mode
        self.official = False
        self.visual_go = False
        self.light = 'UNKNOWN'
        self.controller_ready = False
        self.external_emergency = False
        self.camera_time = self.lidar_time = -math.inf
        self.camera_stamp = self.lidar_stamp = None
        self.geometry_motion_time=None
        self.last_key = None
        self.path = self.left = self.right = None
        self.confidence = 0.0
        self.observed = (False, False)
        self.ready_count = 0
        self.path_confirmed = False
        self.pending_jump_direction = 0
        self.pending_jump_count = 0
        self.pending_camera_path = None
        self.obstacles = []
        self.last_selected = None
        self.pass_side = 'CENTER'
        self.pass_id = None
        self.pass_observation = None
        self.pass_observation_time = -math.inf
        self.pass_offset = 0.0
        self.actual_speed = 0.0
        self.last_camera_issue = 'no_camera_path'
        self.last_geometry_delta=None
        self.pass_planner=ObstaclePassPlanner(vehicle_width,wheelbase,lateral_margin,longitudinal_margin,rear_overhang)
        self.path_source='CAMERA_PATH'

    def _reset(self):
        self.pass_planner.reset()
        self.ready_count = 0
        self.path_confirmed = False
        self.path = self.left = self.right = None
        self.last_selected = None
        self.camera_time = -math.inf
        self.last_key = None
        self.pass_side, self.pass_id = 'CENTER', None
        self.pass_observation = None
        self.pending_jump_count = 0
        self.pending_camera_path = None

    def set_official(self, value):
        value = bool(value)
        if self.require_official and self.official != value:
            self._reset()
        self.official = value

    def set_light(self, state):
        self.light = state
        stop = state == 'RED' or (state == 'YELLOW' and self.yellow_behavior == 'stop')
        if stop:
            if self.visual_go:
                self._reset()
            self.visual_go = False
        elif state == ('LIGHTS_OUT' if self.start_mode == 'red_lights_out' else 'GREEN'):
            if not self.visual_go:
                self._reset()
            self.visual_go = True
        # UNKNOWN cannot grant a start. After passing the gantry it does not
        # revoke a previously observed, confirmed visual start.

    def camera(self, path, left, right, confidence, observed, key, now, stamp=None):
        if key == self.last_key:
            return
        self.last_key = key
        self.last_camera_issue='path_invalid'
        self.last_geometry_delta=None
        good = valid_path(path, self.horizon) and valid_boundary(left, self.horizon) and valid_boundary(right, self.horizon)
        if not good or not math.isfinite(confidence) or not self.min_confidence <= confidence <= 1.0:
            self.last_camera_issue='low_path_confidence' if good else 'boundary_invalid' if valid_path(path,self.horizon) else ('invalid_target' if isinstance(path,np.ndarray) and len(path) and np.any(path[:,0]<=0) else 'path_invalid')
            self.ready_count = 0
            self.pending_camera_path = None
            self.pending_jump_count = 0
            return
        if not inside_corridor(path,left,right,0.):
            self.ready_count=0
            return
        old = self.path
        if old is not None and now-self.camera_time <= self.history_sec:
            old_y = temporal_y(path[:,0],old)
            common=(path[:,0]>=old[0,0])&(path[:,0]<=old[-1,0]-.15)
            jump = float(np.max(np.abs(path[common, 1]-old_y[common]))) if common.any() else 0.
            # The acceptance guard must use the same finite spatial support
            # as speed/steering checks, rather than endpoint derivative spikes.
            current_support=path[common]
            old_support=old[(old[:,0]>=current_support[0,0])&(old[:,0]<=current_support[-1,0])] if len(current_support)>=3 else old[:0]
            current_curvature=float(np.max(abs(supported_curvature(current_support)))) if len(current_support)>=3 else 0.
            previous_curvature=float(np.max(abs(supported_curvature(old_support)))) if len(old_support)>=3 else 0.
            self.raw_jump_curvature=float(np.max(abs(path_geometry(path)[1])))
            self.supported_jump_curvature=current_curvature
            new_heading,_=path_geometry(path)
            old_heading,_=path_geometry(old)
            new_near=new_heading[common][:15];old_near=np.interp(path[common,0],old[:,0],old_heading)[:15]
            delta=float(np.median(new_near)-np.median(old_near)) if len(new_near) else 0.
            heading_delta=abs(math.atan2(math.sin(delta),math.cos(delta)))
            if len(current_support)>=3 and len(old_support)>=3:
                self.last_geometry_delta=dict(lateral_jump=jump,heading_jump=math.atan2(math.sin(delta),math.cos(delta)),
                    curvature_jump=current_curvature-previous_curvature,overlap_points=len(current_support))
            suspicious = jump > .20 or heading_delta>math.radians(30.) or (previous_curvature < .20 and common.any() and current_curvature > .75)
            if suspicious:
                # Vehicle-frame lateral offsets change as the vehicle turns.
                # Confirm a repeated camera measurement, including a straight
                # offset or a temporarily inferred wall, rather than requiring
                # a curve heading and both walls forever.
                consistent = False
                if self.pending_camera_path is not None:
                    difference = path[:,1]-np.interp(path[:,0],
                        self.pending_camera_path[:,0], self.pending_camera_path[:,1])
                    # The whole vehicle-coordinate corridor translates/rotates
                    # between images. Compare its shape after that affine
                    # change; a fixed 12 cm far-point test rejects normal yaw.
                    motion = np.polyfit(path[:,0], difference, 1)
                    consistent = (abs(motion[1]) <= .30 and abs(math.atan(motion[0])) <= .35
                        and np.max(np.abs(difference-np.polyval(motion,path[:,0]))) <= .12)
                self.pending_jump_count = self.pending_jump_count+1 if consistent else 1
                self.pending_camera_path = path.copy()
                if self.pending_jump_count < 2:
                    self.last_camera_issue='trajectory_jump'
                    self.ready_count = 0
                    return
            else:
                self.pending_jump_count = 0
                self.pending_camera_path = None
            path=temporal_merge(path,old,self.alpha)
        self.path, self.left, self.right = path.copy(), left.copy(), right.copy()
        self.last_camera_issue=''
        self.confidence, self.observed = confidence, observed
        self.camera_time, self.camera_stamp = now, stamp
        self.geometry_motion_time=now
        if self.visual_go and (self.official or not self.require_official):
            self.ready_count = min(self.confirm_frames, self.ready_count+1)
            if self.ready_count >= self.confirm_frames:
                self.path_confirmed = True
        else:
            self.ready_count = 0
            self.path_confirmed = False

    def lidar(self, obstacles, now, stamp=None):
        self.obstacles = obstacles
        self.lidar_time, self.lidar_stamp = now, stamp

    def _corridor_obstacles(self):
        result = []
        for obstacle in self.obstacles:
            if not 0 < obstacle.x <= self.horizon:
                continue
            point=np.array([[obstacle.x,obstacle.y]])
            _,inside,_=corridor_membership(point,self.path,self.left,self.right)
            if not inside[0]:continue
            from .lidar_filter import polyline_distance
            if min(polyline_distance(point,self.left)[0],polyline_distance(point,self.right)[0])<=.050001:continue
            if obstacle.confidence >= 1.0:
                result.append(obstacle)
        return sorted(result, key=lambda o: o.x)

    def _blocks_path(self, obstacle, path):
        return footprint_distance(path,obstacle)<self.width/2+self.margin

    def front_emergency(self, obstacles, path):
        # Only confirmed, wall-separated footprints from the detector are used.
        # Distance is to the footprint, never minimum raw scan range.
        # Include the vehicle-to-first-target segment when measured path support
        # starts beyond the near window. It is not extra steering support.
        corridor=np.vstack((np.zeros((1,2)),path))
        return next((o for o in obstacles if o.confidence>=1. and o.x>0 and
                     o.closest_distance<=self.front_obstacle_stop_distance and self._blocks_path(o,corridor)),None)

    def _candidate(self,obstacle,side):
        path,gap,_=self.pass_planner.candidate(self.path,self.left,self.right,obstacle,side)
        return path,gap

    def update_motion(self,now,twist):
        # Temporal smoothing compares geometry in one vehicle frame. Move the
        # old geometry with measured body twist; source stamps/freshness stay
        # unchanged. This is independent of the PP law and IMU pitch role.
        if twist is not None and self.geometry_motion_time is not None and self.path is not None:
            from .corridor_geometry import resample_x,drive_prefix
            dt=max(0.,min(.25,now-self.geometry_motion_time));vx,vy,w=twist;angle=w*dt
            dx=vx*dt if abs(w)<1e-6 else (vx*math.sin(angle)+vy*(math.cos(angle)-1))/w
            dy=vy*dt if abs(w)<1e-6 else (vx*(1-math.cos(angle))+vy*math.sin(angle))/w
            c,z=math.cos(angle),math.sin(angle);rotation=np.array([[c,-z],[z,c]])
            self.path=drive_prefix(resample_x((self.path-[dx,dy])@rotation))
            for name in ('left','right'):
                wall=getattr(self,name)
                if wall is not None:
                    moved=(wall-[dx,dy])@rotation
                    setattr(self,name,moved[(moved[:,0]>0)&(moved[:,0]<=self.horizon)])
            if not all(valid_boundary(getattr(self,name)) for name in ('left','right')):self.path=None
            self.geometry_motion_time=now
        self.pass_planner.integrate(now,twist)
        if self.last_selected is not None and self.pass_planner.last_motion is not None:
            from .corridor_geometry import resample_x,drive_prefix
            translation,rotation=self.pass_planner.last_motion
            self.last_selected=drive_prefix(resample_x((self.last_selected-translation)@rotation))

    def step(self, now):
        out = dict(path=None, mode='WAIT_LIGHT', reason='waiting_for_visual_start', speed=0.0,
                   enabled=False, left_gap=0.0, right_gap=0.0, obstacle_count=0, confidence=0.0,
                   ready_count=self.ready_count, ttc=math.inf, nearest_obstacle=None)
        if self.external_emergency:
            out.update(mode='EMERGENCY_STOP', reason='external_emergency')
            return out
        if not self.visual_go:
            if self.light == 'RED':out.update(mode='STOP_RED', reason='confirmed_visual_red')
            elif self.light == 'YELLOW':out.update(mode='STOP_YELLOW', reason='configured_yellow_stop')
            return out
        if self.require_official and not self.official:
            out['reason'] = 'waiting_for_official_start'
            return out
        age = now-self.camera_time
        if self.path is None or age > self.history_sec:
            self.path_confirmed = False
            self.ready_count = 0
            out.update(mode='WAIT_PATH' if self.path is None else 'EMERGENCY_STOP', reason='camera_path_lost')
            return out
        if now-self.lidar_time > self.lidar_timeout:
            out.update(mode='EMERGENCY_STOP', reason='lidar_timeout')
            return out
        if not self.controller_ready or not self.path_confirmed:
            out.update(mode='WAIT_PATH', reason='waiting_for_controller_or_path_confirmation')
            return out
        # An empty, fresh scan contains no footprint to spatially fuse. Do
        # not invalidate bounded camera history merely because a newer empty
        # scan arrived. Nonempty footprints still require synchronized geometry.
        if self.obstacles and self.camera_stamp is not None and self.lidar_stamp is not None:
            if abs(self.camera_stamp-self.lidar_stamp) > self.max_sensor_skew:
                out.update(mode='WAIT_PATH', reason='sensor_timestamp_skew')
                return out
        confidence = self.confidence
        if age > self.camera_timeout:
            confidence *= max(0.0, (self.history_sec-age)/(self.history_sec-self.camera_timeout))
        if confidence < self.min_confidence:
            out.update(mode='EMERGENCY_STOP', reason='camera_confidence_expired')
            return out
        path = self.path.copy()
        obstacles = self._corridor_obstacles()
        out['obstacle_count'] = len(obstacles)
        blocked = [o for o in obstacles if self._blocks_path(o,path)]
        if blocked:
            out['nearest_obstacle'] = (blocked[0].x,blocked[0].y)
        executed=self.pass_planner.safe_path if valid_path(self.pass_planner.safe_path) else self.last_selected if valid_path(self.last_selected) else path
        emergency=self.front_emergency(self.obstacles,executed)
        if emergency is not None:
            self.pass_planner.debug.update(obstacle_count=len(obstacles),blocking_obstacle=emergency.id,
                closest_obstacle_distance=emergency.closest_distance,obstacle_in_drive_corridor=True,
                emergency_0p5m=True,stop_reason='FRONT_OBSTACLE_0P5M')
            out.update(mode='EMERGENCY_STOP',reason='FRONT_OBSTACLE_0P5M',nearest_obstacle=(emergency.x,emergency.y))
            return out
        for obstacle in obstacles:
            if not self._blocks_path(obstacle,executed):continue
            available = max(0.0, obstacle.x-obstacle.depth/2-self.long_margin)
            ttc = available/max(obstacle.closing_speed,self.actual_speed,.1)
            if ttc <= self.emergency_ttc:
                out.update(mode='EMERGENCY_STOP',reason='critical_ttc',ttc=ttc)
                return out
        key=self.lidar_stamp if self.lidar_stamp is not None else now
        path,mode,reason=self.pass_planner.plan(path,self.left,self.right,obstacles,blocked,key,now)
        self.pass_side=self.pass_planner.side
        self.pass_observation=self.pass_planner.observation
        self.pass_id=self.pass_observation.id if self.pass_observation is not None else None
        self.pass_offset=self.pass_planner.offset
        debug=self.pass_planner.debug
        out.update(left_gap=debug.get('left_gap',0.),right_gap=debug.get('right_gap',0.))
        if path is None:
            out.update(mode=mode,reason=reason)
            return out
        if self.last_selected is not None:
            path=temporal_merge(path,self.last_selected,self.alpha)
        if not valid_path(path,self.horizon):
            out.update(mode='RECOVERY_STOP',reason='invalid_target')
            return out
        if mode in ('LEFT_PASS','RIGHT_PASS') and (not inside_corridor(path,self.left,self.right,self.width/2+self.margin) or any(self._blocks_path(o,path) for o in obstacles)):
            live=list(obstacles)
            if self.pass_observation is not None and all(o.id!=self.pass_observation.id for o in live):live.append(self.pass_observation)
            warm=self.pass_planner.validated_previous(self.left,self.right,live)
            if warm is not None:path=warm;reason='live_validated_previous_pass_geometry'
        if mode in ('LEFT_PASS','RIGHT_PASS','MERGE_CENTER') and not inside_corridor(path,self.left,self.right,self.width/2+self.margin):
            out.update(mode='STOP_BLOCKED',reason='blended_pass_wall_clearance')
            return out
        self.last_selected = path.copy()
        # Temporal merging must not turn a collision-free pass candidate back
        # into a path through the obstacle. Wait while the merge settles.
        for obstacle in obstacles:
            if self._blocks_path(obstacle,path):
                available = max(0.0, obstacle.x-obstacle.depth/2-self.long_margin)
                ttc = available/max(obstacle.closing_speed, self.actual_speed, .1)
                out.update(mode='EMERGENCY_STOP' if ttc <= self.emergency_ttc else 'WAIT_PASS_PATH',
                           reason='pass_trajectory_not_clear', ttc=ttc)
                return out
        self.pass_planner.note_selected(path,self.path,key)
        raw_curvature=float(np.max(np.abs(path_geometry(path)[1])))
        curvature=float(np.max(np.abs(supported_curvature(path))))
        lateral_limit=math.sqrt(self.max_lateral_accel/max(curvature,1e-6))
        speed=min(self.cruise,lateral_limit)
        out.update(raw_path_curvature=raw_curvature,supported_path_curvature=curvature,
                   lateral_acceleration_speed_limit=lateral_limit,
                   base_speed_limiter='PATH_CURVATURE' if lateral_limit<self.cruise else 'CRUISE')
        visibility_limit=self.cruise
        if path[-1,0]<self.horizon-1e-5:
            visible=path[0,0]+float(np.sum(np.linalg.norm(np.diff(path,axis=0),axis=1)))
            visible=max(visible,self.supported_preview_distance)
            visibility_limit=math.sqrt(2*self.max_decel*max(0.,visible-self.long_margin))
            if visibility_limit<speed:out['base_speed_limiter']='VISIBLE_PATH_LENGTH'
            speed=min(speed,visibility_limit)
        factor=float(np.clip((confidence-self.min_confidence)/(getattr(self,'full_speed_confidence',1.)-self.min_confidence), .25, 1.0))
        out.update(visible_path_speed_limit=visibility_limit,confidence_speed_factor=factor,
                   confidence_speed_limit=speed*factor)
        if factor<1.-1e-6:out['base_speed_limiter']='PATH_CONFIDENCE'
        speed*=factor
        if mode != 'CENTER':
            speed = min(speed, self.pass_planner.c.obstacle_pass_speed_mps)
        out['ttc_speed_limit']=self.cruise
        for obstacle in obstacles:
            if not self._blocks_path(obstacle,path):
                continue
            available = max(0.0, obstacle.x-obstacle.depth/2-self.long_margin)
            closing = max(obstacle.closing_speed, self.actual_speed, .1)
            ttc = available/closing
            out['ttc'] = min(out['ttc'], ttc)
            if ttc <= self.emergency_ttc:
                out.update(mode='EMERGENCY_STOP', reason='critical_ttc',ttc_speed_limit=0.)
                return out
            out['ttc_speed_limit']=min(out['ttc_speed_limit'],math.sqrt(2*self.max_decel*available))
            speed = min(speed, math.sqrt(2*self.max_decel*available))
            if ttc < self.caution_ttc:
                speed *= float(np.clip(ttc/self.caution_ttc, .2, 1.0))
                out['ttc_speed_limit']=min(out['ttc_speed_limit'],speed)
        out.update(path=path, mode=mode, reason=reason, speed=speed, enabled=True,
                   confidence=confidence, ready_count=self.ready_count)
        return out


    def recovery_step(self,now,context):
        """Use bounded predicted geometry; never bypass live obstacle safety."""
        out=dict(path=None,mode='EMERGENCY_STOP',reason='',speed=0.,enabled=False,left_gap=0.,right_gap=0.,
                 obstacle_count=len(self.obstacles),confidence=0.,ready_count=self.ready_count,ttc=math.inf,nearest_obstacle=None)
        if self.external_emergency:out['reason']='external_emergency';return out
        if not self.visual_go:out.update(mode='STOP_RED' if self.light=='RED' else 'WAIT_LIGHT',reason='confirmed_visual_red' if self.light=='RED' else 'waiting_for_visual_start');return out
        if self.require_official and not self.official:out['reason']='waiting_for_official_start';return out
        if not self.controller_ready:out['reason']='waiting_for_controller_or_path_confirmation';return out
        if now-self.lidar_time>self.lidar_timeout:out['reason']='lidar_timeout';return out
        paths=context['paths'];path=paths['selected']
        if not all((valid_path if name in ('center','selected') else valid_boundary)(p) for name,p in paths.items()):out['reason']='invalid_target';return out
        margin=self.width/2+self.margin
        if not inside_corridor(path,paths['left'],paths['right'],margin):
            out['reason']='corridor_lost';return out
        if self.obstacles and self.lidar_stamp is not None and abs(context['projection_stamp']-self.lidar_stamp)>self.max_sensor_skew:
            out['reason']='sensor_timestamp_skew';return out
        emergency=self.front_emergency(self.obstacles,path)
        if emergency is not None:
            out.update(reason='FRONT_OBSTACLE_0P5M',nearest_obstacle=(emergency.x,emergency.y));return out
        for obstacle in self.obstacles:
            if not self._blocks_path(obstacle,path):continue
            available=max(0.,obstacle.x-obstacle.depth/2-self.long_margin)
            ttc=available/max(obstacle.closing_speed,self.actual_speed,.1)
            out.update(ttc=min(out['ttc'],ttc),nearest_obstacle=(obstacle.x,obstacle.y))
            out['reason']='critical_ttc' if ttc<=self.emergency_ttc else 'confirmed_collision_risk'
            return out
        curvature=float(np.max(np.abs(supported_curvature(path))))
        speed=min(context['hold_speed'],math.sqrt(self.max_lateral_accel/max(curvature,1e-6)))
        mode=self.pass_side if self.pass_side!='CENTER' else 'CORNER_PATH_HOLD'
        out.update(path=path.copy(),mode=mode,reason=context['issue'],speed=speed,enabled=True,
                   confidence=context['source_confidence'],ready_count=self.ready_count)
        return out
