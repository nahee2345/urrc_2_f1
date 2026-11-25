"""Camera-defined curved corridors; LiDAR supplies compact objects only."""
from dataclasses import dataclass
import math
import numpy as np
from .corridor_geometry import corridor_membership,boundary_ok,wall_tangent


@dataclass
class WallFilterConfig:
    use_camera_wall_filter: bool = True
    wall_reject_margin_m: float = .06
    min_obstacle_wall_separation_m: float = .08
    path_obstacle_half_width_m: float = 0.0  # SDF width/2 + existing lateral margin
    max_camera_lidar_age_diff_sec: float = .15
    camera_timeout_sec: float = .25
    min_boundary_confidence: float = .40
    high_boundary_confidence: float = .75
    wall_alignment_cosine: float = .94
    wall_aspect_ratio: float = 5.0
    wall_shape_region_m: float = .12


def good_polyline(path):
    return (isinstance(path, np.ndarray) and path.ndim == 2 and path.shape[1] == 2 and
            len(path) >= 3 and np.isfinite(path).all() and np.all(np.diff(path[:, 0]) > 0))


def polyline_distance(points, path):
    points = np.asarray(points, dtype=float).reshape(-1, 2)
    if len(points) == 0:
        return np.empty(0)
    a, delta = path[:-1], np.diff(path, axis=0)
    v = points[:, None, :]-a[None, :, :]
    t = np.clip(np.sum(v*delta, axis=2)/np.maximum(np.sum(delta*delta, axis=1), 1e-12), 0, 1)
    return np.sqrt(np.min(np.sum((v-t[:, :, None]*delta)**2, axis=2), axis=1))


def scan_points(ranges, angle_min, angle_increment, range_min, range_max,
                rotation, translation, horizon=5.0, min_valid_beams=20):
    if (not ranges or not all(math.isfinite(v) for v in (angle_min, angle_increment, range_min, range_max))
            or angle_increment <= 0 or range_min < 0 or range_max <= range_min):
        return False, np.empty((0, 2)), np.empty(0, dtype=int)
    r = np.asarray(ranges, dtype=float)
    a = angle_min+np.arange(len(r))*angle_increment
    front = (a >= -math.pi/2) & (a <= math.pi/2)
    finite = np.isfinite(r) & (r >= range_min) & (r <= range_max)
    valid = np.count_nonzero(front & (finite | np.isposinf(r))) >= min_valid_beams
    indices = np.flatnonzero(front & finite & (r <= horizon))
    local = np.column_stack((r[indices]*np.cos(a[indices]), r[indices]*np.sin(a[indices]), np.zeros(len(indices))))
    points = (local@np.asarray(rotation).T+np.asarray(translation))[:, :2]
    keep = points[:, 0] > 0
    return bool(valid), points[keep], indices[keep]


def groups(points, indices, gap):
    if len(points) == 0:
        return []
    splits = np.flatnonzero((np.diff(indices) != 1) | (np.linalg.norm(np.diff(points, axis=0), axis=1) > gap))+1
    return np.split(np.arange(len(points)), splits)


class CameraWallFilter:
    def __init__(self, vehicle_width, lateral_margin=.15, config=None):
        self.c = config or WallFilterConfig()
        self.path_half_width = self.c.path_obstacle_half_width_m or vehicle_width/2+lateral_margin
        if (self.c.wall_reject_margin_m < 0 or self.c.min_obstacle_wall_separation_m < 0 or
                self.path_half_width < vehicle_width/2 or self.c.camera_timeout_sec <= 0 or
                self.c.max_camera_lidar_age_diff_sec <= 0):
            raise ValueError('invalid camera wall filter parameters')
        self.camera = None
        self.selected = None
        self.latest_invalid = False
        self.recovery_context = None
        self.lidar_context = None

    def update_lidar_corridor(self,center,left,right,confidence,observed,stamp,now):
        """Independent measured walls, usable only with their exact raw scan."""
        if self.lidar_context is not None and stamp<self.lidar_context['stamp']:return False
        if (not good_polyline(center) or not all(boundary_ok(p) for p in (left,right)) or
                not math.isfinite(confidence) or not self.c.min_boundary_confidence<=confidence<=1 or not any(observed)):
            return False
        from .obstacle_pass import inside_corridor
        if not inside_corridor(center,left,right,0.):return False
        self.lidar_context=dict(center=center.copy(),left=left.copy(),right=right.copy(),confidence=confidence,
            observed=observed,stamp=stamp,time=now,lo=center[0,0],hi=center[-1,0],independent_lidar=True)
        return True

    def update_camera(self, center, left, right, confidence, observed, stamp, now):
        if self.camera is not None and stamp <= self.camera['stamp']:
            return False
        if (not good_polyline(center) or not all(boundary_ok(p) for p in (left,right)) or
                not math.isfinite(confidence) or not self.c.min_boundary_confidence <= confidence <= 1 or
                not any(observed)):
            self.latest_invalid = True
            return False
        from .obstacle_pass import inside_corridor
        if not inside_corridor(center,left,right,0.):
            self.latest_invalid=True
            return False
        self.camera = dict(center=center.copy(),left=left.copy(),right=right.copy(),confidence=confidence,
                           observed=observed,stamp=stamp,time=now,lo=center[0,0],hi=center[-1,0])
        self.latest_invalid = False
        return True

    def update_selected(self, path, stamp, now):
        if good_polyline(path):
            self.selected = dict(path=path.copy(), stamp=stamp, time=now)

    def context(self, stamp, now):
        if self.recovery_context is not None:
            cam=self.recovery_context
            if now-cam['time']<=self.c.camera_timeout_sec and abs(stamp-cam['stamp'])<=self.c.max_camera_lidar_age_diff_sec:return cam
        cam = self.camera
        if (cam is None or now-cam['time'] > self.c.camera_timeout_sec or
                abs(stamp-cam['stamp']) > self.c.max_camera_lidar_age_diff_sec):
            measured=self.lidar_context
            if (measured is not None and 0<=now-measured['time']<=self.c.camera_timeout_sec and
                    abs(stamp-measured['stamp'])<=1e-6):return measured
            return None
        return cam

    def distances(self, points, cam):
        return polyline_distance(points, cam['left']), polyline_distance(points, cam['right'])

    def path_distances(self, points, cam, stamp, now):
        distance = polyline_distance(points, cam['center'])
        if cam.get('recovery'):
            return np.minimum(distance,polyline_distance(points,cam['selected']))
        selected = self.selected
        if (selected is not None and now-selected['time'] <= self.c.camera_timeout_sec and
                abs(stamp-selected['stamp']) <= self.c.max_camera_lidar_age_diff_sec):
            # Keep base-path objects relevant while a pass trajectory moves
            # away from them; otherwise feedback would erase an active track.
            distance = np.minimum(distance, polyline_distance(points, selected['path']))
        return distance

    def tracked_relevant(self, obstacle, stamp, now):
        cam = self.context(stamp, now)
        if cam is None:return False
        point=np.array([[obstacle.x,obstacle.y]])
        _,inside,_=corridor_membership(point,cam['center'],cam['left'],cam['right'])
        separation=min(float(d[0]) for d in self.distances(point,cam))
        return bool(inside[0] and separation>=self.c.min_obstacle_wall_separation_m and self.path_distances(point,cam,stamp,now)[0]<=self.path_half_width+obstacle.width/2)

    def classify(self, points, indices, stamp, now, *, cluster_gap=.14, min_width=.12,
                 max_width=.80, min_depth=.10, max_depth=1.20):
        cam = self.context(stamp, now)
        debug = dict(raw_points=len(points), wall_rejected_points=0, corridor_points=0, clusters=0,
                     wall_clusters=0, obstacle_candidates=0, confirmed_obstacles=0, path_blocking_obstacles=0,
                     state='NO_FRESH_CAMERA_CORRIDOR', camera_age=None, camera_scan_skew=None, details=[])
        if cam is None or not self.c.use_camera_wall_filter:
            # No fallback to raw frontal ranges. Withhold observations so the
            # existing Decision sensor/camera watchdogs provide the safe stop.
            return False, [], debug
        debug.update(state='LIDAR_CORRIDOR' if cam.get('independent_lidar') else 'CORNER_PATH_HOLD_CORRIDOR' if cam.get('recovery') else 'HELD_CAMERA_CORRIDOR' if self.latest_invalid else 'CAMERA_CORRIDOR',
                     trajectory_origin_stamp=cam.get('source_stamp'),trajectory_hold_age=cam.get('source_age'),
                     camera_age=now-cam['time'], camera_scan_skew=abs(stamp-cam['stamp']))
        if len(points) == 0:
            return True, [], debug
        dl,dr=self.distances(points,cam);nearest=np.minimum(dl,dr)
        in_x,inside,normal_width=corridor_membership(points,cam['center'],cam['left'],cam['right'])
        high = cam['confidence'] >= self.c.high_boundary_confidence and all(cam['observed'])
        margin = self.c.wall_reject_margin_m*(1.0 if high else .5)
        wall = in_x & (nearest <= margin)
        # Same-stamp track-branch labels are shared, but only a supported
        # wall band may suppress a return; a near-wall compact car is retained.
        shared=np.isin(indices,list(getattr(self,'shared_wall_indices',())))
        shared_confirmed=shared & in_x & (nearest<=margin)
        wall |= shared_confirmed
        debug['shared_track_wall_points']=int(np.count_nonzero(shared_confirmed))
        keep = inside & ~wall
        debug['wall_rejected_points'] = int(np.count_nonzero(wall))
        debug['corridor_points'] = int(np.count_nonzero(keep))
        # Raw groups are diagnostics/context only. Obstacle clustering below
        # uses only the remaining corridor points, never these wall returns.
        for ids in groups(points, indices, cluster_gap):
            if np.any(keep[ids]):
                continue
            classification = 'WALL_REGION' if np.any(wall[ids]) else 'OUTSIDE_CORRIDOR'
            cloud = points[ids]
            centered = cloud-cloud.mean(axis=0)
            _, eigenvectors = np.linalg.eigh(centered.T@centered/max(1,len(cloud)))
            direction = eigenvectors[:,-1]
            length = float(np.ptp(centered@direction))
            thickness = float(np.ptp(centered@np.array([-direction[1],direction[0]])))
            debug['details'].append(dict(cluster_id='raw_'+str(int(indices[ids[0]])), classification=classification,
                point_count=len(ids), center_x=float(cloud[:,0].mean()), center_y=float(cloud[:,1].mean()),
                longitudinal_extent=float(np.ptp(cloud[:,0])), lateral_extent=float(np.ptp(cloud[:,1])),
                length=length,width=thickness,aspect_ratio=length/max(thickness,.01),
                distance_to_left_wall=float(np.min(dl[ids])),distance_to_right_wall=float(np.min(dr[ids])),
                inside_drivable_corridor=bool(np.mean(inside[ids])>.5),
                inside_path_corridor=bool(np.any(self.path_distances(cloud,cam,stamp,now)<=self.path_half_width)),
                wall_connected=bool(np.mean(wall[ids])>.8)))
            debug['wall_clusters'] += int(classification == 'WALL_REGION')
        kept_points, kept_indices = points[keep], indices[keep]
        candidates = []
        for ids in groups(kept_points, kept_indices, cluster_gap):
            cloud = kept_points[ids]
            debug['clusters'] += 1
            low, high_pt = cloud.min(axis=0), cloud.max(axis=0)
            depth, width = high_pt-low
            center = (low+high_pt)/2
            a, b = self.distances(cloud, cam)
            closest = np.minimum(a, b)
            centered = cloud-cloud.mean(axis=0)
            covariance = centered.T@centered/max(1,len(cloud))
            eigenvalues, eigenvectors = np.linalg.eigh(covariance)
            direction = eigenvectors[:,-1]
            length = float(np.ptp(centered@direction))
            thickness = float(np.ptp(centered@np.array([-direction[1],direction[0]])))
            aspect = length/max(thickness,.01)
            boundary = cam['left'] if np.mean(a) < np.mean(b) else cam['right']
            tangent=wall_tangent(center,boundary)
            aligned = abs(float(direction@tangent)) >= self.c.wall_alignment_cosine
            near_fraction = float(np.mean(closest < self.c.wall_shape_region_m))
            wall_connected = (near_fraction >= .8 and aligned and aspect >= self.c.wall_aspect_ratio and
                              (length > max_width or float(np.median(closest)) < self.c.min_obstacle_wall_separation_m))
            separated = float(np.median(closest)) >= self.c.min_obstacle_wall_separation_m
            path_blocking = bool(np.any(self.path_distances(cloud, cam, stamp, now) <= self.path_half_width))
            # A transverse barrier can exceed a competing car's nominal
            # width. Never discard a separated, path-blocking footprint merely
            # for being wide; its gaps must reach STOP_BLOCKED downstream.
            corridor_width=float(corridor_membership(center[None,:],cam['center'],cam['left'],cam['right'])[2][0])
            broad_blocker=path_blocking and not aligned and width<=corridor_width and depth<=max_depth
            compact = len(cloud) >= 3 and min_width <= width and (width<=max_width or broad_blocker) and depth <= max_depth
            if wall_connected:
                classification = 'WALL_CONNECTED'; debug['wall_clusters'] += 1
            elif not separated:
                classification = 'WALL_REGION'; debug['wall_clusters'] += 1
            elif not compact:
                classification = 'NON_COMPACT_CLUSTER'
            elif not path_blocking:
                classification = 'NON_BLOCKING_OBJECT'
            else:
                classification = 'PATH_BLOCKING_OBSTACLE'
                candidates.append((float(center[0]),float(center[1]),float(width),float(max(depth,min_depth))))
            debug['details'].append(dict(cluster_id='scan_'+str(int(kept_indices[ids[0]])), center_x=float(center[0]),
                center_y=float(center[1]), point_count=len(cloud), longitudinal_extent=float(depth), lateral_extent=float(width),
                length=length, width=thickness, aspect_ratio=aspect, distance_to_left_wall=float(np.min(a)),
                distance_to_right_wall=float(np.min(b)), inside_drivable_corridor=True, inside_path_corridor=path_blocking,
                wall_connected=wall_connected, classification=classification))
        debug['obstacle_candidates'] = len(candidates)
        return True, candidates, debug
