"""Measured RGB-D wall geometry. No image-plane curve, map or steering output."""
from dataclasses import dataclass
import math
import numpy as np
from .camera_mount import rotation
from .path_curvature import supported_curvature

@dataclass
class DepthConfig:
    horizon_m: float = 32.
    station_m: float = .5
    min_boundary_points: int = 3
    min_wall_height_m: float = .04
    max_wall_height_m: float = .32
    min_width_m: float = .55
    max_width_m: float = 4.
    min_support_length_m: float = 3.
    min_curve_length_m: float = 2.
    min_curve_heading_deg: float = 5.
    min_confidence: float = .75
    timeout_sec: float = .35
    corner_confirm_frames: int = 3
    straight_confirm_frames: int = 4
    latch_timeout_sec: float = 2.
    odom_timeout_sec: float = .25
    association_distance_m: float = 2.
    correction_gain: float = .25
    fallback_speed_mps: float = 3.5

    def validate(self):
        values=vars(self)
        if not all(math.isfinite(v) and v>0 for v in values.values()):raise ValueError('invalid Depth parameters')
        if not (self.horizon_m<=32 and self.min_width_m<self.max_width_m and self.min_wall_height_m<self.max_wall_height_m and
            self.min_confidence<=1 and self.corner_confirm_frames>=2 and self.straight_confirm_frames>=2 and
            self.correction_gain<=1 and self.fallback_speed_mps<=3.5 and self.timeout_sec<=.5):raise ValueError('invalid Depth safety parameters')

def depth_array(msg):
    if msg.encoding not in ('32FC1','16UC1'):raise ValueError('unsupported depth encoding')
    size=4 if msg.encoding=='32FC1' else 2
    dtype=np.dtype(('>' if msg.is_bigendian else '<')+('f4' if size==4 else 'u2'))
    if min(msg.width,msg.height)<=0 or msg.step<msg.width*size or len(msg.data)<msg.step*msg.height:raise ValueError('invalid depth stride')
    a=np.ndarray((msg.height,msg.width),dtype=dtype,buffer=bytes(msg.data),strides=(msg.step,size)).astype(float)
    return a if size==4 else a*.001


def project_depth(depth,k,camera_pose,min_range,max_range,*,z_bounds=None,x_bounds=None):
    """Optical axial Z -> base (+forward,+left,+up), with actual mount/K."""
    k=np.asarray(k,dtype=float).reshape(3,3)
    if not np.isfinite(k).all() or min(k[0,0],k[1,1])<=0:raise ValueError('invalid intrinsics')
    v,u=np.indices(depth.shape)
    valid=np.isfinite(depth)&(depth>min_range)&(depth<max_range)
    z=depth[valid]
    optical=np.column_stack(((u[valid]-k[0,2])*z/k[0,0],(v[valid]-k[1,2])*z/k[1,1],z))
    r=rotation(*camera_pose[3:])@rotation(-math.pi/2,0,-math.pi/2)
    # A 3x3 rotation does not need a threaded BLAS job per camera frame.
    # Componentwise arithmetic has identical pinhole/TF geometry and bounded
    # scheduling overhead alongside Gazebo and perception workers.
    # Remove the ground/sky band before allocating/translating all XYZ points.
    # Use the identical arithmetic and strict height limits as the consumers.
    height=optical[:,0]*r[2,0]+optical[:,1]*r[2,1]+optical[:,2]*r[2,2]+camera_pose[2]
    if z_bounds is not None:
        keep=(height>z_bounds[0])&(height<z_bounds[1])
        optical=optical[keep];height=height[keep]
    base=np.empty_like(optical)
    for axis in (0,1):
        base[:,axis]=optical[:,0]*r[axis,0]+optical[:,1]*r[axis,1]+optical[:,2]*r[axis,2]+camera_pose[axis]
    base[:,2]=height
    if x_bounds is not None:base=base[(base[:,0]>=x_bounds[0])&(base[:,0]<=x_bounds[1])]
    return base


def center_geometry(points,ground_z,c=None):
    c=c or DepthConfig();p=np.asarray(points,dtype=float).reshape(-1,3)
    p=p[np.isfinite(p).all(1)]
    p=p[(p[:,0]>=.5)&(p[:,0]<=c.horizon_m)&
        (p[:,2]>ground_z+c.min_wall_height_m)&(p[:,2]<ground_z+c.max_wall_height_m)]
    # Sort once instead of scanning the complete cloud for every station.
    p=p[np.argsort(p[:,0],kind='stable')]
    rows=[];counts=[];width=None
    # Each station uses actual observed wall pairs. No single-wall width invention.
    for x in np.r_[np.arange(1.,min(8.,c.horizon_m)+.001,c.station_m),np.arange(9.,c.horizon_m+.001,1.)]:
        lo=np.searchsorted(p[:,0],np.nextafter(x-.5,-np.inf),side='left')
        hi=np.searchsorted(p[:,0],np.nextafter(x+.5,np.inf),side='right')
        a=p[lo:hi];a=a[abs(a[:,0]-x)<.5]
        if len(a)<2*c.min_boundary_points:continue
        a=a[np.argsort(a[:,1])];groups=np.split(a,np.flatnonzero(np.diff(a[:,1])>.25)+1)
        groups=[g for g in groups if len(g)>=c.min_boundary_points and np.ptp(g[:,2])>=.025]
        candidates=[]
        prediction=rows[-1][1] if rows else 0.
        if len(rows)>1:
            prediction+=(x-rows[-1][0])*(rows[-1][1]-rows[-2][1])/(rows[-1][0]-rows[-2][0])
        for i,right in enumerate(groups):
            for left in groups[i+1:]:
                ry,ly=np.median(right[:,1]),np.median(left[:,1]);w=ly-ry;mid=(ry+ly)/2
                if not c.min_width_m<=w<=c.max_width_m:continue
                if width is not None and abs(w-width)>max(.6,.3*width):continue
                if rows and abs(mid-prediction)>.7:continue
                cost=abs(mid-prediction)+(abs(w-width) if width else 0.)
                candidates.append((cost,mid,w,len(right)+len(left)))
        if not candidates:continue
        _,y,w,n=min(candidates)
        if rows and x-rows[-1][0]>1.6:break
        rows.append([float(x),float(y)]);counts.append(n);width=w if width is None else .8*width+.2*w
    a=np.asarray(rows,dtype=float).reshape(-1,2)
    return a,counts


def analyze_center(points,counts,c=None,curvature_thresholds=(.08,.18,.30),heading_thresholds=(8.,18.,30.)):
    """Segments require observed continuity and >=2m of coherent curvature."""
    c=c or DepthConfig();a=np.asarray(points,dtype=float).reshape(-1,2)
    result=dict(depth_valid=False,depth_supported_distance_m=float(a[-1,0]) if len(a) else 0.,
                depth_curvature_supported_distance_m=0.,segments=[],center=a.tolist(),
                depth_corner_candidate=False,level=0,direction=0,distance=None,confidence=0.,curvature=0.)
    if len(a)<5 or np.any(np.diff(a[:,0])>1.6):return result
    # Reject isolated station errors; do not manufacture a replacement observation.
    residual=np.abs(a[1:-1,1]-(a[:-2,1]+a[2:,1])/2)
    isolated=np.flatnonzero((residual>.18)&(np.abs(a[2:,1]-a[:-2,1])<.2))+1
    if len(isolated):
        keep=np.ones(len(a),dtype=bool);keep[isolated]=False
        a=a[keep];counts=np.asarray(counts)[keep].tolist()
    if len(a)<5 or np.any(np.diff(a[:,0])>2.1):return result
    # Local quadratic regression over five actual forward stations suppresses
    # depth raster quantization without extending observed support.
    y=np.array([np.polyval(np.polyfit(a[max(0,i-2):min(len(a),i+3),0]-a[i,0],a[max(0,i-2):min(len(a),i+3),1],2),0.) for i in range(len(a))])
    smooth=np.column_stack((a[:,0],y));h=np.unwrap(np.arctan2(np.gradient(y),np.gradient(a[:,0])))
    k=np.gradient(h)/np.hypot(np.gradient(a[:,0]),np.gradient(y))
    # Curvature endpoints lack symmetric support; never certify them.
    observed=np.zeros(len(a),bool);observed[1:-1]=True
    support=float(a[-2,0]);segments=[]
    for lo in np.arange(0., c.horizon_m, 5.):
        hi = min(lo+5., c.horizon_m)
        mask=observed&(a[:,0]>=lo)&(a[:,0]<=hi);idx=np.flatnonzero(mask)
        if len(idx)<3:continue
        q=smooth[idx];length=float(np.linalg.norm(np.diff(q,axis=0),axis=1).sum())
        # Reuse finite-chord Menger curvature as an independent robust metric.
        chord=supported_curvature(q)
        segments.append(dict(start_m=float(q[0,0]),end_m=float(q[-1,0]),support_count=int(sum(counts[i] for i in idx)),
            station_count=len(idx),support_length=length,median_curvature=float(np.median(chord)),
            p90_curvature=float(np.percentile(abs(chord),90)),heading_change=float(math.degrees(h[idx[-1]]-h[idx[0]])),
            lateral_change=float(q[-1,1]-q[0,1]),confidence=.9))
    # Runs of significant signed curvature. At least three distinct stations,
    # supported arc >=2m and same-direction accumulated heading are required.
    runs=[];run=[]
    for i in range(1,len(a)-1):
        sign=1 if k[i]>0 else -1
        significant=abs(k[i])>=curvature_thresholds[0]
        if significant and (not run or k[run[-1]]*k[i]>0):run.append(i)
        else:
            if run:runs.append(run)
            run=[i] if significant else []
    if run:runs.append(run)
    for run in runs:
        if len(run)<3:continue
        # Each interior tangent/curvature is supported by its adjacent measured
        # stations; include that observed chord support in the arc-length test.
        first,last=max(0,run[0]-1),min(len(a)-1,run[-1]+1)
        length=float(np.linalg.norm(np.diff(smooth[first:last+1],axis=0),axis=1).sum())
        heading=abs(float(math.degrees(h[last]-h[first])))
        if length<c.min_curve_length_m or heading<c.min_curve_heading_deg:continue
        robust=float(np.percentile(abs(k[run]),75));level=min(sum(robust>=v for v in curvature_thresholds),sum(heading>=v for v in heading_thresholds))
        if level:
            result.update(depth_corner_candidate=True,level=level,direction=1 if np.median(k[run])>0 else -1,
                          distance=float(a[run[0],0]),curvature=robust);break
    length=float(a[-1,0]-a[0,0]);confidence=.9 if length>=c.min_support_length_m and segments else .65
    result.update(depth_valid=length>=c.min_support_length_m and bool(segments) and confidence>=c.min_confidence,
        depth_curvature_supported_distance_m=support,segments=segments,center=smooth.tolist(),confidence=confidence,
        straight_geometry=bool(max(abs(k[1:-1]),default=0.)<.06 and math.degrees(np.ptp(h[1:-1]))<6.),
        heading_change_deg=float(math.degrees(np.ptp(h[1:-1]))))
    return result

def wall_preview(points,ground_z,c=None,curvature_thresholds=(.08,.18,.30),heading_thresholds=(8.,18.,30.)):
    """Normal-offset preview from a continuous observed wall and measured width.

    Allows a bend whose two walls stop sharing forward X. Width must be
    independently re-observed from both near walls in this same frame.
    """
    from .lidar_track import LidarTrack,LidarTrackConfig
    from .lidar_filter import groups
    from .corridor_geometry import basis,progress
    c=c or DepthConfig();q=np.asarray(points)
    q=q[(q[:,2]>ground_z+c.min_wall_height_m)&(q[:,2]<ground_z+c.max_wall_height_m)&(q[:,0]>.5)&(q[:,0]<=c.horizon_m)]
    if len(q)<30:return None
    key=np.round(np.arctan2(q[:,1],q[:,0])/.004).astype(int)
    # Exact per-bin coordinate medians, with two bulk sorts instead of hundreds
    # of Python/NumPy median calls. Keep the same ray bins and minimum counts.
    _,counts=np.unique(key,return_counts=True)
    end=np.cumsum(counts);start=end-counts
    lower=(start+end-1)//2;upper=(start+end)//2
    medians=[]
    for axis in (0,1):
        values=q[np.lexsort((q[:,axis],key)),axis]
        medians.append((values[lower]+values[upper])/2)
    xy=np.column_stack(medians)[counts>=c.min_boundary_points]
    if len(xy)<12:return None
    near=xy[(xy[:,0]>=1)&(xy[:,0]<=4)]
    right=near[near[:,1]<-.25];left=near[near[:,1]>.25]
    if min(len(left),len(right))<6:return None
    width=float(np.median(left[:,1])-np.median(right[:,1]))
    if not c.min_width_m<=width<=c.max_width_m:return None
    tracker=LidarTrack(LidarTrackConfig(horizon_m=c.horizon_m,min_wall_points=6,cluster_gap_m=.6,single_wall_enabled=False),batch_smoothing=True)
    candidates=[]
    for ids in groups(xy,np.arange(len(xy)),.6):
        a=xy[ids]
        if len(a)<8:continue
        near_a=a[a[:,0]<=np.percentile(a[:,0],25)+.2]
        side=1 if np.median(near_a[:,1])>0 else -1
        if abs(np.median(near_a[:,1])-side*width/2)>.6:continue
        wall,error=tracker._boundary(a)
        if wall is None or progress(wall)[-1]<4:continue
        _,normal=basis(wall);center=wall-side*width/2*normal
        # An X-fold is beyond forward preview, never turn it into a polynomial.
        bad=np.flatnonzero(np.diff(center[:,0])<=0)
        if len(bad):center=center[:bad[0]+1]
        if len(center)<6:continue
        start=max(1.,center[0,0]);end=min(c.horizon_m,center[-1,0])
        x=np.arange(math.ceil(start*2)/2,end+1e-8,.5)
        if len(x)<5:continue
        sampled=np.column_stack((x,np.interp(x,center[:,0],center[:,1])))
        # Audit independent measured rays, rather than counting resampled wall
        # points as new observations.
        nearest=np.argmin(np.linalg.norm(a[:,None,:]-wall[None,:,:],axis=2),axis=1)
        raw_center=a-side*width/2*normal[nearest]
        counts=[int(np.count_nonzero(abs(raw_center[:,0]-xi)<.5)) for xi in x]
        r=analyze_center(sampled,counts,c,curvature_thresholds,heading_thresholds)
        for segment in r.get('segments',[]):
            segment['support_count']=int(np.count_nonzero((raw_center[:,0]>=segment['start_m'])&(raw_center[:,0]<=segment['end_m'])))
        if not r['depth_valid'] or not r['depth_corner_candidate']:continue
        # Count original, independent depth rays across the detected curve.
        support=a[(a[:,0]>=r['distance']-.5)&(a[:,0]<=min(end,r['distance']+4))]
        if len(support)<8 or np.linalg.norm(np.ptp(support,axis=0))<c.min_curve_length_m:continue
        r.update(confidence=.8,geometry_source='DEPTH_WALL_NORMAL_OFFSET',observed_track_width_m=width)
        candidates.append(r)
    return min(candidates,key=lambda r:r['distance']) if candidates else None
